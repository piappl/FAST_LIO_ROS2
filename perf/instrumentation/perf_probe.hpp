// perf/instrumentation/perf_probe.hpp
//
// Opt-in, dependency-free instrumentation for FAST_LIO_ROS2.
//
// WHY: from the outside you can only see that fastlio_mapping died. This probe
// records the internal state that explains WHY -- above all the two buffer
// depths and the LiDAR/IMU association, which is where a live sensor differs
// from a bag replay.
//
// ACTIVATION: entirely by environment variable, decided once at first use.
//
//     export FASTLIO_PERF_LOG=/tmp/run1/perf_probe      # writes _scan.csv + _events.csv
//     unset  FASTLIO_PERF_LOG                           # probe off
//
// When off, every hook is an inlined load of a bool and a return.
//
// Deliberately depends on nothing but the C++ standard library: no ROS, no PCL,
// no Eigen. That keeps it compilable and testable in isolation, and means it
// cannot perturb the types it observes.
//
// THREAD SAFETY: counters are atomics; the two output streams are guarded by
// one mutex each. In the stock build everything runs on the single
// rclcpp::spin() thread anyway.

#ifndef FASTLIO_PERF_PROBE_HPP
#define FASTLIO_PERF_PROBE_HPP

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

namespace flperf {

// ---------------------------------------------------------------- utilities --
inline double wall_now()
{
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// Resident set size in MB, straight from /proc/self/statm (page count).
inline double rss_mb()
{
  std::FILE* f = std::fopen("/proc/self/statm", "r");
  if (!f) return -1.0;
  long total = 0, resident = 0;
  const int n = std::fscanf(f, "%ld %ld", &total, &resident);
  std::fclose(f);
  if (n != 2) return -1.0;
  static const double page_mb =
      static_cast<double>(::sysconf(_SC_PAGESIZE)) / (1024.0 * 1024.0);
  return static_cast<double>(resident) * page_mb;
}

// ------------------------------------------------------------- scan record --
// One of these is filled per processed LiDAR frame and handed to on_scan_done().
struct ScanRecord
{
  // stage timings [s]
  double t_imu_process = 0.0;   // p_imu->Process (undistort + propagate)
  double t_downsample = 0.0;    // VoxelGrid
  double t_icp = 0.0;           // kf.update_iterated_dyn_share_modified
  double t_incremental = 0.0;   // map_incremental (ikd-Tree insert)
  double t_publish = 0.0;       // all publish_* calls
  double t_total = 0.0;         // whole timer_callback body

  // sizes
  unsigned long pts_in = 0;      // feats_undistort
  unsigned long pts_down = 0;    // feats_down_body
  unsigned long eff_feat = 0;    // effct_feat_num
  unsigned long tree_size = 0;   // ikdtree.size()

  // state (for divergence detection)
  double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
  double vel_norm = 0.0;
  double bg_norm = 0.0;
  double ba_norm = 0.0;

  // Online-estimated LiDAR->IMU translation (state_point.offset_T_L_I). With
  // mapping.extrinsic_est_en:true these converge to the sensor's true internal
  // offset, so a run started from zeros MEASURES the extrinsic for you.
  double ext_t_x = 0.0, ext_t_y = 0.0, ext_t_z = 0.0;

  // ---- pose stability -----------------------------------------------------
  // Filled only when the probe is enabled: the caller computes all of it inside
  // its own `if (flperf::enabled())` block, so this costs nothing when off.

  // Attitude, degrees. A stationary sensor whose POSITION wanders usually has an
  // ATTITUDE problem underneath: a fraction of a degree of roll/pitch error
  // leaves a residual gravity component that the EKF integrates into velocity,
  // and from there into position.
  double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

  // Estimated gravity in the world frame. Should sit at (0, 0, -9.81); a
  // persistent horizontal component is the tilt error described above.
  double grav_x = 0.0, grav_y = 0.0, grav_z = 0.0;

  // Full bias vectors, not just the norms: drift confined to one axis is
  // usually a bias on one axis.
  double bg_x = 0.0, bg_y = 0.0, bg_z = 0.0;
  double ba_x = 0.0, ba_y = 0.0, ba_z = 0.0;

  // Mean point-to-plane residual over the accepted correspondences.
  double res_mean = 0.0;

  // Raw IMU motion, straight off the wire and independent of the filter. This is
  // what says "the platform was actually still" -- the estimated velocity cannot
  // be trusted to say it, since that is the thing under suspicion.
  double imu_gyr_mean = 0.0;   // mean |omega| over this scan's IMU batch, rad/s
  double imu_acc_std = 0.0;    // std of |a| over the same batch, m/s^2

  // Translation observability: eigenvalues (ascending) of the mean plane-normal
  // scatter matrix over the correspondences the EKF actually used. Three roughly
  // equal values near 0.33 means every direction is constrained. A small obs_min
  // means the geometry does not pin the pose down along obs_weak_*, and the
  // estimate is free to slide there -- a corridor, a single flat wall, an
  // empty room.
  double obs_min = 0.0, obs_mid = 0.0, obs_max = 0.0;
  double obs_weak_x = 0.0, obs_weak_y = 0.0, obs_weak_z = 0.0;
};

// -------------------------------------------------------------------- probe --
class Probe
{
 public:
  static Probe& get()
  {
    static Probe p;
    return p;
  }

  bool enabled() const { return enabled_; }

  // ---- message callbacks -------------------------------------------------
  // Records the wall gap between consecutive invocations. A large gap means the
  // executor did not get around to us: with rclcpp::spin() (single threaded)
  // the 100 Hz timer_callback and these callbacks share ONE thread, so a long
  // ICP directly starves the IMU intake.
  void on_imu_msg(double header_stamp_sec)
  {
    if (!enabled_) return;
    const double now = wall_now();
    imu_msgs_.fetch_add(1, std::memory_order_relaxed);
    const double prev = last_imu_cb_wall_.exchange(now, std::memory_order_relaxed);
    if (prev > 0.0) {
      const double gap_ms = (now - prev) * 1e3;
      atomic_max(imu_cb_gap_max_ms_, gap_ms);
      // nominal 200 Hz = 5 ms; 3x that is a real starvation event
      if (gap_ms > 15.0) {
        imu_cb_starve_.fetch_add(1, std::memory_order_relaxed);
        event("imu_cb_starved", "gap_ms=%.2f", gap_ms);
      }
    }
    if (header_stamp_sec > 0.0) {
      const double lat_ms = (now - header_stamp_sec) * 1e3;
      atomic_max(imu_latency_max_ms_, lat_ms);
      const double prev_stamp =
          last_imu_stamp_.exchange(header_stamp_sec, std::memory_order_relaxed);
      if (prev_stamp > 0.0) {
        const double dt_ms = (header_stamp_sec - prev_stamp) * 1e3;
        if (dt_ms < 0.0) {
          imu_stamp_regress_.fetch_add(1, std::memory_order_relaxed);
          event("imu_stamp_regress", "dt_ms=%.3f", dt_ms);
        } else {
          atomic_max(imu_stamp_dt_max_ms_, dt_ms);
        }
      }
    }
  }

  void on_lidar_msg(double header_stamp_sec, unsigned long points,
                    double preprocess_s)
  {
    if (!enabled_) return;
    const double now = wall_now();
    lidar_msgs_.fetch_add(1, std::memory_order_relaxed);
    const double prev = last_lidar_cb_wall_.exchange(now, std::memory_order_relaxed);
    if (prev > 0.0) atomic_max(lidar_cb_gap_max_ms_, (now - prev) * 1e3);
    if (header_stamp_sec > 0.0)
      atomic_max(lidar_latency_max_ms_, (now - header_stamp_sec) * 1e3);
    atomic_max(preprocess_max_ms_, preprocess_s * 1e3);
    last_lidar_points_.store(points, std::memory_order_relaxed);
  }

  // A buffer clear throws away data the EKF needed. In imu_cbk/*_pcl_cbk this
  // fires on a timestamp regression, which is exactly what two unsynchronised
  // sensors publishing to one topic produce.
  void on_buffer_clear(const char* which)
  {
    if (!enabled_) return;
    buffer_clears_.fetch_add(1, std::memory_order_relaxed);
    event("buffer_clear", "which=%s", which ? which : "?");
  }

  // ---- sync_packages -----------------------------------------------------
  void on_sync(bool ok, unsigned long lidar_buf, unsigned long imu_buf,
               unsigned long time_buf, unsigned long meas_imu, double lidar_beg,
               double lidar_end, double last_imu_stamp)
  {
    if (!enabled_) return;
    sync_calls_.fetch_add(1, std::memory_order_relaxed);
    lidar_buf_last_.store(lidar_buf, std::memory_order_relaxed);
    imu_buf_last_.store(imu_buf, std::memory_order_relaxed);
    time_buf_last_.store(time_buf, std::memory_order_relaxed);
    atomic_max_ul(lidar_buf_max_, lidar_buf);
    atomic_max_ul(imu_buf_max_, imu_buf);
    meas_imu_last_.store(meas_imu, std::memory_order_relaxed);
    lidar_beg_last_.store(lidar_beg, std::memory_order_relaxed);
    lidar_end_last_.store(lidar_end, std::memory_order_relaxed);

    if (!ok) {
      sync_fail_.fetch_add(1, std::memory_order_relaxed);
      // Classify the stall: this is the difference between "no data arriving"
      // and "IMU has not caught up to the end of the scan yet".
      const char* why = "unknown";
      if (lidar_buf == 0 && imu_buf == 0)      why = "both_buffers_empty";
      else if (lidar_buf == 0)                 why = "lidar_buffer_empty";
      else if (imu_buf == 0)                   why = "imu_buffer_empty";
      else if (last_imu_stamp < lidar_end)     why = "imu_behind_scan_end";
      // sync_packages() clears meas.imu when it drops an uncoverable scan; on a
      // plain stall the previous batch is still in there. See the paired
      // meas_imu_EMPTY event for the scan that was dropped.
      else if (meas_imu == 0)                  why = "scan_dropped_no_imu_coverage";
      const double n = static_cast<double>(sync_fail_.load(std::memory_order_relaxed));
      // Do not spam: log the reason at most ~1 Hz.
      const double now = wall_now();
      if (now - last_sync_fail_log_ > 1.0) {
        last_sync_fail_log_ = now;
        event("sync_stall", "why=%s lidar_buf=%lu imu_buf=%lu imu_lag_s=%.4f n=%.0f",
              why, lidar_buf, imu_buf, lidar_end - last_imu_stamp, n);
      }
      return;
    }

    // sync succeeded -- but with how much IMU?
    if (meas_imu == 0) {
      // Should be unreachable: sync_packages() drops a scan no IMU covers rather
      // than returning it (see on_scan_dropped_no_imu). Kept as a tripwire -- if
      // this ever fires, an un-propagated scan reached the EKF.
      meas_imu_empty_.fetch_add(1, std::memory_order_relaxed);
      event("meas_imu_EMPTY",
            "action=PROCESSED lidar_beg=%.6f lidar_end=%.6f last_imu=%.6f "
            "lidar_buf=%lu imu_buf=%lu",
            lidar_beg, lidar_end, last_imu_stamp, lidar_buf, imu_buf);
    } else if (meas_imu < 3) {
      meas_imu_thin_.fetch_add(1, std::memory_order_relaxed);
      event("meas_imu_thin", "n=%lu span_s=%.4f", meas_imu, lidar_end - lidar_beg);
    }

    // A scan whose duration is implausible means max(curvature) -- and hence
    // lidar_end_time -- came out wrong in the preprocess handler.
    const double dur_ms = (lidar_end - lidar_beg) * 1e3;
    if (dur_ms <= 0.0 || dur_ms > 500.0) {
      bad_scan_span_.fetch_add(1, std::memory_order_relaxed);
      event("bad_scan_span", "dur_ms=%.4f (from max point curvature)", dur_ms);
    }
  }

  // ---- per-scan completion ----------------------------------------------
  void on_scan_done(const ScanRecord& r)
  {
    if (!enabled_) return;
    scans_.fetch_add(1, std::memory_order_relaxed);

    const bool diverged = !std::isfinite(r.pos_x) || !std::isfinite(r.pos_y) ||
                          !std::isfinite(r.pos_z) || !std::isfinite(r.vel_norm) ||
                          !std::isfinite(r.bg_norm) || !std::isfinite(r.ba_norm);
    if (diverged) {
      nonfinite_state_.fetch_add(1, std::memory_order_relaxed);
      event("state_NONFINITE", "pos=(%g,%g,%g) vel=%g bg=%g ba=%g", r.pos_x,
            r.pos_y, r.pos_z, r.vel_norm, r.bg_norm, r.ba_norm);
    } else if (r.vel_norm > 30.0) {
      event("state_velocity_high", "vel_norm=%.3f m/s", r.vel_norm);
    }

    const double now = wall_now();
    // deltas since the previous scan row
    const unsigned long imu_n = imu_msgs_.exchange(0, std::memory_order_relaxed);
    const unsigned long lid_n = lidar_msgs_.exchange(0, std::memory_order_relaxed);
    const double imu_gap = exchange_double(imu_cb_gap_max_ms_, 0.0);
    const double lid_gap = exchange_double(lidar_cb_gap_max_ms_, 0.0);
    const double imu_lat = exchange_double(imu_latency_max_ms_, 0.0);
    const double lid_lat = exchange_double(lidar_latency_max_ms_, 0.0);
    const double imu_dt = exchange_double(imu_stamp_dt_max_ms_, 0.0);
    const double prep = exchange_double(preprocess_max_ms_, 0.0);

    std::lock_guard<std::mutex> lk(scan_mu_);
    if (!scan_f_) return;
    std::fprintf(
        scan_f_,
        "%.6f,%.3f,%lu,"                     // wall, t_rel, scan_idx
        "%lu,%lu,%lu,%lu,%lu,"               // buffers + meas_imu + tree
        "%.6f,%.6f,%.4f,"                    // lidar_beg,end, pipeline_age_ms
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"     // stage timings ms
        "%lu,%lu,%lu,"                       // pts_in, pts_down, eff_feat
        "%.4f,%.4f,%.4f,%.5f,%.6f,%.6f,%d,"  // state
        "%.5f,%.5f,%.5f,"                    // online extrinsic translation
        "%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"  // stream deltas
        "%.2f,"                              // rss_mb
        "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"    // cumulative anomaly counters
        "%.4f,%.4f,%.4f,"                    // attitude deg
        "%.5f,%.5f,%.5f,"                    // gravity
        "%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,"     // bg, ba
        "%.6f,%.6f,%.5f,"                    // res_mean, imu_gyr_mean, imu_acc_std
        "%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",   // observability
        now, now - t0_, scans_.load(std::memory_order_relaxed),
        lidar_buf_last_.load(std::memory_order_relaxed),
        imu_buf_last_.load(std::memory_order_relaxed),
        time_buf_last_.load(std::memory_order_relaxed),
        meas_imu_last_.load(std::memory_order_relaxed), r.tree_size,
        lidar_beg_last_.load(std::memory_order_relaxed),
        lidar_end_last_.load(std::memory_order_relaxed),
        (now - lidar_end_last_.load(std::memory_order_relaxed)) * 1e3,
        r.t_imu_process * 1e3, r.t_downsample * 1e3, r.t_icp * 1e3,
        r.t_incremental * 1e3, r.t_publish * 1e3, r.t_total * 1e3,
        r.pts_in, r.pts_down, r.eff_feat,
        r.pos_x, r.pos_y, r.pos_z, r.vel_norm, r.bg_norm, r.ba_norm,
        diverged ? 1 : 0,
        r.ext_t_x, r.ext_t_y, r.ext_t_z,
        imu_n, lid_n, imu_gap, lid_gap, imu_lat, lid_lat, imu_dt, prep,
        rss_mb(),
        lidar_buf_max_.load(std::memory_order_relaxed),
        imu_buf_max_.load(std::memory_order_relaxed),
        sync_fail_.load(std::memory_order_relaxed),
        meas_imu_empty_.load(std::memory_order_relaxed),
        meas_imu_thin_.load(std::memory_order_relaxed),
        buffer_clears_.load(std::memory_order_relaxed),
        imu_cb_starve_.load(std::memory_order_relaxed),
        imu_stamp_regress_.load(std::memory_order_relaxed),
        r.roll_deg, r.pitch_deg, r.yaw_deg,
        r.grav_x, r.grav_y, r.grav_z,
        r.bg_x, r.bg_y, r.bg_z, r.ba_x, r.ba_y, r.ba_z,
        r.res_mean, r.imu_gyr_mean, r.imu_acc_std,
        r.obs_min, r.obs_mid, r.obs_max,
        r.obs_weak_x, r.obs_weak_y, r.obs_weak_z);
    // Flush every scan: at 10-20 Hz the cost is negligible and it means a
    // SIGSEGV cannot swallow the rows that explain it.
    std::fflush(scan_f_);
  }

  // A lidar scan discarded by sync_packages() because every buffered IMU sample
  // is newer than the scan's end, so nothing can ever undistort or propagate it.
  // Unlike the tripwire above, nothing downstream ever sees this scan -- it is
  // lost data, not a corrupted state update. One at startup is normal.
  void on_scan_dropped_no_imu(double lidar_beg, double lidar_end,
                              double next_imu_stamp, unsigned long imu_buf)
  {
    if (!enabled_) return;
    meas_imu_empty_.fetch_add(1, std::memory_order_relaxed);
    event("meas_imu_EMPTY",
          "action=dropped lidar_beg=%.6f lidar_end=%.6f next_imu_gap_ms=%.1f imu_buf=%lu",
          lidar_beg, lidar_end, (next_imu_stamp - lidar_end) * 1e3, imu_buf);
  }

  // A scan the timer_callback bailed out of before finishing.
  void on_skip(const char* why)
  {
    if (!enabled_) return;
    skips_.fetch_add(1, std::memory_order_relaxed);
    event("scan_skipped", "why=%s", why ? why : "?");
  }

  // ---- events ------------------------------------------------------------
  void event(const char* kind, const char* fmt, ...)
      __attribute__((format(printf, 3, 4)));

  void finish()
  {
    if (!enabled_) return;
    {
      std::lock_guard<std::mutex> lk(scan_mu_);
      if (scan_f_) { std::fflush(scan_f_); std::fclose(scan_f_); scan_f_ = nullptr; }
    }
    std::lock_guard<std::mutex> lk(ev_mu_);
    if (ev_f_) {
      std::fprintf(ev_f_,
                   "%.6f,%.3f,SUMMARY,scans=%lu sync_fail=%lu meas_imu_empty=%lu "
                   "meas_imu_thin=%lu buffer_clears=%lu imu_cb_starve=%lu "
                   "imu_stamp_regress=%lu bad_scan_span=%lu skips=%lu "
                   "nonfinite_state=%lu lidar_buf_max=%lu imu_buf_max=%lu "
                   "rss_mb=%.1f\n",
                   wall_now(), wall_now() - t0_,
                   scans_.load(), sync_fail_.load(), meas_imu_empty_.load(),
                   meas_imu_thin_.load(), buffer_clears_.load(),
                   imu_cb_starve_.load(), imu_stamp_regress_.load(),
                   bad_scan_span_.load(), skips_.load(), nonfinite_state_.load(),
                   lidar_buf_max_.load(), imu_buf_max_.load(), rss_mb());
      std::fflush(ev_f_);
      std::fclose(ev_f_);
      ev_f_ = nullptr;
    }
  }

 private:
  Probe()
  {
    const char* prefix = std::getenv("FASTLIO_PERF_LOG");
    if (!prefix || !*prefix) { enabled_ = false; return; }
    enabled_ = true;
    t0_ = wall_now();
    const std::string scan_path = std::string(prefix) + "_scan.csv";
    const std::string ev_path = std::string(prefix) + "_events.csv";
    scan_f_ = std::fopen(scan_path.c_str(), "w");
    ev_f_ = std::fopen(ev_path.c_str(), "w");
    if (scan_f_) {
      std::fprintf(
          scan_f_,
          "wall_unix,t_rel_s,scan_idx,"
          "lidar_buf,imu_buf,time_buf,meas_imu,tree_size,"
          "lidar_beg_time,lidar_end_time,pipeline_age_ms,"
          "t_imu_ms,t_downsample_ms,t_icp_ms,t_incr_ms,t_publish_ms,t_total_ms,"
          "pts_in,pts_down,eff_feat,"
          "pos_x,pos_y,pos_z,vel_norm,bg_norm,ba_norm,nonfinite,"
          "ext_t_x,ext_t_y,ext_t_z,"
          "imu_msgs_delta,lidar_msgs_delta,imu_cb_gap_max_ms,"
          "lidar_cb_gap_max_ms,imu_latency_max_ms,lidar_latency_max_ms,"
          "imu_stamp_dt_max_ms,preprocess_max_ms,rss_mb,"
          "cum_lidar_buf_max,cum_imu_buf_max,cum_sync_fail,cum_meas_imu_empty,"
          "cum_meas_imu_thin,cum_buffer_clears,cum_imu_cb_starve,"
          "cum_imu_stamp_regress,"
          // pose stability -- appended, so existing column positions do not move
          "roll_deg,pitch_deg,yaw_deg,"
          "grav_x,grav_y,grav_z,"
          "bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,"
          "res_mean,imu_gyr_mean,imu_acc_std,"
          "obs_min,obs_mid,obs_max,obs_weak_x,obs_weak_y,obs_weak_z\n");
      std::fflush(scan_f_);
    }
    if (ev_f_) {
      std::fprintf(ev_f_, "wall_unix,t_rel_s,kind,detail\n");
      std::fflush(ev_f_);
    }
    std::fprintf(stderr,
                 "[flperf] probe ENABLED -> %s , %s\n"
                 "[flperf] unset FASTLIO_PERF_LOG to disable\n",
                 scan_path.c_str(), ev_path.c_str());
  }

  ~Probe() { finish(); }

  Probe(const Probe&) = delete;
  Probe& operator=(const Probe&) = delete;

  static void atomic_max(std::atomic<double>& dst, double v)
  {
    double cur = dst.load(std::memory_order_relaxed);
    while (v > cur &&
           !dst.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
  }
  static void atomic_max_ul(std::atomic<unsigned long>& dst, unsigned long v)
  {
    unsigned long cur = dst.load(std::memory_order_relaxed);
    while (v > cur &&
           !dst.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
  }
  static double exchange_double(std::atomic<double>& d, double v)
  {
    return d.exchange(v, std::memory_order_relaxed);
  }

  bool enabled_ = false;
  double t0_ = 0.0;
  double last_sync_fail_log_ = 0.0;

  std::FILE* scan_f_ = nullptr;
  std::FILE* ev_f_ = nullptr;
  std::mutex scan_mu_;
  std::mutex ev_mu_;

  // rolling (reset each scan row)
  std::atomic<unsigned long> imu_msgs_{0}, lidar_msgs_{0};
  std::atomic<double> imu_cb_gap_max_ms_{0.0}, lidar_cb_gap_max_ms_{0.0};
  std::atomic<double> imu_latency_max_ms_{0.0}, lidar_latency_max_ms_{0.0};
  std::atomic<double> imu_stamp_dt_max_ms_{0.0}, preprocess_max_ms_{0.0};

  // last observed
  std::atomic<double> last_imu_cb_wall_{0.0}, last_lidar_cb_wall_{0.0};
  std::atomic<double> last_imu_stamp_{0.0};
  std::atomic<unsigned long> lidar_buf_last_{0}, imu_buf_last_{0},
      time_buf_last_{0}, meas_imu_last_{0}, last_lidar_points_{0};
  std::atomic<double> lidar_beg_last_{0.0}, lidar_end_last_{0.0};

  // cumulative
  std::atomic<unsigned long> scans_{0}, sync_calls_{0}, sync_fail_{0};
  std::atomic<unsigned long> meas_imu_empty_{0}, meas_imu_thin_{0};
  std::atomic<unsigned long> buffer_clears_{0}, imu_cb_starve_{0};
  std::atomic<unsigned long> imu_stamp_regress_{0}, bad_scan_span_{0};
  std::atomic<unsigned long> skips_{0}, nonfinite_state_{0};
  std::atomic<unsigned long> lidar_buf_max_{0}, imu_buf_max_{0};
};

// Written immediately, so the last lines before a crash are on disk.
inline void Probe::event(const char* kind, const char* fmt, ...)
{
  if (!enabled_) return;
  char detail[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(detail, sizeof(detail), fmt, ap);
  va_end(ap);
  // commas would break the CSV
  for (char* c = detail; *c; ++c)
    if (*c == ',') *c = ';';
  const double now = wall_now();
  std::lock_guard<std::mutex> lk(ev_mu_);
  if (!ev_f_) return;
  std::fprintf(ev_f_, "%.6f,%.3f,%s,%s\n", now, now - t0_, kind, detail);
  std::fflush(ev_f_);  // unbuffered on purpose: we are hunting a crash
}

// ------------------------------------------------------- free-function API --
// These are what laserMapping.cpp calls. All become no-ops when the probe is
// disabled, and none of them can throw.
inline bool enabled() { return Probe::get().enabled(); }

inline void on_imu_msg(double stamp) { Probe::get().on_imu_msg(stamp); }
inline void on_lidar_msg(double stamp, unsigned long pts, double prep_s)
{
  Probe::get().on_lidar_msg(stamp, pts, prep_s);
}
inline void on_buffer_clear(const char* which)
{
  Probe::get().on_buffer_clear(which);
}
inline void on_sync(bool ok, unsigned long lidar_buf, unsigned long imu_buf,
                    unsigned long time_buf, unsigned long meas_imu,
                    double lidar_beg, double lidar_end, double last_imu)
{
  Probe::get().on_sync(ok, lidar_buf, imu_buf, time_buf, meas_imu, lidar_beg,
                       lidar_end, last_imu);
}
inline void on_scan_done(const ScanRecord& r) { Probe::get().on_scan_done(r); }
inline void on_scan_dropped_no_imu(double lidar_beg, double lidar_end,
                                   double next_imu_stamp, unsigned long imu_buf)
{
  Probe::get().on_scan_dropped_no_imu(lidar_beg, lidar_end, next_imu_stamp, imu_buf);
}
inline void on_skip(const char* why) { Probe::get().on_skip(why); }
inline void finish() { Probe::get().finish(); }

}  // namespace flperf

#endif  // FASTLIO_PERF_PROBE_HPP
