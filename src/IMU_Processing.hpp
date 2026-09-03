#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "use-ikfom.hpp"

/// *************Preconfiguration

// Minimum number of IMU samples for init. This used to be the ONLY stopping
// condition, which at 200 Hz meant init finished after 0.05 s of data -- and in
// practice ~20 samples / 0.1 s, because timer_callback discards the first synced
// package before Process() ever sees it. You cannot average noise out of 20
// samples, and you certainly cannot estimate a gyro bias from them. It is now a
// FLOOR, guarding a slow IMU; `mapping.imu_init_time` sets the real window.
#define MAX_INI_COUNT (10)

// Cap on the samples buffered for the init statistics: 100 s at 200 Hz. Only
// guards an absurd imu_init_time -- the stats themselves are converged long
// before this.
static const size_t IMU_INIT_BUF_MAX = 20000;

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

// Per-axis robust spread: 1.4826 * median-absolute-deviation, which equals the
// standard deviation for Gaussian data but -- unlike the plain std -- barely
// moves when one or two samples are garbage.
//
// WHY THIS EXISTS: the plain std cannot distinguish "the platform was vibrating"
// from "one sample was a spike", and those call for opposite responses. If the
// robust spread is much smaller than the plain std, the plain std is describing
// outliers, not noise, and re-doing the init will not help.
// Middle of three. Used instead of the largest when a single axis is
// misbehaving: "worst axis" and "broken axis" are the same axis then, and any
// recommendation built on it is a recommendation about the fault.
static double median3(const V3D &v)
{
  return v(0) + v(1) + v(2) - v.minCoeff() - v.maxCoeff();
}

static V3D robust_spread(const vector<V3D> &v)
{
  V3D out(0, 0, 0);
  if (v.size() < 4) return out;
  vector<double> a(v.size());
  for (int ax = 0; ax < 3; ++ax)
  {
    for (size_t i = 0; i < v.size(); ++i) a[i] = v[i](ax);
    const size_t mid = a.size() / 2;
    nth_element(a.begin(), a.begin() + mid, a.end());
    const double med = a[mid];
    for (size_t i = 0; i < a.size(); ++i) a[i] = fabs(v[i](ax) - med);
    nth_element(a.begin(), a.begin() + mid, a.end());
    out(ax) = 1.4826 * a[mid];
  }
  return out;
}

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_imu_init_time(double seconds);
  // True until the init window has closed. While it is true, Process() returns
  // an empty cloud for EVERY scan by design, so the caller must not report those
  // scans as faults.
  bool initialising() const { return imu_need_init_; }
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  // sensor_msgs::ImuConstPtr last_imu_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
  // Init window, in seconds of IMU data (mapping.imu_init_time). Gravity comes
  // from the mean acceleration over this window and the gyro bias IS its mean,
  // so the window length sets how well either is known.
  double imu_init_time_ = 1.0;
  double init_first_imu_time_ = -1.0;
  double init_last_imu_time_  = -1.0;
  // Kept only during init, for the robust statistics in the completion log.
  vector<V3D> init_acc_, init_gyr_;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  init_first_imu_time_ = -1.0;
  init_last_imu_time_  = -1.0;
  init_acc_.clear();
  init_gyr_.clear();
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::set_imu_init_time(double seconds)
{
  // 0 or negative means "sample floor only", i.e. the old behaviour. Kept so a
  // config can ask for it explicitly rather than by accident.
  imu_init_time_ = seconds;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
    init_first_imu_time_ =
        rclcpp::Time(meas.imu.front()->header.stamp).seconds();
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    if (init_acc_.size() < IMU_INIT_BUF_MAX)
    {
      init_acc_.push_back(cur_acc);
      init_gyr_.push_back(cur_gyr);
    }

    N ++;
  }
  // Advances every call, so the completion test below measures the span of IMU
  // data actually accumulated rather than wall time.
  init_last_imu_time_ = rclcpp::Time(meas.imu.back()->header.stamp).seconds();
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
    double head_stamp = rclcpp::Time(head->header.stamp).seconds();

    if (tail_stamp < last_lidar_end_time_)    continue;
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head_stamp < last_lidar_end_time_)
    {
      dt = tail_stamp - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail_stamp - head_stamp;
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail_stamp - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  // Clear the output on every early return: cur_pcl_un_ still holds the PREVIOUS
  // scan's cloud, and the caller would otherwise re-register it as a new scan.
  if(meas.imu.empty()) { cur_pcl_un_->clear(); return; };
  assert(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    // Init completes on TIME, with the sample count as a floor. Both conditions
    // matter: the time window is what makes the mean acceleration (= gravity)
    // and the mean angular rate (= gyro bias) worth anything, and the sample
    // floor keeps a slow or stuttering IMU from finishing on three samples that
    // happen to span a second.
    const double init_span =
        (init_first_imu_time_ > 0.0 && init_last_imu_time_ > init_first_imu_time_)
            ? (init_last_imu_time_ - init_first_imu_time_)
            : 0.0;
    if (init_iter_num > MAX_INI_COUNT && init_span >= imu_init_time_)
    {
      V3D acc_std = cov_acc.cwiseSqrt();
      V3D gyr_std = cov_gyr.cwiseSqrt();
      double acc_std_ratio = acc_std.norm() / mean_acc.norm();
      const V3D acc_rob = robust_spread(init_acc_);
      const V3D gyr_rob = robust_spread(init_gyr_);
      imu_need_init_ = false;

      // The measured variances are DISCARDED here: the filter runs on the
      // configured mapping.acc_cov / mapping.gyr_cov. Everything above is
      // diagnostics -- which is exactly why they are logged in full below.
      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      RCLCPP_INFO(rclcpp::get_logger("laser_mapping"),
                  "IMU Initial Done: %d samples over %.2f s (mapping.imu_init_time %.2f s) | "
                  "gravity [%.3f %.3f %.3f] | |mean acc| %.4f (raw units) | "
                  "gyro bias [%.4f %.4f %.4f] | acc std [%.4f %.4f %.4f] (%.2f%% of |acc|) | gyr std [%.5f %.5f %.5f]",
                  init_iter_num - 1, init_span, imu_init_time_,
                  imu_state.grav[0], imu_state.grav[1], imu_state.grav[2],
                  mean_acc.norm(),
                  mean_gyr(0), mean_gyr(1), mean_gyr(2),
                  acc_std(0), acc_std(1), acc_std(2), acc_std_ratio * 100.0,
                  gyr_std(0), gyr_std(1), gyr_std(2));
      RCLCPP_INFO(rclcpp::get_logger("laser_mapping"),
                  "IMU init robust spread (1.4826*MAD, outlier-resistant): "
                  "acc [%.4f %.4f %.4f] gyr [%.5f %.5f %.5f]",
                  acc_rob(0), acc_rob(1), acc_rob(2),
                  gyr_rob(0), gyr_rob(1), gyr_rob(2));

      // WHAT KIND of signal is on each axis, not just how big it is.
      // 1.4826*MAD equals the standard deviation for GAUSSIAN data, so the ratio
      // robust/plain is a shape test that costs nothing:
      //     ~1.0  Gaussian noise -- what an IMU at rest should look like
      //     <0.5  a few outliers inflating the plain std
      //     ~1.5  a DETERMINISTIC PERIODIC or two-state signal (a sinusoid gives
      //           exactly 1.048A / 0.707A = 1.48): vibration at a frequency, not
      //           noise. No covariance setting makes this go away; it is
      //           mechanical.
      V3D acc_shape(0, 0, 0), gyr_shape(0, 0, 0);
      for (int i = 0; i < 3; ++i) {
        if (acc_std(i) > 0.0) acc_shape(i) = acc_rob(i) / acc_std(i);
        if (gyr_std(i) > 0.0) gyr_shape(i) = gyr_rob(i) / gyr_std(i);
      }
      RCLCPP_INFO(rclcpp::get_logger("laser_mapping"),
                  "IMU init noise SHAPE robust/plain per axis (1.0 = Gaussian, "
                  "<0.5 = outliers, ~1.5 = periodic/vibration): "
                  "acc [%.2f %.2f %.2f] gyr [%.2f %.2f %.2f]",
                  acc_shape(0), acc_shape(1), acc_shape(2),
                  gyr_shape(0), gyr_shape(1), gyr_shape(2));
      for (int i = 0; i < 3; ++i) {
        if (acc_shape(i) > 1.3) {
          // A sinusoid of amplitude A has plain std A/sqrt(2).
          RCLCPP_WARN(rclcpp::get_logger("laser_mapping"),
                      "Accelerometer axis %c looks PERIODIC, not noisy "
                      "(robust/plain = %.2f, vs 1.0 for Gaussian): a deterministic "
                      "vibration of roughly %.3f m/s2 amplitude. Find the source "
                      "(fan, pump, motor, resonating mount) -- raising acc_cov only "
                      "tells the filter to ignore the accelerometer that much more.",
                      "xyz"[i], acc_shape(i), acc_std(i) * 1.41421356);
        }
      }

      // THE TUNING LINE. mapping.acc_cov / mapping.gyr_cov go straight onto the
      // process-noise diagonal Q (IMU_Processing.hpp: Q.block(0,0)=cov_gyr,
      // Q.block(3,3)=cov_acc), so they are VARIANCES in (m/s^2)^2 and (rad/s)^2
      // -- the square of the numbers just logged. FAST-LIO ships 0.1/0.1, which
      // is a std of 0.32 m/s^2 and 0.32 rad/s (18 deg/s): fine for a cheap
      // embedded MEMS unit, often far too pessimistic for a good external IMU.
      // Too pessimistic and the attitude has no faith in the gyro, so it gets
      // driven by the lidar plane-fit residuals instead and never settles --
      // which is what "roll/pitch swings while stationary" in the perf report
      // means.
      //
      // The MEDIAN axis, not the worst. This used to use maxCoeff(), which on a
      // platform with one vibrating axis meant the recommendation was computed
      // from the fault: a 0.35 m/s2 shake on y gave "acc_cov 1.4", i.e. distrust
      // the accelerometer 14x more than the stock default. The spread across
      // axes is reported below so a lopsided unit is visible rather than
      // averaged away.
      const double a_rob_med = median3(acc_rob);
      const double g_rob_med = median3(gyr_rob);
      const double a_meas = a_rob_med * a_rob_med;
      const double g_meas = g_rob_med * g_rob_med;
      RCLCPP_INFO(rclcpp::get_logger("laser_mapping"),
                  "IMU noise measured on THIS unit -> MEDIAN-axis variance: "
                  "acc %.3e (m/s2)^2, gyr %.3e (rad/s)^2 "
                  "(per-axis acc variance [%.3e %.3e %.3e]). Configured: "
                  "acc_cov %.3e (%.3gx measured), gyr_cov %.3e (%.3gx measured). "
                  "A 10x margin over measured is a reasonable starting point: "
                  "acc_cov %.2e, gyr_cov %.2e. A/B it -- see 'Pose stability' in "
                  "perf/README.md.",
                  a_meas, g_meas,
                  acc_rob(0) * acc_rob(0), acc_rob(1) * acc_rob(1),
                  acc_rob(2) * acc_rob(2),
                  cov_acc_scale.maxCoeff(),
                  a_meas > 0.0 ? cov_acc_scale.maxCoeff() / a_meas : 0.0,
                  cov_gyr_scale.maxCoeff(),
                  g_meas > 0.0 ? cov_gyr_scale.maxCoeff() / g_meas : 0.0,
                  a_meas * 10.0, g_meas * 10.0);
      if (acc_rob.minCoeff() > 0.0 && acc_rob.maxCoeff() > 5.0 * acc_rob.minCoeff())
      {
        RCLCPP_WARN(rclcpp::get_logger("laser_mapping"),
                    "That acc_cov recommendation is from the MEDIAN axis and the "
                    "three axes disagree by %.0fx -- it describes the quiet axes, "
                    "not what the accelerometer is actually being subjected to. "
                    "Fix the noisy axis before tuning acc_cov, or set acc_cov to "
                    "%.2e (10x the WORST axis) and accept that the accelerometer "
                    "is then contributing almost nothing.",
                    acc_rob.maxCoeff() / acc_rob.minCoeff(),
                    acc_rob.maxCoeff() * acc_rob.maxCoeff() * 10.0);
      }

      if (acc_std_ratio > 0.02)
      {
        RCLCPP_WARN(rclcpp::get_logger("laser_mapping"),
                    "High accelerometer variance during IMU init (std = %.2f%% of gravity) — platform was likely "
                    "moving or vibrating. Gravity/bias estimate may be wrong and the filter can diverge "
                    "('No Effective Points'). Keep the platform still for the first %.1f s after launch.",
                    acc_std_ratio * 100.0, imu_init_time_);
      }
      // A spike and a vibrating platform produce the same plain std and need
      // opposite responses, so say which one this was.
      if (acc_rob.maxCoeff() > 0.0 && acc_std.maxCoeff() > 4.0 * acc_rob.maxCoeff())
      {
        RCLCPP_WARN(rclcpp::get_logger("laser_mapping"),
                    "Accelerometer std (%.4f) is %.0fx its robust spread (%.4f) — a FEW OUTLIER SAMPLES "
                    "dominate, not vibration. Re-doing the init will not help; look at the IMU driver "
                    "and the link for dropped or corrupted samples.",
                    acc_std.maxCoeff(), acc_std.maxCoeff() / acc_rob.maxCoeff(),
                    acc_rob.maxCoeff());
      }
      // Vibration couples into all three axes. A single noisy axis does not.
      if (acc_rob.minCoeff() > 0.0 && acc_rob.maxCoeff() > 5.0 * acc_rob.minCoeff())
      {
        RCLCPP_WARN(rclcpp::get_logger("laser_mapping"),
                    "Accelerometer noise is %.0fx larger on one axis than another "
                    "(robust spread [%.4f %.4f %.4f]) — broadband vibration does not do that. "
                    "Suspect a loose mount on that axis, a resonating fixture, or a unit/scaling "
                    "problem in the driver.",
                    acc_rob.maxCoeff() / acc_rob.minCoeff(),
                    acc_rob(0), acc_rob(1), acc_rob(2));
      }
      if (mean_gyr.norm() > 0.1)
      {
        RCLCPP_WARN(rclcpp::get_logger("laser_mapping"),
                    "Large mean angular velocity during IMU init (%.3f rad/s) — platform was rotating; "
                    "the gyro bias estimate is invalid.", mean_gyr.norm());
      }
      // Nothing reads these after init; a long window would otherwise hold on
      // to a few hundred kB for the life of the process.
      init_acc_.clear();
      init_acc_.shrink_to_fit();
      init_gyr_.clear();
      init_gyr_.shrink_to_fit();
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }
    else
    {
      // Every scan is DROPPED while init runs (cur_pcl_un_->clear() below), so
      // a longer window means a longer silence before the first odometry. Say
      // what is happening -- otherwise it reads as a hang.
      static rclcpp::Clock init_log_clock(RCL_STEADY_TIME);
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("laser_mapping"), init_log_clock, 500,
                           "IMU init in progress: %d samples, %.2f of %.2f s. "
                           "Keep the platform still. Lidar scans are dropped until this completes.",
                           init_iter_num - 1, init_span, imu_init_time_);
    }

    cur_pcl_un_->clear();
    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
