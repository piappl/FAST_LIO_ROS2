# FAST_LIO_ROS2 performance & crash diagnosis (Jetson Orin NX)

Tooling to find out why `fastlio_mapping` runs fine on a recorded bag but dies
after a while on the live rig (2x Livox Mid-360, or 1x Livox HAP) on ROS 2 Jazzy
with Cyclone DDS.

Everything here runs **on the target**. Nothing needs to be installed: the
monitors are plain `rclpy` + stdlib, the analyser is stdlib only, and the C++
probe is header-only and compiles to no-ops unless you switch it on.

---

## TL;DR — read the code first, then measure

Reading this fork turned up five concrete things that differ between "replay a
short bag" and "run live for twenty minutes". They are ranked by how well they
match your symptom. **Test them in this order — the first one is close to a
smoking gun.**

### 1. `publish.map_en: true` republishes an ever-growing cloud  ← start here

`config/mid360.yaml:42` sets `map_en: true`. That arms a 1 Hz timer
(`laserMapping.cpp:988`) which calls `publish_map()`:

```cpp
// src/laserMapping.cpp:632
*pcl_wait_pub += *laserCloudWorld;      // appended -- and NEVER cleared
pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);   // serialise the WHOLE thing
pubLaserCloudMap->publish(laserCloudmsg);      // ...every single second
```

`pcl_wait_pub` is only ever appended to — grep it: `laserMapping.cpp:526` (init),
`632` (append), `635` (serialise), `651` (pcd write). There is no `clear()`.

So once a second the node appends ~4k points and then serialises and publishes
the entire accumulation. After 10 minutes that is ~2.5M points, i.e. a ~100 MB
PointCloud2 **every second**; after 30 minutes, three times that. All of it:

* grows RSS without bound,
* burns ever more time in `pcl::toROSMsg` **on the single thread that also has
  to drain the 200 Hz IMU queue** (see 2), and
* hands Cyclone DDS a 100 MB+ sample per second to fragment, which is exactly
  how you overflow a socket receive buffer and lose IMU and lidar samples.

A 60-second bag replay never accumulates enough for any of that to bite. A live
run does. **First experiment: `map_en: false`.**

### 2. One thread does everything — FIXED, kept here as history

This was true and is no longer. It is left in because H1 keeps getting
re-diagnosed from these paragraphs; read the "now" at the bottom before you
spend a day on it.

**Then:** `main()` used `rclcpp::spin()` — a **single-threaded** executor. So
`timer_callback` (IMU integration, voxel downsample, ICP, ikd-Tree insert,
publishing) shared one thread with `imu_cbk` at 200 Hz and the lidar callback at
10 Hz. While a scan was being processed, no sensor callback ran at all. The IMU
subscription was `create_subscription<Imu>(imu_topic, 10, imu_cbk)` — depth 10,
which at 200 Hz is **50 ms of buffer**. Any processing stall longer than that and
IMU samples were dropped by the middleware before your code saw them.

**Now** (`laserMapping.cpp`): a `MultiThreadedExecutor` with 3 threads and one
mutually-exclusive `CallbackGroup` each for IMU, lidar and the mapping timers,
so a scan in ICP no longer blocks `imu_cbk` at all. The IMU queue is **1000**
deep — 5 s at 200 Hz. `imu_cbk` takes `mtx_buffer` only to `push_back`, and
cloud preprocessing happens *before* the lock is taken, so the two sensor
callbacks barely contend. The lidar subscription is
`SensorDataQoS().keep_last(100)`.

The consequence for reading reports: a large `imu_cb_gap_max_ms` is now a
**latency** symptom, not a loss mechanism. With 5 s of queue, a 20 ms late drain
loses nothing. The number that decides H1 is `imu_msgs_delta` — how many
messages actually reached `imu_cbk` — and `meas_imu`, how many the EKF got per
scan. `analyze.py` reports both under H1.

`stream_monitor.py`'s `--qos loam` profile mirrors those subscriptions and
**must be kept in sync with the code**, or the A/B measures a subscription the
node no longer has.

> **Do not "fix" this by swapping in a `MultiThreadedExecutor`.**
> `sync_packages()` reads and pops `lidar_buffer` / `imu_buffer` / `time_buffer`
> **without holding `mtx_buffer`**, while the callbacks push under the lock. That
> is only safe because there is exactly one thread today. Add threads without
> adding locking and you trade a slow failure for a memory-corruption crash.

### 3. The buffers are unbounded

`lidar_buffer`, `imu_buffer` and `time_buffer` (`laserMapping.cpp:110-112`) are
plain `std::deque`s with no size cap. They are only ever emptied by consumption
or by the "lidar loop back" clears. If processing cannot keep up, they grow
until memory runs out — and latency grows with them, so the pose you publish
gets older and older. The probe records both depths every scan.

### 4. `meas.imu` can be empty while `sync_packages()` returns `true`

```cpp
double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
meas.imu.clear();
while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) { ... }
```

If the front IMU sample is already **newer** than `lidar_end_time`, the loop body
never executes, `meas.imu` stays empty, and the function still returns `true`.
`ImuProcess::Process()` then hits its `if (meas.imu.empty()) return;` guard and
returns **without touching `feats_undistort`** — which therefore still holds the
*previous* scan's points. That stale cloud is then matched against the new state.
It does not crash immediately; it corrupts the estimate. The probe counts this as
`meas_imu_EMPTY`.

This is precisely what dropped/late IMU samples (1 and 2) produce.

### 5. Per-point timestamps decide `lidar_end_time`

`Preprocess::mid360_handler` turns each point's `timestamp` into `curvature`:

```cpp
added_pt.curvature = (pl_orig.points[i].timestamp - ref_timestamp) / 1e6;  // ms
added_pt.curvature = std::max(0.0f, added_pt.curvature);                   // clamp
```

and `sync_packages()` uses **`max(curvature)`** as the scan duration to compute
`lidar_end_time`. Consequences:

* timestamps absent or zero → all curvatures clamp to 0 → `lidar_end_time`
  collapses onto `lidar_beg_time` → tiny or empty IMU batches (see 4);
* **two sensors merged with unsynchronised clocks** → that `max` is the clock
  offset, not the scan duration → `lidar_end_time` lands far in the future,
  `lidar_mean_scantime` is poisoned, and the IMU association breaks.

`stream_monitor.py` parses this field straight off the wire and reports
`off_span_ms_max`, `zero_frac_max` and friends. At 10 Hz the span should be
~100 ms.

### Also worth knowing

* **Two Mid-360s, one topic.** FAST-LIO subscribes to exactly one lidar and one
  IMU topic. If both units' IMUs publish to `imu_topic`, their interleaved
  stamps trip `if (timestamp < last_timestamp_imu) imu_buffer.clear();` over and
  over. The monitor's `pubs=` column must read **1**. See
  `perf/config/mid360_dual_perf.yaml`.
* **OpenMP is off on your target.** `CMakeLists.txt` only defines `MP_EN` for
  x86, so on aarch64 the one parallel region in the codebase — the
  residual/nearest-neighbour loop in `h_share_model()`, the dominant per-scan
  cost, run `max_iteration` times per scan — executes on **one core** of your
  eight. There is now an opt-in knob for it (experiment E5); the loop writes only
  per-index data, so enabling it is race-free.
* **`pcd_save_en: true` with `interval: -1`** writes that same unbounded
  `pcl_wait_pub` at shutdown. (The other accumulation site, in
  `publish_frame_world`, is commented out in this fork, so it is only the
  `publish_map` path that grows.)

---

## Preflight

```bash
# 1. the ikd-Tree submodule must be checked out (it is a submodule, and empty
#    in a plain clone -- the build needs include/ikd-Tree/ikd_Tree.cpp)
git submodule update --init --recursive

# 2. build with symbols, so a core dump gives you line numbers
colcon build --packages-select fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 3. host tuning + a preflight report (sysctl, core dumps, clocks, RMW check)
./perf/setup_target.sh                  # add --report-only to change nothing
./perf/setup_target.sh --validate-dds   # check the Cyclone config loads

# 4. in EVERY terminal you use afterwards
source /opt/ros/jazzy/setup.bash
source ~/loam_test/install/setup.bash      # your workspace
source ~/loam_test/FAST_LIO_ROS2/perf/config/perf_env.sh
```

`perf_env.sh` sets `RMW_IMPLEMENTATION`, points `CYCLONEDDS_URI` at the tuned
config, keeps discovery local, and makes console logging unbuffered and
timestamped so a crash lines up with the CSVs.

---

## The protocol

Each phase answers one question. Do not skip ahead — later phases are much
harder to read if you have not established the earlier answers.

### Phase 0 — is the harness itself trustworthy? (no hardware needed)

```bash
# terminal 1: a synthetic Mid-360 with faults injected on purpose
./perf/monitors/fake_livox_pub.py --second-imu-publisher --drop-imu-frac 0.03

# terminal 2
./perf/monitors/stream_monitor.py --tag selftest --qos greedy --out-dir /tmp/selftest
```

You should see `stamp_regression` and `stamp_gap` events, and with
`--zero-point-ts` also `timestamp_all_zero` and `curvature_max_nonpositive`. Run
it once clean (no flags) to see the false-positive floor. Now you know what a
real detection looks like.

#### How to read those numbers

**`missed` is only trustworthy when `regress` is 0.** The gap estimator works on
stamp deltas, so an out-of-order stream confounds it: every backward jump is
followed by an oversized forward jump, which counts as a gap. With
`--second-imu-publisher` you get `gap_events ≈ regressions`, one for one, and
`missed` inflated to roughly that same number — even though nothing was actually
lost. Measured on the reference run: 4196 regressions, 4195 gaps, and a
`missed` estimate ~10% above the gap count, where the ~10% excess is the *real*
`--drop-imu-frac` loss.

So read the columns in this order:

1. **`pubs=`** — more than 1 invalidates everything below it.
2. **`regress`** — if non-zero, fix ordering before believing any loss figure.
3. **`missed`** — only now does it mean loss.

To measure real loss independently of ordering, compare the rate the monitor
*received* against what the publisher (or driver) says it *sent*. On the
reference run that was 368.58 Hz sent vs 368.07 Hz received: **0.14%**, i.e. no
transport loss at all, despite `missed` reporting ~35%.

> Note: `fake_livox_pub.py` stamps messages from a Python timer, so even a clean
> run shows a small spurious `stamp_gap` count on the 200 Hz IMU — measured floor
> is ~0.25% (11 gaps in 4335 messages), and it is higher on a slower box because
> the Python timer slips more. Real Livox stamps are hardware-derived and far
> more regular. Raise `--gap-factor` if the floor is noisy on your target.
> A clean run must show **`regress=0`** and **zero timestamp events** — those have
> no false-positive floor.

### Phase 1 — is the sensor stream itself clean? (driver only, no LOAM)

Start only the Livox driver, then:

```bash
./perf/monitors/stream_monitor.py --tag phase1 --qos greedy \
  --out-dir perf/runs/phase1 --report-period 10
```

Let it run a few minutes, Ctrl-C, then get the verdict:

```bash
./perf/analyze.py perf/runs/phase1
```

Section 2 of that report, **"sensor stream checklist"**, computes every Phase 1
criterion per topic and marks each `PASS` / `WARN` / `FAIL` / `?`. That is the
thing to read — you do not have to cross-reference the console against the CSVs
by hand. A `?` means the data could not answer the question, *not* that it
passed.

#### Where the numbers actually live

Only some of this is on the console. Three of the five criteria are file-only:

| criterion | expected | console? | file |
|---|---|---|---|
| one publisher per topic | **1** | only as `!! N PUBLISHERS` when it is wrong | `publishers` in `phase1_meta.json`, `publishers` column in `phase1_agg.csv` |
| IMU / cloud rate | ~200 / ~10 Hz | yes, `hz` | `hz` in `phase1_agg.csv` |
| stamp regressions | **0** | only as `!! regress=N` | `stamp_regressions` in both |
| per-point time span | ~100 ms at 10 Hz | **no** | `off_span_ms_max` in `phase1_agg.csv` |
| points with `timestamp==0` | **0** | **no** | `zero_frac_max` in `phase1_agg.csv` |

Note the asymmetry: `pubs=1` and `regress=0` never print on the console — silence
is the pass. So a clean console does not confirm those two; the checklist does.

Files written to `--out-dir`:

```
phase1_agg.csv      one row per topic per --report-period; all the columns above
phase1_events.csv   one row per anomaly (gap, regression, timestamp problem)
phase1_meta.json    totals, per-topic, plus the publisher count
```

Quick look without the analyser:

```bash
# the two cloud columns the console never shows
python3 -c "
import csv
for r in csv.DictReader(open('perf/runs/phase1/phase1_agg.csv')):
    if r['points_mean']:
        print(r['t_rel_s'], 'pubs', r['publishers'], 'span', r['off_span_ms_max'],
              'zero', r['zero_frac_max'], 'neg', r['neg_frac_max'])"

# what went wrong, grouped
tail -n +2 perf/runs/phase1/phase1_events.csv | cut -d, -f4 | sort | uniq -c | sort -rn
```

Any failure here is a driver/sensor/PTP problem and LOAM is downstream of it.
Fix it before going on.

### Phase 2 — does LOAM's own consumption lose messages?

This is the central experiment: two subscribers on the same topics, one with
LOAM's exact QoS, one with a deep reliable queue.

```bash
./perf/run_test.sh --name phase2 --duration 1800 \
  --loam-cmd "ros2 launch fast_lio mapping.launch.py \
              config_file:=mid360.yaml rviz:=false"
```

* greedy clean **and** loamqos gappy → the loss is created at the subscriber
  (cause 2 — but see the "FIXED" note there before assuming it).
* both gappy → the loss is upstream: driver, network, or socket buffers.
* **both `n=0`** → neither reader was on a live topic. This is not a result;
  see "The `pubs=0 / 0.00 Hz` checklist FAILs" below. `run_test.sh` takes the
  topics from the config now, so prefer `--config config/<yours>.yaml` (or a
  `--loam-cmd` containing `config_file:=`) over passing them by hand.

Add `--cloud-topic /Laser_map --cloud-rate 1` to watch the accumulating map
directly; a climbing `points_mean` is cause 1 caught in the act.

### Phase 3 — what does LOAM look like from the inside?

`run_test.sh` sets `FASTLIO_PERF_LOG` automatically, so Phase 2 already produced
`perf_probe_scan.csv` (one row per processed scan) and `perf_probe_events.csv`
(one row per anomaly, flushed immediately so a SIGSEGV cannot swallow it).

Watch, in `perf_probe_scan.csv`:

| column | meaning | bad sign |
|---|---|---|
| `lidar_buf`, `imu_buf` | the unbounded deques | monotonic growth (cause 3) |
| `meas_imu` | IMU samples for this scan | 0, or < 3 (cause 4) |
| `t_total_ms` | whole timer_callback | > 100 ms at 10 Hz |
| `t_icp_ms` | the EKF update | dominates, and grows with map size |
| `pipeline_age_ms` | now − `lidar_end_time` | grows monotonically = backlog |
| `imu_cb_gap_max_ms` | gap between `imu_cbk` calls | ≫ 5 ms = starvation (cause 2) |
| `rss_mb` | resident memory | monotonic growth |
| `vel_norm`, `nonfinite` | state sanity | > 30 m/s, or 1 |

### Phase 4 — let it crash, then read the wreck

Run until it actually dies (`--duration 0` records until Ctrl-C; `run_test.sh`
also stops on its own if the node exits). It then collects the log tail, the
OOM/segfault lines from `dmesg`, any core file, and — if `gdb` is present — a
full backtrace, into `postmortem.txt` and `backtrace.txt`.

The distinction that matters: **OOM kill** (SIGKILL, no core, `dmesg` shows
`Out of memory`) points at causes 1/3; a **segfault with a core** points
somewhere specific, and the backtrace names it.

### Phase 5 — analysis

```bash
./perf/analyze.py perf/runs/<the run>
```

`run_test.sh` does this for you and writes `report.txt`. It prints a verdict per
hypothesis with the numbers behind it, then the last 30 seconds before the end —
which is usually where the answer is.

---

## Hypotheses and fixes

`analyze.py` labels each of these `LIKELY` / `POSSIBLE` / `not supported`.

**H1, H8 and H9 have code fixes in the tree** — the rows below say what was
applied. Measured on `20260902_075418_phase2`, the re-run after the H1/H9 work:

| | `055532` (before) | `075418` (after) |
|---|---|---|
| worst `imu_cbk` gap | 22.7 ms | 10.5 ms |
| `imu_cb_starved` | 593 | 0 |
| RSS trend | +7.8 MB/min | +0.4 MB/min |
| per-scan mean | 11.1 ms | 7.4 ms |

H8 then came up `LIKELY` on that second run, on the strength of a single
startup event — the first cloud landed before the first IMU sample. Two things
were wrong with that: `ImuProcess::Process()` had already been fixed to clear
`cur_pcl_un_` on its empty-IMU return, so the "stale cloud" the finding
described could not happen; and one bounded startup transient is not
divergence. `sync_packages()` now drops such a scan outright and `analyze.py`
grades startup drops apart from sustained ones.

Still open: both runs used `CYCLONEDDS_URI=file:///usr/config/CycloneDDS.xml`,
i.e. `perf_env.sh` was never sourced, so the `WhcHigh` half of the H1 fix has
never actually been in effect. Source it for the next run.

### The `pubs=0 / 0.00 Hz` checklist FAILs — root-caused, fixed

Both `075418` and `20260903_065639_phase2` reported `pubs=0` and `0.00 Hz` on
every topic in both monitors while the probe happily processed thousands of
scans off those same topics. The monitors saw nothing; LOAM saw everything.

The cause was not discovery and not the sensors: **`run_test.sh` defaulted the
monitored topics to `/livox/imu` and `/livox/lidar`, while the node reads
`common.lid_topic` / `common.imu_topic` from its config YAML.** `hap.yaml`
points at `/livox/hap_4/lidar` and `/imu/data` (external SBG Ellipse A), so both
monitors subscribed to topics nobody publishes and dutifully recorded nothing.

Two things made that worse than a cosmetic bug:

* section 2 rendered the emptiness as **`FAIL`**, which reads as "the sensor is
  broken" when it means "the monitor looked in the wrong place";
* **H1's `loam` vs `greedy` cross-check silently compared two empty datasets**
  and found `missed~0 vs missed~0`, i.e. it reported no loss because it had no
  data. That cross-check is the *only* thing that can place a loss at the
  subscriber rather than upstream, so H1 was left resting entirely on the probe's
  starvation counter.

Fixed in three places:

* `run_test.sh` now reads `lid_topic` / `imu_topic` / `lidar_type` / `scan_rate`
  out of the config YAML — auto-detected from `config_file:=` in `--loam-cmd`,
  or given explicitly with `--config` — and routes a `lidar_type: 1` topic to
  `--custom-topic` (CustomMsg) instead of `--cloud-topic` (PointCloud2), which
  is another way to get `n=0`. `--imu-topic` / `--cloud-topic` still override.
  If it has no config to read and has to guess, it now says so loudly.
* `analyze.py` prints a `NO DATA` block instead of `FAIL` for a topic with zero
  messages, names the topics, and states that nothing there is a verdict on the
  sensors.
* H1 refuses to draw a conclusion from a monitor that received nothing, and
  reports the delivered IMU rate from the probe instead.

### IMU init and the process-noise covariances

Two things upstream FAST-LIO gets wrong for any IMU better than the one inside
the lidar, both found in `20260903_065639_phase2` (HAP + external SBG Ellipse A
on `/imu/data`) and both fixed or made measurable in the tree:

**1. Init used ~0.1 s of data.** `MAX_INI_COUNT` is 10, and `timer_callback`
throws away the first synced package before `Process()` sees it, so init
finished on the second package — 20 samples at 200 Hz. Gravity is the mean
acceleration over that window and the initial gyro bias *is* its mean, so 20
samples put a tilt error and a bias error straight into the state, and a single
bad sample moves gravity. There is now a **`mapping.imu_init_time`** parameter
(seconds, default 1.0); `MAX_INI_COUNT` survives as a sample floor for slow
IMUs. Scans are dropped while the window is open — that is deliberate, was
always true, and is now logged (`IMU init in progress: N samples, X of Y s`)
instead of showing up as one `No point, skip this scan!` warning per scan.

One consequence to know about: a replay shorter than `imu_init_time` now
produces **no odometry at all**, where the old 0.1 s window would have
initialised (badly) and run. The progress log says so every 500 ms rather than
failing silently, but if you are replaying very short bags, lower the parameter
for that job rather than wondering why nothing came out.

**2. The init log could not tell vibration from a bad sample.** The phase2 run
reported `acc std [0.0097 0.2171 0.0387]` — the y axis 22x the x axis — and
warned "platform was likely moving or vibrating". Broadband vibration does not
do that to one axis. The init now also logs a **robust spread** (1.4826·MAD,
outlier-resistant) next to the plain std, and warns differently for the two
cases. Verified against the fake publisher:

| injected fault | plain acc std | robust spread | warning |
|---|---|---|---|
| clean stationary | `[0.010 0.010 0.010]` | flat | none |
| 2 bad samples on y | `[0.010 0.057 0.010]` | flat | "a FEW OUTLIER SAMPLES dominate, not vibration" |
| y axis resonating | `[0.010 0.078 0.010]` | lopsided | "9x larger on one axis than another" |
| broadband vibration | `[0.251 0.243 0.243]` | high, flat | "High accelerometer variance" |

**3. `acc_cov` / `gyr_cov` are variances, and the defaults are for a different
IMU.** They go straight onto the process-noise diagonal `Q`
(`IMU_Processing.hpp`: `Q(0,0) = cov_gyr`, `Q(3,3) = cov_acc`), so the shipped
`0.1` means an assumed noise std of 0.32 m/s² and 0.32 rad/s — 18 °/s. Against
the SBG's measured per-sample noise that is ~60x (acc) and ~30000x (gyr) the
real variance in variance terms. A gyro that pessimistic contributes almost
nothing to attitude, so attitude gets solved from the lidar plane fits instead;
with 541 effective points at a 2.89 cm residual those are noisy, and the result
is section 4's "roll/pitch swings 0.774 deg while stationary".

The init log now prints the noise measured on the actual unit and what it
implies, so this is a reading exercise rather than a guess:

```
IMU noise measured on THIS unit -> worst-axis variance: acc 1.223e-04 (m/s2)^2,
gyr 4.456e-07 (rad/s)^2. Configured: acc_cov 1.000e-01 (818x measured),
gyr_cov 1.000e-01 (224422x measured). A 10x margin over measured is a
reasonable starting point: acc_cov 1.22e-03, gyr_cov 4.46e-06.
```

The recommendation is computed from the *robust* spread, so a couple of bad
samples cannot inflate it.

`config/hap.yaml` is the **A/B baseline** and deliberately keeps `0.1/0.1`;
`config/hap_sbg.yaml` is the treatment and carries the derivation, the measured
three-step ladder, and the run commands. The short version of that ladder,
measured on a synthetic stationary scene tuned to a comparable residual:

| `acc_cov`/`gyr_cov`/`b_acc_cov` | roll p2p | \|ba\| drift |
|---|---|---|
| `0.1 / 0.1 / 1e-4` (baseline) | 0.265° | 0.0069 |
| `0.02 / 4e-5 / 1e-4` | 0.147° | 0.0121 |
| `0.02 / 4e-5 / 1e-5` (treatment) | 0.096° | 0.0077 |

The covariances halve the attitude wander; doing that also gives the accel bias
more to absorb, which is why `b_acc_cov` has to come with them rather than
after them. **That table is synthetic — it validates the direction, not the
magnitudes.** Re-measure on the target.

**Reproducing this fault class locally:** `fake_livox_pub.py` needs
`--range-noise-std` (try `0.03`) for any pose-stability or IMU-covariance
experiment. Its default scene is geometrically exact, so the plane fits have a
zero residual, the lidar pins the pose outright, and no IMU covariance can make
any difference — an A/B there returns sub-2 mm and 0.008° for every setting and
looks like a null result. `--imu-still`, `--acc-noise-std x,y,z`,
`--acc-spike-count/-mag/-axis` inject the rest.

| id | hypothesis | fix if confirmed |
|---|---|---|
| **H1** | loss at the subscriber (shallow QoS + starved executor) | **applied** — IMU depth 1000, one `CallbackGroup` each for IMU / lidar / the mapping timers, `MultiThreadedExecutor` in `main()`, `mtx_buffer` held across `sync_packages()`, and cloud preprocessing moved out of the buffer lock. Still on you: run with `perf/config/cyclonedds_jetson.xml` so `WhcHigh` is 8 MB (see below) — a throttled reliable writer blocks `publish()` *on that same thread* |
| **H2** | loss in the transport (socket buffers, NIC) | `perf/setup_target.sh` (raises `net.core.rmem_max`) plus `SocketReceiveBufferSize` in `perf/config/cyclonedds_jetson.xml`; jumbo frames on the lidar link if available |
| **H3** | two sensors on one topic / unsynchronised clocks | one topic per sensor, or merge in a node that reorders by stamp; PTP both lidars off one master; point `imu_topic` at a single IMU |
| **H4** | per-point timestamps corrupting `lidar_end_time` | fix the driver's timestamp mode; verify `off_span_ms_max` ≈ 100 ms; treat a non-positive `max(curvature)` as a hard error rather than clamping it |
| **H5** | cannot keep up → growing backlog | raise `point_filter_num`, raise `filter_size_surf`/`filter_size_map`, lower `max_iteration`, cap `cube_side_length`; enable OpenMP (E5); bound the deques and drop oldest instead of growing |
| **H6** | out of memory | `map_en: false`, `pcd_save_en: false`; bound the deques; watch ikd-Tree growth |
| **H7** | Jetson throttling | `nvpmodel -m 0` + `jetson_clocks`, check cooling; `setup_target.sh --max-perf` |
| **H8** | state divergence (bad IMU association) | **applied** — `sync_packages()` now DROPS a lidar scan that no buffered IMU sample covers, instead of emitting a package the EKF cannot propagate; the probe records it as `meas_imu_EMPTY action=dropped`. Otherwise still usually a *consequence* of H1/H3/H4 — fix those first |
| **H9** | an accumulating cloud republished (`publish_map`) | **applied** — `publish_map()` and `save_to_pcd()` now serialise the ikd-Tree instead of the `pcl_wait_pub` accumulator, and `publish_map()` returns early when nothing subscribes; `map_en` also defaults to `false` in `config/hap.yaml` and `config/mid360.yaml` |

### Experiments worth running as clean A/Bs

| id | change | command |
|---|---|---|
| **E1** | no accumulating map | `--loam-cmd "... config_path:=$PWD/perf/config config_file:=mid360_perf_baseline.yaml"` |
| **E2** | DDS config, three ways | merged (default): plain `source perf/config/perf_env.sh`; stock: `LOAM_PERF_DDS_URI=`; pre-merge baseline: write the snippet below to `perf/config/CycloneDDS.xml` and set `LOAM_PERF_DDS_URI=file://$PWD/perf/config/CycloneDDS.xml` |
| **E3** | no RViz (it subscribes to every cloud) | `rviz:=false` |
| **E4** | decimate harder | `point_filter_num: 6` (or 8) — **no effect for `lidar_type: 4`**: the decimation line in `Preprocess::mid360_handler()` is commented out (`src/preprocess.cpp:514`), so for a Mid-360/HAP the only thinning knob is `filter_size_surf` |
| **E5** | OpenMP residual loop on | `colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFASTLIO_ENABLE_OPENMP_MP=ON -DFASTLIO_MP_PROC_NUM=4` |
| **E6** | denser constraints, for pose stability | `--loam-cmd "... config_path:=$PWD/perf/config config_file:=mid360_dense.yaml"` — `filter_size_surf`/`filter_size_map` 0.5 → 0.25. Compare section 4 against a baseline run of equal length |

Change **one** thing per run and keep the run directories — they are all
self-describing (`run_info.txt` records the config, env, and git revision).

---

## Pose stability

A different question from the rest of this document: not "did the process die"
but "does the pose sit still when the sensor does". Section 4 of the report
answers it, and needs no special run mode — stationary spans are found from the
**raw IMU** (`imu_gyr_mean < 0.02 rad/s`, `imu_acc_std < 0.15 m/s2`), never from
the estimated velocity, which is the quantity under suspicion.

To measure drift deliberately: start the node, leave the platform completely
still for at least a minute, and run the analyzer. Keep it still for the *first*
few seconds too — IMU init estimates gravity and the gyro bias from the first 20
samples, and a platform that is moving then poisons the whole run (the node warns
about this at init).

### Reading section 4

The verdict uses the **settled** column — the second half of the stationary span
— because the first seconds still hold the post-init convergence ramp and would
otherwise be scored as drift. That distinction matters: in one 60 s test the raw
peak-to-peak was 228 mm and the settled figure 5.6 mm. Same data.

Under 10 mm peak-to-peak is solid; over 50 mm is flagged. Then three
explanations are checked, in the order worth believing:

| what it prints | what it means | what to do |
|---|---|---|
| `observability: min eigenvalue X of 0.333` | eigenvalues of the mean plane-normal scatter matrix over the correspondences the EKF used. 0.333 each = every direction constrained. Small = the scene does not pin down `obs_weak_*` | geometry, not tuning. A corridor, one flat wall or an open field cannot constrain the axis. Add structure, or accept drift along it |
| `gravity leaks N m/s2 sideways` | estimated gravity has a horizontal component, i.e. an attitude error. That residual is integrated twice into position — this is the classic cause of a stationary pose sliding | re-init with the platform genuinely still; check the `acc_std` warning printed at IMU init |
| `ba/bg still moving` | the bias states have not converged over the span | let it settle longer before judging drift |

`res_mean` and `eff_feat` come last as a fit-quality sanity check: a residual
that is a large fraction of `filter_size_map` means the scan is not really
locking onto the map.

### A reference measurement

Run `20260902_105419_phase2` (600 s, Mid-360, mostly stationary, `mid360.yaml`),
after the H1/H8/H9 and correspondence-reset fixes:

```
  longest span: t_rel 373..427s (54s, 541 scans)
    axis   peak-to-peak      std      drift rate     settled p2p
    x          14.1 mm      2.3 mm       +0.6 mm/min       10.7 mm
    y          13.5 mm      2.2 mm       -0.9 mm/min       10.5 mm
    z           7.5 mm      1.3 mm       -1.8 mm/min        6.0 mm
    attitude peak-to-peak: roll=0.229 pitch=0.190 yaw=0.223 deg
    fit: mean residual 2.17 cm over 393 effective points
```

Drift rates under 2 mm/min, so it is bounded wander rather than a ramp. The
number that stands out is **393 effective points** against an ikd-Tree of 1902 —
a very thin problem — while the same run used 7.9 ms of its 100 ms per-scan
budget. That is what experiment E6 (`perf/config/mid360_dense.yaml`) tests.

### The columns behind it

Appended to `perf_probe_scan.csv` (existing column positions are unchanged):

`roll_deg pitch_deg yaw_deg` · `grav_x/y/z` · `bg_x/y/z ba_x/y/z` · `res_mean` ·
`imu_gyr_mean imu_acc_std` · `obs_min obs_mid obs_max obs_weak_x/y/z`

All of it is computed inside the probe's `if (flperf::enabled())` block, so it
costs nothing when `FASTLIO_PERF_LOG` is unset.

### The correspondence-reset bug

Found while adding the above, and worth knowing if you are comparing against an
older build. `laserCloudOri` and `corr_normvect` are filled with `push_back` in
`h_share_model()`, which runs once per **EKF iteration**, but they were cleared
once per **scan** in `timer_callback`. With `max_iteration: 3` (and 10 in
`hap.yaml`), iterations 2+ appended behind iteration 1's data while the Jacobian
loop indexes `points[0 .. effct_feat_num)` — so every iteration after the first
solved against the *first* iteration's points and normals. They are now cleared
per iteration. Measured on a 60 s synthetic stationary run:

| | before | after |
|---|---|---|
| attitude peak-to-peak | 6.54 deg | 0.29 deg |
| estimated speed while still | 28.9 mm/s mean, 283 mm/s max | 8.6 mm/s mean, 67 mm/s max |
| mean residual | 2.66 cm | 1.22 cm |
| effective points per scan | 614 | 1955 |
| settled worst-axis p2p | 25.4 mm | 13.7 mm |

Attitude, residual and match count are unambiguous. Position peak-to-peak varied
between single runs — measure it on your own hardware before drawing a
conclusion about the centimetres.

---

## Your Cyclone DDS config vs the merged one

`perf/config/cyclonedds_jetson.xml` is the merge of the pre-existing config with
the changes below; `perf_env.sh` points at it. The pre-merge config is reproduced
at the end of this section so experiment E2 stays runnable. Effective
differences:

| setting | yours | merged | why |
|---|---|---|---|
| `Interfaces/NetworkInterface` | `lo` | `lo` *(kept)* | loopback is the right call for a single box and better than the `autodetermine` originally proposed here: no NIC, no 1500-byte MTU, no IP fragmentation under 64 kB, no physical loss |
| `Discovery/*` | `auto` / `1000` | *(kept)* | needed with this many participants on one host; the original proposal omitted it and was worse for it |
| `Watermarks/WhcHigh` | **500 kB** | **8 MB** | **the consequential one** — see below |
| `SocketReceiveBufferSize` | `min=10MB` | `min=10MB max=64MB` | `min` is what Cyclone accepts, `max` is what it *requests*; and either way `SO_RCVBUF` is capped by `net.core.rmem_max` (stock **212992**, i.e. ~1/50th of 10 MB), so run `setup_target.sh` or this element is decorative |
| `FragmentSize` | *(unset → ~1344B default)* | `64000B` | 1344 is sized for Ethernet. On loopback it chops a 520 kB cloud into ~390 fragments for no benefit. **Only safe because of the `lo` binding** — revert to `1344B` if you ever unbind |
| `Domain@id` | *(unset)* | `any` | no-op, `any` is the default; explicit for readability |

### Why `WhcHigh: 500kB` matters

`WhcHigh` is the writer history cache high-water mark. When a **reliable**
writer's unacknowledged backlog crosses it, Cyclone throttles the writer:
`write()` blocks until the reader catches up or `max_blocking_time` expires.

Every FAST-LIO publisher is reliable — `create_publisher<PointCloud2>(topic, 20)`
takes the rclcpp default. And one Mid-360 cloud is ~520 kB, i.e. **a single
sample at or above your watermark**. Two Mid-360 merged is ~1 MB, over it. With
`publish.map_en: true` the `/Laser_map` sample reaches 100 MB+ — 200x the
watermark.

So the writer throttles on essentially every publish. And `publish_frame_world()`
is called from `timer_callback`, on the **same single executor thread** that must
service `imu_cbk` at 200 Hz. A blocking write there starves IMU intake directly —
which is cause 2 and cause 4, reached by a different route.

`t_publish_ms` in `perf_probe_scan.csv` measures exactly this cost, so the A/B is
already instrumented.

### Confirm what you actually got

Do not trust either file. Uncomment the `Tracing` block at the bottom of
`cyclonedds_jetson.xml` (`Verbosity=config`) and run any node once:

```bash
CYCLONEDDS_URI=file://$PWD/perf/config/cyclonedds_jetson.xml RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp talker
grep -iE "rbuf|receive buffer|fragment|whc" /tmp/cyclonedds_config.log
```

Cyclone dumps the fully resolved configuration, including the socket buffer size
it really obtained. That output is authoritative for your Cyclone version — the
defaults quoted above are from memory and your build may differ.

### The pre-merge config (E2 baseline)

Save as `perf/config/CycloneDDS.xml` when you want to A/B against it:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>true</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="lo" multicast="true" />
      </Interfaces>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB" />
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>1000</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

---

## Files

```
perf/
├── README.md                      this document
├── setup_target.sh                one-time host tuning + preflight report
├── run_test.sh                    orchestrator: runs a experiment, collects everything
├── analyze.py                     run directory -> verdict per hypothesis
├── config/
│   ├── perf_env.sh                source this in every terminal
│   ├── cyclonedds_jetson.xml      merged config (see the comparison section)
│   ├── mid360_perf_baseline.yaml  control config: no accumulating map, no pcd
│   ├── mid360_dual_perf.yaml      2x Mid-360 (read its header before using)
│   └── hap_perf.yaml              1x HAP
├── monitors/
│   ├── stream_monitor.py          rates, gaps, latency, publisher count,
│   │                              per-point timestamp sanity; QoS selectable
│   ├── resource_monitor.py        RSS/CPU/per-thread, thermals, clocks,
│   │                              UDP rcvbuf errors, NIC drops, dmesg
│   └── fake_livox_pub.py          synthetic sensor with fault injection
├── instrumentation/
│   └── perf_probe.hpp             in-process probe (opt-in, header-only)
└── runs/                          output, one directory per run (git-ignored)
```

### The probe

Header-only, no ROS/PCL/Eigen dependency, and gated on one environment variable
read once:

```bash
export FASTLIO_PERF_LOG=/path/to/prefix   # writes _scan.csv and _events.csv
unset  FASTLIO_PERF_LOG                   # off; every hook is an inlined bool test
```

It is already wired into `src/laserMapping.cpp` (`git diff` shows the whole
change) and `CMakeLists.txt` adds the include path. Leaving it in the build costs
nothing when the variable is unset.

### Measuring `extrinsic_T` instead of guessing it

`extrinsic_T` is the LiDAR origin expressed in the IMU frame, in metres —
verified against `pointBodyToWorld()`, which computes
`p_imu = extrinsic_R * p_lidar + extrinsic_T`. If a datasheet gives you the IMU
position *relative to the lidar origin*, that is the opposite direction: with
`extrinsic_R = identity`, negate it.

With `mapping.extrinsic_est_en: true` (the default when the key is absent)
FAST-LIO estimates the three translation states online, so a run started from
zeros measures the true offset for you. Read it back either way:

```bash
# from the probe CSV (lands in the run directory)
python3 -c "
import csv
r=list(csv.DictReader(open('perf/runs/<run>/perf_probe_scan.csv')))[-1]
print(r['ext_t_x'], r['ext_t_y'], r['ext_t_z'])"

# or from the node's own log, written every scan regardless of the probe
awk '{print \$11, \$12, \$13}' Log/mat_pre.txt | tail -20
```

`Log/mat_pre.txt` columns: `$1` time, `$2..$4` euler, `$5..$7` pos, `$8..$10`
extrinsic rotation (euler), **`$11..$13` `offset_T_L_I`**, `$14..$16` vel,
`$17..$19` bg, `$20..$22` ba, `$23..$25` grav.

Let it converge over a minute or two of varied motion, pin the value, then set
`extrinsic_est_en: false` — for crash diagnosis you want those six states fixed,
so a timestamp fault cannot be absorbed as an apparent mounting error.

---

## Caveats

* The Cyclone config uses well-established elements, but Cyclone **refuses to
  start** on one it does not recognise and prints the offending path. If that
  happens, comment that line out; `setup_target.sh --validate-dds` checks it for
  you first.
* `latency_*` columns compare the sensor clock to the host clock. If the lidar is
  not PTP-synced to the host, the absolute value is meaningless but
  `latency_jitter_ms` (relative to the best latency seen) is still valid.
* `resource_monitor.py` needs root for the `dmesg` OOM/segfault check. Without
  it, the rest still works and that section reads `<needs root>`.
* `stream_monitor.py` parses every cloud's timestamp field in Python. At 2x
  Mid-360 rates that costs real CPU on a Jetson — which perturbs the very thing
  you are measuring. For the longest runs, drop the greedy monitor
  (`--no-greedy`) and rely on the in-process probe.
