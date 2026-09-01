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

### 2. One thread does everything

`main()` uses `rclcpp::spin()` — a **single-threaded** executor. So
`timer_callback` (IMU integration, voxel downsample, ICP, ikd-Tree insert,
publishing) shares one thread with `imu_cbk` at 200 Hz and the lidar callback at
10 Hz. While a scan is being processed, no sensor callback runs at all.

The IMU subscription is `create_subscription<Imu>(imu_topic, 10, imu_cbk)` —
depth 10, which at 200 Hz is **50 ms of buffer**. Any processing stall longer
than that and IMU samples are dropped by the middleware before your code sees
them. The lidar subscription uses `SensorDataQoS()`: best-effort, depth 5.

That is what the `--qos loam` vs `--qos greedy` A/B in `stream_monitor.py`
measures directly.

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

Check:

| what | where | expected |
|---|---|---|
| exactly one publisher per topic | `pubs=` | **1** |
| IMU rate | `hz` | ~200 |
| stamp regressions | `stamp_regressions` | **0** |
| per-point time span | `off_span_ms_max` | ~100 ms at 10 Hz |
| points with `timestamp==0` | `zero_frac_max` | 0 |

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

* greedy clean **and** loamqos gappy → the loss is created at the subscriber:
  shallow queue + starved single thread (causes 1 and 2).
* both gappy → the loss is upstream: driver, network, or socket buffers.

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

| id | hypothesis | fix if confirmed |
|---|---|---|
| **H1** | loss at the subscriber (shallow QoS + starved single thread) | raise the IMU subscription depth well above 10 (200 Hz × worst stall); give the sensor callbacks their own `CallbackGroup` — **and add `mtx_buffer` locking to `sync_packages()` before introducing threads**. Also raise `WhcHigh` (see below): a throttled reliable writer blocks `publish()` *on that same thread* |
| **H2** | loss in the transport (socket buffers, NIC) | `perf/setup_target.sh` (raises `net.core.rmem_max`) plus `SocketReceiveBufferSize` in `perf/config/cyclonedds_jetson.xml`; jumbo frames on the lidar link if available |
| **H3** | two sensors on one topic / unsynchronised clocks | one topic per sensor, or merge in a node that reorders by stamp; PTP both lidars off one master; point `imu_topic` at a single IMU |
| **H4** | per-point timestamps corrupting `lidar_end_time` | fix the driver's timestamp mode; verify `off_span_ms_max` ≈ 100 ms; treat a non-positive `max(curvature)` as a hard error rather than clamping it |
| **H5** | cannot keep up → growing backlog | raise `point_filter_num`, raise `filter_size_surf`/`filter_size_map`, lower `max_iteration`, cap `cube_side_length`; enable OpenMP (E5); bound the deques and drop oldest instead of growing |
| **H6** | out of memory | `map_en: false`, `pcd_save_en: false`; bound the deques; watch ikd-Tree growth |
| **H7** | Jetson throttling | `nvpmodel -m 0` + `jetson_clocks`, check cooling; `setup_target.sh --max-perf` |
| **H8** | state divergence (bad IMU association) | usually a *consequence* of H1/H3/H4 — fix those first |
| **H9** | an accumulating cloud republished (`publish_map`) | `map_en: false`; or clear `pcl_wait_pub` after publishing / publish the ikd-Tree instead |

### Experiments worth running as clean A/Bs

| id | change | command |
|---|---|---|
| **E1** | no accumulating map | `--loam-cmd "... config_path:=$PWD/perf/config config_file:=mid360_perf_baseline.yaml"` |
| **E2** | DDS config, three ways | merged (default): plain `source perf/config/perf_env.sh`; stock: `LOAM_PERF_DDS_URI=`; pre-merge baseline: write the snippet below to `perf/config/CycloneDDS.xml` and set `LOAM_PERF_DDS_URI=file://$PWD/perf/config/CycloneDDS.xml` |
| **E3** | no RViz (it subscribes to every cloud) | `rviz:=false` |
| **E4** | decimate harder | `point_filter_num: 6` (or 8) |
| **E5** | OpenMP residual loop on | `colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFASTLIO_ENABLE_OPENMP_MP=ON -DFASTLIO_MP_PROC_NUM=4` |

Change **one** thing per run and keep the run directories — they are all
self-describing (`run_info.txt` records the config, env, and git revision).

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

It is already wired into `src/laserMapping.cpp` (15 call sites; `git diff` shows
the whole change) and `CMakeLists.txt` adds the include path. Leaving it in the
build costs nothing when the variable is unset.

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
