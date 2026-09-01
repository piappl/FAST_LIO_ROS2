#!/usr/bin/env python3
"""
stream_monitor.py -- FAST_LIO_ROS2 sensor-stream integrity monitor.

Measures, per topic, everything that can starve or poison the FAST-LIO front end:

  * arrival rate + inter-arrival jitter (wall clock)
  * header-stamp gaps  -> estimated MISSED messages (the thing that kills the EKF)
  * header-stamp regressions / duplicates (the signature of two sensors on one topic)
  * transport latency (now - header.stamp) and its jitter
  * publisher count on the topic (definitive check for "2x Mid-360 -> one /livox/imu")
  * for PointCloud2: point count, and per-point `timestamp` field sanity
    (offset span, zeros, negatives, cross-sensor spread) which is what
    Preprocess::mid360_handler turns into `curvature` -> `lidar_end_time`.

The QoS profile is selectable, and that is the core of the experiment:

  --qos loam    reproduces exactly what laserMapping.cpp asks for
                (SensorDataQoS for PointCloud2, reliable/depth-10 for Imu)
  --qos greedy  reliable, KEEP_LAST depth 2000 -- drops (almost) nothing

Run BOTH at once. Then:
  greedy clean + loam gappy  -> the drops are made by the subscriber side
                               (executor starvation / queue too shallow)
  both gappy                 -> the drops are upstream (driver, network, sensor)

Outputs (in --out-dir):
  <tag>_agg.csv     periodic aggregate, one row per --report-period
  <tag>_events.csv  one row per anomaly (gap / regression / bad timestamps)
  <tag>_meta.json   run metadata + final totals

Usage:
  ./stream_monitor.py --tag loam  --qos loam   --out-dir runs/foo
  ./stream_monitor.py --tag greedy --qos greedy --out-dir runs/foo
"""

import argparse
import csv
import json
import os
import signal
import statistics
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

# Livox CustomMsg is optional -- only present if livox_ros_driver2 is on the path.
try:
    from livox_ros_driver2.msg import CustomMsg

    HAVE_CUSTOM_MSG = True
except Exception:  # pragma: no cover - depends on target workspace
    CustomMsg = None
    HAVE_CUSTOM_MSG = False


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def make_qos(kind: str, msg_kind: str) -> QoSProfile:
    """kind: loam|greedy ; msg_kind: cloud|imu"""
    if kind == "greedy":
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2000,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
    # "loam" -- mirror laserMapping.cpp exactly.
    if msg_kind == "cloud":
        # create_subscription<PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), ...)
        return qos_profile_sensor_data
    # create_subscription<Imu>(imu_topic, 10, ...)  -> rclcpp default = reliable
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


CLOUD_EXTRA_KEYS = ("points", "off_max", "off_span", "neg", "zero")


class TopicStats:
    """Rolling + cumulative statistics for one topic."""

    # A gap is declared when the header-stamp delta exceeds this multiple of
    # nominal. Overridable per run via --gap-factor: hardware-stamped sensors
    # tolerate 1.5, but a software-timed source (fake_livox_pub.py, or a driver
    # that stamps on receipt) needs more headroom to avoid false positives.
    GAP_FACTOR = 1.5

    def __init__(self, name: str, nominal_hz: float, extra_keys=(),
                 gap_factor: float = None):
        self.name = name
        self.nominal_hz = float(nominal_hz)
        # keys of the per-window, message-kind specific accumulators; they must
        # survive reset_window(), otherwise the callbacks KeyError after the
        # first report tick.
        self.extra_keys = tuple(extra_keys)
        if gap_factor is not None:
            self.GAP_FACTOR = float(gap_factor)
        self.nominal_dt = 1.0 / self.nominal_hz if self.nominal_hz > 0 else 0.0

        self.first_wall = None
        self.last_wall = None
        self.last_stamp = None

        # cumulative
        self.count = 0
        self.gap_events = 0
        self.missed_est = 0
        self.regressions = 0
        self.duplicates = 0
        self.min_latency = float("inf")
        self.last_publishers = -1  # cached; finish() must not touch rcl context

        # window
        self.reset_window()

    def reset_window(self):
        self.w_count = 0
        self.w_arr_dt = []      # wall inter-arrival
        self.w_stamp_dt = []    # header-stamp inter-arrival
        self.w_latency = []
        self.w_gap_events = 0
        self.w_missed_est = 0
        self.w_regressions = 0
        self.w_duplicates = 0
        # message-kind specific accumulators, rebuilt every window
        self.w_extra = {k: [] for k in self.extra_keys}

    def update(self, stamp_sec: float, wall: float):
        """Returns list of (kind, detail) anomalies for this message."""
        anomalies = []
        self.count += 1
        self.w_count += 1

        if self.first_wall is None:
            self.first_wall = wall

        if self.last_wall is not None:
            self.w_arr_dt.append(wall - self.last_wall)
        self.last_wall = wall

        latency = wall - stamp_sec
        self.w_latency.append(latency)
        if latency < self.min_latency:
            self.min_latency = latency

        if self.last_stamp is not None:
            dt = stamp_sec - self.last_stamp
            self.w_stamp_dt.append(dt)
            if dt < 0.0:
                self.regressions += 1
                self.w_regressions += 1
                anomalies.append(("stamp_regression", f"dt={dt:+.6f}s"))
            elif dt == 0.0:
                self.duplicates += 1
                self.w_duplicates += 1
                anomalies.append(("stamp_duplicate", "dt=0"))
            elif self.nominal_dt > 0.0 and dt > self.GAP_FACTOR * self.nominal_dt:
                missed = int(round(dt / self.nominal_dt)) - 1
                missed = max(missed, 1)
                self.gap_events += 1
                self.w_gap_events += 1
                self.missed_est += missed
                self.w_missed_est += missed
                anomalies.append(
                    ("stamp_gap", f"dt={dt*1e3:.2f}ms missed~{missed}")
                )
        self.last_stamp = stamp_sec
        return anomalies

    # ---- reporting helpers -------------------------------------------------
    @staticmethod
    def _stat(vals, fn, default=float("nan")):
        return fn(vals) if vals else default

    def window_row(self, t_rel: float, window_s: float, publishers: int) -> dict:
        arr = self.w_arr_dt
        st = self.w_stamp_dt
        lat = self.w_latency
        return {
            "t_rel_s": round(t_rel, 3),
            "topic": self.name,
            "publishers": publishers,
            "msgs": self.w_count,
            "hz": round(self.w_count / window_s, 3) if window_s > 0 else 0.0,
            "arr_dt_mean_ms": self._fmt_ms(self._stat(arr, statistics.fmean)),
            "arr_dt_max_ms": self._fmt_ms(self._stat(arr, max)),
            "stamp_dt_mean_ms": self._fmt_ms(self._stat(st, statistics.fmean)),
            "stamp_dt_max_ms": self._fmt_ms(self._stat(st, max)),
            "stamp_dt_min_ms": self._fmt_ms(self._stat(st, min)),
            "latency_mean_ms": self._fmt_ms(self._stat(lat, statistics.fmean)),
            "latency_max_ms": self._fmt_ms(self._stat(lat, max)),
            # latency relative to the best latency ever seen -> clock-offset free
            "latency_jitter_ms": self._fmt_ms(
                (self._stat(lat, max) - self.min_latency)
                if lat and self.min_latency != float("inf")
                else float("nan")
            ),
            "gap_events": self.w_gap_events,
            "missed_est": self.w_missed_est,
            "stamp_regressions": self.w_regressions,
            "stamp_duplicates": self.w_duplicates,
        }

    @staticmethod
    def _fmt_ms(v: float) -> float:
        if v != v:  # NaN
            return ""
        return round(v * 1e3, 4)

    def totals(self) -> dict:
        span = (
            (self.last_wall - self.first_wall)
            if (self.first_wall is not None and self.last_wall is not None)
            else 0.0
        )
        return {
            "topic": self.name,
            "nominal_hz": self.nominal_hz,
            "messages": self.count,
            "duration_s": round(span, 3),
            "mean_hz": round(self.count / span, 3) if span > 0 else 0.0,
            "gap_events": self.gap_events,
            "missed_est": self.missed_est,
            "loss_pct_est": (
                round(100.0 * self.missed_est / (self.count + self.missed_est), 4)
                if (self.count + self.missed_est) > 0
                else 0.0
            ),
            "stamp_regressions": self.regressions,
            "stamp_duplicates": self.duplicates,
            "min_latency_ms": (
                round(self.min_latency * 1e3, 4)
                if self.min_latency != float("inf")
                else ""
            ),
        }


class CloudFieldAnalyzer:
    """
    Extracts the per-point `timestamp` (float64, ns since epoch) that
    Preprocess::mid360_handler consumes:

        curvature = (point.timestamp - header_stamp_ns) / 1e6   # -> ms
        curvature = max(0, curvature)

    and sync_packages() then does:

        lidar_end_time = lidar_beg_time + max(curvature)/1000

    So a cloud whose `timestamp` field is absent, zero, or drawn from two
    unsynchronised sensors directly corrupts lidar_end_time and therefore the
    whole IMU/LiDAR association. This class reports exactly that.
    """

    def __init__(self):
        self.warned_no_field = False

    def analyze(self, msg: PointCloud2, header_sec: float):
        out = {
            "points": msg.width * msg.height,
            "ts_field": 0,
            "off_min_ms": "",
            "off_max_ms": "",
            "off_span_ms": "",
            "neg_frac": "",
            "zero_frac": "",
        }
        n = msg.width * msg.height
        if n == 0:
            return out, [("empty_cloud", "0 points")]

        fld = next((f for f in msg.fields if f.name == "timestamp"), None)
        if fld is None:
            return out, ([("no_timestamp_field",
                           "fields=" + ",".join(f.name for f in msg.fields))]
                         if not self.warned_no_field else [])

        out["ts_field"] = 1
        # datatype 8 == FLOAT64
        if fld.datatype != 8:
            return out, [("timestamp_not_f64", f"datatype={fld.datatype}")]

        try:
            dt = np.dtype({
                "names": ["ts"],
                "formats": [np.float64],
                "offsets": [fld.offset],
                "itemsize": msg.point_step,
            })
            need = n * msg.point_step
            buf = msg.data[:need] if len(msg.data) >= need else msg.data
            usable = len(buf) // msg.point_step
            if usable == 0:
                return out, [("short_cloud_buffer", f"len={len(buf)}")]
            ts = np.frombuffer(bytes(buf[: usable * msg.point_step]),
                               dtype=dt)["ts"]
        except Exception as exc:  # pragma: no cover
            return out, [("timestamp_parse_error", str(exc))]

        anomalies = []
        header_ns = header_sec * 1e9
        # replicate the C++ arithmetic: (ts_ns - header_ns) / 1e6 -> ms
        off_ms = (ts - header_ns) / 1e6
        finite = np.isfinite(off_ms)
        if not finite.all():
            anomalies.append(("timestamp_nonfinite",
                              f"{int((~finite).sum())} pts"))
            off_ms = off_ms[finite]
            if off_ms.size == 0:
                return out, anomalies

        zero_frac = float(np.count_nonzero(ts == 0.0)) / float(ts.size)
        neg_frac = float(np.count_nonzero(off_ms < 0.0)) / float(off_ms.size)
        o_min = float(off_ms.min())
        o_max = float(off_ms.max())

        out["off_min_ms"] = round(o_min, 4)
        out["off_max_ms"] = round(o_max, 4)
        out["off_span_ms"] = round(o_max - o_min, 4)
        out["neg_frac"] = round(neg_frac, 5)
        out["zero_frac"] = round(zero_frac, 5)

        # --- the diagnostics that matter -----------------------------------
        if zero_frac > 0.5:
            anomalies.append(("timestamp_all_zero",
                              f"{zero_frac*100:.1f}% of points ts==0"))
        # max(curvature) is what becomes lidar_end_time; clamped at 0 by C++
        if o_max <= 0.0:
            anomalies.append(("curvature_max_nonpositive",
                              f"max_off={o_max:.4f}ms -> lidar_end_time collapses"))
        # a single Mid-360 frame spans ~100ms at 10Hz; much more => merged
        # clouds from unsynchronised sensors
        span = o_max - o_min
        if span > 250.0:
            anomalies.append(("timestamp_span_huge",
                              f"span={span:.1f}ms -> unsynced multi-sensor merge?"))
        if neg_frac > 0.05:
            anomalies.append(("timestamp_negative",
                              f"{neg_frac*100:.1f}% of points before header stamp"))
        return out, anomalies


class StreamMonitor(Node):
    def __init__(self, args):
        super().__init__(f"stream_monitor_{args.tag}")
        self.args = args
        self.t0 = time.time()
        self.stats = {}
        self.cloud_analyzer = CloudFieldAnalyzer()
        self.event_count = 0

        os.makedirs(args.out_dir, exist_ok=True)
        self.agg_path = os.path.join(args.out_dir, f"{args.tag}_agg.csv")
        self.ev_path = os.path.join(args.out_dir, f"{args.tag}_events.csv")
        self.meta_path = os.path.join(args.out_dir, f"{args.tag}_meta.json")

        self.agg_fields = [
            "t_rel_s", "topic", "publishers", "msgs", "hz",
            "arr_dt_mean_ms", "arr_dt_max_ms",
            "stamp_dt_mean_ms", "stamp_dt_max_ms", "stamp_dt_min_ms",
            "latency_mean_ms", "latency_max_ms", "latency_jitter_ms",
            "gap_events", "missed_est", "stamp_regressions", "stamp_duplicates",
            "points_mean", "points_min", "points_max",
            "off_max_ms_mean", "off_span_ms_max", "neg_frac_max", "zero_frac_max",
        ]
        self.agg_f = open(self.agg_path, "w", newline="")
        self.agg_w = csv.DictWriter(self.agg_f, fieldnames=self.agg_fields)
        self.agg_w.writeheader()

        self.ev_f = open(self.ev_path, "w", newline="")
        self.ev_w = csv.DictWriter(
            self.ev_f,
            fieldnames=["t_rel_s", "wall_unix", "topic", "kind", "detail"],
        )
        self.ev_w.writeheader()

        self.subs = []

        # ---- IMU ----------------------------------------------------------
        for topic in args.imu_topic:
            self.stats[topic] = TopicStats(topic, args.imu_rate,
                                          gap_factor=args.gap_factor)
            self.subs.append(
                self.create_subscription(
                    Imu, topic,
                    self._make_imu_cb(topic),
                    make_qos(args.qos, "imu"),
                )
            )

        # ---- PointCloud2 --------------------------------------------------
        for topic in args.cloud_topic:
            st = TopicStats(topic, args.cloud_rate, extra_keys=CLOUD_EXTRA_KEYS,
                            gap_factor=args.gap_factor)
            self.stats[topic] = st
            self.subs.append(
                self.create_subscription(
                    PointCloud2, topic,
                    self._make_cloud_cb(topic),
                    make_qos(args.qos, "cloud"),
                )
            )

        # ---- Livox CustomMsg ---------------------------------------------
        for topic in args.custom_topic:
            if not HAVE_CUSTOM_MSG:
                self.get_logger().error(
                    f"--custom-topic {topic} requested but livox_ros_driver2 "
                    "python messages are not importable; skipping."
                )
                continue
            st = TopicStats(topic, args.cloud_rate, extra_keys=CLOUD_EXTRA_KEYS,
                            gap_factor=args.gap_factor)
            self.stats[topic] = st
            self.subs.append(
                self.create_subscription(
                    CustomMsg, topic,
                    self._make_custom_cb(topic),
                    make_qos(args.qos, "cloud"),
                )
            )

        self.timer = self.create_timer(args.report_period, self._report)
        if args.duration > 0:
            self.create_timer(args.duration, self._stop)

        self.get_logger().info(
            f"[{args.tag}] qos={args.qos} imu={args.imu_topic} "
            f"cloud={args.cloud_topic} custom={args.custom_topic} "
            f"-> {args.out_dir}"
        )

    # ---- callbacks --------------------------------------------------------
    def _log_events(self, topic, anomalies, wall):
        for kind, detail in anomalies:
            self.event_count += 1
            self.ev_w.writerow({
                "t_rel_s": round(wall - self.t0, 3),
                "wall_unix": round(wall, 6),
                "topic": topic,
                "kind": kind,
                "detail": detail,
            })
            if self.args.verbose_events:
                self.get_logger().warning(f"{topic}: {kind}: {detail}")
        if anomalies:
            self.ev_f.flush()

    def _make_imu_cb(self, topic):
        st = self.stats[topic]

        def cb(msg):
            wall = time.time()
            self._log_events(topic, st.update(stamp_to_sec(msg.header.stamp), wall),
                             wall)

        return cb

    def _make_cloud_cb(self, topic):
        st = self.stats[topic]

        def cb(msg):
            wall = time.time()
            hs = stamp_to_sec(msg.header.stamp)
            anomalies = st.update(hs, wall)
            info, more = self.cloud_analyzer.analyze(msg, hs)
            anomalies += more
            e = st.w_extra
            e["points"].append(info["points"])
            for key, dst in (("off_max_ms", "off_max"), ("off_span_ms", "off_span"),
                             ("neg_frac", "neg"), ("zero_frac", "zero")):
                if info[key] != "":
                    e[dst].append(info[key])
            self._log_events(topic, anomalies, wall)

        return cb

    def _make_custom_cb(self, topic):
        st = self.stats[topic]

        def cb(msg):
            wall = time.time()
            hs = stamp_to_sec(msg.header.stamp)
            anomalies = st.update(hs, wall)
            e = st.w_extra
            n = len(msg.points)
            e["points"].append(n)
            if n:
                # CustomMsg carries offset_time (ns, relative to timebase)
                offs = np.fromiter((p.offset_time for p in msg.points),
                                   dtype=np.float64, count=n) / 1e6  # -> ms
                o_min, o_max = float(offs.min()), float(offs.max())
                e["off_max"].append(round(o_max, 4))
                e["off_span"].append(round(o_max - o_min, 4))
                e["neg"].append(round(float(np.count_nonzero(offs < 0)) / n, 5))
                e["zero"].append(round(float(np.count_nonzero(offs == 0)) / n, 5))
                if o_max <= 0.0:
                    anomalies.append(("curvature_max_nonpositive",
                                      f"max_off={o_max:.4f}ms"))
                if (o_max - o_min) > 250.0:
                    anomalies.append(("timestamp_span_huge",
                                      f"span={o_max-o_min:.1f}ms"))
            else:
                anomalies.append(("empty_cloud", "0 points"))
            self._log_events(topic, anomalies, wall)

        return cb

    def _safe_count_publishers(self, topic: str) -> int:
        try:
            return self.count_publishers(topic)
        except Exception:
            return -1

    # ---- periodic report -------------------------------------------------
    def _report(self):
        now = time.time()
        t_rel = now - self.t0
        window = self.args.report_period
        lines = []
        for topic, st in self.stats.items():
            npub = self._safe_count_publishers(topic)
            st.last_publishers = npub
            row = st.window_row(t_rel, window, npub)
            e = st.w_extra
            row.update({
                "points_mean": round(statistics.fmean(e["points"]), 1)
                if e.get("points") else "",
                "points_min": min(e["points"]) if e.get("points") else "",
                "points_max": max(e["points"]) if e.get("points") else "",
                "off_max_ms_mean": round(statistics.fmean(e["off_max"]), 3)
                if e.get("off_max") else "",
                "off_span_ms_max": max(e["off_span"]) if e.get("off_span") else "",
                "neg_frac_max": max(e["neg"]) if e.get("neg") else "",
                "zero_frac_max": max(e["zero"]) if e.get("zero") else "",
            })
            self.agg_w.writerow({k: row.get(k, "") for k in self.agg_fields})

            warn = ""
            if npub > 1:
                warn += f" !! {npub} PUBLISHERS"
            if row["missed_est"]:
                warn += f" !! missed~{row['missed_est']}"
            if row["stamp_regressions"]:
                warn += f" !! regress={row['stamp_regressions']}"
            lines.append(
                f"  {topic:<28} {row['hz']:>7.2f}Hz  "
                f"lat {row['latency_mean_ms'] or '?'}ms "
                f"(jit {row['latency_jitter_ms'] or '?'}) "
                f"dt_max {row['stamp_dt_max_ms'] or '?'}ms"
                f"{warn}"
            )
            st.reset_window()
        self.agg_f.flush()
        self.get_logger().info(
            f"[{self.args.tag}] t={t_rel:7.1f}s events={self.event_count}\n"
            + "\n".join(lines)
        )

    def _stop(self):
        raise KeyboardInterrupt

    # ---- teardown --------------------------------------------------------
    def finish(self):
        meta = {
            "tag": self.args.tag,
            "qos": self.args.qos,
            "started_unix": self.t0,
            "ended_unix": time.time(),
            "duration_s": round(time.time() - self.t0, 3),
            "gap_factor": self.args.gap_factor,
            "imu_nominal_hz": self.args.imu_rate,
            "cloud_nominal_hz": self.args.cloud_rate,
            "event_count": self.event_count,
            "topics": [st.totals() for st in self.stats.values()],
            # cached from the last report tick: the rcl context can already be
            # torn down by the time we get here (SIGTERM/SIGINT path).
            "publishers": {t: st.last_publishers for t, st in self.stats.items()},
        }
        with open(self.meta_path, "w") as f:
            json.dump(meta, f, indent=2)
        self.agg_f.close()
        self.ev_f.close()

        print(f"\n=== [{self.args.tag}] summary ===", file=sys.stderr)
        for t in meta["topics"]:
            print(
                f"  {t['topic']:<28} n={t['messages']:<8} "
                f"{t['mean_hz']:>7.2f}Hz  gaps={t['gap_events']:<5} "
                f"missed~{t['missed_est']:<6} (~{t['loss_pct_est']}%)  "
                f"regress={t['stamp_regressions']} dup={t['stamp_duplicates']}",
                file=sys.stderr,
            )
        print(f"  -> {self.agg_path}\n  -> {self.ev_path}\n  -> {self.meta_path}",
              file=sys.stderr)


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="FAST_LIO_ROS2 sensor stream integrity monitor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--tag", default="mon", help="prefix for output files")
    p.add_argument("--qos", choices=["loam", "greedy"], default="loam",
                   help="loam = mirror laserMapping.cpp; greedy = deep reliable")
    p.add_argument("--imu-topic", action="append", default=None)
    p.add_argument("--cloud-topic", action="append", default=None)
    p.add_argument("--custom-topic", action="append", default=None,
                   help="livox_ros_driver2/CustomMsg topic (AVIA path)")
    p.add_argument("--imu-rate", type=float, default=200.0,
                   help="nominal IMU rate (Mid-360 / HAP = 200)")
    p.add_argument("--cloud-rate", type=float, default=10.0,
                   help="nominal cloud rate")
    p.add_argument("--gap-factor", type=float, default=1.5,
                   help="declare a gap when the stamp delta exceeds this "
                        "multiple of the nominal period (default 1.5)")
    p.add_argument("--report-period", type=float, default=5.0)
    p.add_argument("--duration", type=float, default=0.0, help="0 = until Ctrl-C")
    p.add_argument("--out-dir", default="runs/manual")
    p.add_argument("--verbose-events", action="store_true",
                   help="also log every anomaly to the console")
    a = p.parse_args(argv)
    if a.imu_topic is None:
        a.imu_topic = ["/livox/imu"]
    if a.cloud_topic is None:
        a.cloud_topic = ["/livox/lidar"]
    if a.custom_topic is None:
        a.custom_topic = []
    # allow --imu-topic "" to disable
    a.imu_topic = [t for t in a.imu_topic if t]
    a.cloud_topic = [t for t in a.cloud_topic if t]
    a.custom_topic = [t for t in a.custom_topic if t]
    return a


def _install_signal_handlers():
    """Turn SIGTERM/SIGINT into KeyboardInterrupt so finish() always runs and
    the CSV/JSON output is complete even when the orchestrator kills us."""

    def handler(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):  # pragma: no cover
            pass


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    _install_signal_handlers()
    rclpy.init(args=None)
    node = StreamMonitor(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
