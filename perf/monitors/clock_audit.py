#!/usr/bin/env python3
"""
clock_audit.py -- where does the time error actually come from?

stream_monitor.py measures stream integrity and says so itself (README, "Reading
section 4"): the latency columns compare the sensor clock to the host clock, and
"if the lidar is not PTP-synced to the host, the absolute value is meaningless".
This script is the missing half: it audits the CLOCK LAYER underneath, so a
timestamp complaint can be attributed to one of

    host clock  ->  PTP/NTP discipline  ->  per-sensor clock  ->  merger  ->  LOAM

rather than guessed at. It answers one question: is the time error made by our
nodes, or is something under them producing/amplifying it?

It reports three layers.

  LAYER 0  host (no ROS needed, --host-only)
      CLOCK_REALTIME / MONOTONIC / TAI / BOOTTIME, and the kernel TAI-UTC offset
      (must be 37 s; 0 means nothing set the offset, which silently breaks every
      PTP<->system conversion), NTP/chrony state, PTP hardware clocks and their
      offset from the system clock, ptp4l/phc2sys liveness, NIC hardware
      timestamping, and recent clock STEP events in the journal.

  LAYER 1  ROS
      ROS time vs system time, use_sim_time, /clock publishers.

  LAYER 2  topics
      Per topic: header stamp vs host arrival time. The MINIMUM of that over the
      window is the interesting number -- transport latency only ever adds, so
      min(now - stamp) is the tightest upper bound on the sensor-to-host clock
      offset. Per cloud it also reads the per-point `timestamp` field with the
      exact arithmetic Preprocess::mid360_handler uses.

      Then the comparisons that actually localise the fault:
        * per-unit offset vs the MEDIAN unit  -> one bad unit vs all units shifted
        * merged header vs its member headers -> what the merger's min() picked
        * IMU vs lidar                        -> the pairing FAST-LIO depends on

Usage
-----
  # everything, reading the lidar list straight out of the merger config
  ./clock_audit.py --from-merge-config \
      ~/claude_ws/src/auto2_bringup/config/autonomy/pointcloud_merge_default.yaml \
      --namespace /piap_ibis_medevac --imu-topic /piap_ibis_medevac/imu/data \
      --duration 20

  # host clock only -- no ROS, run it on the robot and on the PTP master
  ./clock_audit.py --host-only

  # explicit topics
  ./clock_audit.py --lidar-topic /ns/livox/mid360_0/lidar \
                   --lidar-topic /ns/livox/hap_0/lidar \
                   --merged-topic /ns/merged_pointcloud \
                   --imu-topic /ns/imu/data --duration 30

Exit status is 0 when nothing suspicious was found, 1 when at least one finding
was reported, so it can gate a preflight script.
"""

import argparse
import ctypes
import glob
import json
import os
import re
import statistics
import subprocess
import sys
import time

# Current TAI-UTC difference. PTP runs on TAI, the ROS/system clocks on UTC, so
# every PTP-disciplined sensor needs this applied exactly once. Applied twice or
# not at all, the sensor lands a whole 37 s from the host -- which is the single
# most common Livox timestamp fault, and is trivially recognisable.
TAI_UTC_OFFSET = 37.0

CLOCK_TAI = 11
CLOCK_BOOTTIME = 7


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------

def run(cmd, timeout=5.0):
    """Run a command, return (ok, stdout+stderr). Never raises."""
    try:
        p = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True,
                           text=True, timeout=timeout)
        return p.returncode == 0, (p.stdout + p.stderr).strip()
    except FileNotFoundError:
        return False, "(command not found)"
    except subprocess.TimeoutExpired:
        return False, "(timed out)"
    except Exception as exc:                                  # pragma: no cover
        return False, "(%s)" % exc


def have(tool):
    return subprocess.run(["which", tool], capture_output=True).returncode == 0


def fmt_ms(x):
    return "n/a" if x is None else "%+.3f ms" % (x * 1e3)


def fmt_s(x):
    return "n/a" if x is None else "%+.6f s" % x


class Report:
    """Collects sections and findings; prints a single readable report."""

    def __init__(self):
        self.sections = []
        self.findings = []
        self.data = {}

    def section(self, title):
        self.sections.append((title, []))
        return self

    def line(self, text):
        if not self.sections:
            self.section("")
        self.sections[-1][1].append(text)

    def kv(self, key, value, note=""):
        self.line("  %-34s %s%s" % (key, value, ("   " + note) if note else ""))

    def finding(self, severity, where, text):
        """severity: 'CRITICAL' | 'WARN' | 'INFO'"""
        self.findings.append((severity, where, text))

    def render(self):
        out = []
        for title, lines in self.sections:
            if title:
                out.append("")
                out.append(title)
                out.append("-" * len(title))
            out.extend(lines)
        out.append("")
        out.append("=" * 78)
        out.append("FINDINGS")
        out.append("=" * 78)
        if not self.findings:
            out.append("  none -- every clock checked agrees within tolerance.")
        else:
            order = {"CRITICAL": 0, "WARN": 1, "INFO": 2}
            for sev, where, text in sorted(self.findings,
                                           key=lambda f: order.get(f[0], 9)):
                out.append("  [%-8s] %s" % (sev, where))
                for chunk in text.split("\n"):
                    out.append("             %s" % chunk)
        return "\n".join(out)


# --------------------------------------------------------------------------
# LAYER 0 -- host clocks
# --------------------------------------------------------------------------

def phc_clockid(fd):
    """Kernel's dynamic POSIX clock id for a /dev/ptpN file descriptor."""
    return (~fd << 3) | 3


def read_phc_offset(dev):
    """
    Offset of a PTP hardware clock from CLOCK_REALTIME, in seconds.

    Sampled as realtime/phc/realtime so the read latency can be subtracted --
    the same idea as PTP_SYS_OFFSET, without needing the ioctl.
    Returns (offset_sec, read_latency_sec) or (None, reason).
    """
    try:
        fd = os.open(dev, os.O_RDONLY)
    except PermissionError:
        return None, "permission denied (needs root or a udev rule)"
    except OSError as exc:
        return None, str(exc)
    try:
        cid = phc_clockid(fd)
        best = None
        for _ in range(5):
            t0 = time.clock_gettime(time.CLOCK_REALTIME)
            tp = time.clock_gettime(cid)
            t1 = time.clock_gettime(time.CLOCK_REALTIME)
            lat = t1 - t0
            if best is None or lat < best[1]:
                best = (tp - (t0 + t1) / 2.0, lat)
        return best
    except OSError as exc:
        return None, "cannot read as a clock (%s)" % exc
    finally:
        os.close(fd)


def audit_host(rep, args):
    rep.section("LAYER 0 -- host clocks")

    # --- raw clocks --------------------------------------------------------
    realtime = time.clock_gettime(time.CLOCK_REALTIME)
    monotonic = time.clock_gettime(time.CLOCK_MONOTONIC)
    rep.kv("CLOCK_REALTIME", "%.6f" % realtime,
           "(%s UTC)" % time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(realtime)))
    rep.kv("CLOCK_MONOTONIC", "%.6f" % monotonic, "(uptime)")
    try:
        boottime = time.clock_gettime(CLOCK_BOOTTIME)
        rep.kv("CLOCK_BOOTTIME", "%.6f" % boottime)
    except Exception:
        pass

    rep.data["realtime"] = realtime

    # --- the TAI offset, the classic PTP trap ------------------------------
    try:
        tai = time.clock_gettime(CLOCK_TAI)
        tai_off = tai - realtime
        rep.kv("CLOCK_TAI - CLOCK_REALTIME", "%.3f s" % tai_off,
               "(expected %.0f)" % TAI_UTC_OFFSET)
        rep.data["tai_offset"] = tai_off
        if abs(tai_off) < 1.0:
            rep.finding(
                "WARN", "host: kernel TAI offset is 0",
                "PTP time is TAI, the system clock is UTC. With the kernel offset\n"
                "unset, anything converting between them is off by %.0f s. ptp4l sets\n"
                "this via adjtimex when it is the one disciplining the system clock;\n"
                "if phc2sys runs without it, set it explicitly." % TAI_UTC_OFFSET)
        elif abs(tai_off - TAI_UTC_OFFSET) > 1.0:
            rep.finding(
                "WARN", "host: kernel TAI offset is %.0f s, expected %.0f s"
                % (tai_off, TAI_UTC_OFFSET),
                "Leap-second table is stale or wrong; PTP<->UTC conversions inherit it.")
    except Exception:
        rep.kv("CLOCK_TAI", "unavailable")

    # --- NTP / chrony ------------------------------------------------------
    ok, out = run(["timedatectl", "show"])
    if ok:
        props = dict(l.split("=", 1) for l in out.splitlines() if "=" in l)
        rep.kv("timedatectl NTPSynchronized", props.get("NTPSynchronized", "?"))
        rep.kv("timedatectl NTP", props.get("NTP", "?"))
        rep.kv("timedatectl Timezone", props.get("Timezone", "?"))
        rep.data["ntp_synchronized"] = props.get("NTPSynchronized")
        if props.get("NTPSynchronized") == "no" and not glob.glob("/dev/ptp*"):
            rep.finding(
                "WARN", "host: system clock is not NTP-synchronised",
                "and there is no PTP hardware clock either. The host clock is free-\n"
                "running, so every 'sensor stamp vs host' number below is relative to\n"
                "a drifting reference.")
    else:
        rep.kv("timedatectl", "unavailable")

    if have("chronyc"):
        ok, out = run(["chronyc", "-n", "tracking"])
        if ok:
            for key in ("Reference ID", "Stratum", "System time", "Last offset",
                        "RMS offset", "Frequency", "Root dispersion", "Leap status"):
                m = re.search(r"^%s\s*:\s*(.+)$" % re.escape(key), out, re.M)
                if m:
                    rep.kv("chrony %s" % key, m.group(1).strip())
            m = re.search(r"^System time\s*:\s*([0-9.]+)\s+seconds", out, re.M)
            if m and float(m.group(1)) > 0.010:
                rep.finding(
                    "WARN", "host: chrony reports system time %s s from NTP"
                    % m.group(1),
                    "The host reference itself is off by more than 10 ms.")
            m = re.search(r"^Leap status\s*:\s*(.+)$", out, re.M)
            if m and "Normal" not in m.group(1):
                rep.finding("WARN", "host: chrony leap status is %s" % m.group(1).strip(),
                            "A pending leap second changes TAI-UTC while you are running.")
    elif have("ntpq"):
        ok, out = run(["ntpq", "-pn"])
        if ok:
            rep.line("  ntpq -pn:")
            for l in out.splitlines()[:8]:
                rep.line("      " + l)
    else:
        rep.kv("chrony/ntp client", "neither chronyc nor ntpq present")

    # --- PTP ---------------------------------------------------------------
    devs = sorted(glob.glob("/dev/ptp*"))
    rep.kv("PTP hardware clocks", ", ".join(devs) if devs else "none found")
    phc_offsets = {}
    for dev in devs:
        off, extra = read_phc_offset(dev)
        if off is None:
            rep.kv("  %s offset" % dev, "unreadable", "(%s)" % extra)
        else:
            phc_offsets[dev] = off
            rep.kv("  %s vs CLOCK_REALTIME" % dev, fmt_ms(off),
                   "(read latency %.0f us)" % (extra * 1e6))
            # A PHC sitting exactly TAI_UTC_OFFSET from the system clock is
            # normal when phc2sys is not bridging them; flag it as the likely
            # explanation rather than as an error.
            if abs(abs(off) - TAI_UTC_OFFSET) < 1.0:
                rep.finding(
                    "INFO", "host: %s is %.0f s from CLOCK_REALTIME" % (dev, off),
                    "That is the TAI-UTC offset: the PHC is on TAI and nothing is\n"
                    "converting. Expected if phc2sys is not running, a bug if the\n"
                    "sensor stamps are taken straight off this clock.")
            elif abs(off) > 0.010:
                rep.finding(
                    "WARN", "host: %s is %s from the system clock" % (dev, fmt_ms(off)),
                    "phc2sys is not keeping them together. Sensor stamps derived from\n"
                    "the PHC and ROS stamps derived from the system clock will disagree\n"
                    "by this much.")
    rep.data["phc_offsets"] = phc_offsets

    for proc in ("ptp4l", "phc2sys"):
        ok, out = run(["pgrep", "-a", proc])
        if ok and out:
            rep.kv("%s running" % proc, "yes")
            for l in out.splitlines()[:3]:
                rep.line("      " + l)
        else:
            rep.kv("%s running" % proc, "no")
            if devs and proc == "ptp4l":
                rep.finding(
                    "WARN", "host: PTP hardware clocks exist but ptp4l is not running",
                    "Nothing is disciplining them, so the lidars' PTP master (if it is\n"
                    "this host) is free-running.")

    # --- NIC hardware timestamping ----------------------------------------
    if have("ethtool"):
        for iface in sorted(os.listdir("/sys/class/net")):
            if iface == "lo":
                continue
            ok, out = run(["ethtool", "-T", iface])
            if not ok:
                continue
            hw = "hardware-transmit" in out or "HWTSTAMP" in out
            m = re.search(r"PTP Hardware Clock:\s*(\S+)", out)
            rep.kv("NIC %s" % iface,
                   "hw timestamping: %s" % ("yes" if hw else "NO"),
                   "PHC index %s" % (m.group(1) if m else "?"))
            if not hw:
                rep.finding(
                    "INFO", "host: NIC %s has no hardware timestamping" % iface,
                    "Software PTP only -- expect tens of microseconds of jitter, which\n"
                    "is exactly the size of the merged-header non-monotonicity.")
    else:
        rep.kv("ethtool", "absent -- cannot check NIC timestamping")

    # --- recent clock steps -----------------------------------------------
    ok, out = run(["journalctl", "--since", "-2h", "--no-pager", "-q",
                   "-u", "chrony", "-u", "chronyd", "-u", "systemd-timesyncd",
                   "-u", "ptp4l", "-u", "phc2sys"], timeout=10.0)
    steps = []
    if ok and out:
        for l in out.splitlines():
            if re.search(r"step|jump|slew of|clock change|offset\s+-?\d{3,}", l, re.I):
                steps.append(l)
    if steps:
        rep.kv("clock steps in the last 2 h", str(len(steps)))
        for l in steps[-5:]:
            rep.line("      " + l)
        rep.finding(
            "WARN", "host: the clock was stepped %d time(s) in the last 2 hours" % len(steps),
            "A step moves header stamps discontinuously. In FAST-LIO that is a\n"
            "genuine 'lidar loop back'; the queue is flushed and the run restarts\n"
            "from the new timeline. Correlate the timestamps with your log.")
    else:
        rep.kv("clock steps in the last 2 h", "none found (or journal unreadable)")


# --------------------------------------------------------------------------
# LAYER 1+2 -- ROS
# --------------------------------------------------------------------------

def audit_ros(rep, args):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, \
            QoSReliabilityPolicy, QoSHistoryPolicy
        from sensor_msgs.msg import Imu, PointCloud2
        import numpy as np
    except ImportError as exc:
        rep.section("LAYER 1/2 -- ROS")
        rep.kv("rclpy", "unavailable (%s)" % exc)
        rep.finding("INFO", "ros: skipped",
                    "Source your ROS setup and re-run without --host-only.")
        return

    def stamp_sec(st):
        return st.sec + st.nanosec * 1e-9

    class Collector(Node):
        def __init__(self):
            super().__init__("clock_audit")
            self.samples = {}      # topic -> list of (host_recv, header_stamp)
            self.clouds = {}       # topic -> list of dict(point stats)
            self.subs = []

            cloud_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST, depth=50)
            imu_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST, depth=500)

            for t in args.lidar_topic:
                self._add_cloud(t, cloud_qos)
            if args.merged_topic:
                self._add_cloud(args.merged_topic, cloud_qos)
            for t in args.imu_topic:
                self.samples[t] = []
                self.subs.append(self.create_subscription(
                    Imu, t, lambda m, tt=t: self._on_msg(tt, m), imu_qos))

        def _add_cloud(self, topic, qos):
            self.samples[topic] = []
            self.clouds[topic] = []
            self.subs.append(self.create_subscription(
                PointCloud2, topic, lambda m, tt=topic: self._on_cloud(tt, m), qos))

        def _on_msg(self, topic, msg):
            # Host time is taken first: everything after it is our own overhead.
            self.samples[topic].append(
                (time.clock_gettime(time.CLOCK_REALTIME), stamp_sec(msg.header.stamp)))

        def _on_cloud(self, topic, msg):
            now = time.clock_gettime(time.CLOCK_REALTIME)
            hdr = stamp_sec(msg.header.stamp)
            self.samples[topic].append((now, hdr))
            self.clouds[topic].append(self._cloud_stats(msg, hdr))

        @staticmethod
        def _cloud_stats(msg, header_sec):
            """Per-point `timestamp` in exactly the arithmetic mid360_handler uses."""
            n = msg.width * msg.height
            st = {"points": n, "off_min_ms": None, "off_max_ms": None,
                  "frame_id": msg.header.frame_id}
            fld = next((f for f in msg.fields if f.name == "timestamp"), None)
            if fld is None or fld.datatype != 8 or n == 0:
                return st
            try:
                dt = np.dtype({"names": ["ts"], "formats": [np.float64],
                               "offsets": [fld.offset], "itemsize": msg.point_step})
                need = n * msg.point_step
                buf = bytes(msg.data[:need]) if len(msg.data) >= need else bytes(msg.data)
                usable = len(buf) // msg.point_step
                if usable == 0:
                    return st
                ts = np.frombuffer(buf[:usable * msg.point_step], dtype=dt)["ts"]
                off_ms = (ts - header_sec * 1e9) / 1e6
                off_ms = off_ms[np.isfinite(off_ms)]
                if off_ms.size:
                    st["off_min_ms"] = float(off_ms.min())
                    st["off_max_ms"] = float(off_ms.max())
            except Exception:
                pass
            return st

    rclpy.init(args=None)
    node = Collector()

    # ---- LAYER 1 ----------------------------------------------------------
    rep.section("LAYER 1 -- ROS time vs system time")
    sys_t = time.clock_gettime(time.CLOCK_REALTIME)
    ros_t = node.get_clock().now().nanoseconds * 1e-9
    rep.kv("system CLOCK_REALTIME", "%.6f" % sys_t)
    rep.kv("node ROS clock", "%.6f" % ros_t)
    rep.kv("ROS - system", fmt_ms(ros_t - sys_t))
    use_sim = node.get_parameter("use_sim_time").value
    rep.kv("use_sim_time (this node)", str(use_sim))
    if abs(ros_t - sys_t) > 0.001:
        rep.finding(
            "CRITICAL", "ros: ROS time and system time differ by %s" % fmt_ms(ros_t - sys_t),
            "Something is publishing /clock, or use_sim_time is set. Header stamps\n"
            "and rclcpp::Clock::now() then live on different timelines, which is\n"
            "indistinguishable from a sensor clock error.")

    clock_pubs = node.count_publishers("/clock")
    rep.kv("/clock publishers", str(clock_pubs))
    if clock_pubs and not use_sim:
        rep.finding("WARN", "ros: /clock has %d publisher(s) but use_sim_time is false"
                    % clock_pubs,
                    "Any node that DOES set use_sim_time is on a different timeline.")

    # ---- collect ----------------------------------------------------------
    rep.section("LAYER 2 -- topics")
    watched = list(node.samples.keys())
    if not watched:
        rep.kv("topics", "none configured")
        node.destroy_node()
        rclpy.shutdown()
        return

    rep.line("")
    rep.line("  collecting for %.0f s ..." % args.duration)
    t_end = time.time() + args.duration
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    # Publisher counts are read AFTER the collection window: immediately after
    # create_subscription the DDS discovery round has not finished and every
    # topic still reports 0 publishers, which would make this report claim the
    # whole sensor stack is dead.
    rep.line("")
    for t in watched:
        npub = node.count_publishers(t)
        rep.kv("publishers on %s" % t, str(npub))
        if npub == 0:
            rep.finding("WARN", "ros: nothing is publishing %s" % t,
                        "No data to compare -- check the topic name and the namespace.")
        elif npub > 1:
            rep.finding(
                "CRITICAL", "ros: %d publishers on %s" % (npub, t),
                "Two sensors interleaved on one topic produce stamp regressions that\n"
                "look exactly like a clock fault. See perf/README.md H3.")

    # ---- per-topic offsets ------------------------------------------------
    rep.line("")
    rep.line("  %-42s %7s %12s %12s %12s" %
             ("topic", "msgs", "min(now-hdr)", "med", "max"))
    rep.line("  " + "-" * 88)

    offsets = {}
    for t in watched:
        s = node.samples[t]
        if not s:
            rep.line("  %-42s %7d %12s" % (t[-42:], 0, "no messages"))
            continue
        deltas = [host - hdr for host, hdr in s]
        lo, med, hi = min(deltas), statistics.median(deltas), max(deltas)
        offsets[t] = {"n": len(s), "min": lo, "median": med, "max": hi}
        rep.line("  %-42s %7d %12s %12s %12s" %
                 (t[-42:], len(s), fmt_ms(lo), fmt_ms(med), fmt_ms(hi)))

    rep.line("")
    rep.line("  min(now - header) is the tightest bound on the sensor->host clock offset:")
    rep.line("  transport latency only ever ADDS, so the minimum is mostly clock skew.")
    rep.data["offsets"] = offsets

    # a whole-second offset is the TAI/UTC signature
    for t, o in offsets.items():
        if abs(abs(o["min"]) - TAI_UTC_OFFSET) < 1.0:
            rep.finding(
                "CRITICAL", "%s is %.1f s from the host clock" % (t, o["min"]),
                "That is the TAI-UTC offset, not drift: this sensor's stamps are on\n"
                "TAI and nothing converts them (or the conversion is applied twice).\n"
                "Fix it at the driver/PTP layer -- no amount of node-side tolerance\n"
                "will absorb %.0f seconds." % TAI_UTC_OFFSET)
        elif abs(o["min"]) > 1.0:
            rep.finding(
                "CRITICAL", "%s is %s from the host clock" % (t, fmt_s(o["min"])),
                "Far beyond any plausible transport latency -- this is a clock fault,\n"
                "not a queueing effect.")

    # ---- cross-unit comparison -------------------------------------------
    lidar_offsets = {t: offsets[t]["min"] for t in args.lidar_topic if t in offsets}
    if len(lidar_offsets) >= 2:
        rep.line("")
        rep.line("  per-unit offset relative to the MEDIAN unit:")
        med_all = statistics.median(lidar_offsets.values())
        spread = {}
        for t, o in sorted(lidar_offsets.items(), key=lambda kv: kv[1]):
            d = o - med_all
            spread[t] = d
            flag = "  <-- outlier" if abs(d) > args.unit_tolerance else ""
            rep.line("  %-52s %12s%s" % (t[-52:], fmt_ms(d), flag))
        worst = max(spread.items(), key=lambda kv: abs(kv[1]))
        rep.data["unit_spread"] = spread

        if abs(worst[1]) > args.unit_tolerance:
            rep.finding(
                "CRITICAL", "one unit's clock is out: %s by %s" % (worst[0], fmt_ms(worst[1])),
                "The other units agree with each other, so the host clock is fine and\n"
                "this is that unit's PTP sync. The merger's time_window_sec is %.0f ms --\n"
                "a unit outside it is dropped from the batch (and used to be merged in\n"
                "when it was AHEAD, which is what poisons the scan span)."
                % (args.merge_window * 1e3))
        else:
            rep.finding(
                "INFO", "all %d lidar units agree within %s"
                % (len(lidar_offsets), fmt_ms(max(abs(v) for v in spread.values()))),
                "No single unit is drifting. If stamps still look wrong, the fault is\n"
                "common to all of them -- i.e. the host clock or the PTP master, not\n"
                "the sensors and not the nodes.")

        # all units shifted together?
        if abs(med_all) > args.host_tolerance:
            rep.finding(
                "WARN", "ALL lidar units sit %s from the host clock" % fmt_ms(med_all),
                "A common offset is not a sensor fault: either the host clock is wrong\n"
                "relative to the PTP master, or nothing bridges the PHC and the system\n"
                "clock (check phc2sys and the CLOCK_TAI offset in layer 0).")

    # ---- per-point spans --------------------------------------------------
    cloud_topics = [t for t in node.clouds if node.clouds[t]]
    if cloud_topics:
        rep.line("")
        rep.line("  per-point `timestamp` field, as mid360_handler reads it:")
        rep.line("  %-42s %8s %12s %12s" % ("topic", "pts", "off_min_ms", "off_max_ms"))
        rep.line("  " + "-" * 78)
        for t in cloud_topics:
            stats = node.clouds[t]
            pts = statistics.median([s["points"] for s in stats])
            mins = [s["off_min_ms"] for s in stats if s["off_min_ms"] is not None]
            maxs = [s["off_max_ms"] for s in stats if s["off_max_ms"] is not None]
            if not maxs:
                rep.line("  %-42s %8d %12s" % (t[-42:], pts, "no ts field"))
                rep.finding("WARN", "%s has no usable per-point timestamp field" % t,
                            "mid360_handler would set every curvature to 0 and collapse\n"
                            "lidar_end_time onto lidar_beg_time.")
                continue
            rep.line("  %-42s %8d %12.3f %12.3f" %
                     (t[-42:], pts, statistics.median(mins), statistics.median(maxs)))
            span = statistics.median(maxs) - statistics.median(mins)
            nominal = 1000.0 / args.scan_rate
            if span > 2.0 * nominal:
                rep.finding(
                    "CRITICAL", "%s: point offsets span %.1f ms (nominal %.0f ms)"
                    % (t, span, nominal),
                    "This is what FAST-LIO turns into the scan duration. A span this\n"
                    "large means clouds from clocks that disagree were merged into one\n"
                    "message.")
            if max(maxs) > 1000.0:
                rep.finding(
                    "CRITICAL", "%s: worst point offset %.0f ms past the header"
                    % (t, max(maxs)),
                    "Exactly the shape that latches lidar_mean_scantime.")

    # ---- merged vs its inputs --------------------------------------------
    if args.merged_topic and args.merged_topic in offsets and lidar_offsets:
        rep.line("")
        m = offsets[args.merged_topic]["min"]
        oldest_unit = max(lidar_offsets.items(), key=lambda kv: kv[1])
        rep.kv("merged header offset", fmt_ms(m))
        rep.kv("oldest input unit", "%s at %s" % (oldest_unit[0], fmt_ms(oldest_unit[1])))
        rep.line("  The merger stamps each batch with the OLDEST member, so these two")
        rep.line("  should agree. A gap means the batch is not what you think it is.")
        if abs(m - oldest_unit[1]) > args.unit_tolerance:
            rep.finding(
                "WARN", "merged header does not match the oldest input (%s vs %s)"
                % (fmt_ms(m), fmt_ms(oldest_unit[1])),
                "Either a unit is being dropped from the batch by the freshness\n"
                "window, or the merge is not picking min() over the members you think.")

    # ---- IMU vs lidar -----------------------------------------------------
    imu_present = [t for t in args.imu_topic if t in offsets]
    if imu_present and lidar_offsets:
        rep.line("")
        imu_off = statistics.median([offsets[t]["min"] for t in imu_present])
        lid_off = statistics.median(list(lidar_offsets.values()))
        skew = imu_off - lid_off
        rep.kv("IMU offset (median)", fmt_ms(imu_off))
        rep.kv("lidar offset (median)", fmt_ms(lid_off))
        rep.kv("IMU - lidar skew", fmt_ms(skew))
        if abs(skew) > args.imu_tolerance:
            rep.finding(
                "CRITICAL", "IMU and lidar clocks differ by %s" % fmt_ms(skew),
                "sync_packages() pairs each scan with the IMU samples inside its time\n"
                "window; a skew this size pairs scans with the wrong samples and\n"
                "undistortion integrates the wrong motion. Set\n"
                "common.time_offset_lidar_to_imu to %.4f to compensate, or fix the\n"
                "clock source -- compensating hides a drift that will grow."
                % (-skew))

    node.destroy_node()
    rclpy.shutdown()


# --------------------------------------------------------------------------
# config helper
# --------------------------------------------------------------------------

def topics_from_merge_config(path, namespace):
    """Pull lidar topics + merged topic out of pointcloud_merge_default.yaml."""
    try:
        import yaml
    except ImportError:
        return [], None, "pyyaml not installed"
    try:
        with open(path) as fh:
            doc = yaml.safe_load(fh)
    except Exception as exc:
        return [], None, str(exc)

    params = None
    for v in (doc or {}).values():
        if isinstance(v, dict) and "ros__parameters" in v:
            params = v["ros__parameters"]
            break
    if params is None:
        return [], None, "no ros__parameters block"

    ns = namespace.rstrip("/")
    names = params.get("lidars_names", []) or []
    topics = []
    for n in names:
        entry = params.get("lidar.%s" % n) or {}
        topic = entry.get("topic")
        if not topic:
            # flattened form: "lidar.<name>.topic"
            topic = params.get("lidar.%s.topic" % n)
        if topic:
            topics.append(topic if topic.startswith("/") else "%s/%s" % (ns, topic))
    out = params.get("topic_out_name")
    merged = None
    if out:
        merged = out if out.startswith("/") else "%s/%s" % (ns, out)
    return topics, merged, None


# --------------------------------------------------------------------------

def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host-only", action="store_true",
                   help="layer 0 only; no ROS needed")
    p.add_argument("--lidar-topic", action="append", default=[],
                   help="per-unit cloud topic (repeatable)")
    p.add_argument("--merged-topic", default=None)
    p.add_argument("--imu-topic", action="append", default=[])
    p.add_argument("--from-merge-config", default=None,
                   help="read lidar topics and topic_out_name from a "
                        "pointcloud_merge yaml")
    p.add_argument("--namespace", default="",
                   help="prefix for relative topics in that config")
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--scan-rate", type=float, default=10.0,
                   help="nominal lidar rate, for the per-point span check")
    p.add_argument("--merge-window", type=float, default=0.05,
                   help="the merger's time_window_sec, for the report text")
    p.add_argument("--unit-tolerance", type=float, default=0.005,
                   help="per-unit disagreement treated as a fault (s, default 5 ms)")
    p.add_argument("--host-tolerance", type=float, default=0.050,
                   help="common sensor-vs-host offset treated as a fault (s)")
    p.add_argument("--imu-tolerance", type=float, default=0.010,
                   help="IMU-vs-lidar skew treated as a fault (s)")
    p.add_argument("--json", default=None, help="also write the raw numbers here")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    if args.from_merge_config:
        topics, merged, err = topics_from_merge_config(
            args.from_merge_config, args.namespace)
        if err:
            print("could not read %s: %s" % (args.from_merge_config, err),
                  file=sys.stderr)
        args.lidar_topic = list(args.lidar_topic) + topics
        if merged and not args.merged_topic:
            args.merged_topic = merged

    rep = Report()
    rep.section("clock_audit -- %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    rep.kv("host", os.uname().nodename)
    rep.kv("lidar topics", str(len(args.lidar_topic)))
    rep.kv("merged topic", args.merged_topic or "(none)")
    rep.kv("imu topics", ", ".join(args.imu_topic) or "(none)")

    audit_host(rep, args)
    if not args.host_only:
        audit_ros(rep, args)

    print(rep.render())

    if args.json:
        try:
            with open(args.json, "w") as fh:
                json.dump({"data": rep.data,
                           "findings": [{"severity": s, "where": w, "detail": d}
                                        for s, w, d in rep.findings]}, fh, indent=2)
        except Exception as exc:
            print("could not write %s: %s" % (args.json, exc), file=sys.stderr)

    return 1 if any(s in ("CRITICAL", "WARN") for s, _, _ in rep.findings) else 0


if __name__ == "__main__":
    sys.exit(main())
