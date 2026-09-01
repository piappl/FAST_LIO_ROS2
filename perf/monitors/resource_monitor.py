#!/usr/bin/env python3
"""
resource_monitor.py -- host + process resource sampler for the LOAM crash hunt.

No ROS dependency: reads /proc and /sys directly, so it can run before, during
and after the ROS stack (and it keeps sampling while the node is dying).

Per sample it records:

  process (fastlio_mapping, matched by --pid or --name):
    rss_mb, vsz_mb          -> unbounded growth = the deque/map leak path
    threads                 -> confirms the executor thread count
    cpu_pct                 -> whole-process CPU
    top_thread_cpu_pct      -> hottest single thread; ~100% means ONE thread is
                               saturated, i.e. rclcpp::spin() (SingleThreaded)
                               cannot service the IMU/LiDAR callbacks in time
    n_threads_over_90       -> how many threads are pegged
    state, voluntary_ctxt_switches, nonvoluntary_ctxt_switches

  host:
    mem_avail_mb, mem_total_mb, swap_used_mb
    load1
    cpu_pct_total, cpu_freq_mhz_mean/min/max
    temp_max_c + per-zone temps      -> Jetson thermal throttling over time

  network / DDS transport (the "message sending and receiving" evidence):
    udp_in_errors, udp_rcvbuf_errors, udp_sndbuf_errors  (/proc/net/snmp)
        rcvbuf_errors climbing == datagrams dropped because a socket receive
        buffer was full == exactly the DDS loss that starves the IMU buffer
    <iface>_rx_dropped, _rx_errors, _rx_fifo             (/proc/net/dev)
    Counters are reported BOTH cumulative and as per-second deltas.

Outputs:
  <out-dir>/resources.csv
  <out-dir>/resources_meta.json   (static snapshot + final deltas)

Usage:
  ./resource_monitor.py --name fastlio_mapping --out-dir runs/foo --interval 1.0
"""

import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import time

CLK_TCK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
PAGE_KB = 4


def read_file(path, default=""):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


# Our own tooling mentions the tracked name on its command line; never match it.
_SELF_MARKERS = ("resource_monitor.py", "stream_monitor.py", "run_test.sh",
                 "analyze.py", "fake_livox_pub.py")


def find_pid_by_name(name):
    """
    Prefer an exact /proc/<pid>/comm match (comm is the real process name, but
    the kernel truncates it to 15 chars). Only if nothing matches exactly do we
    fall back to a cmdline substring search, and even then we skip our own
    monitor processes -- run_test.sh passes the tracked name to them, so a naive
    substring match finds the monitor instead of the node.
    """
    me = str(os.getpid())
    short = name[:15]
    exact, loose = [], []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or entry == me:
            continue
        cmd = read_file(f"/proc/{entry}/cmdline").replace("\0", " ")
        if any(m in cmd for m in _SELF_MARKERS):
            continue
        comm = read_file(f"/proc/{entry}/comm").strip()
        if comm == short:
            exact.append(int(entry))
        elif name in cmd:
            loose.append(int(entry))
    hits = exact or loose
    return max(hits) if hits else None


class CpuSampler:
    """Delta-based CPU accounting."""

    def __init__(self):
        self.prev = {}

    def delta(self, key, value, now):
        prev = self.prev.get(key)
        self.prev[key] = (value, now)
        if prev is None:
            return None
        pv, pt = prev
        dt = now - pt
        if dt <= 0:
            return None
        return (value - pv) / dt


class ProcSampler:
    def __init__(self, pid):
        self.pid = pid
        self.cpu = CpuSampler()

    def alive(self):
        return os.path.isdir(f"/proc/{self.pid}")

    def sample(self, now):
        out = {}
        st = read_file(f"/proc/{self.pid}/stat")
        if not st:
            return None
        # comm can contain spaces/parens -> split on the last ')'
        try:
            rest = st[st.rindex(")") + 2:].split()
            out["state"] = rest[0]
            utime = int(rest[11])
            stime = int(rest[12])
            out["threads"] = int(rest[17])
        except (ValueError, IndexError):
            return None

        ticks = self.cpu.delta("proc", utime + stime, now)
        out["cpu_pct"] = round(100.0 * ticks / CLK_TCK, 2) if ticks is not None else ""

        status = read_file(f"/proc/{self.pid}/status")
        for key, field in (("VmRSS", "rss_mb"), ("VmSize", "vsz_mb")):
            m = re.search(rf"^{key}:\s+(\d+) kB", status, re.M)
            out[field] = round(int(m.group(1)) / 1024.0, 2) if m else ""
        for key, field in (
            ("voluntary_ctxt_switches", "vol_ctxt_sw"),
            ("nonvoluntary_ctxt_switches", "nonvol_ctxt_sw"),
        ):
            m = re.search(rf"^{key}:\s+(\d+)", status, re.M)
            if m:
                d = self.cpu.delta(field, int(m.group(1)), now)
                out[field + "_per_s"] = round(d, 1) if d is not None else ""
            else:
                out[field + "_per_s"] = ""

        # ---- per-thread CPU: the executor-saturation evidence ----
        top = 0.0
        over90 = 0
        tdir = f"/proc/{self.pid}/task"
        try:
            tids = os.listdir(tdir)
        except OSError:
            tids = []
        for tid in tids:
            tst = read_file(f"{tdir}/{tid}/stat")
            if not tst:
                continue
            try:
                tr = tst[tst.rindex(")") + 2:].split()
                tt = int(tr[11]) + int(tr[12])
            except (ValueError, IndexError):
                continue
            d = self.cpu.delta(f"t{tid}", tt, now)
            if d is None:
                continue
            pct = 100.0 * d / CLK_TCK
            top = max(top, pct)
            if pct > 90.0:
                over90 += 1
        out["top_thread_cpu_pct"] = round(top, 2)
        out["n_threads_over_90"] = over90
        return out


class HostSampler:
    def __init__(self):
        self.cpu = CpuSampler()
        self.zones = self._find_thermal_zones()
        self.ifaces = []

    @staticmethod
    def _find_thermal_zones():
        zones = []
        base = "/sys/devices/virtual/thermal"
        if not os.path.isdir(base):
            base = "/sys/class/thermal"
        if os.path.isdir(base):
            for d in sorted(os.listdir(base)):
                if d.startswith("thermal_zone"):
                    t = read_file(os.path.join(base, d, "type")).strip()
                    zones.append((os.path.join(base, d, "temp"), t or d))
        return zones

    def sample(self, now):
        out = {}

        # ---- memory ----
        mi = read_file("/proc/meminfo")
        def mem(key):
            m = re.search(rf"^{key}:\s+(\d+) kB", mi, re.M)
            return int(m.group(1)) / 1024.0 if m else float("nan")
        out["mem_total_mb"] = round(mem("MemTotal"), 1)
        out["mem_avail_mb"] = round(mem("MemAvailable"), 1)
        st, sf = mem("SwapTotal"), mem("SwapFree")
        out["swap_used_mb"] = round(st - sf, 1) if st == st and sf == sf else ""

        out["load1"] = read_file("/proc/loadavg").split()[0] if read_file("/proc/loadavg") else ""

        # ---- total CPU ----
        first = read_file("/proc/stat").split("\n")[0].split()
        if len(first) > 8 and first[0] == "cpu":
            vals = [int(x) for x in first[1:9]]
            busy = sum(vals) - vals[3] - vals[4]  # minus idle, iowait
            total = sum(vals)
            db = self.cpu.delta("busy", busy, now)
            dt = self.cpu.delta("total", total, now)
            out["cpu_pct_total"] = (
                round(100.0 * db / dt, 2) if (db is not None and dt) else ""
            )
        else:
            out["cpu_pct_total"] = ""

        # ---- cpu freq (throttling) ----
        freqs = []
        for d in sorted(os.listdir("/sys/devices/system/cpu")) if os.path.isdir(
            "/sys/devices/system/cpu") else []:
            if re.fullmatch(r"cpu\d+", d):
                v = read_file(
                    f"/sys/devices/system/cpu/{d}/cpufreq/scaling_cur_freq").strip()
                if v.isdigit():
                    freqs.append(int(v) / 1000.0)
        out["cpu_freq_mhz_mean"] = round(sum(freqs) / len(freqs), 1) if freqs else ""
        out["cpu_freq_mhz_min"] = round(min(freqs), 1) if freqs else ""
        out["cpu_freq_mhz_max"] = round(max(freqs), 1) if freqs else ""

        # ---- thermals ----
        temps = []
        for path, _name in self.zones:
            v = read_file(path).strip()
            if v.lstrip("-").isdigit():
                temps.append(int(v) / 1000.0)
        out["temp_max_c"] = round(max(temps), 2) if temps else ""

        # ---- UDP errors: the DDS-drop smoking gun ----
        snmp = read_file("/proc/net/snmp")
        udp_hdr = udp_val = None
        for i, line in enumerate(snmp.split("\n")):
            if line.startswith("Udp:"):
                if udp_hdr is None:
                    udp_hdr = line.split()[1:]
                else:
                    udp_val = line.split()[1:]
                    break
        udp = {}
        if udp_hdr and udp_val and len(udp_hdr) == len(udp_val):
            udp = dict(zip(udp_hdr, udp_val))
        for key, field in (
            ("InErrors", "udp_in_errors"),
            ("RcvbufErrors", "udp_rcvbuf_errors"),
            ("SndbufErrors", "udp_sndbuf_errors"),
        ):
            v = int(udp.get(key, 0) or 0)
            out[field] = v
            d = self.cpu.delta(field, v, now)
            out[field + "_per_s"] = round(d, 3) if d is not None else ""

        # ---- interface drops ----
        dev = read_file("/proc/net/dev")
        rx_dropped = rx_errors = rx_fifo = 0
        for line in dev.split("\n")[2:]:
            if ":" not in line:
                continue
            name, data = line.split(":", 1)
            name = name.strip()
            if name == "lo":
                continue
            f = data.split()
            if len(f) < 5:
                continue
            rx_errors += int(f[2])
            rx_dropped += int(f[3])
            rx_fifo += int(f[4])
        for field, v in (("rx_errors", rx_errors), ("rx_dropped", rx_dropped),
                         ("rx_fifo", rx_fifo)):
            out["net_" + field] = v
            d = self.cpu.delta("net_" + field, v, now)
            out["net_" + field + "_per_s"] = round(d, 3) if d is not None else ""

        return out


def static_snapshot():
    """One-shot facts worth freezing into the run record."""
    snap = {}
    snap["uname"] = read_file("/proc/version").strip()
    snap["cmdline"] = read_file("/proc/cmdline").strip()
    snap["nproc"] = os.cpu_count()
    snap["model"] = read_file("/proc/device-tree/model", "").replace("\0", "").strip()
    for key, path in (
        ("l4t_release", "/etc/nv_tegra_release"),
        ("jetson_model_sys", "/sys/firmware/devicetree/base/model"),
    ):
        v = read_file(path, "").replace("\0", "").strip()
        if v:
            snap[key] = v
    snap["governor"] = read_file(
        "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "").strip()
    for key, cmd in (
        ("nvpmodel", ["nvpmodel", "-q"]),
        ("jetson_clocks", ["jetson_clocks", "--show"]),
    ):
        try:
            snap[key] = subprocess.run(
                cmd, capture_output=True, text=True, timeout=10
            ).stdout.strip()
        except Exception:
            pass
    for key in ("net.core.rmem_max", "net.core.rmem_default",
                "net.core.wmem_max", "net.core.netdev_max_backlog",
                "net.ipv4.ipfrag_high_thresh", "net.ipv4.ipfrag_time"):
        snap[key] = read_file("/proc/sys/" + key.replace(".", "/"), "").strip()
    for var in ("RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "ROS_DOMAIN_ID",
                "ROS_DISTRO", "ROS_LOCALHOST_ONLY",
                "ROS_AUTOMATIC_DISCOVERY_RANGE"):
        snap["env_" + var] = os.environ.get(var, "")
    snap["core_pattern"] = read_file("/proc/sys/kernel/core_pattern", "").strip()
    return snap


def tail_dmesg_for_oom():
    """After a crash this is the difference between 'OOM killed' and 'segfault'."""
    for cmd in (["dmesg", "-T", "--level=err,warn,crit,alert,emerg"], ["dmesg"]):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if r.returncode == 0:
                lines = [
                    l for l in r.stdout.split("\n")
                    if re.search(r"oom|Out of memory|killed process|segfault|"
                                 r"general protection|traps:", l, re.I)
                ]
                return lines[-40:]
        except Exception:
            continue
    return ["<dmesg unavailable (needs root or CAP_SYSLOG)>"]


_STOP = False


def _handler(signum, _frame):
    global _STOP
    _STOP = True


def main():
    p = argparse.ArgumentParser(
        description="Host + process resource sampler",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--pid", type=int, default=None)
    g.add_argument("--name", default="fastlio_mapping",
                   help="process name/cmdline substring to track")
    p.add_argument("--out-dir", default="runs/manual")
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=0.0, help="0 = until signal")
    p.add_argument("--wait-for-process", type=float, default=60.0,
                   help="seconds to wait for the tracked process to appear")
    p.add_argument("--exit-when-gone", action="store_true",
                   help="stop sampling once the tracked process disappears")
    a = p.parse_args()

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)

    os.makedirs(a.out_dir, exist_ok=True)
    csv_path = os.path.join(a.out_dir, "resources.csv")
    meta_path = os.path.join(a.out_dir, "resources_meta.json")

    host = HostSampler()
    t0 = time.time()

    # ---- locate the process (it may not exist yet) ----
    pid = a.pid
    if pid is None:
        deadline = t0 + a.wait_for_process
        while pid is None and time.time() < deadline and not _STOP:
            pid = find_pid_by_name(a.name)
            if pid is None:
                time.sleep(0.25)
    proc = ProcSampler(pid) if pid else None
    if proc:
        print(f"resource_monitor: tracking pid {pid}", file=sys.stderr)
    else:
        print(f"resource_monitor: no process matching '{a.name}'; "
              "recording host metrics only", file=sys.stderr)

    fields = [
        "t_rel_s", "wall_unix", "pid", "proc_alive",
        "rss_mb", "vsz_mb", "threads", "cpu_pct",
        "top_thread_cpu_pct", "n_threads_over_90", "state",
        "vol_ctxt_sw_per_s", "nonvol_ctxt_sw_per_s",
        "mem_total_mb", "mem_avail_mb", "swap_used_mb", "load1",
        "cpu_pct_total", "cpu_freq_mhz_mean", "cpu_freq_mhz_min",
        "cpu_freq_mhz_max", "temp_max_c",
        "udp_in_errors", "udp_in_errors_per_s",
        "udp_rcvbuf_errors", "udp_rcvbuf_errors_per_s",
        "udp_sndbuf_errors", "udp_sndbuf_errors_per_s",
        "net_rx_errors", "net_rx_errors_per_s",
        "net_rx_dropped", "net_rx_dropped_per_s",
        "net_rx_fifo", "net_rx_fifo_per_s",
    ]
    f = open(csv_path, "w", newline="")
    w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    w.writeheader()

    peak_rss = 0.0
    first_rss = None
    n = 0
    proc_vanished_at = None

    while not _STOP:
        now = time.time()
        row = {"t_rel_s": round(now - t0, 3), "wall_unix": round(now, 3)}
        row.update(host.sample(now))

        if proc is None and a.pid is None:
            # keep looking; the node may be restarted by the operator
            newpid = find_pid_by_name(a.name)
            if newpid:
                proc = ProcSampler(newpid)
                print(f"resource_monitor: picked up pid {newpid}", file=sys.stderr)

        if proc is not None:
            row["pid"] = proc.pid
            if proc.alive():
                ps = proc.sample(now)
                if ps:
                    row.update(ps)
                    row["proc_alive"] = 1
                    if ps.get("rss_mb") not in ("", None):
                        peak_rss = max(peak_rss, ps["rss_mb"])
                        if first_rss is None:
                            first_rss = ps["rss_mb"]
                else:
                    row["proc_alive"] = 0
            else:
                row["proc_alive"] = 0
                if proc_vanished_at is None:
                    proc_vanished_at = now - t0
                    print(f"resource_monitor: tracked pid {proc.pid} is GONE at "
                          f"t={proc_vanished_at:.1f}s", file=sys.stderr)
                    if a.exit_when_gone:
                        w.writerow(row)
                        n += 1
                        break
                proc = None if a.pid is None else proc
        else:
            row["proc_alive"] = 0

        w.writerow(row)
        n += 1
        if n % 10 == 0:
            f.flush()

        if a.duration > 0 and (now - t0) >= a.duration:
            break
        time.sleep(a.interval)

    f.flush()
    f.close()

    meta = {
        "samples": n,
        "duration_s": round(time.time() - t0, 3),
        "interval_s": a.interval,
        "tracked_name": a.name,
        "tracked_pid": proc.pid if proc else pid,
        "proc_vanished_at_s": proc_vanished_at,
        "rss_first_mb": first_rss,
        "rss_peak_mb": peak_rss,
        "rss_growth_mb": (round(peak_rss - first_rss, 2)
                          if first_rss is not None else None),
        "static": static_snapshot(),
        "dmesg_oom_segfault_tail": tail_dmesg_for_oom(),
    }
    with open(meta_path, "w") as mf:
        json.dump(meta, mf, indent=2)

    print(f"resource_monitor: {n} samples -> {csv_path}", file=sys.stderr)
    print(f"resource_monitor: meta -> {meta_path}", file=sys.stderr)
    if first_rss is not None:
        print(f"resource_monitor: RSS {first_rss:.1f} -> peak {peak_rss:.1f} MB "
              f"(growth {peak_rss - first_rss:+.1f} MB)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
