#!/usr/bin/env python3
"""
analyze.py -- turn one run directory into a diagnosis.

    ./perf/analyze.py perf/runs/20260901_120000_phase2

Reads whatever is present (everything is optional, so partial runs still give
an answer):

    loamqos_agg.csv / _events.csv / _meta.json   stream_monitor, LOAM's own QoS
    greedy_agg.csv  / _events.csv / _meta.json   stream_monitor, deep QoS
    resources.csv   / resources_meta.json        host + process sampler
    perf_probe_scan.csv / perf_probe_events.csv  in-process probe
    run_info.txt                                 provenance

and prints:

  1. what the run was
  2. headline numbers
  3. a verdict for each candidate root cause, with the numbers behind it
  4. the last 30 s before the process died -- usually where the answer is

Pure standard library on purpose: it must run on the Jetson with nothing
installed.
"""

import csv
import json
import os
import statistics
import sys
from collections import Counter

# --------------------------------------------------------------------- utils --

LIKELY = "LIKELY"
POSSIBLE = "POSSIBLE"
UNSUPPORTED = "not supported by the data"
NODATA = "no data"

W = 78


def hr(ch="-"):
    return ch * W


def title(s):
    print()
    print(hr("="))
    print(s)
    print(hr("="))


def fnum(v, default=None):
    """CSV cell -> float or default."""
    if v is None:
        return default
    s = str(v).strip()
    if s == "":
        return default
    try:
        return float(s)
    except ValueError:
        return default


def load_csv(path):
    if not os.path.isfile(path):
        return []
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def load_json(path):
    if not os.path.isfile(path):
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def col(rows, name, filt=None):
    """Numeric column, skipping blanks."""
    out = []
    for r in rows:
        if filt and not filt(r):
            continue
        v = fnum(r.get(name))
        if v is not None:
            out.append(v)
    return out


def trend(vals, frac=0.25):
    """(mean of first frac, mean of last frac) -- cheap drift detector."""
    if len(vals) < 8:
        return None
    n = max(2, int(len(vals) * frac))
    return statistics.fmean(vals[:n]), statistics.fmean(vals[-n:])


def slope_per_min(rows, tcol, vcol):
    """Least-squares slope of vcol vs tcol, per minute."""
    pts = []
    for r in rows:
        t = fnum(r.get(tcol))
        v = fnum(r.get(vcol))
        if t is not None and v is not None:
            pts.append((t, v))
    if len(pts) < 8:
        return None
    n = len(pts)
    mt = statistics.fmean(p[0] for p in pts)
    mv = statistics.fmean(p[1] for p in pts)
    num = sum((p[0] - mt) * (p[1] - mv) for p in pts)
    den = sum((p[0] - mt) ** 2 for p in pts)
    if den == 0:
        return None
    return (num / den) * 60.0


def discover_monitors(run):
    """{tag: {meta, agg, events}} for every stream_monitor output in the dir."""
    out = {}
    try:
        names = sorted(os.listdir(run))
    except OSError:
        return out
    for fn in names:
        if not fn.endswith("_meta.json") or fn == "resources_meta.json":
            continue
        tag = fn[: -len("_meta.json")]
        meta = load_json(os.path.join(run, fn))
        if meta is None:
            continue
        out[tag] = {
            "meta": meta,
            "agg": load_csv(os.path.join(run, f"{tag}_agg.csv")),
            "events": load_csv(os.path.join(run, f"{tag}_events.csv")),
        }
    return out


TS_EVENT_KINDS = ("timestamp_all_zero", "curvature_max_nonpositive",
                  "timestamp_span_huge", "timestamp_negative",
                  "no_timestamp_field", "timestamp_not_f64",
                  "timestamp_parse_error", "timestamp_nonfinite",
                  "empty_cloud", "short_cloud_buffer")


def stream_checklist(monitors):
    """
    The Phase 1 table, computed. One block per monitor per topic, so you do not
    have to cross-reference the console against three CSV columns by hand.
    Returns True if every check passed.
    """
    all_ok = True
    for tag in sorted(monitors):
        m = monitors[tag]
        meta, agg, evs = m["meta"], m["agg"] or [], m["events"] or []
        print(f"\n  monitor '{tag}' (qos={meta.get('qos')}, "
              f"{meta.get('duration_s')}s)")
        pubs = meta.get("publishers") or {}
        for t in meta.get("topics", []):
            topic = t["topic"]
            rows = [r for r in agg if r.get("topic") == topic]
            is_cloud = any(fnum(r.get("points_mean")) is not None for r in rows)
            print(f"    {topic}  ({'cloud' if is_cloud else 'imu'})")

            checks = []

            npub = pubs.get(topic, -1)
            checks.append((
                "exactly one publisher",
                npub == 1,
                f"pubs={npub}" + ("" if npub == 1 else
                                  "  <-- FAST-LIO assumes ONE"),
                npub in (-1, None),
            ))

            nom = t.get("nominal_hz") or 0
            got = t.get("mean_hz") or 0
            rate_ok = bool(nom) and abs(got - nom) <= 0.10 * nom
            checks.append((
                f"rate within 10% of {nom:g} Hz", rate_ok,
                f"{got:.2f} Hz", not nom))

            reg = t.get("stamp_regressions", 0)
            checks.append(("no stamp regressions", reg == 0,
                           f"regress={reg}" + ("" if reg == 0 else
                                               "  <-- two sensors on one topic?"),
                           False))
            dup = t.get("stamp_duplicates", 0)
            checks.append(("no duplicate stamps", dup == 0, f"dup={dup}", False))

            # 'missed' is only meaningful once ordering is clean
            if reg == 0:
                loss = t.get("loss_pct_est", 0.0)
                detail = f"missed~{t.get('missed_est')} ({loss}%)"
                if loss < 0.5:
                    st = True
                elif loss < 2.0:
                    # A software-timed publisher (fake_livox_pub.py, or a driver
                    # that stamps on receipt) has a floor of a few tenths of a
                    # percent from timer slip alone. Flag it, do not fail on it.
                    st = "WARN"
                    detail += "  <-- borderline; on a hardware-stamped sensor "
                    detail += "investigate, on fake_livox_pub.py this is the "
                    detail += "timer floor (try --gap-factor 2.0)"
                else:
                    st = False
                checks.append(("estimated loss under 0.5%", st, detail, False))
            else:
                checks.append(("estimated loss", None,
                               "NOT ASSESSABLE while regress>0 "
                               "(out-of-order stamps inflate it)", True))

            if is_cloud:
                spans = col(rows, "off_span_ms_max")
                zeros = col(rows, "zero_frac_max")
                negs = col(rows, "neg_frac_max")
                expect = 1000.0 / nom if nom else 100.0
                if spans:
                    ok = 0.5 * expect <= max(spans) <= 2.5 * expect
                    checks.append((
                        f"per-point span near {expect:.0f} ms", ok,
                        f"off_span_ms_max={max(spans):.1f}", False))
                else:
                    checks.append(("per-point span", None,
                                   "no per-point timestamps parsed", True))
                if zeros:
                    checks.append(("no zero point timestamps", max(zeros) == 0,
                                   f"zero_frac_max={max(zeros)}", False))
                if negs:
                    checks.append(("few negative offsets", max(negs) <= 0.01,
                                   f"neg_frac_max={max(negs)}", False))

            nts = sum(1 for r in evs
                      if r.get("topic") == topic and r.get("kind") in TS_EVENT_KINDS)
            checks.append(("no timestamp anomaly events", nts == 0,
                           f"{nts} events", False))

            for label, ok, detail, unknown in checks:
                if unknown or ok is None:
                    mark = "  ?  "
                elif ok == "WARN":
                    mark = " WARN"
                elif ok:
                    mark = " PASS"
                else:
                    mark = " FAIL"
                    all_ok = False
                print(f"      [{mark}] {label:<34} {detail}")
    return all_ok


class Finding:
    def __init__(self, key, question):
        self.key = key
        self.question = question
        self.verdict = NODATA
        self.lines = []

    def add(self, s):
        self.lines.append(s)

    def set(self, verdict):
        # never downgrade a LIKELY
        order = {NODATA: 0, UNSUPPORTED: 1, POSSIBLE: 2, LIKELY: 3}
        if order[verdict] > order[self.verdict]:
            self.verdict = verdict

    def render(self):
        mark = {LIKELY: "[!!]", POSSIBLE: "[ ?]", UNSUPPORTED: "[ok]",
                NODATA: "[--]"}[self.verdict]
        print(f"\n{mark} {self.question}")
        print(f"     verdict: {self.verdict}")
        for l in self.lines:
            print(f"     {l}")


# ---------------------------------------------------------------- the report --

def main(argv):
    if len(argv) != 1:
        print(__doc__)
        return 2
    run = argv[0].rstrip("/")
    if not os.path.isdir(run):
        print(f"not a directory: {run}", file=sys.stderr)
        return 1

    info = {}
    p = os.path.join(run, "run_info.txt")
    if os.path.isfile(p):
        for line in open(p):
            if "=" in line:
                k, v = line.rstrip("\n").split("=", 1)
                info[k] = v

    # Any <tag>_meta.json is a stream_monitor run, so a directory produced with
    # --tag phase1 (or any other tag) is analysable, not just the loamqos/greedy
    # pair that run_test.sh happens to create.
    monitors = discover_monitors(run)
    loam_meta = (monitors.get("loamqos") or {}).get("meta")
    greedy_meta = (monitors.get("greedy") or {}).get("meta")
    loam_ev = (monitors.get("loamqos") or {}).get("events") or []
    greedy_ev = (monitors.get("greedy") or {}).get("events") or []
    loam_agg = (monitors.get("loamqos") or {}).get("agg") or []
    # every monitor's events/agg, for the checks that do not need the A/B
    all_ev = [e for m in monitors.values() for e in (m.get("events") or [])]
    res = load_csv(os.path.join(run, "resources.csv"))
    res_meta = load_json(os.path.join(run, "resources_meta.json"))
    probe = load_csv(os.path.join(run, "perf_probe_scan.csv"))
    probe_ev = load_csv(os.path.join(run, "perf_probe_events.csv"))

    title(f"LOAM performance run: {os.path.basename(run)}")
    for k in ("date", "name", "duration_requested_s", "host", "rmw",
              "cyclonedds_uri", "ros_domain_id", "imu_topics", "cloud_topics",
              "loam_cmd", "probe", "git_describe", "loam_exited_after_s",
              "proc_vanished_after_s"):
        if k in info:
            print(f"  {k:<24} {info[k]}")
    have = [f"monitor:{t}" for t in sorted(monitors)]
    have += [n for n, ok in (("resources", bool(res)), ("probe", bool(probe))) if ok]
    print(f"  {'data present':<24} {', '.join(have) if have else 'NOTHING'}")
    if not have:
        print("\n  Nothing to analyse. Was the run dir written by run_test.sh?")
        return 1

    # ------------------------------------------------------------ headline ---
    title("1. headline numbers")

    def show_topics(meta, label):
        if not meta:
            return
        print(f"  {label} (qos={meta.get('qos')}, {meta.get('duration_s')}s):")
        for t in meta.get("topics", []):
            npub = (meta.get("publishers") or {}).get(t["topic"], "?")
            print(f"    {t['topic']:<26} n={t['messages']:<8} "
                  f"{t['mean_hz']:>7.2f}Hz (nominal {t['nominal_hz']:g}) "
                  f"pubs={npub}")
            print(f"    {'':<26} gaps={t['gap_events']} missed~{t['missed_est']} "
                  f"(~{t['loss_pct_est']}%) regress={t['stamp_regressions']} "
                  f"dup={t['stamp_duplicates']}")
    for tag in sorted(monitors):
        label = {"loamqos": "LOAM-QoS subscriber",
                 "greedy": "greedy-QoS subscriber"}.get(tag, f"monitor '{tag}'")
        show_topics(monitors[tag]["meta"], label)

    if res_meta:
        print(f"  process RSS: first={res_meta.get('rss_first_mb')}MB "
              f"peak={res_meta.get('rss_peak_mb')}MB "
              f"growth={res_meta.get('rss_growth_mb')}MB")
        if res_meta.get("proc_vanished_at_s") is not None:
            print(f"  !! tracked process vanished at t="
                  f"{res_meta['proc_vanished_at_s']}s")

    if probe:
        tt = col(probe, "t_total_ms")
        icp = col(probe, "t_icp_ms")
        lb = col(probe, "lidar_buf")
        ib = col(probe, "imu_buf")
        print(f"  probe: {len(probe)} scans processed")
        if tt:
            print(f"    per-scan total: mean={statistics.fmean(tt):.1f}ms "
                  f"p95={sorted(tt)[int(len(tt)*0.95)]:.1f}ms max={max(tt):.1f}ms")
        if icp:
            print(f"    ICP:            mean={statistics.fmean(icp):.1f}ms "
                  f"max={max(icp):.1f}ms")
        if lb:
            print(f"    lidar_buf: mean={statistics.fmean(lb):.2f} max={max(lb):.0f}"
                  f"   imu_buf: mean={statistics.fmean(ib):.1f} max={max(ib):.0f}")
        last = probe[-1]
        for k in ("cum_sync_fail", "cum_meas_imu_empty", "cum_meas_imu_thin",
                  "cum_buffer_clears", "cum_imu_cb_starve",
                  "cum_imu_stamp_regress"):
            v = fnum(last.get(k))
            if v:
                print(f"    {k:<24} {int(v)}")
    if probe_ev:
        kinds = Counter(r.get("kind", "?") for r in probe_ev)
        print("  probe events: " + ", ".join(f"{k}={v}" for k, v in
                                             kinds.most_common()))
    for tag in sorted(monitors):
        evs = monitors[tag]["events"] or []
        if evs:
            kinds = Counter(r.get("kind", "?") for r in evs)
            print(f"  {tag} events: " + ", ".join(
                f"{k}={v}" for k, v in kinds.most_common()))

    # ----------------------------------------------------- stream checklist --
    # This is the Phase 1 table from perf/README.md, computed rather than left
    # for you to cross-reference by hand.
    title("2. sensor stream checklist (Phase 1 criteria)")
    if monitors:
        ok = stream_checklist(monitors)
        print()
        print("  ALL CHECKS PASSED -- the streams themselves are clean."
              if ok else
              "  AT LEAST ONE CHECK FAILED -- fix the stream before blaming LOAM.\n"
              "  A '?' means the data could not answer it, not that it passed.")
    else:
        print("  no stream_monitor output in this directory")

    # ----------------------------------------------------------- hypotheses --
    title("3. candidate root causes")

    findings = []

    def mk(key, q):
        f = Finding(key, q)
        findings.append(f)
        return f

    imu_topics = [t.strip() for t in info.get("imu_topics", "").split() if t.strip()]

    def topic_entry(meta, topic):
        if not meta:
            return None
        for t in meta.get("topics", []):
            if t["topic"] == topic:
                return t
        return None

    # ---- H1: subscriber-side loss (QoS depth + single-threaded executor) ----
    f = mk("H1", "H1  Are messages lost on the SUBSCRIBER side "
                 "(shallow QoS + starved executor)?")
    if loam_meta and greedy_meta:
        for topic in (imu_topics or ["/livox/imu"]):
            a = topic_entry(loam_meta, topic)
            b = topic_entry(greedy_meta, topic)
            if not a or not b:
                continue
            f.add(f"{topic}: loamqos missed~{a['missed_est']} "
                  f"({a['loss_pct_est']}%) vs greedy missed~{b['missed_est']} "
                  f"({b['loss_pct_est']}%)")
            if a["stamp_regressions"] > 0 or b["stamp_regressions"] > 0:
                # Out-of-order stamps make every backward jump produce an
                # oversized forward jump, which the gap estimator counts as a
                # gap. The loam-vs-greedy RATIO survives (both are inflated the
                # same way); the absolute percentage does not.
                f.add(f"  CAVEAT: {topic} had stamp regressions, which inflate "
                      "'missed'. Treat the absolute loss % as unreliable and fix "
                      "ordering first (see H3); the loamqos-vs-greedy ratio is "
                      "still meaningful.")
            if a["missed_est"] > 5 and a["missed_est"] > 3 * max(b["missed_est"], 1):
                f.set(LIKELY)
                f.add("  -> the deep-QoS reader saw the data the LOAM-QoS reader "
                      "missed: loss is created at the subscriber, not upstream.")
            elif a["missed_est"] > 5:
                f.set(POSSIBLE)
            else:
                f.set(UNSUPPORTED)
    elif loam_meta:
        f.add("only the LOAM-QoS monitor ran; add the greedy reference "
              "(drop --no-greedy) to separate subscriber loss from upstream loss")
        for topic in (imu_topics or ["/livox/imu"]):
            a = topic_entry(loam_meta, topic)
            if a and a["missed_est"] > 5:
                f.set(POSSIBLE)
                f.add(f"{topic}: missed~{a['missed_est']} ({a['loss_pct_est']}%)")
    if probe:
        starve = fnum(probe[-1].get("cum_imu_cb_starve"), 0.0) or 0.0
        gaps = col(probe, "imu_cb_gap_max_ms")
        if starve > 0:
            f.set(LIKELY if starve > 10 else POSSIBLE)
            f.add(f"probe: {int(starve)} IMU-callback starvation events "
                  f"(>15ms between imu_cbk calls)")
        if gaps:
            f.add(f"probe: worst gap between imu_cbk invocations = "
                  f"{max(gaps):.1f}ms (nominal 5ms at 200Hz)")
    if res:
        top = col(res, "top_thread_cpu_pct")
        thr = col(res, "threads")
        if top:
            hot = sum(1 for v in top if v > 95.0)
            f.add(f"hottest thread >95% CPU in {hot}/{len(top)} samples "
                  f"(max {max(top):.0f}%)"
                  + (f", process threads={int(statistics.fmean(thr))}" if thr else ""))
            if hot > len(top) * 0.3:
                # Saturation is the MECHANISM, not the evidence. Without observed
                # loss it stays POSSIBLE, so a healthy-but-busy run is not
                # mislabelled.
                f.set(POSSIBLE)
                f.add("  -> one thread is saturated. rclcpp::spin() is single "
                      "threaded, so that same thread must also drain the IMU and "
                      "LiDAR queues. On its own this is a risk factor; the "
                      "verdict is driven by whether messages were actually lost.")

    # ---- H2: transport-level loss ----
    f = mk("H2", "H2  Are datagrams dropped by the TRANSPORT "
                 "(socket buffers / NIC)?")
    if res:
        for c, label in (("udp_rcvbuf_errors", "UDP receive-buffer overflows"),
                         ("udp_in_errors", "UDP input errors"),
                         ("net_rx_dropped", "NIC rx_dropped"),
                         ("net_rx_fifo", "NIC rx_fifo overruns")):
            vals = col(res, c)
            if not vals:
                continue
            delta = vals[-1] - vals[0]
            if delta > 0:
                f.set(LIKELY if "rcvbuf" in c or "fifo" in c else POSSIBLE)
                f.add(f"{label}: +{delta:.0f} during the run "
                      f"({vals[0]:.0f} -> {vals[-1]:.0f})")
            else:
                f.add(f"{label}: no increase")
        if f.verdict == NODATA:
            f.set(UNSUPPORTED)
        if f.verdict == LIKELY:
            f.add("  -> raise the socket buffers: run perf/setup_target.sh and "
                  "use perf/config/cyclonedds_jetson.xml (SocketReceiveBufferSize).")
    if greedy_meta:
        for topic in (imu_topics or ["/livox/imu"]):
            b = topic_entry(greedy_meta, topic)
            if b and b["missed_est"] > 5:
                f.set(POSSIBLE)
                f.add(f"{topic}: even the deep-QoS reader missed "
                      f"~{b['missed_est']} ({b['loss_pct_est']}%), so some loss "
                      f"is upstream of the subscriber queue")

    # ---- H3: two sensors on one topic / unsynced clocks ----
    f = mk("H3", "H3  Are two sensors publishing to ONE topic, or are their "
                 "clocks unsynchronised?")
    for lab in sorted(monitors):
        meta = monitors[lab]["meta"]
        for topic, npub in (meta.get("publishers") or {}).items():
            if isinstance(npub, (int, float)) and npub > 1:
                f.set(LIKELY)
                f.add(f"{lab}: {topic} has {int(npub)} publishers -- "
                      "FAST-LIO assumes exactly one")
        for t in meta.get("topics", []):
            if t["stamp_regressions"] > 0:
                f.set(LIKELY)
                f.add(f"{lab}: {t['topic']} had {t['stamp_regressions']} header-stamp "
                      "regressions (out-of-order timestamps)")
            if t["stamp_duplicates"] > 0:
                f.set(POSSIBLE)
                f.add(f"{lab}: {t['topic']} had {t['stamp_duplicates']} duplicate "
                      "timestamps")
    if probe:
        reg = fnum(probe[-1].get("cum_imu_stamp_regress"), 0.0) or 0.0
        clr = fnum(probe[-1].get("cum_buffer_clears"), 0.0) or 0.0
        if reg > 0:
            f.set(LIKELY)
            f.add(f"probe: {int(reg)} IMU stamp regressions seen inside imu_cbk")
        if clr > 0:
            f.set(LIKELY)
            f.add(f"probe: {int(clr)} buffer CLEARS ('lidar loop back') -- each one "
                  "throws away data the EKF needed")
    if f.verdict == NODATA:
        f.set(UNSUPPORTED)
    if f.verdict == LIKELY:
        f.add("  -> give each sensor its own topic, or merge them in a node that "
              "reorders by timestamp. Check PTP/gPTP sync between the lidars.")

    # ---- H4: per-point timestamps -> lidar_end_time ----
    f = mk("H4", "H4  Are the per-point timestamps corrupting lidar_end_time "
                 "(and thus IMU association)?")
    ts_kinds = ("timestamp_all_zero", "curvature_max_nonpositive",
                "timestamp_span_huge", "timestamp_negative",
                "no_timestamp_field", "timestamp_not_f64",
                "timestamp_parse_error", "timestamp_nonfinite")
    seen = Counter()
    for r in all_ev:
        if r.get("kind") in ts_kinds:
            seen[r["kind"]] += 1
    for k, v in seen.items():
        sev = LIKELY if k in ("timestamp_all_zero", "curvature_max_nonpositive",
                              "no_timestamp_field", "timestamp_span_huge") else POSSIBLE
        f.set(sev)
        f.add(f"cloud monitor: {k} x{v}")
    if probe_ev:
        bad = sum(1 for r in probe_ev if r.get("kind") == "bad_scan_span")
        if bad:
            f.set(LIKELY)
            f.add(f"probe: {bad} scans with an implausible duration "
                  "(lidar_end_time - lidar_beg_time), computed from "
                  "max(point curvature)")
    any_agg = loam_agg or next((m["agg"] for m in monitors.values() if m["agg"]),
                               [])
    if any_agg:
        spans = col(any_agg, "off_span_ms_max")
        zeros = col(any_agg, "zero_frac_max")
        if spans:
            f.add(f"per-point offset span: max={max(spans):.1f}ms "
                  f"(a 10Hz sensor should be ~100ms)")
        if zeros and max(zeros) > 0.5:
            f.set(LIKELY)
            f.add(f"up to {max(zeros)*100:.0f}% of points had timestamp==0")
    if f.verdict == NODATA:
        f.set(UNSUPPORTED)
    if f.verdict in (LIKELY, POSSIBLE):
        f.add("  -> mid360_handler turns point.timestamp into `curvature`; "
              "sync_packages then uses max(curvature) as the scan duration. "
              "Bad values here silently break the IMU/LiDAR association.")

    # ---- H5: cannot keep up -> backlog ----
    f = mk("H5", "H5  Is LOAM failing to keep up with the sensor rate "
                 "(growing backlog)?")
    cloud_rate = fnum(info.get("cloud_rate"), 10.0) or 10.0
    budget_ms = 1000.0 / cloud_rate
    if probe:
        tt = col(probe, "t_total_ms")
        if tt:
            over = sum(1 for v in tt if v > budget_ms)
            f.add(f"per-scan budget at {cloud_rate:g}Hz is {budget_ms:.0f}ms; "
                  f"{over}/{len(tt)} scans exceeded it "
                  f"(mean {statistics.fmean(tt):.1f}ms, max {max(tt):.1f}ms)")
            if over > len(tt) * 0.2:
                f.set(LIKELY)
            elif over > 0:
                f.set(POSSIBLE)
            else:
                f.set(UNSUPPORTED)
        lb = col(probe, "lidar_buf")
        if lb:
            tr = trend(lb)
            f.add(f"lidar_buffer depth: max={max(lb):.0f}" +
                  (f", first-quarter mean={tr[0]:.2f} -> last-quarter mean={tr[1]:.2f}"
                   if tr else ""))
            if max(lb) > 5 or (tr and tr[1] > tr[0] + 2):
                f.set(LIKELY)
                f.add("  -> the deque between the callback and the timer is "
                      "filling up. It is unbounded, so this grows memory and "
                      "latency until something gives.")
        age = col(probe, "pipeline_age_ms")
        if age:
            tr = trend(age)
            f.add(f"pipeline age (now - lidar_end_time): mean="
                  f"{statistics.fmean(age):.0f}ms max={max(age):.0f}ms" +
                  (f", drift {tr[0]:.0f} -> {tr[1]:.0f}ms" if tr else ""))
            if tr and tr[1] > tr[0] * 2 and tr[1] > 200:
                f.set(LIKELY)
                f.add("  -> latency is growing monotonically: a true backlog, not "
                      "jitter.")
    if f.verdict == NODATA:
        f.add("no probe data; rebuild with the probe and set FASTLIO_PERF_LOG")

    # ---- H6: memory ----
    f = mk("H6", "H6  Is the process running out of memory?")
    if res:
        rss = col(res, "rss_mb", filt=lambda r: fnum(r.get("proc_alive")) == 1)
        if rss:
            sl = slope_per_min(
                [r for r in res if fnum(r.get("proc_alive")) == 1],
                "t_rel_s", "rss_mb")
            f.add(f"RSS {rss[0]:.0f} -> {rss[-1]:.0f} MB (peak {max(rss):.0f})" +
                  (f", trend {sl:+.1f} MB/min" if sl is not None else ""))
            if sl is not None and sl > 20:
                f.set(LIKELY)
                f.add("  -> sustained growth. Suspects: the unbounded lidar/imu "
                      "deques (see H5), ikd-Tree map growth, and "
                      "pcd_save_en/interval=-1 which accumulates every scan.")
            elif sl is not None and sl > 5:
                f.set(POSSIBLE)
            else:
                f.set(UNSUPPORTED)
        avail = col(res, "mem_avail_mb")
        if avail:
            f.add(f"host MemAvailable {avail[0]:.0f} -> {avail[-1]:.0f} MB "
                  f"(min {min(avail):.0f})")
            if min(avail) < 400:
                f.set(LIKELY)
                f.add("  -> the host got close to OOM.")
        sw = col(res, "swap_used_mb")
        if sw and max(sw) - min(sw) > 50:
            f.set(POSSIBLE)
            f.add(f"swap use moved by {max(sw)-min(sw):.0f} MB (swapping adds "
                  "large latency spikes)")
    if res_meta:
        oom = [l for l in (res_meta.get("dmesg_oom_segfault_tail") or [])
               if "oom" in l.lower() or "out of memory" in l.lower()
               or "killed process" in l.lower()]
        if oom:
            f.set(LIKELY)
            f.add("dmesg shows OOM activity:")
            for l in oom[-3:]:
                f.add("  " + l[:110])
        seg = [l for l in (res_meta.get("dmesg_oom_segfault_tail") or [])
               if "segfault" in l.lower() or "general protection" in l.lower()]
        if seg:
            f.add("dmesg shows a segfault (so NOT an OOM kill):")
            for l in seg[-3:]:
                f.add("  " + l[:110])
    tree = col(probe, "tree_size") if probe else []
    if tree:
        f.add(f"ikd-Tree size {tree[0]:.0f} -> {tree[-1]:.0f} points")

    # ---- H7: thermal / clock throttling ----
    f = mk("H7", "H7  Is the Jetson throttling (so it slows down over time)?")
    if res:
        fr = col(res, "cpu_freq_mhz_mean")
        tp = col(res, "temp_max_c")
        hot_enough = bool(tp) and max(tp) > 80.0
        if fr:
            tr = trend(fr)
            f.add(f"mean CPU freq {min(fr):.0f}..{max(fr):.0f} MHz" +
                  (f", {tr[0]:.0f} -> {tr[1]:.0f} MHz over the run" if tr else ""))
            if len(fr) < 30:
                f.add(f"(only {len(fr)} samples: too short to judge a clock "
                      "trend -- run for a few minutes at least)")
                f.set(NODATA if not tp else UNSUPPORTED)
            elif tr and tr[1] < tr[0] * 0.85:
                # A clock drop alone can just be a change in load. Throttling is
                # a clock drop that coincides with heat.
                f.set(LIKELY if hot_enough else POSSIBLE)
                f.add("  -> clocks dropped >15%"
                      + (" while hot: this looks like thermal throttling. "
                         "Check nvpmodel mode and cooling."
                         if hot_enough else
                         " but temperatures stayed moderate, so this may just be "
                         "a change in load rather than throttling."))
            else:
                f.set(UNSUPPORTED)
        if tp:
            f.add(f"max temperature {max(tp):.1f} C")
            if max(tp) > 85 and f.verdict != LIKELY:
                f.set(POSSIBLE)
                f.add("  -> hot enough for the SoC to start throttling.")
    if probe:
        tt_rows = [r for r in probe if fnum(r.get("t_total_ms")) is not None]
        sl = slope_per_min(tt_rows, "t_rel_s", "t_total_ms")
        if sl is not None:
            f.add(f"per-scan processing time trend: {sl:+.2f} ms/min")
            if sl > 2.0:
                f.set(POSSIBLE)
                f.add("  -> processing is getting slower. Could be throttling (this "
                      "hypothesis) or a growing map (H6).")

    # ---- H8: filter divergence ----
    f = mk("H8", "H8  Did the state estimate diverge (bad IMU association)?")
    if probe:
        nf = sum(1 for r in probe if fnum(r.get("nonfinite")) == 1)
        vel = col(probe, "vel_norm")
        empty = fnum(probe[-1].get("cum_meas_imu_empty"), 0.0) or 0.0
        thin = fnum(probe[-1].get("cum_meas_imu_thin"), 0.0) or 0.0
        if nf:
            f.set(LIKELY)
            f.add(f"{nf} scans with a non-finite state (NaN/Inf) -- the filter blew up")
        if vel:
            f.add(f"velocity norm: max={max(vel):.2f} m/s")
            if max(vel) > 30:
                f.set(LIKELY)
                f.add("  -> implausible velocity: the filter diverged before the crash.")
        if empty:
            f.set(LIKELY)
            f.add(f"{int(empty)} scans were synced with an EMPTY IMU batch.")
            f.add("  -> ImuProcess::Process() returns immediately when meas.imu is "
                  "empty, so feats_undistort keeps the PREVIOUS scan's points and "
                  "that stale cloud is matched against the new state.")
        if thin:
            f.set(POSSIBLE)
            f.add(f"{int(thin)} scans had fewer than 3 IMU samples")
        if f.verdict == NODATA:
            f.set(UNSUPPORTED)
    if probe_ev:
        for k in ("state_NONFINITE", "state_velocity_high", "meas_imu_EMPTY"):
            n = sum(1 for r in probe_ev if r.get("kind") == k)
            if n:
                f.add(f"probe events: {k} x{n}")

    # ---- H9: an accumulating cloud being republished (publish_map) ----
    f = mk("H9", "H9  Is an ever-growing cloud being republished "
                 "(publish_map / pcl_wait_pub)?")
    # Direct evidence: a monitored cloud topic whose point count keeps climbing.
    for lab in sorted(monitors):
        agg = monitors[lab]["agg"] or []
        if not agg:
            continue
        topics = {r["topic"] for r in agg if r.get("topic")}
        for topic in sorted(topics):
            pts = col(agg, "points_mean", filt=lambda r, t=topic: r.get("topic") == t)
            if len(pts) < 6:
                continue
            tr = trend(pts)
            if tr and tr[0] > 0 and tr[1] > tr[0] * 2.0 and tr[1] - tr[0] > 5000:
                f.set(LIKELY)
                f.add(f"{lab}: {topic} point count grew "
                      f"{tr[0]:.0f} -> {tr[1]:.0f} per message during the run")
                f.add("  -> that topic is republishing an accumulating buffer.")
    # Configuration evidence: the run dir keeps a copy of the config files.
    for fn in sorted(os.listdir(run)):
        if not fn.endswith(".yaml"):
            continue
        try:
            txt = open(os.path.join(run, fn)).read()
        except OSError:
            continue
        for pat, why in (
            ("map_en: true",
             "publish.map_en=true -> map_publish_callback() appends to "
             "pcl_wait_pub every second and republishes ALL of it; the buffer "
             "is never cleared"),
            ("pcd_save_en: true",
             "pcd_save.pcd_save_en=true -> save_to_pcd() writes that same "
             "ever-growing pcl_wait_pub"),
        ):
            if pat in txt:
                f.set(POSSIBLE if "pcd" in pat else LIKELY)
                f.add(f"{fn}: {why}")
    if f.verdict == NODATA:
        f.set(UNSUPPORTED)
        f.add("no monitored topic showed cloud growth")
    if f.verdict in (LIKELY, POSSIBLE):
        f.add("  -> test with perf/config/mid360_perf_baseline.yaml "
              "(map_en:false, pcd_save_en:false). Add "
              "'--cloud-topic /Laser_map --cloud-rate 1' to run_test.sh to watch "
              "the growth directly.")

    for fnd in findings:
        fnd.render()

    # ------------------------------------------------------ final 30 seconds --
    title("4. the last 30 s before the end")
    end_t = None
    if probe:
        end_t = max(col(probe, "t_rel_s") or [0])
    if res:
        end_t = max(end_t or 0, max(col(res, "t_rel_s") or [0]))
    if end_t is None:
        print("  no timeline data")
    else:
        lo = end_t - 30.0
        print(f"  (t_rel {lo:.0f}s .. {end_t:.0f}s)")
        rows = [r for r in probe if (fnum(r.get("t_rel_s")) or 0) >= lo]
        if rows:
            print("\n  probe scans (every 5th row):")
            print(f"    {'t_rel':>7} {'lidBuf':>6} {'imuBuf':>6} {'measImu':>7} "
                  f"{'total_ms':>8} {'icp_ms':>7} {'age_ms':>7} {'rss_MB':>7} "
                  f"{'vel':>6}")
            for r in rows[::5][-14:]:
                print(f"    {fnum(r['t_rel_s'], 0):>7.1f} "
                      f"{fnum(r['lidar_buf'], 0):>6.0f} "
                      f"{fnum(r['imu_buf'], 0):>6.0f} "
                      f"{fnum(r['meas_imu'], 0):>7.0f} "
                      f"{fnum(r['t_total_ms'], 0):>8.1f} "
                      f"{fnum(r['t_icp_ms'], 0):>7.1f} "
                      f"{fnum(r['pipeline_age_ms'], 0):>7.0f} "
                      f"{fnum(r['rss_mb'], 0):>7.1f} "
                      f"{fnum(r['vel_norm'], 0):>6.2f}")
        evs = [r for r in probe_ev if (fnum(r.get("t_rel_s")) or 0) >= lo]
        if evs:
            print(f"\n  probe events in the final window ({len(evs)}):")
            for r in evs[-25:]:
                print(f"    {fnum(r.get('t_rel_s'), 0):>7.1f}  "
                      f"{r.get('kind','?'):<26} {r.get('detail','')[:60]}")
        rrows = [r for r in res if (fnum(r.get("t_rel_s")) or 0) >= lo]
        if rrows:
            print("\n  resources (last rows):")
            print(f"    {'t_rel':>7} {'alive':>5} {'rss_MB':>7} {'cpu%':>6} "
                  f"{'topThr%':>7} {'availMB':>8} {'freqMHz':>8} {'rcvbufErr':>9}")
            for r in rrows[::max(1, len(rrows)//10)][-12:]:
                print(f"    {fnum(r['t_rel_s'], 0):>7.1f} "
                      f"{fnum(r.get('proc_alive'), 0):>5.0f} "
                      f"{fnum(r.get('rss_mb'), 0):>7.1f} "
                      f"{fnum(r.get('cpu_pct'), 0):>6.1f} "
                      f"{fnum(r.get('top_thread_cpu_pct'), 0):>7.1f} "
                      f"{fnum(r.get('mem_avail_mb'), 0):>8.0f} "
                      f"{fnum(r.get('cpu_freq_mhz_mean'), 0):>8.0f} "
                      f"{fnum(r.get('udp_rcvbuf_errors'), 0):>9.0f}")

    # ------------------------------------------------------------- next step --
    title("5. what to do next")
    likely = [f for f in findings if f.verdict == LIKELY]
    possible = [f for f in findings if f.verdict == POSSIBLE]
    if likely:
        print("  LIKELY causes, in the order worth acting on:")
        for f in likely:
            print(f"    {f.key}: {f.question.split('  ', 1)[-1]}")
        print("\n  perf/README.md maps each hypothesis to its fix "
              "(see 'Hypotheses and fixes').")
    elif possible:
        print("  Nothing conclusive. Weak signals:")
        for f in possible:
            print(f"    {f.key}: {f.question.split('  ', 1)[-1]}")
        print("\n  Run longer, or until it actually crashes -- the final-window "
              "section is where the answer usually is.")
    else:
        print("  This run looks clean. Either it did not run long enough to")
        print("  reproduce the failure, or the configuration under test is fine.")
        print("  Re-run with the real sensors and let it reach the crash.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
