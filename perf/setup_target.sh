#!/usr/bin/env bash
#
# setup_target.sh -- one-time preparation of the Jetson Orin NX for a LOAM
# performance/crash run, plus a preflight report.
#
# Everything is idempotent. Nothing here is permanent unless you pass --persist
# (which writes /etc/sysctl.d/99-loam-perf.conf).
#
#   ./perf/setup_target.sh                 # report + apply runtime tuning
#   ./perf/setup_target.sh --report-only   # change nothing, just tell me
#   ./perf/setup_target.sh --persist       # also survive reboot
#   ./perf/setup_target.sh --validate-dds  # check the Cyclone config parses
#
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPORT_ONLY=0
PERSIST=0
VALIDATE_DDS=0
MAX_PERF=0

for arg in "$@"; do
  case "$arg" in
    --report-only)  REPORT_ONLY=1 ;;
    --persist)      PERSIST=1 ;;
    --validate-dds) VALIDATE_DDS=1 ;;
    --max-perf)     MAX_PERF=1 ;;
    -h|--help)
      sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[33m!! %s\033[0m\n' "$*"; }
ok()   { printf '\033[32mok  %s\033[0m\n' "$*"; }

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  if command -v sudo >/dev/null 2>&1; then SUDO="sudo"; fi
fi

# ---------------------------------------------------------------------------
bold "=== 1. platform ==="
MODEL=""
[ -r /proc/device-tree/model ] && MODEL="$(tr -d '\0' < /proc/device-tree/model)"
echo "model          : ${MODEL:-unknown}"
echo "kernel         : $(uname -r)"
echo "cores          : $(nproc)"
echo "ros distro     : ${ROS_DISTRO:-<not sourced>}"
echo "rmw            : ${RMW_IMPLEMENTATION:-<default>}"
if [ -n "${ROS_DISTRO:-}" ]; then
  if [ -f "/opt/ros/${ROS_DISTRO}/lib/librmw_cyclonedds_cpp.so" ] \
     || ldconfig -p 2>/dev/null | grep -q librmw_cyclonedds_cpp; then
    echo "cyclonedds rmw : installed"
  else
    warn "rmw_cyclonedds_cpp is NOT installed:"
    warn "  sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
  fi
fi
[ -r /etc/nv_tegra_release ] && echo "l4t            : $(head -1 /etc/nv_tegra_release)"
free -h | sed -n '1,2p' | sed 's/^/mem            : /'

# ---------------------------------------------------------------------------
bold ""
bold "=== 2. power / clocks (a Jetson that throttles looks exactly like a leak) ==="
if command -v nvpmodel >/dev/null 2>&1; then
  nvpmodel -q 2>/dev/null | sed 's/^/  /' || warn "nvpmodel -q failed"
else
  echo "  nvpmodel not present (not a Jetson, or not on PATH)"
fi
GOV="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo '?')"
echo "  cpu0 governor: $GOV"
if [ "$GOV" != "performance" ]; then
  warn "governor is '$GOV'. Frequency ramping adds latency spikes to the LOAM"
  warn "timer callback. Consider 'jetson_clocks' for the duration of the test."
fi
if [ "$MAX_PERF" -eq 1 ] && [ "$REPORT_ONLY" -eq 0 ]; then
  if command -v nvpmodel >/dev/null 2>&1; then
    echo "  applying nvpmodel -m 0 (MAXN)"; $SUDO nvpmodel -m 0 || warn "nvpmodel failed"
  fi
  if command -v jetson_clocks >/dev/null 2>&1; then
    echo "  applying jetson_clocks"; $SUDO jetson_clocks || warn "jetson_clocks failed"
  elif command -v jetson_clocks.sh >/dev/null 2>&1; then
    echo "  applying jetson_clocks.sh"; $SUDO jetson_clocks.sh || warn "failed"
  fi
fi

# ---------------------------------------------------------------------------
bold ""
bold "=== 3. network buffers (this is where DDS drops your point clouds) ==="
# A 2x Mid-360 frame pair is ~1 MB at 10 Hz. 16 MB of socket buffer gives the
# LOAM thread ~10 frames of slack while it is stuck in ICP.
declare -A SYSCTL=(
  [net.core.rmem_max]=33554432
  [net.core.rmem_default]=16777216
  [net.core.wmem_max]=33554432
  [net.core.wmem_default]=16777216
  [net.core.netdev_max_backlog]=30000
  [net.ipv4.ipfrag_high_thresh]=16777216
)
for key in "${!SYSCTL[@]}"; do
  want="${SYSCTL[$key]}"
  cur="$(sysctl -n "$key" 2>/dev/null || echo '?')"
  if [ "$cur" = "?" ]; then
    warn "$key not readable, skipping"
    continue
  fi
  if [ "$REPORT_ONLY" -eq 1 ]; then
    printf '  %-32s current=%-12s want=%s\n' "$key" "$cur" "$want"
    continue
  fi
  if [ "$cur" -ge "$want" ] 2>/dev/null; then
    ok "$(printf '%-32s already %s (>= %s)' "$key" "$cur" "$want")"
  else
    if $SUDO sysctl -q -w "$key=$want" 2>/dev/null; then
      ok "$(printf '%-32s %s -> %s' "$key" "$cur" "$want")"
    else
      warn "$(printf '%-32s could not set (need root?) still %s' "$key" "$cur")"
    fi
  fi
done

if [ "$PERSIST" -eq 1 ] && [ "$REPORT_ONLY" -eq 0 ]; then
  CONF=/etc/sysctl.d/99-loam-perf.conf
  {
    echo "# written by FAST_LIO_ROS2 perf/setup_target.sh"
    for key in "${!SYSCTL[@]}"; do echo "$key = ${SYSCTL[$key]}"; done
  } | $SUDO tee "$CONF" >/dev/null && ok "persisted to $CONF"
fi

# ---------------------------------------------------------------------------
bold ""
bold "=== 4. core dumps (so a crash leaves a backtrace, not just a message) ==="
CP="$(cat /proc/sys/kernel/core_pattern 2>/dev/null || echo '?')"
echo "  core_pattern : $CP"
echo "  ulimit -c    : $(ulimit -c)"
case "$CP" in
  \|*) warn "cores are piped to a handler ($CP)."
       warn "With systemd-coredump, read them back with:  coredumpctl gdb fastlio_mapping"
       ;;
  *)   if [ "$REPORT_ONLY" -eq 0 ]; then
         $SUDO sysctl -q -w kernel.core_pattern=/tmp/core.%e.%p.%t 2>/dev/null \
           && ok "core_pattern -> /tmp/core.%e.%p.%t" \
           || warn "could not set core_pattern"
       fi
       ;;
esac
echo "  NOTE: 'ulimit -c unlimited' is per-shell and cannot be set by this"
echo "        script for your shell. run_test.sh does it for the node it spawns."

# ---------------------------------------------------------------------------
bold ""
bold "=== 5. build type check ==="
FLIO_BIN="$(command -v fastlio_mapping 2>/dev/null || true)"
if [ -z "$FLIO_BIN" ]; then
  for c in "$HOME"/*_ws/install/fast_lio/lib/fast_lio/fastlio_mapping \
           "$HOME"/loam_test/*/install/fast_lio/lib/fast_lio/fastlio_mapping \
           ./install/fast_lio/lib/fast_lio/fastlio_mapping; do
    [ -x "$c" ] && FLIO_BIN="$c" && break
  done
fi
if [ -n "$FLIO_BIN" ]; then
  echo "  binary       : $FLIO_BIN"
  if file "$FLIO_BIN" 2>/dev/null | grep -q "not stripped"; then
    ok "has symbols (backtraces will be readable)"
  else
    warn "stripped or unknown: rebuild with"
    warn "  colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
    warn "so a core dump gives you line numbers."
  fi
else
  warn "fastlio_mapping not found; build the workspace first."
fi

# ---------------------------------------------------------------------------
if [ "$VALIDATE_DDS" -eq 1 ]; then
  bold ""
  bold "=== 6. Cyclone DDS config validation ==="
  XML="${CYCLONEDDS_URI#file://}"
  [ -n "$XML" ] && [ -f "$XML" ] || XML="$HERE/config/cyclonedds_jetson.xml"
  echo "  checking: $XML"
  python3 -c "import xml.dom.minidom,sys; xml.dom.minidom.parse('$XML')" \
    && ok "XML is well-formed" || { warn "XML is malformed"; exit 1; }

  # The trap: SocketReceiveBufferSize can never exceed net.core.rmem_max, and
  # Cyclone does not fail loudly when it silently gets less than asked for.
  RMEM_MAX="$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)"
  python3 - "$XML" "$RMEM_MAX" <<'PY'
import re, sys, xml.etree.ElementTree as ET
xml_path, rmem_max = sys.argv[1], int(sys.argv[2] or 0)

def to_bytes(v):
    m = re.fullmatch(r"\s*([0-9.]+)\s*([KkMmGg]?)[Bb]?\s*", v or "")
    if not m:
        return None
    return int(float(m.group(1)) * {"": 1, "k": 1 << 10, "m": 1 << 20,
                                    "g": 1 << 30}[m.group(2).lower()])

root = ET.parse(xml_path).getroot()
found = False
for el in root.iter():
    if el.tag.split("}")[-1] != "SocketReceiveBufferSize":
        continue
    found = True
    for attr in ("min", "max"):
        raw = el.attrib.get(attr)
        if not raw:
            print(f"    note: SocketReceiveBufferSize has no '{attr}' attribute")
            continue
        want = to_bytes(raw)
        if want is None:
            print(f"    note: could not parse {attr}={raw!r}")
        elif rmem_max and want > rmem_max:
            print(f"    !! {attr}={raw} ({want} B) EXCEEDS net.core.rmem_max "
                  f"({rmem_max} B).")
            print(f"    !! Cyclone will warn: 'failed to increase socket receive "
                  f"buffer size'.")
            print(f"    !! Fix:  sudo sysctl -w net.core.rmem_max={max(want, 33554432)}")
        else:
            print(f"    ok  {attr}={raw} fits under net.core.rmem_max "
                  f"({rmem_max} B)")
if not found:
    print("    note: no SocketReceiveBufferSize element in this config")
else:
    # Reading Cyclone's numbers: Linux stores 2x the requested SO_RCVBUF (the
    # extra is bookkeeping overhead) and getsockopt reports the doubled value,
    # while net.core.rmem_max caps the value BEFORE doubling. So the size
    # Cyclone reports as "current" is roughly 2 * min(requested, rmem_max) --
    # e.g. "current is 2097152" means rmem_max is about 1 MB, not 2 MB.
    print("    note: Cyclone's reported size is ~2x min(request, rmem_max);")
    print("          Linux doubles SO_RCVBUF internally. 'current is 2097152'")
    print("          therefore means rmem_max is ~1MB.")
PY
  if [ -z "${ROS_DISTRO:-}" ]; then
    warn "ROS not sourced; skipping the live load test."
  else
    echo "  starting a throwaway node with CYCLONEDDS_URI set..."
    OUT="$(RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$XML" \
            timeout 12 ros2 run demo_nodes_cpp talker 2>&1 | head -25)"
    if echo "$OUT" | grep -qi "RMW implementation not installed"; then
      # Not a config problem at all: distinguish it, or the real message is lost.
      warn "cannot test the config: rmw_cyclonedds_cpp is not installed."
      warn "  sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
    elif echo "$OUT" | grep -qiE "unknown element|not a valid|parse error|syntax error|invalid configuration|error in configuration"; then
      warn "Cyclone rejected the config. Comment out the element it names:"
      echo "$OUT" | sed 's/^/    /'
    elif echo "$OUT" | grep -qi "config" ; then
      echo "  Cyclone printed config-related output; check it reads as expected:"
      echo "$OUT" | grep -i config | sed 's/^/    /' | head -10
      ok "no hard configuration error"
    else
      ok "Cyclone accepted the config (no configuration error on startup)"
    fi
  fi
fi

bold ""
bold "=== next ==="
cat <<'EOT'
  1. source perf/config/perf_env.sh          (in every terminal)
  2. read  perf/README.md                    (the test protocol)
  3. run   perf/run_test.sh --help
EOT
