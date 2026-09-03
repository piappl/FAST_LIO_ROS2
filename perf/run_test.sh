#!/usr/bin/env bash
#
# run_test.sh -- run one instrumented LOAM performance/crash experiment and
# collect everything needed to explain a failure into a single run directory.
#
# Two modes:
#
#   MONITOR-ONLY (default) -- you start the driver and LOAM yourself in other
#   terminals; this script only runs the monitors and attaches to the LOAM
#   process by name. Best for long "let it crash" sessions.
#
#       ./perf/run_test.sh --name phase2 --duration 1800
#
#   ORCHESTRATED -- this script also starts LOAM (and optionally the driver),
#   so the run is reproducible and the exit code is captured.
#
#       ./perf/run_test.sh --name phase2 --duration 1800 \
#         --loam-cmd "ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false"
#
# Everything lands in perf/runs/<timestamp>_<name>/ and analyze.py is run at
# the end.
#
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"

NAME="run"
DURATION=600
CONFIG_FILE=""
IMU_TOPICS=()
CLOUD_TOPICS=()
CUSTOM_TOPICS=()
IMU_RATE=200
CLOUD_RATE=10
LOAM_CMD=""
DRIVER_CMD=""
PROC_NAME="fastlio_mapping"
WITH_GREEDY=1
WITH_PROBE=1
SAMPLE_INTERVAL=1.0
REPORT_PERIOD=10

usage() { sed -n '2,30p' "$0"; cat <<'EOT'

Options:
  --name NAME             run label (default: run)
  --duration SEC          how long to record (default: 600; 0 = until Ctrl-C)
  --loam-cmd CMD          command that starts LOAM (omit = monitor-only mode)
  --driver-cmd CMD        command that starts the Livox driver (optional)
  --proc-name NAME        process to track for RSS/CPU (default: fastlio_mapping)
  --config FILE           node config YAML to read common.lid_topic /
                          common.imu_topic from. Auto-detected from
                          --loam-cmd "... config_file:=NAME ..." when omitted.
  --imu-topic T           repeatable; overrides the config (default: from
                          --config, else /livox/imu)
  --cloud-topic T         repeatable; overrides the config (default: from
                          --config, else /livox/lidar)
  --custom-topic T        repeatable, livox CustomMsg topic (AVIA path)
  --imu-rate HZ           nominal IMU rate (default: 200)
  --cloud-rate HZ         nominal cloud rate (default: 10)
  --no-greedy             skip the deep-QoS reference subscriber
  --no-probe              do not set FASTLIO_PERF_LOG
  --interval SEC          resource sampling interval (default: 1.0)
  --report-period SEC     monitor console/CSV aggregation period (default: 10)
EOT
}

while [ $# -gt 0 ]; do
  case "$1" in
    --name)          NAME="$2"; shift 2 ;;
    --duration)      DURATION="$2"; shift 2 ;;
    --loam-cmd)      LOAM_CMD="$2"; shift 2 ;;
    --driver-cmd)    DRIVER_CMD="$2"; shift 2 ;;
    --proc-name)     PROC_NAME="$2"; shift 2 ;;
    --config)        CONFIG_FILE="$2"; shift 2 ;;
    --imu-topic)     IMU_TOPICS+=("$2"); shift 2 ;;
    --cloud-topic)   CLOUD_TOPICS+=("$2"); shift 2 ;;
    --custom-topic)  CUSTOM_TOPICS+=("$2"); shift 2 ;;
    --imu-rate)      IMU_RATE="$2"; shift 2 ;;
    --cloud-rate)    CLOUD_RATE="$2"; shift 2 ;;
    --no-greedy)     WITH_GREEDY=0; shift ;;
    --no-probe)      WITH_PROBE=0; shift ;;
    --interval)      SAMPLE_INTERVAL="$2"; shift 2 ;;
    --report-period) REPORT_PERIOD="$2"; shift 2 ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

# ------------------------------------------------------- topic resolution --
# The monitors must subscribe to the topics THE NODE READS, which live in the
# node's config YAML (common.lid_topic / common.imu_topic). Defaulting to
# /livox/imu and /livox/lidar and hoping means that the moment a config uses
# non-default names -- an external IMU on /imu/data, a namespaced lidar on
# /livox/hap_4/lidar -- both monitors subscribe to dead topics, receive nothing,
# and the report fills up with "FAIL: pubs=0 / 0.00 Hz" for a run whose streams
# were in fact perfect. Worse, H1's loam-vs-greedy cross-check silently compares
# two empty datasets and reports no loss.
#
# So: read the topics out of the config instead of guessing.

# Auto-detect the config from `... config_file:=hap.yaml ...` in --loam-cmd.
if [ -z "$CONFIG_FILE" ] && [ -n "$LOAM_CMD" ]; then
  _cf="$(printf '%s' "$LOAM_CMD" | grep -oE 'config_file:=[^[:space:]]+' | head -1 | cut -d= -f2-)"
  _cf="${_cf#:}"
  if [ -n "$_cf" ]; then
    if [ -f "$_cf" ]; then CONFIG_FILE="$_cf"
    elif [ -f "$REPO/config/$_cf" ]; then CONFIG_FILE="$REPO/config/$_cf"
    else echo "WARNING: config_file:=$_cf named in --loam-cmd but not found" >&2
    fi
  fi
fi

yaml_scalar() {  # $1=file $2=key -> bare value, quotes and trailing comment stripped
  grep -E "^[[:space:]]*$2:" "$1" 2>/dev/null \
    | grep -vE '^[[:space:]]*#' | head -1 \
    | sed -e "s/^[[:space:]]*$2:[[:space:]]*//" -e 's/[[:space:]]*#.*$//' \
          -e 's/^"//' -e 's/"$//' -e "s/^'//" -e "s/'\$//" \
          -e 's/[[:space:]]*$//'
}

CFG_LID=""; CFG_IMU=""; CFG_LTYPE=""; CFG_RATE=""
if [ -n "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ]; then
  CFG_LID="$(yaml_scalar "$CONFIG_FILE" lid_topic)"
  CFG_IMU="$(yaml_scalar "$CONFIG_FILE" imu_topic)"
  CFG_LTYPE="$(yaml_scalar "$CONFIG_FILE" lidar_type)"
  CFG_RATE="$(yaml_scalar "$CONFIG_FILE" scan_rate)"
  echo "config: $CONFIG_FILE"
  echo "  lid_topic=${CFG_LID:-<unset>} imu_topic=${CFG_IMU:-<unset>}" \
       "lidar_type=${CFG_LTYPE:-?} scan_rate=${CFG_RATE:-?}"
fi

if [ ${#IMU_TOPICS[@]} -eq 0 ]; then
  if [ -n "$CFG_IMU" ]; then IMU_TOPICS=("$CFG_IMU")
  else
    IMU_TOPICS=("/livox/imu")
    echo "WARNING: no --imu-topic and no config to read it from; guessing" \
         "/livox/imu. If the node reads a different topic the monitors will" \
         "record NOTHING and section 1/2 of the report will be meaningless." >&2
  fi
fi

# lidar_type 1 (AVIA) means the lidar topic carries livox CustomMsg, not
# PointCloud2. Subscribing with the wrong type is another way to get n=0.
if [ ${#CLOUD_TOPICS[@]} -eq 0 ] && [ ${#CUSTOM_TOPICS[@]} -eq 0 ]; then
  if [ -n "$CFG_LID" ]; then
    if [ "$CFG_LTYPE" = "1" ]; then CUSTOM_TOPICS=("$CFG_LID")
    else                            CLOUD_TOPICS=("$CFG_LID")
    fi
  else
    CLOUD_TOPICS=("/livox/lidar")
    echo "WARNING: no --cloud-topic and no config to read it from; guessing" \
         "/livox/lidar (see the IMU warning above)." >&2
  fi
fi

# scan_rate is the nominal cloud rate the checklist compares against.
if [ -n "$CFG_RATE" ] && [ "$CLOUD_RATE" = "10" ]; then CLOUD_RATE="$CFG_RATE"; fi

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ERROR: ROS is not sourced. Do:" >&2
  echo "  source /opt/ros/jazzy/setup.bash && source <your_ws>/install/setup.bash" >&2
  echo "  source $HERE/config/perf_env.sh" >&2
  exit 1
fi

RUN_DIR="$HERE/runs/$(date +%Y%m%d_%H%M%S)_${NAME}"
mkdir -p "$RUN_DIR"
echo "run dir: $RUN_DIR"

# --------------------------------------------------------------- provenance --
{
  echo "name=$NAME"
  echo "date=$(date -Is)"
  echo "duration_requested_s=$DURATION"
  echo "host=$(hostname)"
  echo "kernel=$(uname -r)"
  echo "ros_distro=${ROS_DISTRO:-}"
  echo "rmw=${RMW_IMPLEMENTATION:-<default>}"
  echo "cyclonedds_uri=${CYCLONEDDS_URI:-<stock>}"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-0}"
  echo "discovery_range=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<default>}"
  echo "config_file=${CONFIG_FILE:-<none>}"
  echo "imu_topics=${IMU_TOPICS[*]}"
  echo "cloud_topics=${CLOUD_TOPICS[*]}"
  echo "custom_topics=${CUSTOM_TOPICS[*]:-}"
  echo "imu_rate=$IMU_RATE"
  echo "cloud_rate=$CLOUD_RATE"
  echo "loam_cmd=${LOAM_CMD:-<external>}"
  echo "driver_cmd=${DRIVER_CMD:-<external>}"
  echo "probe=$WITH_PROBE"
  echo "greedy_monitor=$WITH_GREEDY"
  echo "git_describe=$(git -C "$REPO" describe --always --dirty 2>/dev/null || echo n/a)"
  echo "git_rev=$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo n/a)"
} > "$RUN_DIR/run_info.txt"
git -C "$REPO" diff > "$RUN_DIR/repo.diff" 2>/dev/null || true
cp "$REPO/config/"*.yaml "$RUN_DIR/" 2>/dev/null || true

# probe output prefix
if [ "$WITH_PROBE" -eq 1 ]; then
  export FASTLIO_PERF_LOG="$RUN_DIR/perf_probe"
  echo "probe: FASTLIO_PERF_LOG=$FASTLIO_PERF_LOG"
else
  unset FASTLIO_PERF_LOG
  echo "probe: disabled"
fi

# core dumps for whatever we spawn
ulimit -c unlimited 2>/dev/null || echo "warn: could not raise core limit"

PIDS=()
LOAM_PID=""

cleanup() {
  echo ""
  echo "--- stopping ---"
  # monitors first so they write their summaries, then the workload
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  if [ -n "$LOAM_PID" ]; then
    kill -TERM "-$LOAM_PID" 2>/dev/null || kill -TERM "$LOAM_PID" 2>/dev/null || true
  fi
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  [ -n "$LOAM_PID" ] && { kill -KILL "-$LOAM_PID" 2>/dev/null || true; }
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ------------------------------------------------------------------ driver ---
if [ -n "$DRIVER_CMD" ]; then
  echo "starting driver: $DRIVER_CMD"
  setsid bash -c "$DRIVER_CMD" > "$RUN_DIR/driver.log" 2>&1 &
  PIDS+=("$!")
  sleep 5
fi

# -------------------------------------------------------------------- LOAM ---
if [ -n "$LOAM_CMD" ]; then
  echo "starting LOAM: $LOAM_CMD"
  setsid bash -c "ulimit -c unlimited; exec $LOAM_CMD" \
    > "$RUN_DIR/loam.log" 2>&1 &
  LOAM_PID=$!
  echo "loam launcher pid=$LOAM_PID"
  sleep 3
else
  echo "monitor-only mode: start the driver and LOAM yourself now."
fi

# ---------------------------------------------------------------- monitors ---
MON_ARGS=()
for t in "${IMU_TOPICS[@]}";    do MON_ARGS+=(--imu-topic "$t"); done
for t in "${CLOUD_TOPICS[@]}";  do MON_ARGS+=(--cloud-topic "$t"); done
for t in "${CUSTOM_TOPICS[@]:-}"; do [ -n "$t" ] && MON_ARGS+=(--custom-topic "$t"); done
MON_ARGS+=(--imu-rate "$IMU_RATE" --cloud-rate "$CLOUD_RATE"
           --report-period "$REPORT_PERIOD" --out-dir "$RUN_DIR")

echo "starting stream_monitor (loam QoS)"
python3 "$HERE/monitors/stream_monitor.py" --tag loamqos --qos loam \
  "${MON_ARGS[@]}" > "$RUN_DIR/monitor_loamqos.log" 2>&1 &
PIDS+=("$!")

if [ "$WITH_GREEDY" -eq 1 ]; then
  echo "starting stream_monitor (greedy QoS reference)"
  python3 "$HERE/monitors/stream_monitor.py" --tag greedy --qos greedy \
    "${MON_ARGS[@]}" > "$RUN_DIR/monitor_greedy.log" 2>&1 &
  PIDS+=("$!")
fi

echo "starting resource_monitor (tracking '$PROC_NAME')"
python3 "$HERE/monitors/resource_monitor.py" --name "$PROC_NAME" \
  --out-dir "$RUN_DIR" --interval "$SAMPLE_INTERVAL" \
  > "$RUN_DIR/monitor_resources.log" 2>&1 &
PIDS+=("$!")

# ------------------------------------------------------------------- wait ----
echo ""
if [ "$DURATION" -gt 0 ] 2>/dev/null; then
  echo "recording for ${DURATION}s (Ctrl-C to stop early)..."
else
  echo "recording until Ctrl-C..."
fi
echo "watch progress:  tail -f $RUN_DIR/monitor_loamqos.log"
echo ""

START=$(date +%s)
while :; do
  sleep 5
  NOW=$(date +%s); EL=$((NOW - START))
  if [ "$DURATION" -gt 0 ] 2>/dev/null && [ "$EL" -ge "$DURATION" ]; then
    echo "duration reached (${EL}s)"; break
  fi
  # if we launched LOAM and it died, stop and record that fact
  if [ -n "$LOAM_PID" ] && ! kill -0 "$LOAM_PID" 2>/dev/null; then
    echo "!! LOAM process group exited after ${EL}s -- this is the event we want"
    echo "loam_exited_after_s=$EL" >> "$RUN_DIR/run_info.txt"
    sleep 3   # let the monitors capture the aftermath
    break
  fi
  # -x matches the process NAME exactly, so our python monitors (whose command
  # lines mention $PROC_NAME) are not mistaken for the node itself.
  if ! pgrep -x "$PROC_NAME" > /dev/null 2>&1; then
    if [ -f "$RUN_DIR/.saw_proc" ]; then
      echo "!! '$PROC_NAME' disappeared after ${EL}s"
      echo "proc_vanished_after_s=$EL" >> "$RUN_DIR/run_info.txt"
      sleep 3
      break
    fi
  else
    touch "$RUN_DIR/.saw_proc"
  fi
done

cleanup
trap - EXIT INT TERM
rm -f "$RUN_DIR/.saw_proc"

# ------------------------------------------------------- crash post-mortem ---
echo ""
echo "--- post-mortem ---"
{
  echo "== tail of loam.log =="
  tail -60 "$RUN_DIR/loam.log" 2>/dev/null || echo "(monitor-only mode: no loam.log)"
  echo ""
  echo "== dmesg: OOM / segfault =="
  (dmesg -T 2>/dev/null || dmesg 2>/dev/null || echo "<needs root>") \
    | grep -iE "oom|out of memory|killed process|segfault|general protection" \
    | tail -30
  echo ""
  echo "== core files =="
  ls -la /tmp/core.* 2>/dev/null || echo "none in /tmp"
  command -v coredumpctl >/dev/null 2>&1 && coredumpctl list 2>/dev/null | tail -5
} > "$RUN_DIR/postmortem.txt" 2>&1
sed -n '1,40p' "$RUN_DIR/postmortem.txt"

CORE="$(ls -t /tmp/core.${PROC_NAME}* 2>/dev/null | head -1)"
BIN="$(command -v "$PROC_NAME" 2>/dev/null || true)"
if [ -n "$CORE" ] && [ -n "$BIN" ] && command -v gdb >/dev/null 2>&1; then
  echo "extracting backtrace from $CORE"
  gdb -batch -ex "thread apply all bt full" -ex quit "$BIN" "$CORE" \
    > "$RUN_DIR/backtrace.txt" 2>&1
  echo "  -> $RUN_DIR/backtrace.txt"
elif command -v coredumpctl >/dev/null 2>&1; then
  coredumpctl info "$PROC_NAME" > "$RUN_DIR/coredumpctl_info.txt" 2>&1 || true
  echo "  hint: coredumpctl gdb $PROC_NAME   (then: thread apply all bt full)"
fi

# ---------------------------------------------------------------- analysis ---
echo ""
python3 "$HERE/analyze.py" "$RUN_DIR" | tee "$RUN_DIR/report.txt"
echo ""
echo "run dir: $RUN_DIR"
