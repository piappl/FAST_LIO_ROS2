# shellcheck shell=bash
#
# perf/config/perf_env.sh -- source this in EVERY terminal that runs any part of
# the test (driver, LOAM, monitors, rviz). If the processes disagree on RMW or
# domain they will not see each other, and you will misdiagnose that as loss.
#
#   source ~/loam_test/FAST_LIO_ROS2/perf/config/perf_env.sh
#
# Override anything by exporting it BEFORE sourcing, e.g.
#   LOAM_PERF_DDS_URI=  source .../perf_env.sh      # run with stock DDS tuning

_perf_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LOAM_PERF_ROOT="$(dirname "$_perf_env_dir")"

# ---------------------------------------------------------------- ROS / DDS ---
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Point Cyclone at the tuned config. Set LOAM_PERF_DDS_URI to empty to get the
# stock behaviour -- that A/B is one of the tests in the protocol.
if [ -z "${LOAM_PERF_DDS_URI+x}" ]; then
  LOAM_PERF_DDS_URI="file://${_perf_env_dir}/cyclonedds_jetson.xml"
fi
if [ -n "$LOAM_PERF_DDS_URI" ]; then
  export CYCLONEDDS_URI="$LOAM_PERF_DDS_URI"
else
  unset CYCLONEDDS_URI
fi

# Keep discovery on this host unless you actually need a remote RViz. Cuts
# discovery chatter and multicast load. (ROS_LOCALHOST_ONLY is deprecated in
# Jazzy; this is its replacement.)
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

# Unbuffered, timestamped console output -- essential for lining a crash up
# against the monitor CSVs.
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=0
# NOTE: this must not be written as ${VAR:-[{severity}] ...} -- the first '}'
# would close the parameter expansion and mangle the format.
if [ -z "${RCUTILS_CONSOLE_OUTPUT_FORMAT:-}" ]; then
  export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'
fi

# ------------------------------------------------------- FAST_LIO perf probe ---
# Empty = probe off (zero overhead). run_test.sh sets this per run.
#   export FASTLIO_PERF_LOG=/path/to/perf_probe.csv
export FASTLIO_PERF_LOG="${FASTLIO_PERF_LOG:-}"

# ------------------------------------------------------------------- cores ----
# Leave core 0 for the kernel/IRQs; see README "CPU affinity" before using.
export LOAM_PERF_TASKSET="${LOAM_PERF_TASKSET:-}"

# Guard: a missing RMW produces a wall of rcl text that reads like a transport
# fault. Say plainly what is wrong instead.
if [ -n "${ROS_DISTRO:-}" ] && [ "$RMW_IMPLEMENTATION" = "rmw_cyclonedds_cpp" ]; then
  if ! ldconfig -p 2>/dev/null | grep -q librmw_cyclonedds_cpp \
     && [ ! -f "/opt/ros/${ROS_DISTRO}/lib/librmw_cyclonedds_cpp.so" ]; then
    echo "perf_env: WARNING rmw_cyclonedds_cpp does not appear to be installed." >&2
    echo "perf_env:   sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" >&2
    echo "perf_env:   (or: export RMW_IMPLEMENTATION=rmw_fastrtps_cpp before sourcing)" >&2
  fi
fi

echo "perf_env: RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID" \
     "CYCLONEDDS_URI=${CYCLONEDDS_URI:-<stock>}" \
     "DISCOVERY=$ROS_AUTOMATIC_DISCOVERY_RANGE" \
     "PERF_LOG=${FASTLIO_PERF_LOG:-<off>}"
unset _perf_env_dir
