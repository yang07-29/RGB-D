#!/usr/bin/env bash
set -euo pipefail

dataset="${1:-data/rgbd_dataset_freiburg1_xyz}"
output="${2:-artifacts/ros2_cpp_linux_full}"
install_prefix="${3:-install/ros2}"
expected_frames="${4:-790}"

if [[ ! -f "${dataset}/rgb.txt" || ! -f "${dataset}/depth.txt" || ! -f "${dataset}/groundtruth.txt" ]]; then
  echo "TUM dataset files are missing under ${dataset}" >&2
  exit 2
fi
if [[ ! -f "${install_prefix}/setup.bash" ]]; then
  echo "ROS2 workspace setup is missing: ${install_prefix}/setup.bash" >&2
  exit 2
fi

mkdir -p "${output}"
output_abs="$(cd "${output}" && pwd)"
dataset_abs="$(cd "${dataset}" && pwd)"
install_abs="$(cd "${install_prefix}" && pwd)"

# ROS-generated setup files may probe unset AMENT variables. Keep strict mode
# for this script, but temporarily disable nounset while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
source "${install_abs}/setup.bash"
set -u
package_prefix="$(ros2 pkg prefix rgbd_odometry_ros)"
parameter_file="${package_prefix}/share/rgbd_odometry_ros/config/odometry.yaml"
if [[ ! -f "${parameter_file}" ]]; then
  echo "ROS2 parameter file is missing: ${parameter_file}" >&2
  exit 2
fi

node_pid=""
publisher_pid=""
stop_process_group() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi
  kill -INT -- "-${pid}" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
    sleep 1
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}
cleanup() {
  stop_process_group "${publisher_pid}"
  stop_process_group "${node_pid}"
}
trap cleanup EXIT

setsid ros2 run rgbd_odometry_ros rgbd_odometry_node --ros-args \
  --params-file "${parameter_file}" \
  -p reliable_qos:=true \
  -p metrics_csv:="${output_abs}/metrics.csv" \
  -p trajectory_tum:="${output_abs}/trajectory_local.txt" \
  >"${output_abs}/odometry_node.log" 2>&1 &
node_pid=$!

sleep 2
setsid ros2 run rgbd_odometry_ros tum_rgbd_publisher --ros-args \
  -p dataset:="${dataset_abs}" \
  -p publish_hz:=30.0 \
  -p startup_delay_s:=3.0 \
  -p reliable_qos:=true \
  -p flush_after_done:=true \
  -p max_frames:="${expected_frames}" \
  >"${output_abs}/publisher.log" 2>&1 &
publisher_pid=$!

sleep 2
ros2 topic list | sort >"${output_abs}/topic_list.txt"
timeout 30 ros2 topic echo /odom --once >"${output_abs}/odom_once.yaml" 2>"${output_abs}/odom_once.stderr" || true

deadline=$((SECONDS + 180))
while true; do
  if [[ -f "${output_abs}/metrics.csv" ]]; then
    rows=$(( $(wc -l <"${output_abs}/metrics.csv") - 1 ))
    if (( rows >= expected_frames )); then
      break
    fi
  fi
  if ! kill -0 "${node_pid}" 2>/dev/null; then
    echo "ROS2 C++ odometry node exited before processing ${expected_frames} frames" >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    rows=0
    [[ ! -f "${output_abs}/metrics.csv" ]] || rows=$(( $(wc -l <"${output_abs}/metrics.csv") - 1 ))
    echo "Timed out waiting for ${expected_frames} processed frames; observed ${rows}" >&2
    exit 1
  fi
  sleep 1
done

sleep 2
cleanup
trap - EXIT

python3 -m src.summarize_ros2_metrics \
  --input "${output_abs}/metrics.csv" \
  --output "${output_abs}/performance.json"
python3 -m src.evaluate_saved_trajectory \
  --dataset "${dataset_abs}" \
  --trajectory "${output_abs}/trajectory_local.txt" \
  --output "${output_abs}/evaluation" \
  --method ros2_cpp_point_to_point

python3 - "${output_abs}" "${expected_frames}" <<'PY'
import csv
import json
import pathlib
import sys

output = pathlib.Path(sys.argv[1])
expected = int(sys.argv[2])
with (output / "metrics.csv").open(newline="", encoding="utf-8") as handle:
    rows = list(csv.DictReader(handle))
trajectory = [
    line for line in (output / "trajectory_local.txt").read_text(encoding="utf-8").splitlines()
    if line.strip() and not line.startswith("#")
]
performance = json.loads((output / "performance.json").read_text(encoding="utf-8"))
evaluation = json.loads((output / "evaluation" / "evaluation.json").read_text(encoding="utf-8"))
if len(rows) != expected or len(trajectory) != expected:
    raise SystemExit(f"expected {expected} metrics/poses, got {len(rows)}/{len(trajectory)}")
if performance["unsynchronized_or_pending_final"] not in (0, 1):
    raise SystemExit("unexpected final pending count after the deliberate synchronizer flush pair")
if performance["rgb_received_final"] < expected or performance["depth_received_final"] < expected:
    raise SystemExit("fewer than the expected source messages were received")
if evaluation["frames"] != expected:
    raise SystemExit("evaluation frame count mismatch")
summary = {
    "backend": "ROS2 Jazzy C++",
    "frames": expected,
    "metrics_rows": len(rows),
    "trajectory_poses": len(trajectory),
    "status_counts": performance["status_counts"],
    "callback_latency_ms": performance["callback_latency_ms"],
    "process_peak_rss_bytes": performance["process_peak_rss_bytes"],
    "ate_rpe": evaluation["metrics"],
    "synchronizer_flush": "one later timestamp pair is sent only to release the final real pair; it is not in the trajectory",
    "ground_truth_usage": "evaluation only; poses are not published or read by the odometry node",
}
(output / "run_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2))
PY

echo "ROS2 C++ full-sequence evidence written to ${output_abs}"
