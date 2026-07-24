#!/usr/bin/env bash
set -euo pipefail

cmake -S cpp -B build/cpp -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/cpp --parallel
ctest --test-dir build/cpp --output-on-failure

if [[ $# -ge 1 ]]; then
  dataset="$1"
  output="${2:-artifacts/cpp_full_voxel_0.05_corr_0.12}"
  ./build/cpp/rgbd_odometry \
    --dataset "$dataset" \
    --output "$output" \
    --voxel 0.05 \
    --max-correspondence 0.12
fi
