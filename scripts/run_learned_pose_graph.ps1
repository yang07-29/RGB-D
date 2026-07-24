$ErrorActionPreference = "Stop"
$env:OMP_NUM_THREADS = "1"
$env:OPENBLAS_NUM_THREADS = "1"

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

& ".\.conda-open3d\python.exe" -m src.pose_graph_slam `
    --dataset data/rgbd_dataset_freiburg1_xyz `
    --output artifacts/pose_graph_learned_fr1_xyz `
    --candidate-mode descriptor `
    --descriptor-file artifacts/loop_learning/mobilenetv3_se_test_descriptors.npy `
    --descriptor-threshold 0.7909525632858276 `
    --descriptor-top-k-per-target 1 `
    --max-loop-candidates 30
