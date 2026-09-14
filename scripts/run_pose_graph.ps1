param(
    [string]$Dataset = "data\rgbd_dataset_freiburg1_xyz",
    [string]$Output = "artifacts\pose_graph_one_to_one_edge_quality_v2",
    [string]$ResultOutput = "results\pose_graph_one_to_one_edge_quality_v2"
)

$ErrorActionPreference = "Stop"
$env:OMP_NUM_THREADS = "1"
$env:OPENBLAS_NUM_THREADS = "1"
$python = ".\.conda-open3d\python.exe"
if (-not (Test-Path $python)) {
    throw "Open3D environment not found. Create .conda-open3d and install requirements-open3d.lock.txt first."
}

& $python -m src.pose_graph_slam `
    --dataset $Dataset `
    --output $Output `
    --hard-negative-candidates 10 `
    --quiet
if ($LASTEXITCODE -ne 0) { throw "Pose-graph experiment failed" }

& $python -m src.build_pose_graph_report --input $Output --output $ResultOutput
if ($LASTEXITCODE -ne 0) { throw "Pose-graph report generation failed" }
