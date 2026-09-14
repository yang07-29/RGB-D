param(
    [string]$Dataset = "data\rgbd_dataset_freiburg1_xyz",
    [string]$Output = "artifacts\performance_streaming_v2",
    [string]$ResultOutput = "results\performance_streaming_v2"
)

$ErrorActionPreference = "Stop"
$python = ".\.conda-open3d\python.exe"
if (-not (Test-Path $python)) {
    throw "Open3D environment not found: $python"
}

foreach ($run in 1..3) {
    & $python -m src.run_odometry `
        --dataset $Dataset `
        --output "$Output\numpy\run_$run" `
        --voxel 0.05 `
        --max-correspondence 0.08 `
        --max-acceptable-rmse 0.08 `
        --warmup-frames 30 `
        --quiet
    if ($LASTEXITCODE -ne 0) { throw "NumPy performance repeat $run failed" }

    & $python -m src.open3d_odometry `
        --dataset $Dataset `
        --output "$Output\open3d\run_$run" `
        --voxel 0.05 `
        --max-correspondence 0.08 `
        --max-acceptable-rmse 0.08 `
        --warmup-frames 30 `
        --quiet
    if ($LASTEXITCODE -ne 0) { throw "Performance repeat $run failed" }
}

& $python -m src.open3d_odometry `
    --dataset $Dataset `
    --output "$Output\open3d\soak_3x" `
    --voxel 0.05 `
    --max-correspondence 0.08 `
    --max-acceptable-rmse 0.08 `
    --warmup-frames 30 `
    --input-repeats 3 `
    --no-evaluation `
    --quiet
if ($LASTEXITCODE -ne 0) { throw "Three-cycle soak run failed" }

& $python -m src.build_performance_report --input $Output --output $ResultOutput --repeats 3
if ($LASTEXITCODE -ne 0) { throw "Performance report generation failed" }
