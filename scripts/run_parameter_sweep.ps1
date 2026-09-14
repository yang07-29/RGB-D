param(
    [string]$Dataset = "data\rgbd_dataset_freiburg1_xyz",
    [string]$Output = "artifacts\parameter_sweep_one_to_one_quality_v2",
    [string]$ResultOutput = "results\parameter_sweep_one_to_one_quality_v2"
)

$ErrorActionPreference = "Stop"
$python = ".\.conda-open3d\python.exe"
$voxels = @(0.03, 0.05, 0.08)
$correspondences = @(0.08, 0.12, 0.20)

foreach ($voxel in $voxels) {
    foreach ($correspondence in $correspondences) {
        $tag = "voxel_{0:N2}_corr_{1:N2}" -f $voxel, $correspondence
        & $python -m src.run_odometry --dataset $Dataset --output "$Output\numpy_point_to_point\$tag" --voxel $voxel --max-correspondence $correspondence --max-acceptable-rmse $correspondence --quiet
        if ($LASTEXITCODE -ne 0) { throw "NumPy point-to-point failed: $tag" }
        & $python -m src.open3d_odometry --dataset $Dataset --output "$Output\open3d_point_to_plane\$tag" --voxel $voxel --max-correspondence $correspondence --max-acceptable-rmse $correspondence --quiet
        if ($LASTEXITCODE -ne 0) { throw "Open3D point-to-plane failed: $tag" }
    }
}

& $python -m src.build_benchmark_report --input $Output --output $ResultOutput
if ($LASTEXITCODE -ne 0) { throw "Benchmark report aggregation failed" }
