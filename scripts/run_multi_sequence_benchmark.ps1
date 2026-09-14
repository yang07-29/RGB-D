param(
    [string]$DataRoot = "data",
    [string]$Output = "artifacts\multi_sequence_one_to_one_quality_v2",
    [string]$ResultOutput = "results\multi_sequence_one_to_one_quality_v2"
)

$ErrorActionPreference = "Stop"
$python = ".\.conda-open3d\python.exe"
$datasets = @(
    "rgbd_dataset_freiburg1_xyz",
    "rgbd_dataset_freiburg1_desk",
    "rgbd_dataset_freiburg1_desk2"
)
$voxel = 0.05
$correspondence = 0.08

foreach ($datasetName in $datasets) {
    $dataset = Join-Path $DataRoot $datasetName
    if (-not (Test-Path (Join-Path $dataset "groundtruth.txt"))) {
        throw "Dataset is incomplete: $dataset"
    }
    $sequence = $datasetName.Replace("rgbd_dataset_freiburg", "fr")
    & $python -m src.run_odometry --dataset $dataset --output "$Output\$sequence\numpy_point_to_point" --voxel $voxel --max-correspondence $correspondence --max-acceptable-rmse $correspondence --quiet
    if ($LASTEXITCODE -ne 0) { throw "NumPy point-to-point failed: $sequence" }
    & $python -m src.open3d_odometry --dataset $dataset --output "$Output\$sequence\open3d_point_to_plane" --voxel $voxel --max-correspondence $correspondence --max-acceptable-rmse $correspondence --quiet
    if ($LASTEXITCODE -ne 0) { throw "Open3D point-to-plane failed: $sequence" }
}

& $python -m src.build_multi_sequence_report --input $Output --output $ResultOutput
if ($LASTEXITCODE -ne 0) { throw "Multi-sequence report aggregation failed" }
