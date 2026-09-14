param(
    [string]$GroundTruth = "data\rgbd_dataset_freiburg1_xyz\groundtruth.txt",
    [string]$EstimatedTrajectory = "artifacts\parameter_sweep_one_to_one_quality_v2\open3d_point_to_plane\voxel_0.05_corr_0.08\trajectory_open3d_local.txt",
    [string]$ProjectSummary = "results\parameter_sweep_one_to_one_quality_v2\summaries\open3d_point_to_plane\voxel_0.05_corr_0.08\summary.json",
    [string]$Output = "artifacts\evo_crosscheck_one_to_one_v2",
    [string]$ResultOutput = "results\evo_crosscheck_one_to_one_v2"
)

$ErrorActionPreference = "Stop"
$python = ".\.conda-open3d\python.exe"
$evoApe = ".\.conda-open3d\Scripts\evo_ape.exe"
$evoRpe = ".\.conda-open3d\Scripts\evo_rpe.exe"

foreach ($path in @($GroundTruth, $EstimatedTrajectory, $ProjectSummary, $python, $evoApe, $evoRpe)) {
    if (-not (Test-Path -LiteralPath $path)) { throw "Required input is missing: $path" }
}
New-Item -ItemType Directory -Force $Output | Out-Null
$env:USERPROFILE = (Resolve-Path -LiteralPath $Output).Path
$env:HOME = $env:USERPROFILE

& $evoApe tum $GroundTruth $EstimatedTrajectory --align --pose_relation trans_part --t_max_diff 0.02 --save_results "$Output\ape_translation.zip" --no_warnings
if ($LASTEXITCODE -ne 0) { throw "evo APE failed" }

foreach ($delta in @(1, 30)) {
    $translationArgs = @("tum", $GroundTruth, $EstimatedTrajectory, "--pose_relation", "trans_part", "--delta", $delta, "--delta_unit", "f")
    $rotationArgs = @("tum", $GroundTruth, $EstimatedTrajectory, "--pose_relation", "angle_deg", "--delta", $delta, "--delta_unit", "f")
    if ($delta -eq 30) {
        $translationArgs += "--all_pairs"
        $rotationArgs += "--all_pairs"
    }
    $translationArgs += @("--t_max_diff", "0.02", "--save_results", "$Output\rpe_delta${delta}_translation.zip", "--no_warnings")
    $rotationArgs += @("--t_max_diff", "0.02", "--save_results", "$Output\rpe_delta${delta}_rotation.zip", "--no_warnings")
    & $evoRpe @translationArgs
    if ($LASTEXITCODE -ne 0) { throw "evo translation RPE failed for delta=$delta" }
    & $evoRpe @rotationArgs
    if ($LASTEXITCODE -ne 0) { throw "evo rotation RPE failed for delta=$delta" }
}

& $python -m src.build_evo_crosscheck_report --evo-results $Output --project-summary $ProjectSummary --output $ResultOutput
if ($LASTEXITCODE -ne 0) { throw "evo report generation failed" }
