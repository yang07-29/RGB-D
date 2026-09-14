param(
    [string]$Dataset = "data\rgbd_dataset_freiburg1_xyz",
    [string]$Output = "artifacts\tracking_stress_one_to_one_v2",
    [string]$ResultOutput = "results\tracking_stress_one_to_one_v2"
)

$ErrorActionPreference = "Stop"
$python = ".\.conda-open3d\python.exe"
foreach ($step in @(1, 2, 3)) {
    foreach ($mode in @("identity", "predictive_recovery")) {
        & $python -m src.run_odometry `
            --dataset $Dataset `
            --output "$Output\step_$step\$mode" `
            --frame-step $step `
            --voxel 0.05 `
            --max-correspondence 0.08 `
            --max-acceptable-rmse 0.08 `
            --tracking-mode $mode `
            --recovery-coarse-voxel 0.12 `
            --recovery-distance-multiplier 4.0 `
            --lost-after 2 `
            --quiet
        if ($LASTEXITCODE -ne 0) { throw "Tracking stress run failed: step=$step mode=$mode" }
    }
}

& $python -m src.build_tracking_stress_report --input $Output --output $ResultOutput
if ($LASTEXITCODE -ne 0) { throw "Tracking stress report generation failed" }
