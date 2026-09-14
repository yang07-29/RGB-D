param(
    [string]$Output = "artifacts\loop_learning_multiseed_v2",
    [string]$ResultOutput = "results\loop_learning_multiseed_v2"
)

$ErrorActionPreference = "Stop"
$learningPython = ".\.conda-learning\python.exe"
$open3dPython = ".\.conda-open3d\python.exe"
$seeds = @(42, 20260724, 20260914)

foreach ($seed in $seeds) {
    $runOutput = "$Output\seed_$seed"
    & $learningPython -m src.train_loop_descriptor `
        --train-dataset data\rgbd_dataset_freiburg1_desk `
        --validation-dataset data\rgbd_dataset_freiburg1_desk2 `
        --test-dataset data\rgbd_dataset_freiburg1_xyz `
        --output $runOutput `
        --epochs 5 `
        --batch-size 32 `
        --device cuda `
        --seed $seed `
        --negative-sampling geometric_hard `
        --hard-negative-fraction 0.25
    if ($LASTEXITCODE -ne 0) { throw "Descriptor training failed for seed $seed" }
}

$poseJobs = @()
$firstSeed = $seeds[0]
$firstMetrics = Import-Csv "$Output\seed_$firstSeed\metrics.csv"
foreach ($method in @("hsv_histogram", "mobilenetv3_imagenet_frozen")) {
    $row = $firstMetrics | Where-Object method -eq $method
    $poseJobs += [pscustomobject]@{Seed=$firstSeed; Method=$method; Threshold=$row.validation_threshold}
}
foreach ($seed in $seeds) {
    $metrics = Import-Csv "$Output\seed_$seed\metrics.csv"
    foreach ($method in @("mobilenetv3_no_se", "mobilenetv3_se")) {
        $row = $metrics | Where-Object method -eq $method
        $poseJobs += [pscustomobject]@{Seed=$seed; Method=$method; Threshold=$row.validation_threshold}
    }
}

foreach ($job in $poseJobs) {
    $descriptor = "$Output\seed_$($job.Seed)\$($job.Method)_test_descriptors.npy"
    $poseOutput = "$Output\pose_graph\$($job.Method)_seed_$($job.Seed)"
    & $open3dPython -m src.pose_graph_slam `
        --dataset data\rgbd_dataset_freiburg1_xyz `
        --output $poseOutput `
        --candidate-mode descriptor `
        --descriptor-file $descriptor `
        --descriptor-threshold $job.Threshold `
        --descriptor-top-k-per-target 1 `
        --max-loop-candidates 30 `
        --hard-negative-candidates 10 `
        --quiet
    if ($LASTEXITCODE -ne 0) { throw "Pose graph failed for $($job.Method), seed $($job.Seed)" }
}

& $open3dPython -m src.build_learning_ablation_report `
    --input $Output `
    --output $ResultOutput `
    --seeds $seeds
if ($LASTEXITCODE -ne 0) { throw "Learning ablation report generation failed" }
