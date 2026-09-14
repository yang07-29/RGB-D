param(
    [int]$Frames = 30,
    [double]$PublishHz = 5.0,
    [int]$TimeoutSeconds = 60,
    [int]$RosDomainId = 42,
    [string]$RmwImplementation = "rmw_cyclonedds_cpp",
    [string]$OutputDirectory = "artifacts/ros2_smoke"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

if ($Frames -le 0) { throw "Frames must be positive" }
if ($PublishHz -le 0) { throw "PublishHz must be positive" }

$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$rosPrefix = (Resolve-Path (Join-Path $root ".conda-ros2")).Path
$installPrefix = (Resolve-Path (Join-Path $root "ros2_ws\install")).Path
$dataset = (Resolve-Path (Join-Path $root "data\rgbd_dataset_freiburg1_xyz")).Path
$output = Join-Path $root $OutputDirectory
New-Item -ItemType Directory -Path $output -Force | Out-Null

$metrics = Join-Path $output "ros2_metrics.csv"
$nodeStdout = Join-Path $output "odometry.stdout.log"
$nodeStderr = Join-Path $output "odometry.stderr.log"
$publisherStdout = Join-Path $output "publisher.stdout.log"
$publisherStderr = Join-Path $output "publisher.stderr.log"
$echoStdout = Join-Path $output "odom_once.yaml"
$echoStderr = Join-Path $output "odom_echo.stderr.log"
$topicList = Join-Path $output "topic_list.txt"
$runMetadata = Join-Path $output "run_metadata.txt"
$summaryJson = Join-Path $output "summary.json"
$trajectoryTum = Join-Path $output "trajectory_local.txt"

@($metrics, $nodeStdout, $nodeStderr, $publisherStdout, $publisherStderr,
  $echoStdout, $echoStderr, $topicList, $runMetadata, $summaryJson, $trajectoryTum) | ForEach-Object {
    Remove-Item -LiteralPath $_ -Force -ErrorAction SilentlyContinue
}

# RoboStack's Windows environment must be isolated from incompatible DLLs on
# the user's normal PATH. Keep only the project prefixes and Windows runtime.
$env:PATH = @(
    (Join-Path $installPrefix "Lib\rgbd_odometry_py"),
    $rosPrefix,
    (Join-Path $rosPrefix "Library\bin"),
    (Join-Path $rosPrefix "Scripts"),
    (Join-Path $rosPrefix "bin"),
    (Join-Path $env:WINDIR "System32"),
    $env:WINDIR
) -join ";"
$env:PYTHONPATH = @(
    (Join-Path $installPrefix "Lib\site-packages"),
    (Join-Path $rosPrefix "Lib\site-packages")
) -join ";"
$env:AMENT_PREFIX_PATH = "$installPrefix;$($rosPrefix)\Library"
$env:CMAKE_PREFIX_PATH = $env:AMENT_PREFIX_PATH
$env:COLCON_PREFIX_PATH = $installPrefix
$env:RMW_IMPLEMENTATION = $RmwImplementation
$env:ROS_DOMAIN_ID = [string]$RosDomainId
$env:ROS_DISTRO = "jazzy"
$env:ROS_LOG_DIR = Join-Path $output "ros_logs"
$env:PYTHONNOUSERSITE = "1"
$env:OMP_NUM_THREADS = "1"
$env:OPENBLAS_NUM_THREADS = "1"

$pythonExecutable = Join-Path $rosPrefix "python.exe"
$ros2Script = Join-Path $rosPrefix "Library\bin\ros2-script.py"
$paramsFile = Join-Path $installPrefix "share\rgbd_odometry_ros\config\odometry.yaml"

$started = Get-Date
$node = $null
$publisher = $null
$echo = $null
try {
    $nodeArgs = @(
        "-m", "rgbd_odometry_ros.rgbd_odometry_py_node",
        "--ros-args", "--params-file", $paramsFile,
        "-p", "metrics_csv:=$metrics",
        "-p", "trajectory_tum:=$trajectoryTum"
    )
    $node = Start-Process -FilePath $pythonExecutable -ArgumentList $nodeArgs -PassThru `
        -RedirectStandardOutput $nodeStdout -RedirectStandardError $nodeStderr -WindowStyle Hidden
    Start-Sleep -Seconds 2
    if ($node.HasExited) { throw "Odometry node exited early; inspect $nodeStderr" }

    # Subscribe before publishing so the artifact proves a real odometry message
    # crossed the ROS graph, rather than merely proving that processes started.
    $echo = Start-Process -FilePath $pythonExecutable `
        -ArgumentList @($ros2Script, "topic", "echo", "/odom", "--once") `
        -PassThru -RedirectStandardOutput $echoStdout -RedirectStandardError $echoStderr -WindowStyle Hidden

    $publishHzArgument = $PublishHz.ToString(
        "R", [System.Globalization.CultureInfo]::InvariantCulture
    )
    if (-not $publishHzArgument.Contains(".")) { $publishHzArgument += ".0" }
    $publisherArgs = @(
        "-m", "rgbd_odometry_ros.tum_rgbd_publisher",
        "--ros-args",
        "-p", "dataset:=$dataset",
        "-p", "publish_hz:=$publishHzArgument",
        "-p", "max_frames:=$Frames"
    )
    $publisher = Start-Process -FilePath $pythonExecutable -ArgumentList $publisherArgs -PassThru `
        -RedirectStandardOutput $publisherStdout -RedirectStandardError $publisherStderr -WindowStyle Hidden

    Start-Sleep -Seconds 2
    (& $pythonExecutable $ros2Script topic list) | Set-Content -LiteralPath $topicList -Encoding UTF8

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    $processed = 0
    while ((Get-Date) -lt $deadline) {
        if (Test-Path -LiteralPath $metrics) {
            $processed = [Math]::Max(0, @(Get-Content -LiteralPath $metrics).Count - 1)
        }
        if ($processed -ge $Frames) { break }
        if ($node.HasExited) { throw "Odometry node exited after $processed frames; inspect $nodeStderr" }
        if ($publisher.HasExited) { throw "TUM publisher exited unexpectedly; inspect $publisherStderr" }
        Start-Sleep -Milliseconds 500
    }
    if ($processed -lt $Frames) {
        throw "Timed out: received $processed of $Frames frames in $TimeoutSeconds seconds"
    }
    $trajectoryRows = if (Test-Path -LiteralPath $trajectoryTum) {
        @(Get-Content -LiteralPath $trajectoryTum).Count
    } else { 0 }
    if ($trajectoryRows -ne $Frames) {
        throw "Trajectory contains $trajectoryRows rows; expected $Frames"
    }
    if (-not $echo.WaitForExit(10000)) {
        throw "No /odom message was observed within the timeout"
    }
    # On this RoboStack/Windows build the CLI can return a non-zero shutdown
    # code after --once even though a complete message was emitted. The
    # captured message itself is the authoritative transport evidence.
    if (-not (Test-Path -LiteralPath $echoStdout) -or (Get-Item -LiteralPath $echoStdout).Length -eq 0) {
        throw "No serialized /odom message was captured; inspect $echoStderr"
    }
    $echoExitCode = if ($null -eq $echo.ExitCode) { "unknown" } else { [string]$echo.ExitCode }

    & $pythonExecutable (Join-Path $root "src\summarize_ros2_metrics.py") `
        --input $metrics --output $summaryJson | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "ROS2 metrics summary validation failed" }

    $elapsed = ((Get-Date) - $started).TotalSeconds
    @(
        "status=success",
        "requested_frames=$Frames",
        "processed_frames=$processed",
        "publish_hz=$PublishHz",
        "ros_domain_id=$RosDomainId",
        "rmw_implementation=$env:RMW_IMPLEMENTATION",
        "ros2_topic_echo_exit_code=$echoExitCode",
        "elapsed_wall_s=$elapsed",
        "dataset=$dataset",
        "metrics_csv=$metrics"
        "trajectory_tum=$trajectoryTum"
    ) | Set-Content -LiteralPath $runMetadata -Encoding UTF8
    Write-Output "ROS2 demo completed: $processed/$Frames frames"
    Write-Output "Metrics: $metrics"
    Write-Output "Topic evidence: $topicList"
    Write-Output "Odometry evidence: $echoStdout"
}
finally {
    foreach ($process in @($publisher, $echo, $node)) {
        if ($null -ne $process -and -not $process.HasExited) {
            Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
            $process.WaitForExit(5000) | Out-Null
        }
    }
}
