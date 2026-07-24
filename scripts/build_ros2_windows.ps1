param(
    [switch]$BuildCpp
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$prefix = (Resolve-Path (Join-Path $root ".conda-ros2")).Path
$workspace = Join-Path $root "ros2_ws"

$env:PATH = @(
    (Join-Path $prefix "Library\opt\rviz_ogre_vendor\bin"),
    (Join-Path $prefix "Library\opt\rviz_ogre_vendor\lib"),
    (Join-Path $prefix "Library\opt\rviz_ogre_vendor\lib64"),
    $prefix,
    (Join-Path $prefix "Library\bin"),
    (Join-Path $prefix "Scripts"),
    (Join-Path $prefix "bin"),
    (Join-Path $env:WINDIR "System32"),
    $env:WINDIR
) -join ";"
$env:PYTHONPATH = Join-Path $prefix "Lib\site-packages"
$env:AMENT_PREFIX_PATH = Join-Path $prefix "Library"
$env:CMAKE_PREFIX_PATH = $env:AMENT_PREFIX_PATH
$env:PYTHONNOUSERSITE = "1"
$env:ROS_DISTRO = "jazzy"

$cppOption = if ($BuildCpp) { "ON" } else { "OFF" }
Push-Location $workspace
try {
    & (Join-Path $prefix "Scripts\colcon.exe") build --merge-install `
        --packages-select rgbd_odometry_ros rgbd_odometry_py `
        --cmake-args "-DRGBD_ROS_BUILD_CPP=$cppOption" -DCMAKE_BUILD_TYPE=Release -G Ninja
    if ($LASTEXITCODE -ne 0) { throw "colcon build failed with exit code $LASTEXITCODE" }
}
finally {
    Pop-Location
}

Write-Output "ROS2 packages built successfully (RGBD_ROS_BUILD_CPP=$cppOption)"
