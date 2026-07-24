param(
    [string]$Dataset = "data\rgbd_dataset_freiburg1_xyz",
    [string]$Output = "artifacts\cpp_full_voxel_0.05_corr_0.12",
    [int]$MaxFrames = 0
)

$ErrorActionPreference = "Stop"
$mingw = "C:\msys64\mingw64"
$cmakeArgs = @(
    "-S", "cpp",
    "-B", "build\cpp",
    "-G", "Ninja",
    "-DCMAKE_BUILD_TYPE=Release",
    "-DCMAKE_C_COMPILER=$mingw\bin\gcc.exe",
    "-DCMAKE_CXX_COMPILER=$mingw\bin\g++.exe",
    "-DCMAKE_MAKE_PROGRAM=$mingw\bin\ninja.exe",
    "-DCMAKE_PREFIX_PATH=$mingw"
)

cmake @cmakeArgs
cmake --build build\cpp --config Release --parallel
$env:Path = "$mingw\bin;$env:Path"
ctest --test-dir build\cpp --output-on-failure

$runArgs = @(
    "--dataset", $Dataset,
    "--output", $Output,
    "--voxel", "0.05",
    "--max-correspondence", "0.12"
)
if ($MaxFrames -gt 0) {
    $runArgs += @("--max-frames", $MaxFrames)
}
& ".\build\cpp\rgbd_odometry.exe" @runArgs
