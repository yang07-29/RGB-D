param(
    [string]$Dataset = "data\rgbd_dataset_freiburg1_xyz",
    [string]$Output = "artifacts\cpp_full_voxel_0.05_corr_0.12",
    [int]$MaxFrames = 0,
    [string]$MingwRoot = $env:RGBD_MINGW_ROOT
)

$ErrorActionPreference = "Stop"
if (-not $MingwRoot) {
    $compiler = Get-Command g++.exe -ErrorAction SilentlyContinue
    if ($compiler) {
        $MingwRoot = Split-Path -Parent (Split-Path -Parent $compiler.Source)
    } elseif (Test-Path -LiteralPath "C:\msys64\mingw64\bin\g++.exe") {
        $MingwRoot = "C:\msys64\mingw64"
    } else {
        throw "MinGW toolchain not found. Pass -MingwRoot or set RGBD_MINGW_ROOT."
    }
}
$MingwRoot = (Resolve-Path -LiteralPath $MingwRoot).Path
foreach ($tool in @("gcc.exe", "g++.exe", "ninja.exe")) {
    if (-not (Test-Path -LiteralPath (Join-Path $MingwRoot "bin\$tool"))) {
        throw "Missing $tool under $MingwRoot\bin"
    }
}
$cmakeArgs = @(
    "-S", "cpp",
    "-B", "build\cpp",
    "-G", "Ninja",
    "-DCMAKE_BUILD_TYPE=Release",
    "-DCMAKE_C_COMPILER=$MingwRoot\bin\gcc.exe",
    "-DCMAKE_CXX_COMPILER=$MingwRoot\bin\g++.exe",
    "-DCMAKE_MAKE_PROGRAM=$MingwRoot\bin\ninja.exe",
    "-DCMAKE_PREFIX_PATH=$MingwRoot"
)

cmake @cmakeArgs
cmake --build build\cpp --config Release --parallel
$env:Path = "$MingwRoot\bin;$env:Path"
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
