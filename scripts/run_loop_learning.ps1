$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

& ".\.conda-learning\python.exe" -m src.train_loop_descriptor `
    --train-dataset data/rgbd_dataset_freiburg1_desk `
    --validation-dataset data/rgbd_dataset_freiburg1_desk2 `
    --test-dataset data/rgbd_dataset_freiburg1_xyz `
    --output artifacts/loop_learning `
    --epochs 5 `
    --batch-size 32 `
    --device cuda
