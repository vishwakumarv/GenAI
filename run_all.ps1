# Windows PowerShell, from the project folder with .venv active:
#   .\run_all.ps1            tests + evaluation (uses the trained models\ppo.zip)
#   .\run_all.ps1 -Train     also retrain the RL agent first (~15-30 min)
param([switch]$Train)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
python -m pytest -q tests
if ($Train) { python src/train_rl.py }
python src/evaluate.py
