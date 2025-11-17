# 현재 스크립트가 있는 폴더로 이동
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $scriptDir

$venvPath     = Join-Path $scriptDir "venv"
$activatePs1  = Join-Path $venvPath "Scripts\Activate.ps1"
$reqPath      = Join-Path $scriptDir "requirements.txt"
$ar4Path      = Join-Path $scriptDir "AR4.py"

# 가상환경이 없으면 생성 + requirements 설치
if (-not (Test-Path $activatePs1)) {
    Write-Host "Create Virtual environment"
    python -m venv $venvPath

    Write-Host "Install Requirements"
    # venv 활성화
    & $activatePs1

    # requirements 설치
    pip install -r $reqPath
}

# 항상 마지막에 AR4 실행
Write-Host "Run AR4"
& $activatePs1
python $ar4Path
