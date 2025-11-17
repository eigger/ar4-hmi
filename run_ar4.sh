#!/usr/bin/env bash

# 스크립트가 있는 폴더로 이동
scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$scriptDir" || exit 1

venvPath="$scriptDir/venv"
activateSh="$venvPath/bin/activate"
reqPath="$scriptDir/requirements.txt"
ar4Path="$scriptDir/AR4.py"

# 가상환경이 없으면 생성 + requirements 설치
if [ ! -f "$activateSh" ]; then
    echo "Create Virtual environment"
    python3 -m venv "$venvPath"

    echo "Install Requirements"
    # venv 활성화
    # shellcheck source=/dev/null
    source "$activateSh"

    # requirements 설치
    pip install -r "$reqPath"
fi

# 항상 마지막에 AR4 실행
echo "Run AR4"
# shellcheck source=/dev/null
source "$activateSh"
python3 "$ar4Path"

