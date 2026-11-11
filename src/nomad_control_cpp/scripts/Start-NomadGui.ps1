# 기존 실행 경로 호환용. 설정/접속 로직은 한 실행기로 관리합니다.
& (Join-Path $PSScriptRoot "Launch-NomadGui.ps1") @args
exit $LASTEXITCODE
