param([string]$SettingsFile=(Join-Path $PSScriptRoot 'connection.json'))
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'Nomad.Common.ps1')
Write-Host 'NOMAD 연결 설정 (비밀번호는 저장하지 않습니다)' -ForegroundColor Cyan
$piAddress=Read-Host 'Pi IP 주소 또는 호스트 이름'
$piUser=Read-Host 'Pi 로그인 계정'
$workspace=Read-Host "Pi 작업 폴더 [/home/$piUser/nomad_cpp_ws]"
if(!$workspace){$workspace="/home/$piUser/nomad_cpp_ws"}
$keyPath=Read-Host 'SSH 개인 키 경로 (비밀번호 로그인은 Enter)'
$tempFile=$SettingsFile+'.new'
@{hostName=$piAddress;userName=$piUser;workspace=$workspace;identityFile=$keyPath} | ConvertTo-Json | Set-Content -LiteralPath $tempFile -Encoding UTF8
$null=Read-NomadSettings $tempFile
Move-Item -LiteralPath $tempFile -Destination $SettingsFile -Force
Write-Host '저장 완료. NOMAD GUI 시작을 실행하세요.'
