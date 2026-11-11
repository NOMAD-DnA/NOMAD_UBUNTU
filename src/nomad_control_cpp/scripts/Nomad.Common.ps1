# 접속 설정과 런타임 로그는 배포 소스/인증 키에서 분리합니다.
function Read-NomadSettings([string]$Path) {
    $s=Get-Content -LiteralPath $Path -Raw -Encoding UTF8 | ConvertFrom-Json
    if($s.hostName -notmatch '^[a-zA-Z0-9][a-zA-Z0-9.-]*$' -or $s.userName -notmatch '^[a-z_][a-z0-9_-]*$' -or
       $s.workspace -notmatch '^/[a-zA-Z0-9_./-]+$' -or $s.workspace -match '(^|/)\.\.(/|$)'){throw '연결 설정의 주소·계정·작업 폴더를 확인하세요.'}
    if($s.identityFile -and !(Test-Path -LiteralPath $s.identityFile -PathType Leaf)){throw 'SSH 키 파일이 없습니다.'}
    return $s
}
function Quote-NomadArgument([string]$Value) {
    if($Value -match '["\r\n]'){throw '명령 인자에 따옴표나 줄바꿈을 사용할 수 없습니다.'}
    return '"'+$Value+'"'
}
function Get-NomadSshArguments($Settings) {
    $a=@('-o','ConnectTimeout=6','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2')
    if($Settings.identityFile){$a+=@('-i',[string]$Settings.identityFile,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes')}
    return $a
}
function Get-NomadStatus([int]$Port) {
    try{return Invoke-RestMethod "http://127.0.0.1:$Port/api/status" -TimeoutSec 1}catch{return $null}
}
function Test-NomadProcessCreationTime($Actual, $Saved) {
    # PowerShell 7 may deserialize the JSON timestamp as DateTime instead of string.
    try {
        $expected = if($Saved -is [datetime]) { $Saved.ToUniversalTime() } else {
            [datetime]::Parse([string]$Saved, [Globalization.CultureInfo]::InvariantCulture,
                [Globalization.DateTimeStyles]::RoundtripKind).ToUniversalTime()
        }
        return $Actual.ToUniversalTime().Ticks -eq $expected.Ticks
    } catch { return $false }
}
