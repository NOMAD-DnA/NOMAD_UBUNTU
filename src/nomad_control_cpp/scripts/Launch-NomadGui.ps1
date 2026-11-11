param([ValidateSet('Control','View','Preview')][string]$Mode='Control',
    [string]$SettingsFile=(Join-Path $PSScriptRoot 'connection.json'),[switch]$NoBrowser,[switch]$CheckOnly)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'Nomad.Common.ps1')

# 이미 열린 연결에서도 기록기를 준비합니다. 인증을 다시 묻거나 조종 명령을 보내지 않습니다.
function Start-NomadRecording($Settings, $SshArguments, [string]$StateDirectory) {
    $previousExitCode=$LASTEXITCODE
    $recorder="$($Settings.workspace)/diagnostics/start_recording.sh"
    $remote="if test -f '$recorder'; then timeout 3s bash '$recorder'; else echo 'Pi recording helper is not installed.' >&2; exit 4; fi"
    $arguments=@('-n','-T','-o','BatchMode=yes','-o','NumberOfPasswordPrompts=0',
        '-o','ConnectTimeout=3','-o','ConnectionAttempts=1','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=1')+
        @($SshArguments)+@("$($Settings.userName)@$($Settings.hostName)",$remote)
    try {
        & ssh.exe @arguments *> (Join-Path $StateDirectory 'recording-start.log')
        if($LASTEXITCODE -ne 0){Write-Warning '시험 자동 기록을 시작하지 못했습니다. recording-start.log를 확인하세요.'}
    } catch {
        Write-Warning '시험 자동 기록을 시작하지 못했습니다. Pi 기록기 설치와 SSH 연결을 확인하세요.'
    } finally {
        $global:LASTEXITCODE=$previousExitCode
    }
}
try {
    if($Mode -eq 'Preview') {
        $exe=Join-Path $PSScriptRoot '../bin/nomad_gui_preview.exe'
        if(!(Test-Path -LiteralPath $exe)){$exe=Join-Path $PSScriptRoot '../build/nomad_gui_preview.exe'}
        $page=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../web/index.html'))
        if(!(Test-Path -LiteralPath $exe)){throw '미리보기 실행 파일이 없습니다.'}
        if($CheckOnly){Write-Output 'Preview configuration OK';exit 0}
        $preview=Get-NomadStatus 8766
        if(!$preview){
            Start-Process -FilePath $exe -ArgumentList @((Quote-NomadArgument $page),'8766') -WindowStyle Hidden | Out-Null
            for($i=0;$i -lt 20;$i++){$preview=Get-NomadStatus 8766;if($preview){break};Start-Sleep -Milliseconds 100}
        }
        if(!$preview -or $preview.preview -ne $true){throw '8766 포트의 미리보기를 확인할 수 없습니다.'}
        if(!$NoBrowser){Start-Process 'http://127.0.0.1:8766/'}
        exit 0
    }
    if(!(Test-Path -LiteralPath $SettingsFile)){
        if($CheckOnly){throw 'connection.json이 없습니다. 연결 설정을 실행하세요.'}
        & (Join-Path $PSScriptRoot 'Setup-NomadGui.ps1') -SettingsFile $SettingsFile
    }
    $settings=Read-NomadSettings $SettingsFile
    $sshArgs=@(Get-NomadSshArguments $settings)
    if($CheckOnly){Write-Output 'Connection configuration OK (no network or vehicle command)';exit 0}
    Write-Host 'NOMAD GUI 연결 중' -ForegroundColor Cyan
    $stateDirectory=Join-Path $env:LOCALAPPDATA 'NOMAD-Control'
    New-Item -ItemType Directory -Path $stateDirectory -Force | Out-Null
    $pidFile=Join-Path $stateDirectory 'tunnel.json'
    $existing=Get-NomadStatus 8765
    $listeners=@(Get-NetTCPConnection -State Listen -LocalPort 8765 -ErrorAction SilentlyContinue)
    if($listeners.Count -gt 0){
        $owned=$false
        if(Test-Path -LiteralPath $pidFile){
            $saved=Get-Content -LiteralPath $pidFile -Raw | ConvertFrom-Json
            $process=Get-CimInstance Win32_Process -Filter "ProcessId=$([int]$saved.pid)" -ErrorAction SilentlyContinue
            $owned=$process -and $process.Name -eq 'ssh.exe' -and (Test-NomadProcessCreationTime $process.CreationDate $saved.created) -and
                $saved.hostName -eq $settings.hostName -and $saved.userName -eq $settings.userName -and $saved.workspace -eq $settings.workspace -and
                @($listeners | Where-Object OwningProcess -ne $process.ProcessId).Count -eq 0
        }
        # Recover a legitimate tunnel even when its saved PID metadata is absent/stale.
        # Check the actual listener, exact SSH destination, forward and service path.
        if(!$owned){
            $owners=@($listeners | Select-Object -ExpandProperty OwningProcess -Unique)
            if($owners.Count -eq 1){
                $listenerProcess=Get-CimInstance Win32_Process -Filter "ProcessId=$($owners[0])" -ErrorAction SilentlyContinue
                $line=[string]$listenerProcess.CommandLine
                $destination=[regex]::Escape("$($settings.userName)@$($settings.hostName)")
                $service=[regex]::Escape("$($settings.workspace)/commission_session/ensure_gui.sh")
                $owned=$listenerProcess -and $listenerProcess.Name -eq 'ssh.exe' -and
                    $line -match ('(?:^|[\s"]){0}(?=[\s"]|$)' -f $destination) -and
                    $line -match '(?:^|[\s"])127\.0\.0\.1:8765:127\.0\.0\.1:8765(?=[\s"]|$)' -and
                    $line -match ('(?:^|[\s"]){0}(?=[\s"]|$)' -f $service)
                if($owned){
                    @{pid=$listenerProcess.ProcessId;created=$listenerProcess.CreationDate.ToUniversalTime().ToString('o');hostName=$settings.hostName;userName=$settings.userName;workspace=$settings.workspace} |
                        ConvertTo-Json | Set-Content -LiteralPath $pidFile -Encoding UTF8
                }
            }
        }
        @{time=[datetime]::UtcNow.ToString('o');version=$PSVersionTable.PSVersion.ToString();script=$PSCommandPath;owned=$owned;listeners=@($listeners | Select-Object LocalAddress,OwningProcess)} |
            ConvertTo-Json -Depth 4 | Set-Content (Join-Path $stateDirectory 'last-launch-check.json') -Encoding UTF8
        if($owned){
            Start-NomadRecording $settings $sshArgs $stateDirectory
            # A single Wi-Fi status timeout does not mean another app owns the port.
            for($attempt=0;$attempt -lt 4;$attempt++){
                if($existing -and $existing.connected -and $existing.telemetry_fresh -and !$existing.preview -and
                   $existing.commissioning_setup_acknowledged -eq ($Mode -eq 'Control')){
                    if(!$NoBrowser){Start-Process 'http://127.0.0.1:8765/'}
                    Write-Host '기존 연결을 열었습니다.';exit 0
                }
                if($attempt -lt 3){Start-Sleep -Milliseconds 300;$existing=Get-NomadStatus 8765}
            }
            # The GUI must remain accessible during a transient outage. Its own
            # freshness/arming checks keep START disabled until the controller is ready.
            Write-Host '기존 SSH 연결로 GUI를 엽니다. 연결 상태가 확인될 때까지 조종은 잠깁니다.' -ForegroundColor Yellow
            if(!$NoBrowser){Start-Process 'http://127.0.0.1:8765/'}
            exit 0
        }
        throw '8765 포트가 사용 중입니다. 기존 GUI SSH 연결을 종료한 뒤 다시 실행하세요.'
    }
    $tcp=[Net.Sockets.TcpClient]::new()
    try {
        $connect=$tcp.ConnectAsync([string]$settings.hostName,22)
        if(!$connect.Wait(1800) -or !$tcp.Connected){throw 'Pi가 응답하지 않습니다. 전원과 Wi-Fi를 확인하세요.'}
    } finally {$tcp.Dispose()}
    $readyValue=if($Mode -eq 'Control'){'YES'}else{'NO'}
    # 한 SSH 연결에서 서비스 준비와 터널을 함께 유지합니다. START는 GUI에서만 요청합니다.
    # 두 번째 환경 변수는 기존 Pi 시작 스크립트와의 호환용입니다.
    $remote="NOMAD_CONTROL_READY=$readyValue NOMAD_LIFTED_TEST=$readyValue bash $($settings.workspace)/commission_session/ensure_gui.sh && exec sleep infinity"
    $arguments=@($sshArgs)+@('-o','ExitOnForwardFailure=yes','-L','127.0.0.1:8765:127.0.0.1:8765',"$($settings.userName)@$($settings.hostName)",$remote)
    $parameters=@{FilePath='ssh.exe';ArgumentList=(($arguments | ForEach-Object {Quote-NomadArgument $_}) -join ' ');PassThru=$true}
    if($settings.identityFile){
        $parameters.WindowStyle='Hidden'
        $parameters.RedirectStandardOutput=Join-Path $stateDirectory 'connection.log'
        $parameters.RedirectStandardError=Join-Path $stateDirectory 'connection-error.log'
    } else {
        # 사용자 비밀번호/최초 호스트 확인이 필요한 SSH 창만 표시합니다.
        $parameters.WindowStyle='Normal'
        Write-Host 'SSH 창에서 로그인하세요. 연결 창을 닫으면 차량 연결도 종료됩니다.'
    }
    $tunnel=Start-Process @parameters
    $process=Get-CimInstance Win32_Process -Filter "ProcessId=$($tunnel.Id)"
    if(!$process){throw 'SSH 실행이 종료됐습니다. 연결 로그를 확인하세요.'}
    @{pid=$tunnel.Id;created=$process.CreationDate.ToUniversalTime().ToString('o');hostName=$settings.hostName;userName=$settings.userName;workspace=$settings.workspace} |
        ConvertTo-Json | Set-Content -LiteralPath $pidFile -Encoding UTF8
    $ready=$false
    for($i=0;$i -lt 90;$i++){
        $tunnel.Refresh();if($tunnel.HasExited){break}
        $status=Get-NomadStatus 8765
        if($status -and $status.connected -and $status.telemetry_fresh -and !$status.preview -and
           $status.commissioning_setup_acknowledged -eq ($Mode -eq 'Control')){$ready=$true;break}
        Start-Sleep -Milliseconds 400
    }
    if(!$ready){throw "Pi 연결 준비가 완료되지 않았습니다. SSH 로그인과 VESC 전원을 확인하세요. 로그: $stateDirectory"}
    Start-NomadRecording $settings $sshArgs $stateDirectory
    if($Mode -eq 'View') {
        Write-Host '연결 완료 · 관찰 전용 · 조종 잠김' -ForegroundColor Green
    } else {
        Write-Host '연결 완료 · GUI에서 수동 제어 시작을 누르세요.' -ForegroundColor Green
    }
    if(!$NoBrowser){Start-Process 'http://127.0.0.1:8765/'}
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    Write-Host 'Pi 없이 화면을 보려면 NOMAD GUI 미리보기를 실행하세요.'
    if(!$NoBrowser -and !$CheckOnly){[void](Read-Host 'Enter로 닫기')}
    exit 1
}
