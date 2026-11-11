param([string]$OutputDirectory=(Join-Path $PSScriptRoot '../../output/nomad-gui-release-20261107'))
$ErrorActionPreference='Stop'
$project=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$output=[IO.Path]::GetFullPath($OutputDirectory)
$package=Join-Path $output 'NOMAD-GUI-Windows'
if(Test-Path -LiteralPath $package){throw '출력 폴더가 이미 존재합니다. 다른 OutputDirectory를 지정하세요.'}
$exe=Join-Path $project 'build-release/nomad_gui_preview.exe'
if(!(Test-Path -LiteralPath $exe)){throw '먼저 Windows Release 빌드를 완료하세요.'}
New-Item -ItemType Directory -Force -Path $package,(Join-Path $package 'bin'),(Join-Path $package 'web'),(Join-Path $package 'scripts'),(Join-Path $package 'licenses') | Out-Null
Copy-Item -LiteralPath $exe -Destination (Join-Path $package 'bin/nomad_gui_preview.exe')
Copy-Item -LiteralPath (Join-Path $project 'web/index.html') -Destination (Join-Path $package 'web/index.html')
$scripts=@('Launch-NomadGui.ps1','Nomad.Common.ps1','Setup-NomadGui.ps1','Install-Shortcuts.ps1','connection.example.json',
    'NOMAD_GUI_Start.cmd','NOMAD_GUI_Preview.cmd','NOMAD_GUI_Settings.cmd','Install-Shortcuts.cmd')
foreach($name in $scripts){Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $package "scripts/$name")}
foreach($name in @('RELEASE.ko.txt','PI_UPDATE.ko.txt')){Copy-Item -LiteralPath (Join-Path $project $name) -Destination $package}
foreach($name in @('httplib.LICENSE','json.LICENSE','SOURCES.txt')){Copy-Item -LiteralPath (Join-Path $project "third_party/$name") -Destination (Join-Path $package 'licenses')}
foreach($name in @('gcc-libs','crt','headers','winpthreads','libwinpthread')){
    $license="C:/msys64/ucrt64/share/licenses/$name"
    if(Test-Path -LiteralPath $license){Copy-Item -LiteralPath $license -Destination (Join-Path $package 'licenses') -Recurse}
}
$commands=@{'NOMAD GUI 시작.cmd'='NOMAD_GUI_Start.cmd';'NOMAD GUI 미리보기.cmd'='NOMAD_GUI_Preview.cmd';'연결 설정.cmd'='NOMAD_GUI_Settings.cmd';'바탕화면 바로가기 만들기.cmd'='Install-Shortcuts.cmd'}
foreach($item in $commands.GetEnumerator()){
    [IO.File]::WriteAllText((Join-Path $package $item.Key),"@echo off`r`ncall `"%~dp0scripts\$($item.Value)`"`r`n",[Text.Encoding]::ASCII)
}
# 소스/빌드 입력만 포함하고 PC별 접속 설정, 키, 로그, 측정 기록은 제외합니다.
$sourceStage=Join-Path $output 'source/nomad_control_cpp'
New-Item -ItemType Directory -Force -Path $sourceStage,(Join-Path $sourceStage 'scripts') | Out-Null
foreach($name in @('src','include','config','srv','web','test','third_party')){
    Copy-Item -LiteralPath (Join-Path $project $name) -Destination $sourceStage -Recurse
}
foreach($name in @('CMakeLists.txt','package.xml','GUIDE.ko.txt','ROS_TRIAL_GUIDE.ko.txt','GUI_GUIDE.ko.txt','GUI_START.ko.txt','RAW_COMMISSION_GUIDE.ko.txt','STABILITY_GUIDE.ko.txt','STEERING_CALIBRATION.ko.txt','RELEASE.ko.txt','PI_UPDATE.ko.txt')){
    Copy-Item -LiteralPath (Join-Path $project $name) -Destination $sourceStage
}
foreach($name in $scripts){Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $sourceStage 'scripts')}
foreach($name in @('ros_env.sh','start_gui.sh','ensure_gui.sh','run_controller_ready.sh')){Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $sourceStage 'scripts')}
& tar -czf (Join-Path $package 'nomad_control_cpp-source.tar.gz') -C (Join-Path $output 'source') nomad_control_cpp
if($LASTEXITCODE -ne 0){throw '소스 압축 실패'}
$hashes=Get-ChildItem -LiteralPath $package -File -Recurse | Sort-Object FullName | ForEach-Object {
    (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLower()+'  '+$_.FullName.Substring($package.Length+1).Replace('\','/')
}
[IO.File]::WriteAllLines((Join-Path $package 'SHA256SUMS.txt'),$hashes,[Text.UTF8Encoding]::new($false))
$zip=Join-Path $output 'NOMAD-GUI-Windows-20261107.zip'
Compress-Archive -LiteralPath $package -DestinationPath $zip -CompressionLevel Optimal
Write-Output $zip
