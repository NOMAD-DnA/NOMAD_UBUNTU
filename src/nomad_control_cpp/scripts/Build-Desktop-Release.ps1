param([string]$QtBin='C:\msys64\ucrt64\bin', [string]$ReleaseDate='20261109')
$ErrorActionPreference='Stop'
$project=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$release=Join-Path (Split-Path $project) "output/nomad-qt-$ReleaseDate"
$windows=Join-Path $release 'NOMAD-Qt-Windows'
$linux=Join-Path $release 'NOMAD-Qt-Linux'
$localBin=Join-Path $project 'desktop/bin'
$env:PATH="$QtBin;$env:PATH"
New-Item -ItemType Directory -Force -Path $localBin,$windows,$linux | Out-Null
Copy-Item -LiteralPath (Join-Path $project 'build-desktop/nomad_desktop.exe') -Destination $localBin -Force
Copy-Item -LiteralPath (Join-Path $project 'build-desktop/nomad_desktop.exe') -Destination (Join-Path $localBin 'nomad_simulator.exe') -Force
& (Join-Path $QtBin 'windeployqt.exe') --no-translations --no-opengl-sw --no-system-d3d-compiler (Join-Path $localBin 'nomad_desktop.exe')
if($LASTEXITCODE -ne 0){throw 'windeployqt failed'}
# MinGW와 Qt의 전이 DLL 의존성도 수집합니다. Windows 시스템 DLL은 포함하지 않습니다.
$queue=[Collections.Generic.Queue[string]]::new()
Get-ChildItem -LiteralPath $localBin -Recurse -File | Where-Object {$_.Extension -in '.dll','.exe'} | ForEach-Object {$queue.Enqueue($_.FullName)}
$seen=[Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
while($queue.Count -gt 0){
    $binary=$queue.Dequeue()
    if(!$seen.Add($binary)){continue}
    $imports=& (Join-Path $QtBin 'objdump.exe') -p $binary
    if($LASTEXITCODE -ne 0){throw "Cannot inspect $binary"}
    foreach($line in $imports){
        if($line -match 'DLL Name:\s*(\S+)'){
            $name=$Matches[1];$source=Join-Path $QtBin $name;$target=Join-Path $localBin $name
            if(Test-Path -LiteralPath $source){
                if(!(Test-Path -LiteralPath $target)){Copy-Item -LiteralPath $source -Destination $target}
                $queue.Enqueue($target)
            }
        }
    }
}
foreach($destination in @($windows,$linux)){
    $desktop=Join-Path $destination 'desktop'
    $referenceInclude=Join-Path $destination 'include/nomad_control_cpp'
    New-Item -ItemType Directory -Force -Path $referenceInclude | Out-Null
    Copy-Item -LiteralPath (Join-Path $project 'include/nomad_control_cpp/vehicle_reference.hpp') -Destination $referenceInclude -Force
    New-Item -ItemType Directory -Force -Path $desktop | Out-Null
    foreach($name in @('CMakeLists.txt','main.cpp','simulation.hpp','drive_view.hpp','test_console.cpp','README.ko.txt','start-linux.sh','install-linux-desktop.sh','start-windows.cmd','Simulation.cmd','MOTION_VIEW.ko.txt','STEERING_INPUT.ko.txt')){
        Copy-Item -LiteralPath (Join-Path $project "desktop/$name") -Destination $desktop -Force
    }
    Copy-Item -LiteralPath (Join-Path $project 'AUTONOMY_GUIDE.ko.txt') -Destination $destination -Force
    Copy-Item -LiteralPath (Join-Path $project 'ROS_WSL_VALIDATION.ko.txt') -Destination $destination -Force
    if(Test-Path (Join-Path $project 'OFFLINE_VALIDATION.ko.txt')){
        Copy-Item -LiteralPath (Join-Path $project 'OFFLINE_VALIDATION.ko.txt') -Destination $destination -Force
    }
}
Copy-Item -LiteralPath $localBin -Destination (Join-Path $windows 'desktop') -Recurse -Force
$licenses=Join-Path $windows 'licenses'
New-Item -ItemType Directory -Force -Path $licenses | Out-Null
Copy-Item -Path (Join-Path $QtBin '../share/licenses/*') -Destination $licenses -Recurse -Force
$winScripts=Join-Path $windows 'scripts'
New-Item -ItemType Directory -Force -Path $winScripts | Out-Null
foreach($name in @('Launch-NomadGui.ps1','Nomad.Common.ps1','Setup-NomadGui.ps1','NOMAD_GUI_Settings.cmd')){
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination $winScripts -Force
}
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'connection.example.json') -Destination (Join-Path $winScripts 'connection.json') -Force
Copy-Item -LiteralPath (Join-Path $project 'linux') -Destination $linux -Recurse -Force
$linuxRoot=@'
#!/usr/bin/env bash
set -euo pipefail
dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$dir/desktop/start-linux.sh" "$@"
'@
[IO.File]::WriteAllText((Join-Path $linux 'start.sh'),$linuxRoot.Replace("`r`n","`n")+"`n",[Text.UTF8Encoding]::new($false))
[IO.File]::WriteAllText((Join-Path $windows 'Start-NOMAD.cmd'),"@echo off`r`ncall `"%~dp0desktop\start-windows.cmd`"`r`n",[Text.Encoding]::ASCII)
Copy-Item -LiteralPath (Join-Path $project 'desktop/README.ko.txt') -Destination (Join-Path $linux 'README.ko.txt') -Force
Copy-Item -LiteralPath (Join-Path $project 'desktop/README.ko.txt') -Destination (Join-Path $windows 'README.ko.txt') -Force
Compress-Archive -Path $linux -DestinationPath (Join-Path $release "NOMAD-Qt-Linux-$ReleaseDate.zip") -Force
Compress-Archive -Path $windows -DestinationPath (Join-Path $release "NOMAD-Qt-Windows-$ReleaseDate.zip") -Force
Write-Output "Qt packages: $release"
