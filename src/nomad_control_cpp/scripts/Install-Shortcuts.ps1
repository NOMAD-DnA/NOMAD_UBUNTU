$ErrorActionPreference='Stop'
$desktop=[Environment]::GetFolderPath('Desktop')
$shell=New-Object -ComObject WScript.Shell
foreach($item in @(@('NOMAD GUI 시작','NOMAD_GUI_Start.cmd'),@('NOMAD GUI 미리보기','NOMAD_GUI_Preview.cmd'))){
    $shortcut=$shell.CreateShortcut((Join-Path $desktop ($item[0]+'.lnk')))
    $shortcut.TargetPath=Join-Path $PSScriptRoot $item[1]
    $shortcut.WorkingDirectory=$PSScriptRoot
    $shortcut.IconLocation="$env:SystemRoot\System32\shell32.dll,14"
    $shortcut.Description=if($item[1] -like '*Preview*'){'Pi 없이 NOMAD 화면 보기'}else{'Pi 연결 후 NOMAD 차량 조종'}
    $shortcut.Save()
}
Write-Host '바탕화면에 NOMAD GUI 시작 / 미리보기를 만들었습니다.'
