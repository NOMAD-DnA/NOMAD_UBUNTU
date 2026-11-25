$ErrorActionPreference = 'Stop'
try {
    if (-not [Environment]::Is64BitOperatingSystem -or $env:PROCESSOR_ARCHITECTURE -eq 'ARM64') {
        throw 'This installer supports Windows 10/11 x64 only.'
    }
    $root = Join-Path $env:LOCALAPPDATA 'NOMAD-Spectator'
    New-Item -ItemType Directory -Force -Path $root | Out-Null
    $python = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe'
    if (-not (Test-Path $python)) {
        $launcher = Get-Command py.exe -ErrorAction SilentlyContinue
        if ($launcher) {
            $candidate = & $launcher.Source -3.12 -c 'import sys; print(sys.executable)' 2>$null
            if ($LASTEXITCODE -eq 0 -and $candidate -and (Test-Path $candidate)) { $python = $candidate }
        }
    }
    if (-not (Test-Path $python)) {
        if (-not (Get-Command winget.exe -ErrorAction SilentlyContinue)) {
            throw 'Install Python 3.12 x64 from https://www.python.org/downloads/windows/ and run this installer again.'
        }
        & winget.exe install --id Python.Python.3.12 --exact --source winget --scope user --architecture x64 --accept-source-agreements --accept-package-agreements
        if ($LASTEXITCODE -ne 0) { throw 'Python installation failed. Install Python 3.12 x64 and retry.' }
        $python = Join-Path $env:LOCALAPPDATA 'Programs\Python\Python312\python.exe'
    }
    if (-not (Test-Path $python)) { throw 'Python 3.12 was not found after installation.' }
    $venv = Join-Path $root 'python312'
    if (-not (Test-Path (Join-Path $venv 'Scripts\python.exe'))) {
        & $python -m venv $venv
        if ($LASTEXITCODE -ne 0) { throw 'Could not create the app Python environment.' }
    }
    $appPython = Join-Path $venv 'Scripts\python.exe'
    & $appPython -m pip install --disable-pip-version-check --only-binary=:all: -r (Join-Path $PSScriptRoot 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Package download failed. Check your Internet connection and retry.' }
    $version = (Get-Content (Join-Path $PSScriptRoot 'version.txt') -Raw).Trim()
    if ($version -notmatch '^[a-f0-9]{16}$') { throw 'Invalid package version.' }
    $destination = Join-Path $root ('versions\' + $version)
    New-Item -ItemType Directory -Force -Path $destination | Out-Null
    Copy-Item -Path (Join-Path $PSScriptRoot 'app\*') -Destination $destination -Recurse -Force
    & $appPython -c 'import sys; sys.path.insert(0, sys.argv[1]); import windows_spectator, monitor' (Join-Path $destination 'laptop_monitor')
    if ($LASTEXITCODE -ne 0) { throw 'App dependency check failed.' }
    & $appPython (Join-Path $destination 'laptop_monitor\windows_spectator.py') --check-installation
    if ($LASTEXITCODE -ne 0) { throw 'Installed viewer startup check failed. Shortcuts were not updated.' }
    & $appPython (Join-Path $destination 'laptop_monitor\windows_spectator.py') --init-demo-config
    if ($LASTEXITCODE -ne 0) { throw 'Could not create the demo address settings file.' }
    $demoConfig = Join-Path $root 'demo.ini'
    $shell = New-Object -ComObject WScript.Shell
    $locations = @([Environment]::GetFolderPath('Desktop'), (Join-Path ([Environment]::GetFolderPath('Programs')) 'NOMAD'))
    foreach ($folder in $locations) {
        New-Item -ItemType Directory -Force -Path $folder | Out-Null
        $shortcut = $shell.CreateShortcut((Join-Path $folder 'NOMAD Spectator.lnk'))
        $shortcut.TargetPath = Join-Path $venv 'Scripts\pythonw.exe'
        $shortcut.Arguments = '"' + (Join-Path $destination 'laptop_monitor\windows_spectator.py') + '"'
        $shortcut.WorkingDirectory = $destination
        $shortcut.Description = 'NOMAD read-only spectator for the Ubuntu operator PC'
        $shortcut.Save()
        $demoShortcut = $shell.CreateShortcut((Join-Path $folder 'NOMAD Demo.lnk'))
        $demoShortcut.TargetPath = $shortcut.TargetPath
        $demoShortcut.Arguments = $shortcut.Arguments + ' --demo'
        $demoShortcut.WorkingDirectory = $destination
        $demoShortcut.Description = 'NOMAD demo spectator - auto connect using demo.ini'
        $demoShortcut.Save()
        $settingsShortcut = $shell.CreateShortcut((Join-Path $folder 'NOMAD Demo Settings.lnk'))
        $settingsShortcut.TargetPath = Join-Path $env:WINDIR 'System32\notepad.exe'
        $settingsShortcut.Arguments = '"' + $demoConfig + '"'
        $settingsShortcut.WorkingDirectory = $root
        $settingsShortcut.Description = 'Edit the NOMAD Demo relay address, save, then reopen NOMAD Demo'
        $settingsShortcut.Save()
    }
    Write-Host 'Installed. Open NOMAD Demo to auto-connect using demo.ini.' -ForegroundColor Green
    Write-Host 'Use NOMAD Demo Settings to edit the address. Save, then reopen NOMAD Demo.'
    Write-Host 'NOMAD Spectator is also available for manual address entry.'
    Write-Host 'The shared folder is no longer required to run this installed copy.'
    exit 0
} catch {
    Write-Host ('Installation failed: ' + $_.Exception.Message) -ForegroundColor Red
    exit 1
}
