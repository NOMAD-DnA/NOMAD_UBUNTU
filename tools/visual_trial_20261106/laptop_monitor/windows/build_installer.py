"""Build one self-contained Windows CMD installer from the shared GUI source."""
import base64
import hashlib
import io
from pathlib import Path
import zipfile


def build():
    here = Path(__file__).resolve().parent
    gui = here.parent
    application = gui.parent
    entries = {}
    # Only source and display settings. Never include recordings or credentials.
    for source in sorted(gui.glob('*.py')):
        entries['app/laptop_monitor/' + source.name] = source.read_bytes()
    for folder in ('indoor_config', 'mapper2_config', 'mapper3_config'):
        for source in sorted((gui / folder).glob('*.json')):
            entries['app/laptop_monitor/' + folder + '/' + source.name] = source.read_bytes()
    entries['app/config/oak_basalt_calib.json'] = (application / 'config/oak_basalt_calib.json').read_bytes()
    for name in ('install.ps1', 'requirements.txt', 'README.txt'):
        entries[name] = (here / name).read_bytes()
    digest = hashlib.sha256()
    for name, data in sorted(entries.items()):
        digest.update(name.encode() + b'\0' + data)
    version = digest.hexdigest()[:16]
    entries['version.txt'] = version.encode()
    payload = io.BytesIO()
    with zipfile.ZipFile(payload, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name, data in sorted(entries.items()):
            item = zipfile.ZipInfo(name, date_time=(2026, 11, 25, 0, 0, 0))
            item.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(item, data)
    encoded = base64.b64encode(payload.getvalue()).decode('ascii')
    bootstrap = r'''@echo off
setlocal
set "NOMAD_SETUP_FILE=%~f0"
echo NOMAD Windows Spectator Setup - Windows 10/11 x64
echo Internet is required for the first installation.
powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Stop'; $stage=Join-Path ([IO.Path]::GetTempPath()) ('nomad-setup-'+[guid]::NewGuid().ToString('N')); $code=1; try { New-Item -ItemType Directory -Path $stage | Out-Null; $text=[IO.File]::ReadAllText($env:NOMAD_SETUP_FILE); $parts=$text -split '\r?\n:NOMAD_PAYLOAD\r?\n',2; if($parts.Count -ne 2){throw 'Invalid installer payload'}; $zip=Join-Path $stage 'payload.zip'; [IO.File]::WriteAllBytes($zip,[Convert]::FromBase64String($parts[1])); Expand-Archive -LiteralPath $zip -DestinationPath (Join-Path $stage 'payload'); $job=Start-Process -FilePath (Join-Path $PSHOME 'powershell.exe') -ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass','-File',([char]34+(Join-Path $stage 'payload\install.ps1')+[char]34)) -NoNewWindow -Wait -PassThru; $code=$job.ExitCode } catch { Write-Host $_ -ForegroundColor Red } finally { if(Test-Path $stage){Remove-Item -LiteralPath $stage -Recurse -Force} }; exit $code"
set "NOMAD_SETUP_RESULT=%ERRORLEVEL%"
if not "%NOMAD_SETUP_RESULT%"=="0" echo Installation failed. Read the message above.
pause
exit /b %NOMAD_SETUP_RESULT%
:NOMAD_PAYLOAD
'''
    body = bootstrap.replace('\n', '\r\n') + '\r\n'.join(encoded[i:i+120] for i in range(0, len(encoded), 120)) + '\r\n'
    destination = application.parent / 'NOMAD_Windows_Spectator_Setup.cmd'
    destination.write_bytes(body.encode('ascii'))
    (application.parent / 'NOMAD_Windows_관전_설치안내.txt').write_bytes((here / 'README.txt').read_bytes())
    print(destination)
    print('version=' + version)
    print('sha256=' + hashlib.sha256(destination.read_bytes()).hexdigest())
    print('payload_files=' + str(len(entries)))


if __name__ == '__main__':
    build()
