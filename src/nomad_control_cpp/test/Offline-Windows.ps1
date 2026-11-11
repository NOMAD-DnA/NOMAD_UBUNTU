param([string]$QtBin='C:\msys64\ucrt64\bin')
$ErrorActionPreference='Stop'
$project=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$env:PATH="$QtBin;$env:PATH"
foreach($file in (Get-ChildItem (Join-Path $project 'scripts') -Filter *.ps1)){
    $tokens=$null; $parseErrors=$null
    [System.Management.Automation.Language.Parser]::ParseFile($file.FullName,[ref]$tokens,[ref]$parseErrors) | Out-Null
    if($parseErrors.Count){throw ($parseErrors | Out-String)}
}
& "$QtBin/cmake.exe" -S $project -B "$project/build" -DNOMAD_BUILD_ROS2=OFF
if($LASTEXITCODE){throw 'Core configure failed'}
& "$QtBin/cmake.exe" --build "$project/build" -j2
if($LASTEXITCODE){throw 'Core build failed'}
& "$QtBin/ctest.exe" --test-dir "$project/build" --output-on-failure
if($LASTEXITCODE){throw 'Core tests failed'}
& "$QtBin/cmake.exe" -S "$project/desktop" -B "$project/build-desktop" -G Ninja "-DCMAKE_PREFIX_PATH=$(Split-Path $QtBin)" -DCMAKE_BUILD_TYPE=Release
if($LASTEXITCODE){throw 'Qt configure failed'}
& "$QtBin/cmake.exe" --build "$project/build-desktop" -j2
if($LASTEXITCODE){throw 'Qt build failed'}
& "$QtBin/ctest.exe" --test-dir "$project/build-desktop" --output-on-failure
if($LASTEXITCODE){throw 'Qt tests failed'}
