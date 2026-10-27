# 编译 L103 关节电机调试上位机 (Qt 5.14.0 MinGW 64)
#   用法:  pwsh -NoProfile -File scripts\build.ps1
$ErrorActionPreference = 'Stop'

$Qt     = 'C:\Qt\Qt5.14.0\5.14.0\mingw73_64'
$Mgw    = 'C:\Qt\Qt5.14.0\Tools\mingw730_64'
$Root   = Split-Path -Parent $PSScriptRoot
$Build  = Join-Path $Root 'build'

if (-not (Test-Path "$Qt\bin\qmake.exe")) { throw "找不到 $Qt\bin\qmake.exe" }
if (-not (Test-Path "$Mgw\bin\mingw32-make.exe")) { throw "找不到 $Mgw\bin\mingw32-make.exe" }

$env:PATH = "$Qt\bin;$Mgw\bin;$env:PATH"
New-Item -ItemType Directory -Force -Path $Build | Out-Null

Push-Location $Build
try {
    & "$Qt\bin\qmake.exe" (Join-Path $Root 'L103MotorControl.pro')
    if ($LASTEXITCODE -ne 0) { throw "qmake 失败 ($LASTEXITCODE)" }

    & "$Mgw\bin\mingw32-make.exe" -j4
    if ($LASTEXITCODE -ne 0) { throw "make 失败 ($LASTEXITCODE)" }
} finally {
    Pop-Location
}

Write-Host "编译完成 -> $Build\L103MotorControl.exe"
Write-Host "要能双击运行, 还得先跑一次 scripts\deploy.ps1"