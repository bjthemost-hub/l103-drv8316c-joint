# 部署 L103 上位机 —— 拷 Qt 运行时 + USBCANx64.dll 到 build\, 让 exe 能直接双击跑。
#
# 为什么不用 windeployqt: 本机这套 Qt 5.14.0 跑 windeployqt 会报
#   "Unable to find the platform plugin."
# 然后一个 DLL 都不拷。所以这里按依赖手工拷 —— 清单是实测出来的
# (2026-10-05: 拷了下面这几个, exe 就能起窗口; 少任何一个都不行)。
#
# 用法:  pwsh -NoProfile -File scripts\deploy.ps1
$ErrorActionPreference = 'Stop'

$Qt      = 'C:\Qt\Qt5.14.0\5.14.0\mingw73_64'
$Mgw     = 'C:\Qt\Qt5.14.0\Tools\mingw730_64'
$Root    = Split-Path -Parent $PSScriptRoot
$Build   = Join-Path $Root 'build'
# USBCANx64.dll 来自 CH9433 EVT 包里的 USB-CAN 适配器驱动。可用环境变量 USBCAN_DLL 指定。
$UsbCan  = if ($env:USBCAN_DLL) { $env:USBCAN_DLL } else { 'C:\path\to\USBCANx64.dll' }

New-Item -ItemType Directory -Force -Path $Build | Out-Null
New-Item -ItemType Directory -Force -Path "$Build\platforms" | Out-Null

$files = @{
    "$Qt\bin\Qt5Core.dll"                       = $Build
    "$Qt\bin\Qt5Gui.dll"                        = $Build
    "$Qt\bin\Qt5Widgets.dll"                    = $Build
    "$Qt\bin\Qt5SerialPort.dll"                 = $Build
    "$Qt\plugins\platforms\qwindows.dll"        = "$Build\platforms"
    "$Mgw\bin\libgcc_s_seh-1.dll"               = $Build
    "$Mgw\bin\libstdc++-6.dll"                  = $Build
    "$Mgw\bin\libwinpthread-1.dll"              = $Build
}
foreach ($src in $files.Keys) {
    if (-not (Test-Path $src)) { throw "缺少 $src" }
    Copy-Item -LiteralPath $src -Destination $files[$src] -Force
}

# USBCANx64.dll: 没有它程序照样跑, 只是 CAN 页枚举不到设备
if (Test-Path $UsbCan) {
    Copy-Item -LiteralPath $UsbCan -Destination $Build -Force
    Write-Host "已拷 USBCANx64.dll"
} else {
    Write-Warning "没找到 $UsbCan —— CAN 页会显示「没找到设备 / USBCANx64.dll」"
}

Write-Host "部署完成 -> $Build"
Write-Host "直接双击 $Build\L103MotorControl.exe 即可 (不需要设 PATH)"
