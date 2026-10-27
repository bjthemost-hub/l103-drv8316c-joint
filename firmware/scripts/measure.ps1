# 一次测量 = 设速度 -> 进速度环 -> SD1 原始角直出 -> 截 N ms -> 停
# 用法:
#   powershell -ExecutionPolicy Bypass -File scripts\measure.ps1 -Vel 1.047 -Ms 5000 -Out work\m10.log
#   python scripts\scorecap.py work\m10.log
param(
  [double]$Vel = 1.047,      # 给定速度 rad/s
  [int]$Ms = 5000,           # 采集时长 ms
  [string]$Out = "work\m.log",
  [string]$Port = "COM15",
  [int]$Baud = 115200,
  [string]$Pre = "",         # 采集前补发命令, 分号分隔, 如 "PB150;S0.05,4.0"
  [int]$SettleMs = 900,      # 进 M3 后等多久再开采
  [switch]$Keep              # 测完不自动 M0
)
$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,"None",8,"One"
$sp.ReadTimeout = 50
$sp.Open(); $sp.DiscardInBuffer(); Start-Sleep -Milliseconds 200; $sp.DiscardInBuffer()
$echo = New-Object System.Text.StringBuilder
function Send([string]$c, [int]$wait = 150) {
  $sp.Write($c + "`r"); Start-Sleep -Milliseconds $wait
  [void]$echo.Append($sp.ReadExisting())
}
if ($Pre -ne "") { foreach ($c in $Pre.Split(";")) { if ($c -ne "") { Send $c } } }
Send "SD0"
Send ("V" + $Vel.ToString("0.####"))
Send "M3" $SettleMs
Send "SD1" 150
# 采集期的数据行在打印时按 OK/ERR 过滤掉
$sb = New-Object System.Text.StringBuilder
$end = (Get-Date).AddMilliseconds($Ms)
while ((Get-Date) -lt $end) {
  $n = $sp.BytesToRead
  if ($n -gt 0) {
    $b = New-Object byte[] $n
    $null = $sp.Read($b, 0, $n)
    [void]$sb.Append([Text.Encoding]::ASCII.GetString($b))
  } else { Start-Sleep -Milliseconds 5 }
}
Send "SD0" 150
if (-not $Keep) { Send "M0" 200 }
$sp.Close()
[IO.File]::WriteAllText((Join-Path (Resolve-Path ".").Path $Out), $sb.ToString())
$lines = ($sb.ToString() -split "`n" | Where-Object { $_ -match "^[0-9A-Fa-f]+,[0-9A-Fa-f]+" }).Count
"--- 回显 ---"
(($echo.ToString() -split "`n") | Where-Object { $_ -match "^(OK|ERR|FAULT)" }) -join "`n"
"--- $Out : $($sb.Length) bytes, $lines 样本 ($([math]::Round($lines*0.002,2)) s @500Hz) ---"
