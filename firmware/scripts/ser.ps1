param([string]$Cmds = "", [int]$Ms = 1500, [switch]$Reset, [int]$KeyMs = 150)
$sp = New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
$sp.ReadTimeout = 50
$sp.Open()
$sp.DiscardInBuffer()
if ($Reset) {
  $ocd = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin/openocd.exe"
  $scr = "c:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/bin"
  Start-Process -FilePath $ocd -WindowStyle Hidden -Wait -ArgumentList @("-f","debug/openocd.cfg","-s",$scr,"-c","init","-c","reset","-c","exit") | Out-Null
  Start-Sleep -Milliseconds 400
}
if ($Cmds -ne "") {
  foreach ($c in $Cmds.Split(";")) {
    if ($c -ne "") { $sp.Write($c + "`r"); Start-Sleep -Milliseconds $KeyMs }
  }
}
$end = (Get-Date).AddMilliseconds($Ms)
$sb = New-Object System.Text.StringBuilder
while ((Get-Date) -lt $end) {
  try {
    $n = $sp.BytesToRead
    if ($n -gt 0) {
      $buf = New-Object byte[] $n
      $null = $sp.Read($buf, 0, $n)
      [void]$sb.Append([System.Text.Encoding]::ASCII.GetString($buf))
    } else { Start-Sleep -Milliseconds 20 }
  } catch { Start-Sleep -Milliseconds 20 }
}
$sp.Close()
$sb.ToString()
