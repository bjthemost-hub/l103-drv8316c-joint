param([string]$Pre="M0;S0.02,0.20;V1.047",[int]$Ms=5000,[string]$Out="work\cap.log")
$sp = New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
$sp.ReadTimeout = 30
$sp.Open(); $sp.DiscardInBuffer(); Start-Sleep -Milliseconds 200; $sp.DiscardInBuffer()
foreach($c in $Pre.Split(";")){ if($c -ne ""){ $sp.Write($c+"`r"); Start-Sleep -Milliseconds 150 } }
$null = $sp.ReadExisting()
$sb = New-Object System.Text.StringBuilder
$end=(Get-Date).AddMilliseconds($Ms)
while((Get-Date) -lt $end){
  $n = $sp.BytesToRead
  if($n -gt 0){
    $buf = New-Object byte[] $n
    $null = $sp.Read($buf,0,$n)
    [void]$sb.Append([System.Text.Encoding]::ASCII.GetString($buf))
  } else { Start-Sleep -Milliseconds 5 }
}
$sp.Write("SD0`r"); Start-Sleep -Milliseconds 200
$null = $sp.ReadExisting()
$sp.Write("M0`r"); Start-Sleep -Milliseconds 200
$sp.Close()
[System.IO.File]::WriteAllText((Resolve-Path -LiteralPath ".").Path + "\" + $Out, $sb.ToString())
"bytes: " + $sb.Length
