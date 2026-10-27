param([string]$Gains="0.05,0.50",[string]$Vel="1.047",[int]$Ms=6000,[string]$Prefix="work\cw10")
$sp = New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
$sp.ReadTimeout = 30
$sp.Open(); $sp.DiscardInBuffer(); Start-Sleep -Milliseconds 200; $sp.DiscardInBuffer()
foreach($g in $Gains.Split(";")){
  if($g -eq ""){ continue }
  $sp.Write("SD0`r"); Start-Sleep -Milliseconds 100
  $sp.Write("M0`r");  Start-Sleep -Milliseconds 150
  $sp.Write("X`r");   Start-Sleep -Milliseconds 100
  $sp.Write("S$g`r"); Start-Sleep -Milliseconds 120
  $sp.Write("V$Vel`r"); Start-Sleep -Milliseconds 120
  $null = $sp.ReadExisting()
  $sp.Write("M3`r");  Start-Sleep -Milliseconds 120
  $sp.Write("SD1`r"); Start-Sleep -Milliseconds 120
  $null = $sp.ReadExisting()
  $sb = New-Object System.Text.StringBuilder
  $end=(Get-Date).AddMilliseconds($Ms)
  while((Get-Date) -lt $end){
    $n = $sp.BytesToRead
    if($n -gt 0){ $buf = New-Object byte[] $n; $null = $sp.Read($buf,0,$n); [void]$sb.Append([System.Text.Encoding]::ASCII.GetString($buf)) }
    else { Start-Sleep -Milliseconds 5 }
  }
  $sp.Write("SD0`r"); Start-Sleep -Milliseconds 150
  $sp.Write("M0`r");  Start-Sleep -Milliseconds 250
  $null = $sp.ReadExisting()
  $out = $Prefix + "_" + ($g -replace ",","_") + ".log"
  [System.IO.File]::WriteAllText((Resolve-Path -LiteralPath ".").Path + "\" + $out, $sb.ToString())
  "gains=$g  bytes=$($sb.Length) -> $out"
}
$sp.Close()
