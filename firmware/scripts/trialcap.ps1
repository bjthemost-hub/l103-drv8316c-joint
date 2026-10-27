param([string]$Trials="2:0.02:0.20",[string]$Vel="1.047",[int]$Ms=6000,[string]$Prefix="work\cw")
$sp = New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
$sp.ReadTimeout = 30
$sp.Open(); $sp.DiscardInBuffer(); Start-Sleep -Milliseconds 200; $sp.DiscardInBuffer()
foreach($t in $Trials.Split(";")){
  if($t -eq ""){ continue }
  $f=$t.Split(":"); $vs=$f[0]; $kp=$f[1]; $ki=$f[2]
  $pb = if($f.Count -ge 4){ $f[3] } else { "" }
  $sp.Write("SD0`r"); Start-Sleep -Milliseconds 100
  $sp.Write("M0`r");  Start-Sleep -Milliseconds 150
  $sp.Write("X`r");   Start-Sleep -Milliseconds 100
  $sp.Write("VS$vs`r"); Start-Sleep -Milliseconds 100
  if($pb -ne ""){ $sp.Write("PB$pb`r"); Start-Sleep -Milliseconds 100 }
  $sp.Write("S$kp,$ki`r"); Start-Sleep -Milliseconds 120
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
  $tag = "vs" + $vs + "_" + ($kp -replace "\.","p") + "_" + ($ki -replace "\.","p")
  if($pb -ne ""){ $tag = $tag + "_pb" + $pb }
  $out = $Prefix + "_" + $tag + ".log"
  [System.IO.File]::WriteAllText((Resolve-Path -LiteralPath ".").Path + "\" + $out, $sb.ToString())
  "vs=$vs kp=$kp ki=$ki pb=$pb  bytes=$($sb.Length) -> $out"
}
$sp.Close()
