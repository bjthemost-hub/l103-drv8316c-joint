param([int]$Trials=12,[string]$Prefix="work\rnd")
$speeds = @(0.400,0.700,1.047,2.000,3.142,6.283,10.472,20.944,31.416,52.360,73.304)
$sp = New-Object System.IO.Ports.SerialPort "COM15",115200,"None",8,"One"
$sp.ReadTimeout = 30
$sp.Open(); $sp.DiscardInBuffer(); Start-Sleep -Milliseconds 200; $sp.DiscardInBuffer()
for($i=1; $i -le $Trials; $i++){
  $v = $speeds | Get-Random
  $ms = Get-Random -Minimum 1500 -Maximum 6000
  $sp.Write("SD0`r"); Start-Sleep -Milliseconds 80
  $sp.Write(("V" + $v.ToString("0.###") + "`r")); Start-Sleep -Milliseconds 100
  $sp.Write("M3`r"); Start-Sleep -Milliseconds 900
  $null=$sp.ReadExisting()
  $sp.Write("SD1`r"); Start-Sleep -Milliseconds 120; $null=$sp.ReadExisting()
  $sb = New-Object System.Text.StringBuilder
  $end=(Get-Date).AddMilliseconds($ms)
  while((Get-Date) -lt $end){
    $n=$sp.BytesToRead
    if($n -gt 0){ $b=New-Object byte[] $n; $null=$sp.Read($b,0,$n); [void]$sb.Append([Text.Encoding]::ASCII.GetString($b)) }
    else { Start-Sleep -Milliseconds 5 }
  }
  $sp.Write("SD0`r"); Start-Sleep -Milliseconds 120; $null=$sp.ReadExisting()
  $out = $Prefix + "_" + $i.ToString("00") + "_v" + $v.ToString("0.###") + "_" + $ms + "ms.log"
  [IO.File]::WriteAllText((Resolve-Path ".").Path + "\" + $out, $sb.ToString())
  "{0,-46} cmd {1,7:0.000} rad/s ({2,6:0.0} rpm), {3} ms" -f $out, $v, ($v*60/(2*[math]::PI)), $ms
}
$sp.Close()
