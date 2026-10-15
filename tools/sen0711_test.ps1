# Reads the SEN0711 (NH3 / pH / water temp) through a USB-RS485 adapter.
# Usage: powershell -ExecutionPolicy Bypass -File sen0711_test.ps1 -Port COM10
param([Parameter(Mandatory)][string]$Port, [int]$Baud = 4800, [int]$SlaveId = 1)

if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $Port) {
  $found = ([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object) -join ', '
  if (-not $found) { $found = 'none' }
  Write-Host "$Port not found. Plug in the USB-RS485 adapter (or fix the COM number in the .bat). Ports seen now: $found" -ForegroundColor Red
  exit 1
}
function Crc16([byte[]]$d, [int]$n) {
  $crc = 0xFFFF
  for ($i = 0; $i -lt $n; $i++) {
    $crc = $crc -bxor $d[$i]
    for ($b = 0; $b -lt 8; $b++) {
      if ($crc -band 1) { $crc = ($crc -shr 1) -bxor 0xA001 } else { $crc = $crc -shr 1 }
    }
  }
  return $crc
}

$req = [byte[]]@($SlaveId, 0x03, 0x00, 0x00, 0x00, 0x03, 0, 0)
$c = Crc16 $req 6
$req[6] = $c -band 0xFF; $req[7] = $c -shr 8

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 1000
$sp.Open()
try {
  for ($try = 1; $try -le 5; $try++) {
    $sp.DiscardInBuffer()
    $sp.Write($req, 0, 8)
    Start-Sleep -Milliseconds 500
    $n = $sp.BytesToRead
    $buf = New-Object byte[] $n
    if ($n) { [void]$sp.Read($buf, 0, $n) }
    $hex = ($buf | ForEach-Object { $_.ToString('X2') }) -join ' '
    Write-Host ("Try {0}: sent {1} -> got {2} byte(s): {3}" -f $try, (($req | % { $_.ToString('X2') }) -join ' '), $n, $hex)

    for ($i = 0; $i + 11 -le $n; $i++) {
      if ($buf[$i] -eq $SlaveId -and $buf[$i+1] -eq 3 -and $buf[$i+2] -eq 6) {
        $f = [int[]]$buf[$i..($i+10)]
        if ((Crc16 $f 9) -eq ($f[9] -bor ($f[10] -shl 8))) {
          $nh3 = (($f[3] -shl 8) -bor $f[4]) / 100
          $ph  = (($f[5] -shl 8) -bor $f[6]) / 100
          $raw = ($f[7] -shl 8) -bor $f[8]; if ($raw -ge 32768) { $raw -= 65536 }
          Write-Host ("SENSOR OK: Ammonia={0} mg/L  pH={1}  WaterTemp={2} C" -f $nh3, $ph, ($raw / 10)) -ForegroundColor Green
          return
        }
      }
    }
  }
  Write-Host "No valid reply. 0 bytes = sensor not powered or A/B not connected; garbage/00 = A/B swapped or no common GND." -ForegroundColor Yellow
} finally { $sp.Close() }
