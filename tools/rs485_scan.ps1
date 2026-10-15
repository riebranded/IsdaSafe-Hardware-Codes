# Wide Modbus scan: tries many baud rates and slave IDs and reports ANY bytes heard.
# Usage: powershell -ExecutionPolicy Bypass -File rs485_scan.ps1 -Port COM24 [-Full]
param([Parameter(Mandatory)][string]$Port, [switch]$Full)

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

$bauds = 4800, 9600, 2400, 19200, 1200, 38400, 57600, 115200
$ids = if ($Full) { 1..247 } else { @(1, 2, 3, 4, 5, 10, 0xFF) }
$heard = $false

foreach ($baud in $bauds) {
  $sp = New-Object System.IO.Ports.SerialPort $Port, $baud, 'None', 8, 'One'
  $sp.Open()
  Write-Host "baud $baud ..." -NoNewline
  foreach ($id in $ids) {
    $req = [byte[]]@($id, 0x03, 0x00, 0x00, 0x00, 0x03, 0, 0)
    $c = Crc16 $req 6
    $req[6] = $c -band 0xFF; $req[7] = $c -shr 8
    $sp.DiscardInBuffer()
    $sp.Write($req, 0, 8)
    Start-Sleep -Milliseconds ([Math]::Max(150, 30000 / $baud))
    $n = $sp.BytesToRead
    if ($n) {
      $buf = New-Object byte[] $n; [void]$sp.Read($buf, 0, $n)
      Write-Host ("`n  ID {0} -> {1} byte(s): {2}" -f $id, $n, (($buf | % { $_.ToString('X2') }) -join ' ')) -ForegroundColor Green
      $heard = $true
    }
  }
  Write-Host " done"
  $sp.Close()
}
if (-not $heard) { Write-Host "Nothing heard on any baud/ID." -ForegroundColor Yellow }
