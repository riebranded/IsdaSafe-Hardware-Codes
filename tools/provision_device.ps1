# Factory provisioning for QR-code claiming. Run ONCE per board, at build time.
#
#   1. Resets the ESP32 and reads its chip ID from the boot line.
#   2. Generates a random device secret and writes it to the board
#      (serial command PROVISION; stored in flash, survives reflashing the sketch).
#   3. Registers the secret + a claim code in Supabase
#      (register_device_credentials, service-role key; only hashes are stored).
#   4. Writes a printable QR sticker to tools\stickers\<ID>.html and opens it.
#   5. Uploads that sticker to the PRIVATE Supabase Storage bucket
#      "device-stickers" (create it first with Public OFF; the sticker holds
#      the plain claim code, so it must never be readable by app users).
#
# Re-running it on the same board replaces its secret and claim code (old
# stickers stop working); ownership in the app is kept.
#
# Usage (close Arduino's Serial Monitor first; COMxx = the ESP32's port):
#   powershell -ExecutionPolicy Bypass -File provision_device.ps1 -Port COM9
# Optional:
#   -ClaimUrl https://your-site/claim   QR becomes <url>?id=<ID>&code=<CODE>
#                                       (default: plain "ISDASAFE:<ID>:<CODE>")
#   -StickerBucket name                 private bucket to upload to (default device-stickers)
#   -NoUpload                           keep the sticker local only
# The service-role key is read from $env:SUPABASE_SERVICE_ROLE_KEY, or asked
# for (hidden input). Never commit it.
param(
  [Parameter(Mandatory)][string]$Port,
  [string]$ClaimUrl = "",
  [string]$SupabaseUrl = "https://vemaphehzjnvspxmweeb.supabase.co",
  [string]$StickerBucket = "device-stickers",
  [switch]$NoUpload
)
$ErrorActionPreference = 'Stop'

function Fail($msg) { Write-Host $msg -ForegroundColor Red; exit 1 }

if ([System.IO.Ports.SerialPort]::GetPortNames() -notcontains $Port) {
  $found = ([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object) -join ', '
  if (-not $found) { $found = 'none' }
  Fail "$Port not found. Plug in the ESP32 (and close Arduino's Serial Monitor). Ports seen now: $found"
}

# --- service-role key -------------------------------------------------------
$key = $env:SUPABASE_SERVICE_ROLE_KEY
if (-not $key) {
  $sec = Read-Host "Supabase service_role / secret key (Dashboard > Project Settings > API Keys)" -AsSecureString
  $key = [Runtime.InteropServices.Marshal]::PtrToStringBSTR([Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec))
}
if (-not $key) { Fail "No service-role key given." }

# --- secrets ----------------------------------------------------------------
$rng = [System.Security.Cryptography.RandomNumberGenerator]::Create()
$bytes = New-Object byte[] 32; $rng.GetBytes($bytes)
$secret = -join ($bytes | ForEach-Object { $_.ToString('x2') })

# 8 chars from 32 unambiguous symbols (no 0/O, 1/I) = 40 bits; byte % 32 is unbiased.
$alphabet = 'ABCDEFGHJKLMNPQRSTUVWXYZ23456789'
$cb = New-Object byte[] 8; $rng.GetBytes($cb)
$raw = -join ($cb | ForEach-Object { $alphabet[$_ % 32] })
$claimCode = $raw.Substring(0, 4) + '-' + $raw.Substring(4, 4)

# --- talk to the board --------------------------------------------------------
$sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
$sp.NewLine = "`n"
$sp.ReadTimeout = 500
$sp.Open()

function Wait-Line([string]$pattern, [int]$timeoutSec) {
  $deadline = (Get-Date).AddSeconds($timeoutSec)
  while ((Get-Date) -lt $deadline) {
    try { $line = $sp.ReadLine().Trim() } catch [TimeoutException] { continue }
    if ($line -match $pattern) { return $Matches }
  }
  return $null
}

try {
  Write-Host "Resetting the board on $Port ..."
  $sp.DtrEnable = $false
  $sp.RtsEnable = $true;  Start-Sleep -Milliseconds 150   # EN low
  $sp.RtsEnable = $false                                   # EN high -> boot
  $sp.DiscardInBuffer()

  $m = Wait-Line '^ID=([0-9A-F]{12})\b' 20
  if (-not $m) { Fail "No 'ID=' boot line within 20 s. Is the IsdaSafe sketch flashed and the Serial Monitor closed?" }
  $deviceId = $m[1]
  Write-Host "Device ID: $deviceId"

  $sp.Write("PROVISION $secret`n")
  $m = Wait-Line '^(PROVISIONED OK|PROVISION ERROR.*)$' 40
  if (-not $m) { Fail "Board did not answer PROVISION within 40 s (it may be stuck in a long upload; try again)." }
  if ($m[1] -ne 'PROVISIONED OK') { Fail "Board refused: $($m[1])" }
  Write-Host "Secret stored on the board." -ForegroundColor Green
} finally { $sp.Close() }

# --- register in Supabase ----------------------------------------------------
$headers = @{ apikey = $key; 'Content-Type' = 'application/json' }
if ($key -notlike 'sb_secret_*') { $headers['Authorization'] = "Bearer $key" }
$body = @{ p_device_id = $deviceId; p_secret = $secret; p_claim_code = $claimCode } | ConvertTo-Json
try {
  Invoke-RestMethod -Method Post -Uri "$SupabaseUrl/rest/v1/rpc/register_device_credentials" -Headers $headers -Body $body | Out-Null
} catch {
  Fail "Supabase registration failed: $($_.Exception.Message)`nThe board now holds an unregistered secret; fix the key/connection and run this again."
}
Write-Host "Registered in Supabase." -ForegroundColor Green

# --- sticker -----------------------------------------------------------------
$payload = if ($ClaimUrl) { "${ClaimUrl}?id=$deviceId&code=$claimCode" } else { "ISDASAFE:${deviceId}:$claimCode" }
$outDir = Join-Path $PSScriptRoot 'stickers'
New-Item -ItemType Directory -Force $outDir | Out-Null
$outFile = Join-Path $outDir "$deviceId.html"
$html = @"
<!doctype html>
<html><head><meta charset="utf-8"><title>IsdaSafe $deviceId</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/qrcode-generator/1.4.4/qrcode.min.js"></script>
<style>
  body { font-family: system-ui, sans-serif; background: #fff; color: #000; margin: 24px; }
  .sticker { width: 50mm; border: 1px dashed #999; padding: 4mm; text-align: center; }
  .sticker svg { width: 42mm; height: 42mm; }
  .id { font-size: 9pt; margin-top: 2mm; }
  .code { font: bold 13pt ui-monospace, monospace; letter-spacing: 1px; }
  .hint { font-size: 7pt; color: #444; }
  @media print { .noprint { display: none; } .sticker { border: none; } }
</style></head>
<body>
<p class="noprint">Print at 100% scale. Keep this file private: the code below lets anyone claim the device while it's unowned.</p>
<div class="sticker">
  <div id="qr"></div>
  <div class="id">IsdaSafe &middot; $deviceId</div>
  <div class="code">$claimCode</div>
  <div class="hint">Scan in the IsdaSafe app to add this device</div>
</div>
<script>
  var qr = qrcode(0, 'M');
  qr.addData('$payload');
  qr.make();
  document.getElementById('qr').innerHTML = qr.createSvgTag({ scalable: true, margin: 0 });
</script>
</body></html>
"@
[IO.File]::WriteAllText($outFile, $html)

# --- upload sticker to private Storage --------------------------------------
# The device is already provisioned + registered at this point, so a failed
# upload only warns; the local file is still there to print.
$uploaded = $false
if (-not $NoUpload) {
  $upHeaders = @{ apikey = $key; 'Content-Type' = 'text/html; charset=utf-8'; 'x-upsert' = 'true' }
  if ($key -notlike 'sb_secret_*') { $upHeaders['Authorization'] = "Bearer $key" }
  try {
    Invoke-RestMethod -Method Post -Uri "$SupabaseUrl/storage/v1/object/$StickerBucket/$deviceId.html" `
      -Headers $upHeaders -Body ([Text.Encoding]::UTF8.GetBytes($html)) | Out-Null
    $uploaded = $true
    Write-Host "Sticker uploaded to Storage: $StickerBucket/$deviceId.html" -ForegroundColor Green
  } catch {
    Write-Host "Sticker upload failed (local copy kept): $($_.Exception.Message)" -ForegroundColor Yellow
    Write-Host "  Does the private bucket '$StickerBucket' exist? Re-upload it by hand from the dashboard if needed." -ForegroundColor Yellow
  }
}

Write-Host ""
Write-Host "Done." -ForegroundColor Green
Write-Host "  Device ID : $deviceId"
Write-Host "  Claim code: $claimCode"
Write-Host "  QR text   : $payload"
Write-Host "  Sticker   : $outFile"
if ($uploaded) { Write-Host "  Storage   : $StickerBucket/$deviceId.html (private)" }
Start-Process $outFile
