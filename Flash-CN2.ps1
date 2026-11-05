param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^COM[1-9][0-9]*$')]
    [string]$Port,
    [Parameter(Mandatory = $true)]
    [ValidateSet('0x20000', '0x410000')]
    [string]$AppOffset
)

$ErrorActionPreference = 'Stop'
$app = Join-Path $PSScriptRoot 'firmware/ESP32-CN2-IO10-IO11-app.bin'
if (-not (Test-Path -LiteralPath $app)) { throw 'Build CN2 firmware with Build-CN2.ps1 first.' }
$installs = (Get-Content -LiteralPath 'C:/Espressif/tools/eim_idf.json' -Raw | ConvertFrom-Json).idfInstalled
$selected = $installs | Where-Object {
    $_.name -match '^v6\.' -and $_.status -eq 'finished' -and (Test-Path -LiteralPath $_.python)
} | Sort-Object { [version]($_.name.TrimStart('v')) } -Descending | Select-Object -First 1
if (-not $selected) { throw 'An installed ESP-IDF 6.x Python environment is required.' }

Write-Host 'Hardware requirement: isolate U12 pin13 CTP_INT before powering CN2 firmware.'
Write-Host 'Do not press USER. Connect IO10->PC11, PC10->1k->IO11, GND common.'
Write-Host "Writing application only: $app to $AppOffset on $Port."
Write-Host 'AppOffset must match the actual active boot partition, verified from device logs.'
& $selected.python -m esptool --chip esp32s3 --port $Port --baud 460800 write-flash $AppOffset $app
if ($LASTEXITCODE -ne 0) { throw "Application flash failed (exit $LASTEXITCODE)." }
