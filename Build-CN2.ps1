param(
    [ValidateSet('gec-esp32-s3-lcd35-camera-stm32-cn2', 'gec-esp32-s3-lcd35-camera-stm32')]
    [string]$Variant = 'gec-esp32-s3-lcd35-camera-stm32-cn2'
)

$ErrorActionPreference = 'Stop'
$sourceProject = Join-Path $PSScriptRoot 'ESP32/xiaozhi-gec-camera'
$buildProject = 'C:/esp-projects/gec-r4-rotate'
robocopy $sourceProject $buildProject /E /XD build managed_components .git releases /XF sdkconfig sdkconfig.old /R:2 /W:1 /NFL /NDL /NJH /NJS /NP
if ($LASTEXITCODE -ge 8) { throw 'Source copy failed' }
Push-Location -LiteralPath $buildProject
try {
    & './main/boards/gec-esp32-s3/build.ps1' -Variant $Variant
    $delivery = Join-Path $PSScriptRoot 'firmware'
    New-Item -ItemType Directory -Path $delivery -Force | Out-Null
    $backup = Join-Path $delivery 'before-cn2-20261104'
    if (-not (Test-Path -LiteralPath $backup)) {
        New-Item -ItemType Directory -Path $backup | Out-Null
        foreach ($name in @('ESP32-app.bin', 'SHA256.json', '固件版本说明.txt')) {
            $old = Join-Path $delivery $name
            if (Test-Path -LiteralPath $old) { Copy-Item -LiteralPath $old -Destination $backup }
        }
    }
    $filename = if ($Variant.EndsWith('-cn2')) { 'ESP32-CN2-IO10-IO11-app.bin' } else { 'ESP32-TF-IO47-IO48-app.bin' }
    Copy-Item -LiteralPath (Join-Path $buildProject 'build/xiaozhi.bin') -Destination (Join-Path $delivery $filename)
    Copy-Item -LiteralPath (Join-Path $buildProject 'build/xiaozhi.bin') -Destination (Join-Path $delivery 'ESP32-app.bin')
    $hashes = [ordered]@{}
    foreach ($name in @('ESP32-app.bin', $filename, 'STM32-verified.hex')) {
        $path = Join-Path $delivery $name
        $hashes[$name] = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    $hashes | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $delivery 'SHA256.json') -Encoding utf8
    $variantNotes = if ($Variant.EndsWith('-cn2')) { @(
        'CN2 wiring: IO10 TX -> PC11 RX; IO11 RX <- PC10 TX through 1k; GND common.',
        'CN2 requires physical CTP_INT isolation at U12 pin13. Do not press USER.',
        'BOOT click: chat; double click: camera; long press 3s: WiFi setup.'
    ) } else { @(
        'TF wiring: GPIO47 TX -> PC11 RX; GPIO48 RX <- PC10 TX; GND common.',
        'No TF card. Touch and USER button remain enabled.'
    ) }
    @(
        "Variant: $Variant",
        "ESP32-app.bin = $filename"
    ) + $variantNotes + @(
        'STM32-verified.hex is the existing compatible UART4 firmware.',
        'Build success does not imply flashing, UART hardware or motion validation.'
    ) | Set-Content -LiteralPath (Join-Path $delivery '固件版本说明.txt') -Encoding utf8
    Write-Host "Built variant: $Variant"
    Write-Host "Application: $(Join-Path $delivery $filename)"
    Write-Host 'STM32 uses Guide_MQTT Keil project / STM32-verified.hex.'
} finally { Pop-Location }
