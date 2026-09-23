#Requires -Version 5.1
<#
.SYNOPSIS
  PNGs unter data\img\ nach LVGL-8 .bin konvertieren (fuer uploadfs / FFat).

.DESCRIPTION
  Kompass_V5.png  -> ui_image_kompass_bg.bin
  Kompass_EL.png  -> ui_image_kompass_el.bin
  windPfeil.png   -> ui_image_pfeil_wind.bin

.EXAMPLE
  .\convert_images.ps1
#>
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

$script = Join-Path $PSScriptRoot 'tools\png_to_lvgl8_bin.py'
if (-not (Test-Path -LiteralPath $script)) {
    Write-Error "Skript fehlt: $script"
}

# PlatformIO-venv bevorzugen (wie build.ps1), sonst python aus PATH
$python = $null
$penvPy = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
if (Test-Path -LiteralPath $penvPy) {
    $python = $penvPy
} else {
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd) {
        $python = $cmd.Source
    }
}
if (-not $python) {
    Write-Error 'Python nicht gefunden (weder PlatformIO-penv noch PATH).'
}

Write-Host "Python: $python"
& $python -m pip install pillow -q
if ($LASTEXITCODE -ne 0) {
    Write-Error 'pip install pillow fehlgeschlagen.'
}

Write-Host 'Konvertiere data\img\ *.png -> ui_image_*.bin ...'
& $python $script --from-data-img
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Host ''
Write-Host 'Fertig. Aufs Geraet:  pio run -t uploadfs'
