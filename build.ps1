#Requires -Version 5.1
<#
.SYNOPSIS
  Firmware flashen; optional LittleFS (Bilder) mit hochladen.
  Zu Beginn: alle *.bin unter src\ui loeschen (EEZ-Exportreste).

.PARAMETER Clean
  Führt zuerst pio run -t clean aus.

.PARAMETER WithFs
  PNGs nach data/img konvertieren und LittleFS (uploadfs) hochladen, danach Firmware.

.EXAMPLE
  .\build.ps1
  Nur Firmware: pio run -t upload -e esp32-s3-viewe

.EXAMPLE
  .\build.ps1 -WithFs
  .\build.ps1 --fs
  PNG -> .bin, uploadfs, dann Firmware-Upload.

.EXAMPLE
  .\build.ps1 --clean --fs
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $false)]
    [switch]$Clean,
    [Parameter(Mandatory = $false)]
    [Alias('Fs')]
    [switch]$WithFs,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$RemainingArguments
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

# EEZ legt beim Export ggf. *.bin unter src/ui ab; die werden hier nicht fuer FAT genutzt.
# Immer entfernen, damit keine veralteten Binaerdateien im UI-Ordner liegen bleiben.
$uiDir = Join-Path $PSScriptRoot 'src\ui'
if (Test-Path -LiteralPath $uiDir) {
    $binsInUi = Get-ChildItem -LiteralPath $uiDir -Filter '*.bin' -Recurse -File -ErrorAction SilentlyContinue
    foreach ($f in $binsInUi) {
        Remove-Item -LiteralPath $f.FullName -Force
        Write-Host "Entfernt (src/ui): $($f.Name)" -ForegroundColor DarkGray
    }
}

foreach ($a in $RemainingArguments) {
    switch -Regex ($a) {
        '^--?clean$' { $Clean = $true }
        '^--fs$' { $WithFs = $true }
        '^--with-fs$' { $WithFs = $true }
        '^--mit-fs$' { $WithFs = $true }
    }
}

function Invoke-Step {
    param([string]$Label, [scriptblock]$Action)
    Write-Host "`n=== $Label ===" -ForegroundColor Cyan
    & $Action
    if ($LASTEXITCODE -ne 0) {
        throw "Schritt fehlgeschlagen (Exit $LASTEXITCODE): $Label"
    }
}

try {
    if ($Clean) {
        Invoke-Step "pio clean" { pio run -t clean }
    }

    if ($WithFs) {
        Invoke-Step "PNG -> LVGL .bin" { python tools/png_to_lvgl8_bin.py }
        Invoke-Step "uploadfs (LittleFS)" { pio run -t uploadfs }
    }

    Invoke-Step "upload Firmware (esp32-s3-viewe)" { pio run -t upload -e esp32-s3-viewe }

    Write-Host "`nFertig." -ForegroundColor Green
}
catch {
    Write-Host "`n$($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
