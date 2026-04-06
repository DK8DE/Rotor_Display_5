#Requires -Version 5.1
<#
.SYNOPSIS
  Firmware flashen; optional LittleFS (Bilder) mit hochladen.
  Zu Beginn: alle *.bin unter src\ui loeschen (EEZ-Exportreste).

.PARAMETER Clean
  Führt zuerst pio run -t clean aus.

.PARAMETER WithFs
  PNGs nach data/img konvertieren und LittleFS (uploadfs) hochladen, danach Firmware.

.PARAMETER Version
  Neue Versionsnummer setzen (z. B. "1.1") — schreibt include\firmware_version.h (APP_VERSION, APP_DATE).
  Ohne Angabe bleibt die Version unverändert (wie RotorTcpBridge/build.ps1).

.EXAMPLE
  .\build.ps1
  Nur Firmware: pio run -t upload -e esp32-s3-viewe

.EXAMPLE
  .\build.ps1 -Version "1.1"
  Version und Datum in firmware_version.h aktualisieren, dann bauen/flashen.

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
    [Parameter(Mandatory = $false)]
    [string]$Version = '',
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$RemainingArguments
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

$VerFile = Join-Path $PSScriptRoot 'include\firmware_version.h'
if (-not (Test-Path -LiteralPath $VerFile)) {
    throw "Nicht gefunden: $VerFile"
}
$verContent = Get-Content -LiteralPath $VerFile -Raw -Encoding UTF8
$verMatch = [regex]::Match($verContent, 'FIRMWARE_APP_VERSION\s+"([^"]+)"')
if (-not $verMatch.Success) { throw 'FIRMWARE_APP_VERSION nicht in firmware_version.h gefunden.' }
$CurrentVer = $verMatch.Groups[1].Value

if ($Version -eq '') {
    $Version = $CurrentVer
    Write-Host "Version: $Version  (unveraendert)" -ForegroundColor Cyan
}
else {
    $today = (Get-Date).ToString('dd.MM.yyyy')
    $verContent = $verContent `
        -replace '(FIRMWARE_APP_VERSION\s+)"[^"]+"', "`$1`"$Version`"" `
        -replace '(FIRMWARE_APP_DATE\s+)"[^"]+"', "`$1`"$today`""
    $utf8NoBom = New-Object System.Text.UTF8Encoding $false
    [System.IO.File]::WriteAllText($VerFile, $verContent, $utf8NoBom)
    Write-Host "Version gesetzt: $CurrentVer  ->  $Version  ($today)" -ForegroundColor Green
}

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
