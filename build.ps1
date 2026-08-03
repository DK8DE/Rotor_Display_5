#Requires -Version 5.1
<#
.SYNOPSIS
  Firmware flashen; optional LittleFS (Bilder) mit hochladen.
  Zu Beginn: alle *.bin unter src\ui loeschen (EEZ-Exportreste).
  Nach dem Build: Ordner IMGs\ neu befuellen (ESP Web Tools / espwebtool).

.PARAMETER Clean
  Führt zuerst pio run -t clean aus.

.PARAMETER WithFs
  PNGs nach data/img konvertieren und LittleFS (uploadfs) hochladen, danach Firmware.
  Zusaetzlich fatfs.bin in IMGs\ ablegen (Offset 0x610000).

.PARAMETER Version
  Neue Versionsnummer setzen (z. B. "1.1") — schreibt include\firmware_version.h (APP_VERSION, APP_DATE).
  Ohne Angabe bleibt die Version unverändert (wie RotorTcpBridge/build.ps1).

.PARAMETER SkipUpload
  Nur bauen und IMGs aktualisieren, kein Upload auf den Controller.

.EXAMPLE
  .\build.ps1
  Build, IMGs aktualisieren, Firmware flashen.

.EXAMPLE
  .\build.ps1 -Version "1.1"
  Version und Datum in firmware_version.h aktualisieren, dann bauen/flashen.

.EXAMPLE
  .\build.ps1 -WithFs
  .\build.ps1 --fs
  PNG -> .bin, uploadfs, dann Firmware-Upload.

.EXAMPLE
  .\build.ps1 -SkipUpload
  Nur Build + IMGs (fuer espwebtool), kein COM-Upload.

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
    [switch]$SkipUpload,
    [Parameter(Mandatory = $false)]
    [string]$Version = '',
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$RemainingArguments
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

# --- PlatformIO / Xtensa-GCC (Windows) ---
# Ohne diesen PATH-Eintrag meldet der Build oft: xtensa-esp32s3-elf-g++ nicht gefunden,
# obwohl das Paket unter %USERPROFILE%\.platformio\packages liegt.
$xtensaBin = Join-Path $env:USERPROFILE '.platformio\packages\toolchain-xtensa-esp-elf\xtensa-esp-elf\bin'
if (Test-Path -LiteralPath $xtensaBin) {
    $env:PATH = "$xtensaBin;$env:PATH"
}
$xtensaAs = Join-Path $xtensaBin 'xtensa-esp32s3-elf-as.exe'
if (-not (Test-Path -LiteralPath $xtensaAs)) {
    Write-Host 'WARNUNG: Toolchain scheint unvollstaendig (kein xtensa-esp32s3-elf-as.exe).' -ForegroundColor Yellow
    Write-Host '  Loeschen: $env:USERPROFILE\.platformio\packages\toolchain-xtensa-esp-elf' -ForegroundColor Yellow
    Write-Host '  Danach: erneut bauen (PlatformIO laedt das Paket neu).' -ForegroundColor Yellow
}
$pioExe = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'
if (-not (Test-Path -LiteralPath $pioExe)) {
    $pioExe = 'pio'
}

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
        '^--?skip-?upload$' { $SkipUpload = $true }
        '^--?no-?upload$' { $SkipUpload = $true }
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

# IMGs\ fuer ESP Web Tools / espwebtool: aktuelle Images + Partitionstabelle + manifest.json
function Update-WebFlasherImgs {
    param(
        [string]$BuildDir,
        [string]$ImgsDir,
        [string]$FwVersion,
        [bool]$IncludeFatfs
    )

    if (-not (Test-Path -LiteralPath $BuildDir)) {
        throw "Build-Verzeichnis fehlt: $BuildDir"
    }

    if (Test-Path -LiteralPath $ImgsDir) {
        Remove-Item -LiteralPath $ImgsDir -Recurse -Force
    }
    New-Item -ItemType Directory -Path $ImgsDir -Force | Out-Null

    $required = @(
        @{ Src = (Join-Path $BuildDir 'bootloader.bin'); Dst = 'bootloader.bin' },
        @{ Src = (Join-Path $BuildDir 'partitions.bin'); Dst = 'partitions.bin' },
        @{ Src = (Join-Path $BuildDir 'firmware.bin');   Dst = 'firmware.bin' }
    )
    foreach ($f in $required) {
        if (-not (Test-Path -LiteralPath $f.Src)) {
            throw "Fehlt nach Build: $($f.Src)"
        }
        Copy-Item -LiteralPath $f.Src -Destination (Join-Path $ImgsDir $f.Dst) -Force
        Write-Host "  IMGs\$($f.Dst)" -ForegroundColor DarkGray
    }

    # Lesbare Partitionstabelle (Quelle im Repo)
    $partCsv = Join-Path $PSScriptRoot 'partitions.csv'
    if (Test-Path -LiteralPath $partCsv) {
        Copy-Item -LiteralPath $partCsv -Destination (Join-Path $ImgsDir 'partitions.csv') -Force
        Write-Host '  IMGs\partitions.csv' -ForegroundColor DarkGray
    }

    # boot_app0 (OTA-Daten) — Standard Arduino-ESP32, Offset 0xE000
    $bootApp0Dst = Join-Path $ImgsDir 'boot_app0.bin'
    $bootApp0Candidates = @(
        (Join-Path $BuildDir 'boot_app0.bin'),
        (Join-Path $env:USERPROFILE '.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin')
    )
    $bootApp0Src = $bootApp0Candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    if ($bootApp0Src) {
        Copy-Item -LiteralPath $bootApp0Src -Destination $bootApp0Dst -Force
        Write-Host '  IMGs\boot_app0.bin' -ForegroundColor DarkGray
    }
    else {
        Write-Host '  WARNUNG: boot_app0.bin nicht gefunden — Manifest ohne OTA-Daten.' -ForegroundColor Yellow
    }

    # Basis-Teile: Bootloader/Partitionstabelle/OTA-Daten/Firmware - flashen NIE die FS-Partition,
    # bestehende Einstellungen (config.json) und Bilder auf dem Geraet bleiben unangetastet.
    $baseParts = [System.Collections.Generic.List[object]]::new()
    $baseParts.Add([ordered]@{ path = 'bootloader.bin'; offset = 0 })
    $baseParts.Add([ordered]@{ path = 'partitions.bin'; offset = 32768 })   # 0x8000
    if (Test-Path -LiteralPath $bootApp0Dst) {
        $baseParts.Add([ordered]@{ path = 'boot_app0.bin'; offset = 57344 }) # 0xE000
    }
    $baseParts.Add([ordered]@{ path = 'firmware.bin'; offset = 65536 })     # 0x10000

    # FATFS-Image (Bilder + config.json-Vorlage) fuer die Komplett-Installation bauen/kopieren
    # (Offset laut partitions.csv: 0x610000). Wird NUR im Full-Install-Manifest referenziert.
    $fatfsAvailable = $false
    if ($IncludeFatfs) {
        $fatSrc = Join-Path $BuildDir 'fatfs.bin'
        if (-not (Test-Path -LiteralPath $fatSrc)) {
            Write-Host '  fatfs.bin fehlt - baue Filesystem-Image ...' -ForegroundColor DarkGray
            & $pioExe run -t buildfs -e esp32-s3-viewe
            if ($LASTEXITCODE -ne 0) {
                throw "buildfs fehlgeschlagen (Exit $LASTEXITCODE)"
            }
        }
        if (Test-Path -LiteralPath $fatSrc) {
            Copy-Item -LiteralPath $fatSrc -Destination (Join-Path $ImgsDir 'fatfs.bin') -Force
            Write-Host '  IMGs\fatfs.bin' -ForegroundColor DarkGray
            $fatfsAvailable = $true
        }
        else {
            Write-Host '  WARNUNG: fatfs.bin nicht erzeugt - Komplett-Installation ohne Filesystem.' -ForegroundColor Yellow
        }
    }

    function New-WebFlasherManifest {
        param([string]$Path, [string]$Name, [bool]$PromptErase, [object[]]$Parts)
        $manifest = [ordered]@{
            name                     = $Name
            version                  = $FwVersion
            new_install_prompt_erase = $PromptErase
            builds                   = @(
                [ordered]@{
                    chipFamily = 'ESP32-S3'
                    parts      = @($Parts)
                }
            )
        }
        $json = $manifest | ConvertTo-Json -Depth 6
        $utf8NoBom = New-Object System.Text.UTF8Encoding $false
        [System.IO.File]::WriteAllText($Path, $json, $utf8NoBom)
    }

    # 1) Update (Standard): nur Bootloader/Partitionstabelle/Firmware - Dateisystem (Einstellungen,
    #    Bilder) bleibt unveraendert, da diese Flash-Region gar nicht beschrieben wird.
    $updateManifestPath = Join-Path $ImgsDir 'manifest.json'
    New-WebFlasherManifest -Path $updateManifestPath -Name 'Rotor Display 5 (Update)' `
        -PromptErase $false -Parts $baseParts.ToArray()
    Write-Host "  IMGs\manifest.json  (v$FwVersion, Update - Einstellungen bleiben erhalten)" -ForegroundColor DarkGray

    # 2) Komplett-Installation: zusaetzlich das Dateisystem-Image (Fabrik-Bilder + Default-Config) -
    #    fuer neue/leere Geraete oder einen bewussten Reset. Ueberschreibt bestehende Einstellungen!
    if ($fatfsAvailable) {
        $fullParts = [System.Collections.Generic.List[object]]::new($baseParts)
        # Partition "ffat" beginnt laut partitions.csv bei 0x610000, aber PlatformIOs FFat-Wear-Leveling
        # reserviert die ersten 4096 Bytes der Partition fuer WL-Metadaten (siehe builder/main.py,
        # fetch_fs_size(): FS_START += 4096 fuer filesystem == "fatfs"). Das eigentliche FAT-Image (auch
        # 4096 Bytes kleiner gebaut) muss deshalb bei 0x611000 geschrieben werden, nicht bei 0x610000 -
        # sonst findet FFat.begin() keinen gueltigen Header und formatiert die Partition leer neu.
        $fullParts.Add([ordered]@{ path = 'fatfs.bin'; offset = 6361088 }) # 0x611000 (0x610000 + 4096)
        $fullManifestPath = Join-Path $ImgsDir 'manifest-full-install.json'
        New-WebFlasherManifest -Path $fullManifestPath -Name 'Rotor Display 5 (Komplett-Installation)' `
            -PromptErase $true -Parts $fullParts.ToArray()
        Write-Host "  IMGs\manifest-full-install.json  (v$FwVersion, Komplett - setzt Einstellungen zurueck)" -ForegroundColor DarkGray
    }
}

try {
    if ($Clean) {
        Invoke-Step "pio clean" { & $pioExe run -t clean }
    }

    if ($WithFs) {
        Invoke-Step "PNG -> LVGL .bin" { python tools/png_to_lvgl8_bin.py }
    }

    Invoke-Step "pio build (esp32-s3-viewe)" { & $pioExe run -e esp32-s3-viewe }

    $buildDir = Join-Path $PSScriptRoot '.pio\build\esp32-s3-viewe'
    $imgsDir = Join-Path $PSScriptRoot 'IMGs'
    Invoke-Step "IMGs aktualisieren (ESP Web Tools)" {
        # Dateisystem (fatfs.bin) gehoert immer ins Web-Flasher-Paket, sonst fehlen einem frisch
        # geflashten Geraet die Bilder (Kompass-Hintergrund, Windpfeil) aus data/img. -WithFs steuert
        # hier nur noch die PNG->bin-Neukonvertierung; das Einpacken passiert unabhaengig davon.
        Update-WebFlasherImgs -BuildDir $buildDir -ImgsDir $imgsDir -FwVersion $Version -IncludeFatfs $true
    }

    if ($WithFs -and -not $SkipUpload) {
        Invoke-Step "uploadfs (FATFS)" { & $pioExe run -t uploadfs -e esp32-s3-viewe }
    }

    if (-not $SkipUpload) {
        Invoke-Step "upload Firmware (esp32-s3-viewe)" { & $pioExe run -t upload -e esp32-s3-viewe }
    }
    else {
        Write-Host "`nUpload uebersprungen (-SkipUpload). IMGs liegt unter: $imgsDir" -ForegroundColor Yellow
    }

    Write-Host "`nFertig." -ForegroundColor Green
}
catch {
    Write-Host "`n$($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
