# Rotor_Display_5

Build und Voraussetzungen für die Firmware (ESP32-S3, VIEWE UEDX46460015, Umgebung `esp32-s3-viewe`).

## Ziel

Firmware für ESP32-S3 bauen und flashen. Bilder für LVGL liegen auf der FAT-Partition (über `uploadfs`); Quellen sind PNGs im Ordner `imgs/`.

## EEZ Studio und Bildnamen (`imgs` vs. `src/ui`)

- EEZ Studio kann beim Export oder Build LVGL-Bilder als `*.bin` unter **`src/ui`** ablegen. Dieses Projekt lädt die Bilder zur Laufzeit von der FAT-Partition **`S:/img/...`** (die passenden `*.bin` liegen nach Konvertierung unter **`data/img/`**).
- Die **PNG-Dateien in `imgs/`** müssen **denselben Basisnamen** tragen wie die Originalbilder bzw. die Asset-Namen in EEZ (z. B. `ui_image_kompass_bg.png`, `ui_image_pfeil_wind.png`), damit `tools/png_to_lvgl8_bin.py` die passenden Dateien **`data/img/<name>.bin`** erzeugt und sie zu den Referenzen in `screens.c` / `images.c` passen.
- **`build.ps1`** löscht **vor jedem Lauf** alle `*.bin` unter **`src/ui`** (rekursiv), falls welche vorhanden sind, damit keine veralteten EEZ-Binärdateien im UI-Ordner liegen bleiben.

## Voraussetzungen (Rechner)

1. **PlatformIO** (CLI oder VS Code/Cursor). Im Projektordner funktionieren die Befehle `pio run …`.
2. **Python 3** (für das PNG-Tool), im PATH als `python` oder `py`.
   - Einmal im Projektroot:
     ```bash
     pip install -r requirements.txt
     ```
   - Benötigt: **Pillow** (siehe `requirements.txt`) für `tools/png_to_lvgl8_bin.py`.
3. **ESP32** per USB; passender USB-Treiber (CP210x, CH340 o. Ä.).
   - Upload/Monitor: siehe `platformio.ini` (z. B. Upload 921600, Monitor 115200).
   - Bei ESP32-S3 + USB-CDC: `monitor_rts` / `monitor_dtr` = 0 in `platformio.ini`, sonst Reset beim Öffnen des seriellen Monitors.
4. **Lokale Bibliotheken** unter `lib/` (LVGL, ESP32_Display_Panel, eez-framework, …). Kein separates Arduino-`libraries`-Setup nötig, wenn `lib/` vollständig ist.

## Empfohlen: `build.ps1` (Windows PowerShell)

Im Projektroot:

**Zuerst (immer):** `*.bin` unter `src/ui` entfernen, falls vorhanden.

| Aktion | Befehl |
|--------|--------|
| Nur Firmware bauen und flashen (ohne neue Bilder / ohne Dateisystem-Upload) | `.\build.ps1` |
| PNGs nach `data/img/*.bin`, FAT flashen, dann Firmware | `.\build.ps1 -WithFs` oder `.\build.ps1 -Fs` oder `.\build.ps1 --fs` (gleichwertig: `--with-fs`, `--mit-fs`) |
| Vorher aufräumen (`pio clean`) | `.\build.ps1 --clean` oder mit FS: `.\build.ps1 --clean --fs` |

**Reihenfolge mit `-WithFs` / `--fs`:**

1. `python tools/png_to_lvgl8_bin.py` — PNG → LVGL-8 `.bin` nach `data/img/`
2. `pio run -t uploadfs` — `data/` auf die Flash-Partition
3. `pio run -t upload -e esp32-s3-viewe` — Firmware

## Manuell (ohne `build.ps1`)

```bash
pip install -r requirements.txt
```

```bash
# Nur Bilder erzeugen
python tools/png_to_lvgl8_bin.py

# Nur Dateisystem
pio run -t uploadfs

# Nur Firmware (Standard-Umgebung: esp32-s3-viewe)
pio run -t upload -e esp32-s3-viewe

# Nur kompilieren (ohne Flash)
pio run -e esp32-s3-viewe

# Aufräumen
pio run -t clean
```

## Partition und `data`

`board_build.filesystem` und Partitionen siehe `platformio.ini` und `partitions.csv`. Der Inhalt von `data/` (inkl. `data/img/*.bin` nach Konvertierung) wird mit `uploadfs` auf den ESP geschrieben.

## Kurzüberblick

- **`build.ps1`:** zuerst Aufräumen von `*.bin` in `src/ui`, dann je nach Schaltern nur Firmware oder zusätzlich PNG-Konvertierung und FS-Upload.
- **Ohne Schalter:** schneller Firmware-Flash.
- **Mit `-Fs` / `--fs`:** Bilder neu erzeugen, FS hochladen, Firmware flashen.
- **Python + pip + Pillow** für die PNG-Pipeline; **PlatformIO** für den ESP-Build.
- **PNGs in `imgs/`** an EEZ-Assetnamen anbinden; FAT-Dateien stammen aus `data/img/` nach `uploadfs`.
