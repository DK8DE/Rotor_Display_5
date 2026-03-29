# Rotor_Display_5

Build und Voraussetzungen für die Firmware (ESP32-S3, VIEWE UEDX46460015, Umgebung `esp32-s3-viewe`).

## Rotorcontroller und Zusammenspiel mit dem PC

Der **Rotorcontroller** (Antennenrotor-Steuerung) steuert einen **Azimut-Rotor** und kann **alleine** am Gerät oder **gemeinsam mit einer PC-Software** betrieben werden. Die Firmware auf diesem Display ist die **Bedienoberfläche** auf dem ESP32; der eigentliche Regler/Motor hängt typischerweise per **RS485** am Bus.

Für die **Windows-Software am PC** ist der Controller (über dieses Board) im Wesentlichen die **USB-zu-RS485-Brücke**: Befehle und Telemetrie laufen zwischen PC und Rotor-Bus. Die Desktop-Anwendung dazu ist **RotorTcpBridge** (Python/Qt):

**https://github.com/DK8DE/RotorTcpBridge**

Dort sind Verbindung (seriell/TCP), Protokoll, Kompass, Karte und Schnittstellen zu anderer Software beschrieben.

## Ziel dieses Repos

Firmware für ESP32-S3 bauen und flashen. Bilder für LVGL liegen auf der FAT-Partition (über `uploadfs`); Quellen sind PNGs im Ordner `imgs/`.

## EEZ Studio und Bildnamen (`imgs` vs. `src/ui`)

- EEZ Studio kann beim Export oder Build LVGL-Bilder als **`*.bin`** unter **`src/ui`** ablegen. **Diese von EEZ erzeugten `.bin`-Dateien haben für dieses Projekt kein passendes Format** (andere LVGL-/Speicher-Erwartung als unsere FAT-Ladepfade). Sie werden daher **entfernt** und **nicht** verwendet.
- Stattdessen erzeugen wir die benötigten **`*.bin`** **selbst aus PNGs** mit **`tools/png_to_lvgl8_bin.py`** — passend zu LVGL 8 und dem Zugriff zur Laufzeit über **`S:/img/...`** auf der FAT-Partition (die fertigen Dateien liegen unter **`data/img/<name>.bin`** nach der Konvertierung).
- Die **PNG-Dateien in `imgs/`** müssen **denselben Basisnamen** tragen wie die Originalbilder bzw. die Asset-Namen in EEZ (z. B. `ui_image_kompass_bg.png`, `ui_image_pfeil_wind.png`), damit das Skript die richtigen **`data/img/<name>.bin`** erzeugt und sie zu den Referenzen in `screens.c` / `images.c` passen.
- **`build.ps1`** löscht **vor jedem Lauf** alle **`*.bin`** unter **`src/ui`** (rekursiv), falls welche vorhanden sind — damit liegen keine veralteten oder **von EEZ exportierten, ungeeigneten** Binärdateien im UI-Ordner.

## Anwendungscode und `src/ui` (nicht manuell bearbeiten)

- Der Ordner **`src/ui`** ist **von EEZ Studio generiert** und wird beim **neuen Export überschrieben**. Dort **keine eigenen Änderungen** vornehmen (keine Fixes, keine Zusatzlogik in `screens.c` usw.).
- Eigene Logik gehört **außerhalb** von `src/ui`, z. B. in `src/main.cpp`, eigene Module unter `src/`, Hooks über die von EEZ vorgesehenen **Actions**, **`ui_tick`**, **`objects`**, **`ui.h`** und ähnliche **öffentliche Eintrittspunkte**.
- So bleibt ein erneuter EEZ-Export möglich, ohne dass Anwendungscode verloren geht.

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

## Cursor / VS Code: C++-Analyse (clangd)

Ohne passende **Kompilierdaten** markiert der Editor oft **Scheinfehler** (z. B. bei `assert(board->begin())`, fehlende Includes), weil die Analyse nicht dieselben Flags und Pfade wie der echte ESP32-Xtensa-Build nutzt.

1. **Einmal (oder nach größeren `platformio.ini`-Änderungen)** im Projektroot die **Compilation Database** erzeugen (steht in `.gitignore`, wird **nicht** ins Repo committet):
   ```bash
   pio run -t compiledb -e esp32-s3-viewe
   ```
   Danach liegt `compile_commands.json` im **Projektroot** (gleicher Ordner wie `platformio.ini`).
2. **Erweiterung:** **clangd** (`llvm-vs-code-extensions.vscode-clangd`). Die Projektdatei **`.vscode/settings.json`** schaltet die Microsoft-**C/C++**-IntelliSense-Squiggles ab und setzt **`--query-driver`** für die PlatformIO-Toolchain unter `%USERPROFILE%\.platformio\packages\toolchain-xtensa-esp32s3\...`, damit clangd die **Xtensa-GCC**-Aufrufe aus `compile_commands.json` nachvollziehen darf.
3. Fenster neu laden (**Developer: Reload Window**), bis die Analyse die neue DB einliest.

Liegt die Toolchain woanders, die Pfade in `.vscode/settings.json` bei `clangd.arguments` → `--query-driver` anpassen.

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
- **`src/ui` nicht von Hand ändern**; nur die EEZ-API von außen nutzen (siehe Abschnitt oben).
