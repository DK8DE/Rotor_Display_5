/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include "sdkconfig.h"
#ifdef CONFIG_ARDUINO_RUNNING_CORE
#include <Arduino.h>
#endif
#include "lvgl.h"

#ifdef __cplusplus
namespace esp_panel::drivers {
class LCD;
class Touch;
}
#endif

// *INDENT-OFF*

/**
 * LVGL related parameters, can be adjusted by users
 */
#define LVGL_PORT_TICK_PERIOD_MS                (2) // The period of the LVGL tick task, in milliseconds

/**
 *
 * LVGL buffer related parameters, can be adjusted by users:
 *
 *  (These parameters will be useless if the avoid tearing function is enabled)
 *
 *  - Memory type for buffer allocation:
 *      - MALLOC_CAP_SPIRAM: Allocate LVGL buffer in PSRAM
 *      - MALLOC_CAP_INTERNAL: Allocate LVGL buffer in SRAM
 *
 *      (The SRAM is faster than PSRAM, but the PSRAM has a larger capacity)
 *      (For SPI/QSPI LCD, it is recommended to allocate the buffer in SRAM, because the SPI DMA does not directly support PSRAM now)
 *
 *  - The size (in bytes) and number of buffers:
 *      - Lager buffer size can improve FPS, but it will occupy more memory. Maximum buffer size is `width * height`.
 *      - The number of buffers should be 1 or 2.
 */
#define LVGL_PORT_BUFFER_MALLOC_CAPS            (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)       // Allocate LVGL buffer in SRAM
// #define LVGL_PORT_BUFFER_MALLOC_CAPS            (MALLOC_CAP_SPIRAM)      // Allocate LVGL buffer in PSRAM
#define LVGL_PORT_BUFFER_SIZE_HEIGHT            (20)
#define LVGL_PORT_BUFFER_NUM                    (2)

/**
 * LVGL timer handle task related parameters, can be adjusted by users
 */
#define LVGL_PORT_TASK_MAX_DELAY_MS             (500)       // The maximum delay of the LVGL timer task, in milliseconds
#define LVGL_PORT_TASK_MIN_DELAY_MS             (2)         // The minimum delay of the LVGL timer task, in milliseconds
#define LVGL_PORT_TASK_STACK_SIZE               (6 * 1024)  // The stack size of the LVGL timer task, in bytes
#define LVGL_PORT_TASK_PRIORITY                 (2)         // The priority of the LVGL timer task
#ifdef ARDUINO_RUNNING_CORE
#define LVGL_PORT_TASK_CORE                     (ARDUINO_RUNNING_CORE)  // Valid if using Arduino
#else
#define LVGL_PORT_TASK_CORE                     (0)                     // Valid if using ESP-IDF
#endif
                                                            // The core of the LVGL timer task, `-1` means the don't specify the core
                                                            // Default is the same as the main core
                                                            // This can be set to `1` only if the SoCs support dual-core,
                                                            // otherwise it should be set to `-1` or `0`

                                                            /**


/**
 * Avoid tering related configurations, can be adjusted by users.
 *
 *  (Currently, This function only supports RGB LCD and the version of LVGL must be >= 8.3.9)
 */
/**
 * Set the avoid tearing mode:
 *      - 0: Disable avoid tearing function
 *      - 1: LCD double-buffer & LVGL full-refresh
 *      - 2: LCD triple-buffer & LVGL full-refresh
 *      - 3: LCD double-buffer & LVGL direct-mode (recommended)
 */
#ifdef CONFIG_LVGL_PORT_AVOID_TEARING_MODE
#define LVGL_PORT_AVOID_TEARING_MODE            (CONFIG_LVGL_PORT_AVOID_TEARING_MODE)
                                                        // Valid if using ESP-IDF
#else
#define LVGL_PORT_AVOID_TEARING_MODE            (0)     // Valid if using Arduino
#endif

#if LVGL_PORT_AVOID_TEARING_MODE != 0
/**
 * When avoid tearing is enabled, the LVGL software rotation `lv_disp_set_rotation()` is not supported.
 * But users can set the rotation degree(0/90/180/270) here, but this function will reduce FPS.
 *
 * Set the rotation degree:
 *      - 0: 0 degree
 *      - 90: 90 degree
 *      - 180: 180 degree
 *      - 270: 270 degree
 */
#ifdef CONFIG_LVGL_PORT_ROTATION_DEGREE
#define LVGL_PORT_ROTATION_DEGREE               (CONFIG_LVGL_PORT_ROTATION_DEGREE)
                                                        // Valid if using ESP-IDF
#else
#define LVGL_PORT_ROTATION_DEGREE               (0)     // Valid if using Arduino
#endif

/**
 * Anti-Tearing-Modus und Drehung legen Pufferlayout und Refresh-Modus fest; an dieser Stelle typisch unverändert lassen.
 *
 * Vor LCD-Bus-Init: `lcd_bus->configRgbFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);`
 * Bilddrift: README, Abschnitt Troubleshooting.
 */
#define LVGL_PORT_AVOID_TEAR                    (1)
// Set the buffer number and refresh mode according to the different modes
#if LVGL_PORT_AVOID_TEARING_MODE == 1
    #define LVGL_PORT_DISP_BUFFER_NUM           (2)
    #define LVGL_PORT_FULL_REFRESH              (1)
#elif LVGL_PORT_AVOID_TEARING_MODE == 2
    #define LVGL_PORT_DISP_BUFFER_NUM           (3)
    #define LVGL_PORT_FULL_REFRESH              (1)
#elif LVGL_PORT_AVOID_TEARING_MODE == 3
    #define LVGL_PORT_DISP_BUFFER_NUM           (2)
    #define LVGL_PORT_DIRECT_MODE               (1)
#else
    #error "Invalid avoid tearing mode, please set macro `LVGL_PORT_AVOID_TEARING_MODE` to one of `LVGL_PORT_AVOID_TEARING_MODE_*`"
#endif
// Check rotation
#if (LVGL_PORT_ROTATION_DEGREE != 0) && (LVGL_PORT_ROTATION_DEGREE != 90) && (LVGL_PORT_ROTATION_DEGREE != 180) && \
    (LVGL_PORT_ROTATION_DEGREE != 270)
    #error "Invalid rotation degree, please set to 0, 90, 180 or 270"
#elif LVGL_PORT_ROTATION_DEGREE != 0
    #ifdef LVGL_PORT_DISP_BUFFER_NUM
        #undef LVGL_PORT_DISP_BUFFER_NUM
        #define LVGL_PORT_DISP_BUFFER_NUM           (3)
    #endif
#endif
#endif /* LVGL_PORT_AVOID_TEARING_MODE */

// *INDENT-ON*

/**
 * Optionaler Versatz der LVGL-Ausgabe auf dem physischen Panel (Pixel).
 *
 * Wenn links Randbereiche des Panels nicht sichtbar sind („Bild zu weit links“),
 * kann ein positiver X-Offset den Inhalt nach rechts schieben – ohne Änderungen an EEZ-/ui-Dateien.
 * Gilt für den Flush-Pfad ohne „avoid tearing“ (drawBitmap).
 *
 * Per build_flags überschreibbar, z. B. -DLVGL_PORT_DISP_OFFSET_X=5
 *
 * Wichtig: Der Flush kopiert nur in den sichtbaren Bereich (Clipping + ggf. Scratch-Puffer).
 * Naives drawBitmap(x+offset, …, volle Breite) würde x_end > Panelbreite verletzen → kein Bild.
 */
#ifndef LVGL_PORT_DISP_OFFSET_X
#define LVGL_PORT_DISP_OFFSET_X  (0)
#endif
#ifndef LVGL_PORT_DISP_OFFSET_Y
#define LVGL_PORT_DISP_OFFSET_Y  (0)
#endif

/**
 * Viele SPI-Panels (z. B. SH8601) verlangen x/y auf 2 Pixel ausgerichtet (x_coord_align).
 * Ungerade x_start (z. B. 5) → Warnung und oft kein sichtbares Bild.
 * Die effektiven Offsets werden auf Vielfache von LVGL_PORT_DISP_PANEL_XY_ALIGN aufgerundet;
 * Touch nutzt dieselben Werte (LVGL_PORT_DISP_OFFSET_*_EFFECTIVE).
 */
#ifndef LVGL_PORT_DISP_PANEL_XY_ALIGN
#define LVGL_PORT_DISP_PANEL_XY_ALIGN  (2)
#endif
#define LVGL_PORT_DISP_OFFSET_X_EFFECTIVE \
    ((((LVGL_PORT_DISP_OFFSET_X) + LVGL_PORT_DISP_PANEL_XY_ALIGN - 1) / LVGL_PORT_DISP_PANEL_XY_ALIGN) * \
     LVGL_PORT_DISP_PANEL_XY_ALIGN)
#define LVGL_PORT_DISP_OFFSET_Y_EFFECTIVE \
    ((((LVGL_PORT_DISP_OFFSET_Y) + LVGL_PORT_DISP_PANEL_XY_ALIGN - 1) / LVGL_PORT_DISP_PANEL_XY_ALIGN) * \
     LVGL_PORT_DISP_PANEL_XY_ALIGN)

/**
 * Max. Pixel pro Flush (Partial-Buffer). Muss >= größtes mögliches lv_area * nach Crop sein.
 */
#ifndef LVGL_PORT_FLUSH_SCRATCH_PIXELS
#define LVGL_PORT_FLUSH_SCRATCH_PIXELS  (464 * (LVGL_PORT_BUFFER_SIZE_HEIGHT + 4))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Porting LVGL with LCD and touch panel. This function should be called after the initialization of the LCD and touch panel.
 *
 * @param lcd The pointer to the LCD panel device, mustn't be nullptr
 * @param tp  The pointer to the touch panel device, set to nullptr if is not used
 *
 * @return true if success, otherwise false
 */
bool lvgl_port_init(esp_panel::drivers::LCD *lcd, esp_panel::drivers::Touch *tp);

/**
 * @brief Deinitialize the LVGL porting.
 *
 * @return true if success, otherwise false
 */
bool lvgl_port_deinit(void);

/**
 * @brief Lock the LVGL mutex. This function should be called before calling any LVGL APIs when not in LVGL task,
 *        and the `lvgl_port_unlock()` function should be called later.
 *
 * @param timeout_ms The timeout of the mutex lock, in milliseconds. If the timeout is set to `-1`, it will wait indefinitely.
 *
 * @return true if success, otherwise false
 */
bool lvgl_port_lock(int timeout_ms);

/**
 * @brief Unlock the LVGL mutex. This function should be called after using LVGL APIs when not in LVGL task, and the
 *        `lvgl_port_lock()` function should be called before.
 *
 * @return true if success, otherwise false
 */
bool lvgl_port_unlock(void);

#ifdef __cplusplus
}
#endif
