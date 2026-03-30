#include <Arduino.h>
#include <FFat.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <stdint.h>
#include "lvgl_v8_port.h"

#include <Button.h>
#include "UltraEncoderPCNT.h"
#include <Signals.h>
// WICHTIG: Lokalen UI-Wrapper verwenden (EEZ-Studio UI liegt unter "src/ui/")
#include "ui.h"
#include "serial_bridge.h"
#include "pwm_config.h"
#include "rotor_app.h"
#include "rotor_error_app.h"
#include "rotor_rs485.h"
#include "signals_ring_app.h"

/** Signals → ATtiny: nur TX, wie im SignalsDemo (Serial1). */
static constexpr int8_t SIGNALS_TX_PIN = 40;
static constexpr int8_t SIGNALS_RX_PIN = -1;
static constexpr uint32_t SIGNALS_BAUD = 115200;
/** NeoPixel-Ring am Empfänger (wie SignalsDemo: 16 LEDs). */
static constexpr uint8_t SIGNALS_NUM_LEDS = 16;

static Signals g_signals(Serial1);

/** Kurze Boot-Melodie + einmal blaues Kreislauflicht (nur beim Start). */
static void signals_play_boot_welcome()
{
    g_signals.begin(SIGNALS_TX_PIN, SIGNALS_RX_PIN, SIGNALS_BAUD);
    g_signals.clear();
    g_signals.stopTone();
    delay(80);

    /* Kleine Melodie (Hertz, Vol 0..50, ms) – C-Dur Aufwärts + Schlusston */
    struct {
        uint16_t freq;
        uint16_t ms;
    } const notes[] = {
        { 523, 140 },  /* C5 */
        { 659, 140 },  /* E5 */
        { 784, 140 },  /* G5 */
        { 1047, 180 }, /* C6 */
        { 784, 120 },
        { 1047, 220 },
    };
    for (size_t i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
        g_signals.tone(notes[i].freq, 18, notes[i].ms);
        delay(notes[i].ms + 35);
    }
    g_signals.stopTone();
    delay(120);

    /* Einmal blau im Kreis (wie Demo-Lauflicht, nur B=255) */
    g_signals.setAutoShow(false);
    g_signals.clear();
    for (uint8_t i = 0; i < SIGNALS_NUM_LEDS; i++) {
        g_signals.clear();
        g_signals.setPixel(i, 0, 0, 255, 100);
        g_signals.show();
        delay(95);
    }
    g_signals.setAutoShow(true);
    g_signals.clear();
}

/**
 * EEZ-Studio UI benötigt (je nach Projekt) einen periodischen Tick.
 *
 * Da in diesem Beispiel LVGL in einem eigenen Task läuft (lv_timer_handler()),
 * registrieren wir einen LVGL-Timer, der ui_tick() regelmäßig im LVGL-Kontext
 * aufruft.
 */
static void ui_tick_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    ui_tick();
}



using namespace esp_panel::drivers;
using namespace esp_panel::board;

/*if you use BOARD_VIEWE_UEDX46460015_MD50ET or UEDX48480021_MD80E/T,please open it*/
/* A/B vertauscht ggü. Board-Silk — Drehrichtung wie gewünscht (Quadratur) */
#define GPIO_NUM_KNOB_PIN_A     5
#define GPIO_NUM_KNOB_PIN_B     6
#define GPIO_BUTTON_PIN         GPIO_NUM_0

/**
 * Skalierung PCNT-Schritt → Zehntelgrad für rotor_app_encoder_step(delta_tenths).
 * 10 = 1,0° pro Rastung; 1 = 0,1° pro Rastung (Fein).
 * Schritte kommen per UltraEncoderStepCallback direkt aus dem PCNT-Service-Task — kein 1-ms-Polling,
 * sonst können feine Schritte mit dem internen Flush (Watch/Timeout) kollidieren.
 */
static constexpr int ENCODER_DELTA_TENTHS_PER_STEP = 10;

/**
 * PCNT-Hardware-Glitchfilter (Nanosekunden): Pulse kürzer als dieser Wert zählen nicht.
 * Vorher 200 ns ≈ aus; mechanische Drehgeber profitieren oft von 2–10 µs.
 * Zu groß: sehr schnelles Drehen kann Schritte „verschlucken“. 0 = Filter aus.
 * Sinnvoll typ. 0,3–1 ms (UEPCNT_DEFAULT_GLITCH_NS in UltraEncoderPCNT.h).
 */
static constexpr uint32_t ENCODER_GLITCH_NS = UEPCNT_DEFAULT_GLITCH_NS;

/**
 * Gegenrichtungs-Filter (UltraEncoderPCNT::setOppositeStepMinMs): nur Einzelschritt |delta|==1.
 * Kurze falsche Richtungsimpulse unterdrücken; 0 = aus. Tuning wie encoder_test (typ. 35–50 ms).
 */
static constexpr uint32_t ENCODER_OPPOSITE_STEP_MIN_MS = 42u;

/** Quadratur-Encoder über PCNT (UltraEncoderPCNT). */
static UltraEncoderPCNT *s_ultra_enc = nullptr;

static void ultra_encoder_on_step(void *user, int32_t step_delta)
{
    (void)user;
    if (step_delta == 0 || ENCODER_DELTA_TENTHS_PER_STEP == 0) {
        return;
    }
    rotor_app_encoder_step(static_cast<int>(step_delta) * ENCODER_DELTA_TENTHS_PER_STEP);
}

static void SingleClickCb(void *button_handle, void *usr_data)
{
    (void)button_handle;
    (void)usr_data;
    /* Fehler aktiv (GETERR/async ERR): immer SETREF (Homing + Quittierung) — vor Snap/Stop/„nur wenn nicht ref.“ */
    if (rotor_error_app_encoder_click_triggers_homing_only()) {
        rotor_rs485_send_setref_homing();
        lvgl_port_lock(-1);
        LVGL_button_event((void *)(intptr_t)BUTTON_SINGLE_CLICK);
        lvgl_port_unlock();
        return;
    }
    if (rotor_rs485_is_position_polling()) {
        /* Soll = Ist: Encoder-Session löschen (sonst kein taget + altes SETPOS aus rotor_app_loop) */
        const float snap = rotor_rs485_get_last_position_deg();
        rotor_app_snap_target_to_deg(snap);
        rotor_rs485_hw_snap_retarget_request(snap);
    } else if (rotor_rs485_is_moving()) {
        rotor_rs485_send_stop();
    } else if (!rotor_rs485_is_referenced()) {
        rotor_rs485_send_setref_homing();
    }
    lvgl_port_lock(-1);
    LVGL_button_event((void *)(intptr_t)BUTTON_SINGLE_CLICK);
    lvgl_port_unlock();
}
static void DoubleClickCb(void *button_handle, void *usr_data)
{
    // Serial.println("Button Double Click");
}
static void LongPressStartCb(void *button_handle, void *usr_data) {
    // Serial.println("Button Long Press Start");
    lvgl_port_lock(-1);
    LVGL_button_event((void*)(intptr_t)BUTTON_LONG_PRESS_START);
    lvgl_port_unlock();
}

void setup()
{
    Serial.begin(115200);
    serial_bridge::begin();

    Serial.println("Signals boot (TX GPIO40) …");
    signals_play_boot_welcome();
    Serial.println("Initializing board");
    /* FFat (partitions.csv: ffat) — muss vor LVGL, damit S:/img/… über LV_FS_STDIO funktioniert */
    if (!FFat.begin(true, "/ffat", 10, "ffat")) {
        Serial.println("FFat mount failed — Bilder unter /ffat/img/ sind nicht erreichbar");
    } else {
        Serial.printf("FFat OK, %.1f KB free\n", FFat.freeBytes() / 1024.0f);
    }
    pwm_config_load();
    // esp_log_level_set("*", ESP_LOG_NONE);
  
  // Initialize UART0
    // MySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    Board *board = new Board();
    board->init();
#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    // When avoid tearing function is enabled, the frame buffer number should be set in the board driver
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    /**
     * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the
     * "bounce buffer" functionality to enhance the RGB data bandwidth.
     * This feature will consume `bounce_buffer_size * bytes_per_pixel * 2` of SRAM memory.
     */
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
    }
#endif
#endif
    assert(board->begin());

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    /* UltraEncoderPCNT: HALF = 2 Flanken pro logischem Schritt (VIEWE-Knob); FULL würde nur jeden 2. Klick zählen */
    Serial.println("Initialize UltraEncoderPCNT (A/B GPIO5/6, ULTRA_MODE_HALF)");
    s_ultra_enc = new UltraEncoderPCNT(
        GPIO_NUM_KNOB_PIN_A,
        GPIO_NUM_KNOB_PIN_B,
        ULTRA_MODE_HALF,
        0,
        1000);
    if (!s_ultra_enc->begin(0, 500.0f, 0, ENCODER_GLITCH_NS)) {
        Serial.println("UltraEncoderPCNT begin failed");
    } else {
        s_ultra_enc->setPositionSteps(0);
        s_ultra_enc->setOppositeStepMinMs(ENCODER_OPPOSITE_STEP_MIN_MS);
        s_ultra_enc->setStepCallback(ultra_encoder_on_step, nullptr);
    }

    Serial.println("Initialize Button device");
    Button *btn = new Button(GPIO_BUTTON_PIN, false);
    btn->attachSingleClickEventCb(&SingleClickCb, NULL);
    btn->attachDoubleClickEventCb(&DoubleClickCb, NULL);
    btn->attachLongPressStartEventCb(&LongPressStartCb, NULL);

    Serial.println("Creating UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);


    // EEZ-Studio UI initialisieren (Screens/Styles/Images werden angelegt)
    ui_init();

    /* Rotor RS485: Homing-Button, LED, Boot-GETREF (Logik außerhalb src/ui) */
    rotor_app_init();
    signals_ring_app_init(&g_signals, SIGNALS_NUM_LEDS);

    /**
     * Sichtbarer Rand / Panel-Versatz: nicht per LVGL-Screen-Padding (unzuverlässig mit EEZ-Root
     * 460×460 + absolute Kinder), sondern LVGL_PORT_DISP_OFFSET_* in src/lvgl_v8_port.h bzw.
     * build_flags – siehe Flush drawBitmap + Touch-Kompensation in lvgl_v8_port.cpp.
     */

    // EEZ-Studio UI tick regelmäßig ausführen (läuft innerhalb des LVGL-Tasks)
    // 10ms ist ein guter Startwert; können wir später je nach Bedarf anpassen.
    lv_timer_create(ui_tick_timer_cb, 10, NULL);
    /* Release the mutex */
    lvgl_port_unlock();
}

void loop()
{
    /* RS485-Empfang zuerst (USB spiegeln + Parser) — kann Pending löschen (ACK) */
    serial_bridge::poll();
    /* Encoder-SETPOS-Retry sofort nach frei werdendem Bus, bevor wieder GETPOSDG gestartet wird */
    rotor_app_loop();
    rotor_pwm_ui_loop();
    rotor_rs485_loop();
    rotor_error_app_loop(millis());
    signals_ring_app_loop(millis());

    rotor_app_loop();
}
