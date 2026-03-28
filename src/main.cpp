
#include <Arduino.h>
#include <FFat.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <stdint.h>
#include "lvgl_v8_port.h"

#include <ESP_Knob.h>
#include <Button.h>
#include <Signals.h>
// WICHTIG: Lokalen UI-Wrapper verwenden (EEZ-Studio UI liegt unter "src/ui/")
#include "ui.h"
#include "serial_bridge.h"

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



/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
// #include <demos/lv_demos.h>
// #include <examples/lv_examples.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

/*if you use BOARD_VIEWE_UEDX46460015_MD50ET or UEDX48480021_MD80E/T,please open it*/
#define GPIO_NUM_KNOB_PIN_A     6
#define GPIO_NUM_KNOB_PIN_B     5
#define GPIO_BUTTON_PIN         GPIO_NUM_0

/*Knob event definition*/
ESP_Knob *knob;
void onKnobLeftEventCallback(int count, void *usr_data)
{
    // Serial.printf("Detect left event, count is %d\n", count);
    lvgl_port_lock(-1);
    LVGL_knob_event((void*)(intptr_t)KNOB_LEFT);
    lvgl_port_unlock();
}

void onKnobRightEventCallback(int count, void *usr_data)
{
    // Serial.printf("Detect right event, count is %d\n", count);
    lvgl_port_lock(-1);
    LVGL_knob_event((void*)(intptr_t)KNOB_RIGHT);
    lvgl_port_unlock();
}

static void SingleClickCb(void *button_handle, void *usr_data) {
    // Serial.println("Button Single Click");
    lvgl_port_lock(-1);
    LVGL_button_event((void*)(intptr_t)BUTTON_SINGLE_CLICK);
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

    /*knob initialization*/
    Serial.println("Initialize Knob device");
    knob = new ESP_Knob(GPIO_NUM_KNOB_PIN_A, GPIO_NUM_KNOB_PIN_B);
    knob->begin();
    knob->attachLeftEventCallback(onKnobLeftEventCallback);
    knob->attachRightEventCallback(onKnobRightEventCallback);

    Serial.println("Initialize Button device");
    Button *btn = new Button(GPIO_BUTTON_PIN, false);
    btn->attachSingleClickEventCb(&SingleClickCb, NULL);
    btn->attachDoubleClickEventCb(&DoubleClickCb, NULL);
    btn->attachLongPressStartEventCb(&LongPressStartCb, NULL);

    Serial.println("Creating UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    /**
     * Create the simple labels
     */

    /**
     * Try an example. Don't forget to uncomment header.
     * See all the examples online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples
     */
    //  lv_example_btn_1();

    /**
     * Or try out a demo.
     * Don't forget to uncomment header and enable the demos in `lv_conf.h`. E.g. `LV_USE_DEMO_WIDGETS`
     */
    // lv_demo_widgets();
    // lv_demo_benchmark();
    // lv_demo_music();
    // lv_demo_stress();
    // EEZ-Studio UI initialisieren (Screens/Styles/Images werden angelegt)
    ui_init();

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
    serial_bridge::poll();
}
