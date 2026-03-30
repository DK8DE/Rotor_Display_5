/**
 * UltraEncoderPCNT testen (gleiche Lib wie Haupt-App).
 * encoder_test:  pio run  |  pio run -t upload  |  pio device monitor
 *
 * - begin(): Default-Glitch (UEPCNT_DEFAULT_GLITCH_NS in UltraEncoderPCNT.h)
 * - setOppositeStepMinMs(): Gegenrichtungs-Filter in der Lib (Einzel-Schritt)
 */

#include <Arduino.h>
#include <stdio.h>

#include "UltraEncoderPCNT.h"

static constexpr int GPIO_KNOB_PIN_A = 5;
static constexpr int GPIO_KNOB_PIN_B = 6;

/** Optional: explizit Glitch (0 = Default aus Header) */
static constexpr uint32_t GLITCH_NS = UEPCNT_DEFAULT_GLITCH_NS;

/** Gegenrichtung: Einzelschritt erst nach so vielen ms (0 = aus) */
static constexpr uint32_t OPPOSITE_STEP_MS = 42u;

static UltraEncoderPCNT *s_enc = nullptr;

static void on_step(void *user, int32_t step_delta)
{
    auto *enc = static_cast<UltraEncoderPCNT *>(user);
    const long pos = enc->getPositionSteps();
    char line[48];
    snprintf(line, sizeof(line), "%ld\t%ld\r\n", (long)step_delta, (long)pos);
    Serial.print(line);
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.printf(
        "encoder-usb-test: UltraEncoderPCNT HALF, glitch=%lu ns, oppMs=%u\r\n",
        (unsigned long)GLITCH_NS,
        (unsigned)OPPOSITE_STEP_MS);
    Serial.println("FORMAT: delta\tposition");

    s_enc = new UltraEncoderPCNT(GPIO_KNOB_PIN_A, GPIO_KNOB_PIN_B, ULTRA_MODE_HALF, 0, 1000);
    if (!s_enc->begin(0, 500.0f, 0, GLITCH_NS)) {
        Serial.println("begin() failed");
        while (true) {
            delay(1000);
        }
    }
    s_enc->setOppositeStepMinMs(OPPOSITE_STEP_MS);
    s_enc->setStepCallback(on_step, s_enc);
}

void loop()
{
    delay(500);
}
