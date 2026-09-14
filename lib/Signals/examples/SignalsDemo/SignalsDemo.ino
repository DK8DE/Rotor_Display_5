/*
  SignalsDemo (ESP32)
  -------------------
  NeoPixel-Daten an GPIO39, Lautsprecher PWM an GPIO40.

  Dieses Beispiel:
  - spielt ein kleines LED-Demo (Farben + Lauflicht)
  - spielt kurze Toene (mit Dauerangabe)

  API:
  - sig.begin(numLeds, neoPin, speakerPin)
  - sig.service()                           // Ton-Dauer (nicht-blockierend)
  - sig.clear() / sig.show() / setAutoShow
  - sig.setAll / sig.setPixel (brightness 0..100)
  - sig.tone(freq, vol, durMs)              // vol 0..50
  - sig.stopTone()
*/

#include <Signals.h>

Signals sig;

void setup()
{
  sig.begin(16, 39, 40);
  sig.clear();
  sig.stopTone();
  delay(200);

  sig.tone(440, 20, 200);
  delay(300);
}

void loop()
{
  sig.service();

  sig.setAll(255, 0, 0, 80);
  sig.tone(440, 10, 150);
  delay(600);
  sig.service();

  sig.setAll(0, 255, 0, 80);
  sig.tone(660, 10, 150);
  delay(600);
  sig.service();

  sig.setAll(0, 0, 255, 80);
  sig.tone(880, 10, 150);
  delay(600);
  sig.service();

  sig.setAutoShow(false);
  sig.clear();

  for (uint8_t i = 0; i < 16; i++) {
    sig.clear();
    sig.setPixel(i, 255, 255, 255, 60);
    sig.show();
    sig.tone(1200, 5, 40);
    delay(80);
    sig.service();
  }

  sig.setAutoShow(true);
  sig.clear();
  delay(400);
}
