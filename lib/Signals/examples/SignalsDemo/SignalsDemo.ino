/*
  SignalsDemo (ESP32)
  -------------------
  TX an GPIO17 -> zum RX-Pin des ATtiny (PA2)
  GND muss gemeinsam sein.

  Dieses Beispiel:
  - spielt ein kleines LED-Demo (Farben + Lauflicht)
  - spielt kurze Toene (mit Dauerangabe)

    Verfügbare Funktionen aus Signals (wie im Beispiel genutzt):
  - sig.begin(txPin, rxPin, baud)           // Startet die serielle Verbindung (ESP32 UART)
  - sig.clear()                             // LEDs aus
  - sig.show()                              // Anzeigen (wenn AutoShow aus)
  - sig.setAutoShow(true/false)             // Auto-Show ein/aus
  - sig.setAll(r,g,b,bright)                // Alle LEDs setzen
  - sig.setPixel(index,r,g,b,bright)        // Einzelne LED setzen
  - sig.tone(freq, vol, durMs)              // Ton (freq 200..2000, vol 0..50, Länge durMs in ms)
  - sig.stopTone()                          // Ton aus

  Hinweis:
  - Der ATtiny-Empfaenger nutzt ein ASCII-Protokoll.
  - Fuer schnelle LED-Updates: AutoShow aus, viele Pixel setzen, dann show().
*/

#include <Signals.h>

// Wir nutzen Serial1 (ESP32). TX = GPIO17
static const int8_t TX_PIN = 17;
// RX brauchen wir nicht
static const int8_t RX_PIN = -1;

Signals sig(Serial1);

void setup()
{
  // Serielle Verbindung zum ATtiny starten
  sig.begin(TX_PIN, RX_PIN, 115200);

  // Alles aus
  sig.clear();
  sig.stopTone();

  delay(200);

  // Kurzer Ton: 440 Hz, Lautstaerke 20/50, 200ms
  sig.tone(440, 20, 200);
  delay(300);
}

void loop()
{
  // 1) Alle LEDs rot/gruen/blau
  sig.setAll(255, 0, 0, 80);
  sig.tone(440, 10, 150);
  delay(600);

  sig.setAll(0, 255, 0, 80);
  sig.tone(660, 10, 150);
  delay(600);

  sig.setAll(0, 0, 255, 80);
  sig.tone(880, 10, 150);
  delay(600);

  // 2) Lauflicht (schnell) -> AutoShow aus, dann show()
  sig.setAutoShow(false);
  sig.clear();

  for (uint8_t i = 0; i < 16; i++)
  {
    // Einen Punkt setzen
    sig.clear();
    sig.setPixel(i, 255, 255, 255, 60);

    // Jetzt anzeigen
    sig.show();

    // Kleiner Klick-Ton
    sig.tone(1200, 5, 40);

    delay(120);
  }

  sig.setAutoShow(true);

  // 3) Alles aus + kurzer Sweep
  sig.clear();
  sig.tone(200, 30, 120);  delay(140);
  sig.tone(400, 30, 120);  delay(140);
  sig.tone(800, 30, 120);  delay(140);
  sig.tone(1200, 30, 120); delay(140);
  sig.tone(1600, 30, 120); delay(140);
  sig.tone(2000, 30, 120); delay(500);
}
