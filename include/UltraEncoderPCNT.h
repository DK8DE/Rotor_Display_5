#pragma once

#include <Arduino.h>
#include <stdint.h>

extern "C" {
  #include "driver/pulse_cnt.h"
}

/** Optional: wird im UltraEnc-Service-Task direkt nach verarbeiteten logischen Schritten aufgerufen (kein Polling). */
typedef void (*UltraEncoderStepCallback)(void *user, int32_t stepDelta);

/*
  UltraEncoderPCNT
  ----------------
  - Quadratur-Encoder A/B wird ueber PCNT (neuer Treiber) gelesen.
  - Optional kann ein Z-Signal (Index / Marker) ueber einen GPIO-Interrupt angebunden werden.
  - Optional kann anhand des erwarteten Schritt-Abstands zwischen zwei Z-Events eine
    Korrektur der Position vorgenommen werden, um Schrittverluste / Fehlzaehlungen zu kompensieren.

  WICHTIG:
  - PCNT dekodiert nur A/B. Z wird separat ueber GPIO-Interrupt erfasst.
  - Fuer Debug gibt es RAW-Position (nur A/B) und CORR-Position (RAW + Offset durch Z-Korrektur).
*/

// ------------------------------------------------------------
// Feature-Konfiguration (LIB-INTERN)
// ------------------------------------------------------------
#define UEPCNT_ENABLE_SPEED       1
#define UEPCNT_ENABLE_ACCEL       0
#define UEPCNT_ENABLE_RAW_TICKS   1
#define UEPCNT_ENABLE_RANGE       0
#define UEPCNT_ENABLE_UI          0

// ------------------------------------------------------------
// Speed-Parameter
// ------------------------------------------------------------
// Hinweis: Diese Werte beeinflussen nur die Speed-Schaetzung (v), nicht das Positionszaehlen selbst.
// Fuer eine schnellere, aber noch stabile Reaktion auf dem ESP32-C3:
// - Speed-Update alle 5 ms (statt 20 ms)
// - Speed wird nach 80 ms ohne Ticks auf 0 gesetzt
#define UEPCNT_SPEED_UPDATE_MIN_TIME_US 5000UL
#define UEPCNT_SPEED_ZERO_TIMEOUT_US    80000UL

// ------------------------------------------------------------
// Watch-Point Parameter
// ------------------------------------------------------------
// Je kleiner, desto haeufiger ISR-Events (mehr CPU), aber schnellere Reaktion.
// Ticks: Watchpoints erzeugen bei schneller Bewegung fruehere Flush-Events (mehr Interrupts, aber noch C3-tauglich).
#define UEPCNT_DEFAULT_WATCHPOINT_TICKS 300

// ms: Task wird spaetestens in diesem Intervall geweckt, um PCNT-Zaehler zu "flushen" (auch bei langsamem Drehen).
#define UEPCNT_TASK_TIMEOUT_MS          1

/** Default PCNT-Glitchfilter (ns). Mechanische Drehgeber: typ. 0,3–1 ms; 0 = aus. */
#ifndef UEPCNT_DEFAULT_GLITCH_NS
#define UEPCNT_DEFAULT_GLITCH_NS 600000u
#endif

// Encoder-Modus (logische Aufloesung)
// ------------------------------------------------------------
enum UltraEncoderMode {
    ULTRA_MODE_SINGLE = 1,
    ULTRA_MODE_HALF   = 2,
    ULTRA_MODE_FULL   = 4
};

class UltraEncoderPCNT {
public:
    UltraEncoderPCNT(int pinA,
                     int pinB,
                     UltraEncoderMode mode = ULTRA_MODE_FULL,
                     uint8_t cpuCore = 0,
                     uint32_t serviceIntervalMicros = 1000);

    ~UltraEncoderPCNT();

    // Initialisierung:
    //  - accelPercent / threshold sind aktuell nur Kompatibilitaets-Parameter (koennen ignoriert werden)
    //  - ticksPerStep:
    //      0 -> Default abhaengig vom Mode
    //      >0 -> explizit
    //  - glitchNs: PCNT Glitch-Filter (max. Pulsdauer in ns, die noch als Rauschen verworfen wird); 0 = aus
    bool begin(uint8_t accelPercent = 0,
               float accelThresholdStepsPerSec = 500.0f,
               uint8_t ticksPerStep = 0,
               uint32_t glitchNs = UEPCNT_DEFAULT_GLITCH_NS);

    /**
     * Gegenrichtungs-Filter (0 = aus): Nur fuer genau **einen** logischen Schritt (|delta|==1).
     * Schneller Richtungswechsel innerhalb von minMs nach dem letzten Schritt wird verworfen
     * (Rest-Ticks bleiben im PCNT-Rest; kein Positions-/Callback-Sprung).
     * Mehrere Schritte gleicher Richtung oder |delta|>1: ungefiltert (echte schnelle Drehung).
     */
    void setOppositeStepMinMs(uint32_t minMs);

    // --------------------------------------------------------
    // Position / Speed
    // --------------------------------------------------------

    // Standard-Position:
    // - Wenn Z-Korrektur aktiv ist, wird die korrigierte Position zurueckgegeben.
    // - Sonst die RAW-Position.
    long  getPositionSteps() const;

    // Nur RAW (A/B ohne Korrektur)
    long  getPositionStepsRaw() const;

    // Korrigierte Position (RAW + Offset)
    long  getPositionStepsCorrected() const;

    // Speed in Schritten / Sekunde (aus A/B)
    float getSpeedStepsPerSec() const;

    // Position setzen (setzt RAW und setzt Korrektur-Offset auf 0)
    void setPositionSteps(long newPosition);

    // ticksPerStep setzen
    void setTicksPerStep(uint8_t ticksPerStep);

    // Watchpoint-Ticks setzen (optional)
    void setWatchPointTicks(int watchTicks);

    /**
     * Schritt-Callback: stepDelta = +N / -N logische Schritte dieses handleDticks-Aufrufs.
     * Läuft im UltraEnc-Task (kurz halten; für LVGL ggf. Mutex nutzen).
     */
    void setStepCallback(UltraEncoderStepCallback cb, void *user = nullptr);

    // --------------------------------------------------------
    // Z-Unterstuetzung (optional)
    // --------------------------------------------------------

    // Z-Pin anbinden:
    //  - pin: GPIO des Z-Signals
    //  - activeHigh: true -> Z ist HIGH wenn aktiv
    //               false -> Z ist LOW wenn aktiv
    bool attachZ(uint8_t pin, bool activeHigh = true);

    // Z wieder abschalten
    void detachZ();

    // Optionaler Software-Filter fuer Z (gegen Stoerflanken/EMI):
    // - minIntervalUs: Mindestabstand zwischen zwei gueltigen Z-Events (0 = aus)
    // - minAbsStepsBetween: Mindestbewegung in Steps zwischen zwei gueltigen Z-Events (0 = aus)
    void configureZFilter(uint32_t minIntervalUs, long minAbsStepsBetween);

    // Z-Historie/Filter/Korrektur-Referenzen zuruecksetzen (z.B. bei Richtungswechsel)
    // - Z-Pulse-Zaehler bleibt erhalten, damit die Ausgabe im Sketch stabil bleibt.
    void resetZHistory();

    // Anzahl erkannter Z-Events (nach Filter)
    uint32_t getZPulseCount() const;

    // Letzter gemessener Abstand zwischen zwei Z-Events (RAW, in Steps)
    long getLastZDistanceSteps() const;

    // Letzter gemessener Abstand zwischen zwei Z-Events (Zeit, in µs)
    uint32_t getLastZDistanceUs() const;

    // Letzter Fehler (gemessen - erwartet) in Steps, wenn Korrektur aktiv ist
    long getLastZErrorSteps() const;

    // --------------------------------------------------------
    // Z-basierte Korrektur (optional)
    // --------------------------------------------------------

    // Aktiviert die Korrektur anhand des erwarteten Step-Abstands zwischen Z-Events.
    //
    // expectedStepsBetweenZ:
    //   z.B. 500, wenn zwischen zwei Z-Events immer ca. 500 Encoder-Schritte liegen.
    //
    // maxAbsErrorSteps:
    //   Korrektur wird nur angewendet, wenn |Fehler| <= maxAbsErrorSteps.
    //   Damit werden Stoerimpulse/Glitches ignoriert.
    //
    // gain:
    //   1.0  -> harte Korrektur (Offset wird sofort angepasst)
    //   0.2  -> sanfte Korrektur (Offset wird langsam nachgefuehrt)
    void enableZDistanceCorrection(bool enable,
                                   long expectedStepsBetweenZ,
                                   long maxAbsErrorSteps,
                                   float gain = 1.0f);

    // Liefert den aktuell angewendeten Korrektur-Offset (in Steps)
    long getCorrectionOffsetSteps() const;

    // Stop: Task beenden + PCNT freigeben (und Z-Interrupt abschalten)
    void stop();

private:
    // FreeRTOS-Task (wartet auf Watch-Point Events oder Timeout)
    static void serviceTask(void *arg);
    void        serviceLoopOnce();

    // PCNT Setup/Teardown
    bool initPcnt(uint32_t glitchNs);
    void deinitPcnt();

    // Watch-Points konfigurieren
    bool setupWatchPoints();
    void clearWatchPoints();

    // Callback aus ISR, wenn Watch-Point erreicht wird
    static bool IRAM_ATTR onPcntWatchPoint(pcnt_unit_handle_t unit,
                                          const pcnt_watch_event_data_t *edata,
                                          void *user_ctx);

    // Z ISR
    static void IRAM_ATTR isrZ(void *arg);

    // Verarbeitung der A/B dticks
    void handleDticks(int32_t dticks, uint32_t dMicros);

    // Verarbeitung eines (oder mehrerer) Z-Events (wird im Task gemacht)
    void handleZInTask();

private:
    // Pins / Konfig
    int              _pinA;
    int              _pinB;
    UltraEncoderMode _mode;
    uint8_t          _cpuCore;
    uint32_t         _serviceIntervalMicros;

    // PCNT Handles
    pcnt_unit_handle_t    _pcntUnit;
    pcnt_channel_handle_t _pcntChanA;
    pcnt_channel_handle_t _pcntChanB;

    // Task
    TaskHandle_t _taskHandle;
    volatile bool _running;

    // Schritte-Aufloesung
    uint8_t _ticksPerStep;
    float   _invTicksPerStep;

    // Watch-Point
    int _watchTicks;
    int _wpPos;
    int _wpNeg;

    UltraEncoderStepCallback _onStepCb;
    void *_onStepUser;

    /** 0 = Gegenrichtungsfilter aus; sonst ms zwischen Einzelschritt und erlaubter Gegenrichtung */
    uint32_t _oppStepMinMs;
    int8_t   _lastEmittedStepSign;
    uint32_t _lastEmittedStepMs;

    // Zustaende
    int64_t  _positionSteps;       // RAW-Position (nur A/B)
    int32_t  _tickRemainder;       // Tick-Rest fuer stepDelta

    // Korrektur (RAW + Offset)
    int64_t  _corrOffsetSteps;     // wird ueber Z nachgefuehrt
    bool     _useCorrectedAsDefault;

#if UEPCNT_ENABLE_SPEED
    float    _speedStepsPerSec;
    int64_t  _speedTicksAcc;
    uint32_t _speedTimeAccMicros;
    uint32_t _noMoveTimeAccMicros;
#endif

    // Zeitmessung
    uint32_t _lastMicros;

    // --------------------------------------------------------
    // Z (optional)
    // --------------------------------------------------------
    bool     _zEnabled;
    uint8_t  _zPin;
    bool     _zActiveHigh;

    // ISR -> Task Uebergabe:
    volatile uint32_t _zPendingCount;   // wie viele Z-Events seit letztem Task-Durchlauf
    volatile uint32_t _zPendingUs;      // Timestamp des letzten Z-Events (µs)

    // Filter / Historie (im Task)
    uint32_t _zMinIntervalUs;
    int64_t  _zMinAbsStepsBetween;

    bool     _hasLastZ;
    uint32_t _lastZUsAccepted;
    int64_t  _lastZPosAcceptedRaw;

    // ausgewertete Z-Daten (nach Filter)
    uint32_t _zPulseCount;
    uint32_t _lastZDistanceUs;
    int64_t  _lastZDistanceStepsRaw;

    // Korrektur-Status
    bool     _zCorrEnabled;
    int64_t  _zExpectedStepsAbs;
    int64_t  _zMaxAbsErrorSteps;
    float    _zCorrGain;
    int64_t  _lastZErrorSteps;
};
