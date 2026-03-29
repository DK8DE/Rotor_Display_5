#include "UltraEncoderPCNT.h"
#include <math.h>

/*
  Implementierungshinweise
  ------------------------
  - Die PCNT-Auswertung laeuft in einer Task, die durch Watch-Points oder Timeout geweckt wird.
  - Z wird per GPIO-Interrupt gezaehlt und per "pending" an die Task uebergeben.
  - Damit vermeiden wir lange Arbeit in ISR und bekommen stabile Auswertung.

  Z-Korrektur:
  - Wir speichern RAW (A/B) und halten einen Offset, der bei jedem gueltigen Z-Event nachgefuehrt wird.
  - Dadurch koennen verlorene/zusaetzliche Schritte kompensiert werden.
*/

static constexpr float UEPCNT_INV_MICROS_TO_SEC = 1.0f / 1000000.0f;

// ------------------------------------------------------------
// Konstruktor / Destruktor
// ------------------------------------------------------------

UltraEncoderPCNT::UltraEncoderPCNT(int pinA,
                                   int pinB,
                                   UltraEncoderMode mode,
                                   uint8_t cpuCore,
                                   uint32_t serviceIntervalMicros)
: _pinA(pinA),
  _pinB(pinB),
  _mode(mode),
  _cpuCore(cpuCore),
  _serviceIntervalMicros(serviceIntervalMicros),
  _pcntUnit(nullptr),
  _pcntChanA(nullptr),
  _pcntChanB(nullptr),
  _taskHandle(nullptr),
  _running(false),
  _ticksPerStep(4),
  _invTicksPerStep(1.0f / 4.0f),
  _watchTicks(UEPCNT_DEFAULT_WATCHPOINT_TICKS),
  _wpPos(UEPCNT_DEFAULT_WATCHPOINT_TICKS),
  _wpNeg(-UEPCNT_DEFAULT_WATCHPOINT_TICKS),
  _onStepCb(nullptr),
  _onStepUser(nullptr),
  _positionSteps(0),
  _tickRemainder(0),
  _corrOffsetSteps(0),
  _useCorrectedAsDefault(true),
#if UEPCNT_ENABLE_SPEED
  _speedStepsPerSec(0.0f),
  _speedTicksAcc(0),
  _speedTimeAccMicros(0),
  _noMoveTimeAccMicros(0),
#endif
  _lastMicros(0),
  _zEnabled(false),
  _zPin(255),
  _zActiveHigh(true),
  _zPendingCount(0),
  _zPendingUs(0),
  _zMinIntervalUs(0),
  _zMinAbsStepsBetween(0),
  _hasLastZ(false),
  _lastZUsAccepted(0),
  _lastZPosAcceptedRaw(0),
  _zPulseCount(0),
  _lastZDistanceUs(0),
  _lastZDistanceStepsRaw(0),
  _zCorrEnabled(false),
  _zExpectedStepsAbs(0),
  _zMaxAbsErrorSteps(0),
  _zCorrGain(1.0f),
  _lastZErrorSteps(0)
{
    if (_serviceIntervalMicros == 0) {
        _serviceIntervalMicros = 1000;
    }
}

UltraEncoderPCNT::~UltraEncoderPCNT() {
    stop();
}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------

bool UltraEncoderPCNT::begin(uint8_t accelPercent,
                            float accelThresholdStepsPerSec,
                            uint8_t ticksPerStep,
                            uint32_t glitchNs)
{
    (void)accelPercent;
    (void)accelThresholdStepsPerSec;

    // ticksPerStep bestimmen
    if (ticksPerStep == 0) {
        switch (_mode) {
            case ULTRA_MODE_SINGLE: _ticksPerStep = 1; break;
            case ULTRA_MODE_HALF:   _ticksPerStep = 2; break;
            case ULTRA_MODE_FULL:
            default:                _ticksPerStep = 4; break;
        }
    } else {
        _ticksPerStep = ticksPerStep;
        if (_ticksPerStep == 0) _ticksPerStep = 1;
    }
    _invTicksPerStep = 1.0f / (float)_ticksPerStep;

    // Wenn schon aktiv: sauber stoppen (inkl. PCNT/Task)
    // Z bleibt dabei deaktiviert, weil stop() detachZ() macht.
    // Falls du Z dauerhaft willst: nach begin() wieder attachZ() aufrufen.
    stop();

    // PCNT initialisieren
    if (!initPcnt(glitchNs)) {
        return false;
    }

    // Watch-Points einrichten
    if (!setupWatchPoints()) {
        deinitPcnt();
        return false;
    }

    // Startwerte
    _positionSteps = 0;
    _tickRemainder = 0;
    _corrOffsetSteps = 0;

#if UEPCNT_ENABLE_SPEED
    _speedStepsPerSec    = 0.0f;
    _speedTicksAcc       = 0;
    _speedTimeAccMicros  = 0;
    _noMoveTimeAccMicros = 0;
#endif

    // Z-Statistiken zuruecksetzen (Z selber ist nach stop() ohnehin aus)
    _zPendingCount = 0;
    _zPendingUs = 0;
    _hasLastZ = false;
    _lastZUsAccepted = 0;
    _lastZPosAcceptedRaw = 0;
    _zPulseCount = 0;
    _lastZDistanceUs = 0;
    _lastZDistanceStepsRaw = 0;
    _lastZErrorSteps = 0;

    // Zeitbasis
    _lastMicros = micros();

    _running = true;

    // Task starten: wartet auf Events oder Timeout (kein Busy-Wait!)
    /* Priorität > idle, damit Schritte zeitnah nach PCNT-Flush verarbeitet werden (feine Auflösung). */
    BaseType_t res = xTaskCreatePinnedToCore(
        UltraEncoderPCNT::serviceTask,
        "UltraEncTask",
        4096,
        this,
        3,
        &_taskHandle,
        _cpuCore
    );

    if (res != pdPASS) {
        _running = false;
        _taskHandle = nullptr;
        clearWatchPoints();
        deinitPcnt();
        return false;
    }

    return true;
}

long UltraEncoderPCNT::getPositionSteps() const {
    return _useCorrectedAsDefault ? (long)(_positionSteps + _corrOffsetSteps) : (long)_positionSteps;
}

long UltraEncoderPCNT::getPositionStepsRaw() const {
    return (long)_positionSteps;
}

long UltraEncoderPCNT::getPositionStepsCorrected() const {
    return (long)(_positionSteps + _corrOffsetSteps);
}

float UltraEncoderPCNT::getSpeedStepsPerSec() const {
#if UEPCNT_ENABLE_SPEED
    return _speedStepsPerSec;
#else
    return 0.0f;
#endif
}

void UltraEncoderPCNT::setPositionSteps(long newPosition) {
    // RAW setzen
    _positionSteps = (int64_t)newPosition;
    _tickRemainder = 0;

    // Korrektur zuruecksetzen (damit "newPosition" auch wirklich der neue Referenzwert ist)
    _corrOffsetSteps = 0;

#if UEPCNT_ENABLE_SPEED
    _speedStepsPerSec    = 0.0f;
    _speedTicksAcc       = 0;
    _speedTimeAccMicros  = 0;
    _noMoveTimeAccMicros = 0;
#endif

    // Z-Historie zuruecksetzen (damit dz beim naechsten Z sauber startet)
    _hasLastZ = false;
    _lastZUsAccepted = 0;
    _lastZPosAcceptedRaw = 0;
    _lastZDistanceUs = 0;
    _lastZDistanceStepsRaw = 0;
    _lastZErrorSteps = 0;
}

void UltraEncoderPCNT::setTicksPerStep(uint8_t ticksPerStep) {
    if (ticksPerStep == 0) ticksPerStep = 1;
    _ticksPerStep = ticksPerStep;
    _invTicksPerStep = 1.0f / (float)_ticksPerStep;
}

void UltraEncoderPCNT::setWatchPointTicks(int watchTicks) {
    if (watchTicks < 10) watchTicks = 10;
    if (watchTicks > 20000) watchTicks = 20000;

    _watchTicks = watchTicks;
    _wpPos = +watchTicks;
    _wpNeg = -watchTicks;

    // Wenn schon aktiv: Watch-Points neu setzen
    if (_pcntUnit) {
        clearWatchPoints();
        setupWatchPoints();
    }
}

// ------------------------------------------------------------
// Z (optional) - Public
// ------------------------------------------------------------

bool UltraEncoderPCNT::attachZ(uint8_t pin, bool activeHigh) {
    detachZ();

    _zPin = pin;
    _zActiveHigh = activeHigh;
    _zEnabled = true;

    // Z-Historie zuruecksetzen (damit beim ersten Z kein Muell entsteht)
    _zPendingCount = 0;
    _zPendingUs = 0;
    _hasLastZ = false;
    _lastZUsAccepted = 0;
    _lastZPosAcceptedRaw = 0;
    _zPulseCount = 0;
    _lastZDistanceUs = 0;
    _lastZDistanceStepsRaw = 0;
    _lastZErrorSteps = 0;

    pinMode(_zPin, INPUT);

    // Welche Flanke ist das "Z-Event"?
    int mode = _zActiveHigh ? RISING : FALLING;

    // attachInterruptArg erlaubt "this" als Context
    attachInterruptArg(_zPin, UltraEncoderPCNT::isrZ, this, mode);

    return true;
}

void UltraEncoderPCNT::detachZ() {
    if (_zEnabled && _zPin != 255) {
        detachInterrupt(_zPin);
    }

    _zEnabled = false;
    _zPin = 255;

    _zPendingCount = 0;
    _zPendingUs = 0;
    _hasLastZ = false;
    _lastZUsAccepted = 0;
    _lastZPosAcceptedRaw = 0;
    _zPulseCount = 0;
    _lastZDistanceUs = 0;
    _lastZDistanceStepsRaw = 0;
    _lastZErrorSteps = 0;
}

void UltraEncoderPCNT::configureZFilter(uint32_t minIntervalUs, long minAbsStepsBetween) {
    _zMinIntervalUs = minIntervalUs;
    _zMinAbsStepsBetween = (int64_t)llabs((long long)minAbsStepsBetween);
}

void UltraEncoderPCNT::resetZHistory() {
    // Pending atomar loeschen
    noInterrupts();
    _zPendingCount = 0;
    _zPendingUs = 0;
    interrupts();

    // Historie/Filter-Referenzen zuruecksetzen
    _hasLastZ = false;
    _lastZUsAccepted = 0;
    _lastZPosAcceptedRaw = 0;

    // Letzte Abstaende/Fehler zuruecksetzen (Z-Pulse-Zaehler bleibt!)
    _lastZDistanceUs = 0;
    _lastZDistanceStepsRaw = 0;
    _lastZErrorSteps = 0;
}

uint32_t UltraEncoderPCNT::getZPulseCount() const {
    return _zPulseCount;
}

long UltraEncoderPCNT::getLastZDistanceSteps() const {
    return (long)_lastZDistanceStepsRaw;
}

uint32_t UltraEncoderPCNT::getLastZDistanceUs() const {
    return _lastZDistanceUs;
}

long UltraEncoderPCNT::getLastZErrorSteps() const {
    return (long)_lastZErrorSteps;
}

void UltraEncoderPCNT::enableZDistanceCorrection(bool enable,
                                                 long expectedStepsBetweenZ,
                                                 long maxAbsErrorSteps,
                                                 float gain)
{
    _zCorrEnabled = enable;

    _zExpectedStepsAbs = (int64_t)llabs((long long)expectedStepsBetweenZ);
    _zMaxAbsErrorSteps = (int64_t)llabs((long long)maxAbsErrorSteps);

    if (gain < 0.0f) gain = 0.0f;
    if (gain > 1.0f) gain = 1.0f;
    _zCorrGain = gain;

    _lastZErrorSteps = 0;

    // Wenn eingeschaltet wird: Offset nicht automatisch aendern, aber Historie resetten,
    // damit die erste Korrektur erst nach zwei gueltigen Z-Events passiert.
    _hasLastZ = false;
    _lastZUsAccepted = 0;
    _lastZPosAcceptedRaw = 0;
}

long UltraEncoderPCNT::getCorrectionOffsetSteps() const {
    return (long)_corrOffsetSteps;
}

// ------------------------------------------------------------
// stop()
// ------------------------------------------------------------

void UltraEncoderPCNT::setStepCallback(UltraEncoderStepCallback cb, void *user) {
    _onStepCb = cb;
    _onStepUser = user;
}

void UltraEncoderPCNT::stop() {
    if (!_running && !_pcntUnit) return;

    _onStepCb = nullptr;
    _onStepUser = nullptr;
    _running = false;

    // Task wecken, falls blockiert
    if (_taskHandle) {
        xTaskNotifyGive(_taskHandle);
    }

    // Task loeschen
    if (_taskHandle) {
        TaskHandle_t h = _taskHandle;
        _taskHandle = nullptr;
        vTaskDelete(h);
    }

    // Z abschalten (damit keine ISR weiterfeuert)
    detachZ();

    clearWatchPoints();
    deinitPcnt();
}

// ------------------------------------------------------------
// FreeRTOS Task (Event-driven via Watch-Points)
// ------------------------------------------------------------

void UltraEncoderPCNT::serviceTask(void *arg) {
    UltraEncoderPCNT *self = static_cast<UltraEncoderPCNT*>(arg);
    if (!self) {
        vTaskDelete(nullptr);
        return;
    }

    while (self->_running) {
        // Warten auf:
        // - Watch-Point Event (ISR -> notify)
        // - oder Timeout (damit langsame Bewegung/Stillstand sauber abgeholt wird)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UEPCNT_TASK_TIMEOUT_MS));

        self->serviceLoopOnce();
    }

    vTaskDelete(nullptr);
}

// ------------------------------------------------------------
// PCNT Callback (ISR) - Watch-Point erreicht
// ------------------------------------------------------------

bool IRAM_ATTR UltraEncoderPCNT::onPcntWatchPoint(pcnt_unit_handle_t unit,
                                                  const pcnt_watch_event_data_t *edata,
                                                  void *user_ctx)
{
    (void)unit;
    (void)edata;

    UltraEncoderPCNT *self = static_cast<UltraEncoderPCNT*>(user_ctx);
    if (!self) return false;

    if (!self->_running || !self->_taskHandle) return false;

    BaseType_t hpTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(self->_taskHandle, &hpTaskWoken);
    return (hpTaskWoken == pdTRUE);
}

// ------------------------------------------------------------
// Z ISR (nur Timestamp + pending count; Auswertung im Task)
// ------------------------------------------------------------

void IRAM_ATTR UltraEncoderPCNT::isrZ(void *arg) {
    UltraEncoderPCNT *self = static_cast<UltraEncoderPCNT*>(arg);
    if (!self) return;

    if (!self->_zEnabled) return;

    uint32_t nowUs = (uint32_t)micros();

    // Pending zaehlen (volatile++ vermeiden)
    self->_zPendingCount = self->_zPendingCount + 1;
    self->_zPendingUs = nowUs;

    // Task wecken, damit Z zeitnah ausgewertet wird
    if (self->_running && self->_taskHandle) {
        BaseType_t hpTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(self->_taskHandle, &hpTaskWoken);
    }
}

// ------------------------------------------------------------
// serviceLoopOnce(): Count abholen und auf 0 setzen
// ------------------------------------------------------------

void UltraEncoderPCNT::serviceLoopOnce() {
    if (!_pcntUnit) return;

    // Zeitdelta
    uint32_t nowMicros = micros();
    uint32_t dMicros = nowMicros - _lastMicros;
    _lastMicros = nowMicros;

    if (dMicros == 0) return;

    // Aktuellen Count lesen
    int cnt32 = 0;
    esp_err_t err = pcnt_unit_get_count(_pcntUnit, &cnt32);
    if (err != ESP_OK) {
        return;
    }

    int32_t dticks = (int32_t)cnt32;

    // Count zuruecksetzen
    pcnt_unit_clear_count(_pcntUnit);

    // A/B verarbeiten
    handleDticks(dticks, dMicros);

    // Z verarbeiten (falls Events anstehen)
    handleZInTask();
}

// ------------------------------------------------------------
// Z-Auswertung im Task
// ------------------------------------------------------------

void UltraEncoderPCNT::handleZInTask() {
    if (!_zEnabled) return;

    // Pending atomar holen und loeschen
    uint32_t pendingCount;
    uint32_t pendingUs;

    noInterrupts();
    pendingCount = _zPendingCount;
    pendingUs = _zPendingUs;
    _zPendingCount = 0;
    interrupts();

    if (pendingCount == 0) return;

    // Wenn mehrere Events in einem Task-Zyklus kamen, koennen wir deren Zwischenzeiten nicht rekonstruieren.
    // Wir behandeln das als EIN Event (mit dem letzten Timestamp) und lassen Korrektur ggf. aus.
    bool burst = (pendingCount > 1);

    // RAW-Position zum Zeitpunkt der Auswertung (Task) - konsistent mit A/B-Verarbeitung
    int64_t posNowRaw = _positionSteps;

    // --------------------------------------------------------
    // Filter: Mindestzeit / Mindestbewegung
    // --------------------------------------------------------
    if (_zMinIntervalUs > 0 && _hasLastZ) {
        uint32_t dtUs = pendingUs - _lastZUsAccepted;
        if (dtUs < _zMinIntervalUs) {
            // Stoerflanke -> ignorieren
            return;
        }
    }

    if (_zMinAbsStepsBetween > 0 && _hasLastZ) {
        int64_t dPos = posNowRaw - _lastZPosAcceptedRaw;
        if (llabs(dPos) < _zMinAbsStepsBetween) {
            // Stoerflanke ohne echte Bewegung -> ignorieren
            return;
        }
    }

    // Event akzeptieren
    _zPulseCount += 1;

    // Abstand in Zeit / Steps (RAW)
    if (_hasLastZ) {
        _lastZDistanceUs = pendingUs - _lastZUsAccepted;
        _lastZDistanceStepsRaw = posNowRaw - _lastZPosAcceptedRaw;
    } else {
        _lastZDistanceUs = 0;
        _lastZDistanceStepsRaw = 0;
    }

    _lastZUsAccepted = pendingUs;
    _lastZPosAcceptedRaw = posNowRaw;
    _hasLastZ = true;

    // --------------------------------------------------------
    // Z-Korrektur
    // --------------------------------------------------------
    // Korrektur nur sinnvoll, wenn:
    // - aktiviert
    // - ein erwarteter Abstand gesetzt ist
    // - wir einen gueltigen Abstand haben (nicht erster Z)
    // - kein Burst (mehrere Z in einem Task-Durchlauf) -> unsicher
    if (_zCorrEnabled && _zExpectedStepsAbs > 0 && _lastZDistanceUs != 0 && !burst) {
        int64_t dz = _lastZDistanceStepsRaw;

        // Richtung anhand Vorzeichen bestimmen
        if (dz != 0) {
            int64_t expected = (dz > 0) ? _zExpectedStepsAbs : -_zExpectedStepsAbs;
            int64_t errSteps = dz - expected;

            _lastZErrorSteps = errSteps;

            // Nur korrigieren, wenn Fehler plausibel klein ist
            if (_zMaxAbsErrorSteps == 0 || llabs(errSteps) <= _zMaxAbsErrorSteps) {
                // Offset so aendern, dass korrigierte Position "auf Raster" liegt:
                // corr = raw + offset
                // Wenn dz zu gross war (wir haben zu viele Steps gezaehlt), dann offset kleiner machen usw.
                float corrF = (float)errSteps * _zCorrGain;

                // Auf ganze Steps runden
                int64_t corrSteps = (int64_t)lroundf(corrF);

                // Offset anpassen (Fehler wegdruecken)
                _corrOffsetSteps -= corrSteps;
            }
        }
    } else {
        // wenn keine Korrektur aktiv: Fehler 0 lassen
        _lastZErrorSteps = 0;
    }
}

// ------------------------------------------------------------
// handleDticks(): Tick->Step + Speed
// ------------------------------------------------------------

/** Floor-Division a/b fuer b>0. C++-Truncation bei negativem a liefert z.B. -1/2 == 0 und frisst den
 * ersten Tick nach jedem Richtungswechsel (Rest + neuer Tick ergeben keinen Schritt). */
static int32_t uepcnt_div_floor_ticks(int32_t a, int32_t b)
{
    if (b <= 0) {
        return 0;
    }
    if (a >= 0) {
        return a / b;
    }
    const int64_t an = -(int64_t)a;
    return -(int32_t)((an + (int64_t)b - 1) / (int64_t)b);
}

void UltraEncoderPCNT::handleDticks(int32_t dticks, uint32_t dMicros) {
    int32_t totalTicks = _tickRemainder + dticks;

    int32_t stepDelta = 0;
    int32_t remainder = 0;

    if (_ticksPerStep > 0) {
        const int32_t b = (int32_t)_ticksPerStep;
        stepDelta = uepcnt_div_floor_ticks(totalTicks, b);
        remainder = totalTicks - stepDelta * b;
    } else {
        stepDelta = totalTicks;
        remainder = 0;
    }

    _tickRemainder = remainder;
    _positionSteps += (int64_t)stepDelta;

    if (stepDelta != 0 && _onStepCb) {
        _onStepCb(_onStepUser, stepDelta);
    }

#if UEPCNT_ENABLE_SPEED
    if (dticks == 0) {
        _noMoveTimeAccMicros += dMicros;
    } else {
        _noMoveTimeAccMicros = 0;
    }

    _speedTicksAcc += dticks;
    _speedTimeAccMicros += dMicros;

    if (_noMoveTimeAccMicros >= UEPCNT_SPEED_ZERO_TIMEOUT_US) {
        _speedStepsPerSec = 0.0f;

        _speedTicksAcc = 0;
        _speedTimeAccMicros = 0;
        _noMoveTimeAccMicros = 0;
        return;
    }

    if (_speedTimeAccMicros >= UEPCNT_SPEED_UPDATE_MIN_TIME_US) {
        if (_speedTicksAcc != 0) {
            float dtSec = (float)_speedTimeAccMicros * UEPCNT_INV_MICROS_TO_SEC;
            float stepsFromTicks = (float)_speedTicksAcc * _invTicksPerStep;
            _speedStepsPerSec = stepsFromTicks / dtSec;
        } else {
            _speedStepsPerSec = 0.0f;
        }

        _speedTicksAcc = 0;
        _speedTimeAccMicros = 0;
    }
#endif
}

// ------------------------------------------------------------
// PCNT Setup/Teardown
// ------------------------------------------------------------

bool UltraEncoderPCNT::initPcnt(uint32_t glitchNs) {
    deinitPcnt();

    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = 32767;
    unit_config.low_limit  = -32768;
    unit_config.flags.accum_count = true;

    esp_err_t err = pcnt_new_unit(&unit_config, &_pcntUnit);
    if (err != ESP_OK || !_pcntUnit) {
        _pcntUnit = nullptr;
        return false;
    }

    if (glitchNs > 0) {
        pcnt_glitch_filter_config_t filter_config = {};
        filter_config.max_glitch_ns = glitchNs;
        pcnt_unit_set_glitch_filter(_pcntUnit, &filter_config);
    }

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num  = (gpio_num_t)_pinA;
    chan_a_config.level_gpio_num = (gpio_num_t)_pinB;
    err = pcnt_new_channel(_pcntUnit, &chan_a_config, &_pcntChanA);
    if (err != ESP_OK || !_pcntChanA) {
        deinitPcnt();
        return false;
    }

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num  = (gpio_num_t)_pinB;
    chan_b_config.level_gpio_num = (gpio_num_t)_pinA;
    err = pcnt_new_channel(_pcntUnit, &chan_b_config, &_pcntChanB);
    if (err != ESP_OK || !_pcntChanB) {
        deinitPcnt();
        return false;
    }

    // 4x Quadratur
    err = pcnt_channel_set_edge_action(
        _pcntChanA,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE
    );
    if (err != ESP_OK) { deinitPcnt(); return false; }

    err = pcnt_channel_set_level_action(
        _pcntChanA,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    );
    if (err != ESP_OK) { deinitPcnt(); return false; }

    err = pcnt_channel_set_edge_action(
        _pcntChanB,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE
    );
    if (err != ESP_OK) { deinitPcnt(); return false; }

    err = pcnt_channel_set_level_action(
        _pcntChanB,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    );
    if (err != ESP_OK) { deinitPcnt(); return false; }

    err = pcnt_unit_enable(_pcntUnit);
    if (err != ESP_OK) { deinitPcnt(); return false; }

    err = pcnt_unit_clear_count(_pcntUnit);
    if (err != ESP_OK) { deinitPcnt(); return false; }

    err = pcnt_unit_start(_pcntUnit);
    if (err != ESP_OK) { deinitPcnt(); return false; }

    return true;
}

void UltraEncoderPCNT::deinitPcnt() {
    if (_pcntUnit) {
        pcnt_unit_stop(_pcntUnit);
        pcnt_unit_disable(_pcntUnit);
    }

    if (_pcntChanA) {
        pcnt_del_channel(_pcntChanA);
        _pcntChanA = nullptr;
    }

    if (_pcntChanB) {
        pcnt_del_channel(_pcntChanB);
        _pcntChanB = nullptr;
    }

    if (_pcntUnit) {
        pcnt_del_unit(_pcntUnit);
        _pcntUnit = nullptr;
    }
}

// ------------------------------------------------------------
// Watch-Points
// ------------------------------------------------------------

bool UltraEncoderPCNT::setupWatchPoints() {
    if (!_pcntUnit) return false;

    esp_err_t err;

    err = pcnt_unit_add_watch_point(_pcntUnit, _wpPos);
    if (err != ESP_OK) return false;

    err = pcnt_unit_add_watch_point(_pcntUnit, _wpNeg);
    if (err != ESP_OK) return false;

    pcnt_event_callbacks_t cbs = {};
    cbs.on_reach = UltraEncoderPCNT::onPcntWatchPoint;

    err = pcnt_unit_register_event_callbacks(_pcntUnit, &cbs, this);
    if (err != ESP_OK) return false;

    return true;
}

void UltraEncoderPCNT::clearWatchPoints() {
    if (!_pcntUnit) return;

    pcnt_unit_remove_watch_point(_pcntUnit, _wpPos);
    pcnt_unit_remove_watch_point(_pcntUnit, _wpNeg);
}
