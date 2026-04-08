/**
 * RS485-Telegramme: #SRC:DST:CMD:PARAMS:CS$
 * CS = SRC + DST + letzte Zahl in PARAMS (float), wie RotorTcpBridge/rs485_protocol.py — kein scaled100.
 * CS-Format wie _fmt_cs: ganzzahlig ohne ",00", sonst bis zu zwei Nachkommastellen (Komma).
 *
 * Bus-Regel: Pro Anfrage ein ausstehendes Telegramm; nächste Anfrage erst nach ACK (oder Timeout).
 * SETASELECT (DST 255): kein Pending/ACK — sofort senden.
 * SETPOSCC: Encoder-Vorschau nur im Mitläufer-Modus an den fremden Master (DST = dessen SRC-ID), nicht an den
 * Rotor-Slave (NOTIMPL). Eigenbetrieb: kein SETPOSCC — niemand konsumiert es.
 * SETCONIDF / SETCONTID (DST 255): neue Controller-master_id → config.json + ACK_SETCONIDF bzw. ACK_SETCONTID.
 * GETANEMO/GETTEMPA/GETWINDDIR/GETTEMPM: nur Stillstand, ohne Fremd-PC; ACKs auch bei PC-Mitlesen.
 * GETCONANO/SETCONANO: Anemometer/Wetter-Tab (1/0) in config.json; bei 0 weiter GETTEMPA (Außentemp Rotor-Info).
 * GETCONDELTA/SETCONDELTA: Encoder-Schritt 1 oder 10 Zehntelgrad (0,1° bzw. 1° pro Raste) → config encoder_delta.
 * GETCONCHA/SETCONCHA: Antennenwechsel 1 = taget behalten + SETPOS, 0 = taget = Ist-Anzeige (config concha).
 * GETCONLEDP/SETCONLEDP: NeoPixel-Ring global 0…100 % (config.json conledp auf FFat).
 * GETTEMPM zyklisch alle ROTOR_MOTOR_TEMP_POLL_MS (5 s); Wetter-GETs unverändert gestaffelt.
 *
 * Verbindung: Fehler 10 (Verbindungstimeout), wenn länger kein Telegramm vom Slave (SRC=Rotor-ID)
 * oder vor erster Antwort nur Timeouts — PC- oder Controller-Abfragen zählen (ROTOR_CONN_LOST_TIMEOUT_MS).
 * Fehler 10: GETERR-Recovery bis ACK_ERR (nicht GETREF zuerst — ref=1 trotz Slave-Fehler möglich).
 * Boot: zuerst 3× TEST:0 (Ping, je ROTOR_RS485_ACK_TIMEOUT_MS); ohne ACK_TEST/NAK_TEST → sofort Fehler 10.
 *
 * Zweiter Master auf demselben RS485-Segment (eigenes PC-Interface, nicht „durch den Controller“ gebündelt):
 * gleiche Master-ID wie Display → ACK_GETPOSDG/ACK_SETPOSDG am Bus nicht zuordenbar, Pending kann falsch
 * gelöst werden. PC dann andere Master-ID als der Controller (config.json master_id).
 * Liest die PC-Software nur den USB-Mitschnitt des Controllers (kein eigener Bus-TX auf dieselbe Leitung),
 * entsteht dieses ACK-Problem durch den PC nicht.
 *
 * Fremd-Master auf dem Bus (z. B. PC ID 1, Display ID 2): früher wurde komplett kein eigenes GETPOSDG
 * mehr gesendet und GETPOSDG-Timeouts löschten das Pending — Ist lief nur über fremde ACKs → Nachlaufen.
 * Während s_poll_pos / s_poll_ref läuft der Display trotzdem eigenes Polling (siehe rotor_rs485_loop / on_ack_timeout).
 */

#include "rotor_rs485.h"

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pwm_config.h"
#include "rotor_app.h"
#include "rotor_error_app.h"
#include "serial_bridge.h"

extern "C" void rotor_app_antenna_offset_changed(void);
extern "C" void rotor_app_apply_remote_antenna_selection(uint8_t n_1_to_3);

#ifndef ROTOR_RS485_MASTER_ID
#define ROTOR_RS485_MASTER_ID 2u
#endif
#ifndef ROTOR_RS485_SLAVE_ID
#define ROTOR_RS485_SLAVE_ID 20u
#endif

/** Toleranz Ist/Soll in Grad (0,2°) — dann GETPOSDG-Polling stoppen */
#ifndef ROTOR_POS_TOL_DEG
#define ROTOR_POS_TOL_DEG 0.2f
#endif

#ifndef ROTOR_RS485_ACK_TIMEOUT_MS
#define ROTOR_RS485_ACK_TIMEOUT_MS 500u
#endif

#ifndef ROTOR_BOOT_QUERY_DELAY_MS
#define ROTOR_BOOT_QUERY_DELAY_MS 2000u
#endif

#ifndef ROTOR_POLL_GAP_MS
#define ROTOR_POLL_GAP_MS 200u
#endif

/** Nach erstem „angekommen“ (Toleranz): GETPOSDG noch so lange (Endlage kann noch leicht nachziehen) */
#ifndef ROTOR_POLL_POS_GRACE_MS
#define ROTOR_POLL_POS_GRACE_MS 600u
#endif

#ifndef ROTOR_PERIODIC_GETREF_MS
#define ROTOR_PERIODIC_GETREF_MS 5000u
#endif

#ifndef ROTOR_POLL_GETERR_MS
/** GETERR zyklisch — wie periodisches GETREF (5 s), nicht Bus fluten */
#define ROTOR_POLL_GETERR_MS 5000u
#endif

/** Kein Fremd-Master → Rotor mehr so lange → eigenes GET-Polling wieder an */
#ifndef ROTOR_PC_FOREIGN_SILENCE_MS
#define ROTOR_PC_FOREIGN_SILENCE_MS 3000u
#endif

/** Referenz + Betrieb: keine Antwort vom Rotor (Slave) seit so lang → Fehler 10 Verbindungstimeout
 * (gilt auch wenn nur der PC fragt und der Controller selbst nicht sendet). */
#ifndef ROTOR_CONN_LOST_TIMEOUT_MS
#define ROTOR_CONN_LOST_TIMEOUT_MS 5000u
#endif

/** Abstand zwischen GETANEMO / GETTEMPA / GETWINDDIR (zyklisch); jede Größe alle 3 s */
#ifndef ROTOR_WEATHER_STAGGER_MS
#define ROTOR_WEATHER_STAGGER_MS 1000u
#endif

/** GETTEMPM Motortemperatur — eigenes Intervall (5 s), unabhängig von Wetter-Rotation */
#ifndef ROTOR_MOTOR_TEMP_POLL_MS
#define ROTOR_MOTOR_TEMP_POLL_MS 5000u
#endif

enum class Pending : uint8_t {
    None,
    GetRef,
    GetPosDg,
    SetPosDg,
    SetRef,
    Stop,
    GetErr,
    SetPwm,
    GetAntOff1,
    GetAntOff2,
    GetAntOff3,
    GetAngle1,
    GetAngle2,
    GetAngle3,
    GetAnemo,
    GetTempA,
    GetWindDir,
    GetTempM,
    Test
};

static uint8_t s_master_id = ROTOR_RS485_MASTER_ID;
static uint8_t s_slave_id = ROTOR_RS485_SLAVE_ID;

static rotor_rs485_ref_cb_t s_ref_cb = nullptr;
static rotor_rs485_pos_cb_t s_pos_cb = nullptr;
static rotor_rs485_target_cb_t s_target_cb = nullptr;

/** Nicht im UART-Parser: rotor_app_antenna_offset_changed (Flash nicht im Parser) */
static bool s_pending_antenna_offset_notify = false;
/** SET* config: pwm_config_save + UI (LVGL) in rotor_rs485_idle_tasks */
static bool s_pending_config_changed_from_bus = false;

/** Viele ACK_GETPOSDG in einem poll(): Ist-Callback nur einmal pro loop() — sonst blockiert LVGL die UART-Verarbeitung. */
static bool s_pos_ui_deferred = false;

/** ACK_GETANEMO / ACK_GETTEMPA / ACK_GETTEMPM / ACK_WINDDIR (Anzeige in loop(), nicht im Parser) */
static float s_last_wind_kmh = 0.0f;
static float s_last_tempa_c = 0.0f;
static float s_last_tempm_c = 0.0f;
static float s_last_wind_dir_deg = 0.0f;
static uint8_t s_weather_dirty_mask = 0;

static Pending s_pending = Pending::None;
static uint32_t s_pending_since_ms = 0;

static bool s_poll_ref = false;
static uint32_t s_next_ref_poll_ms = 0;

static bool s_poll_pos = false;
static float s_target_deg = 0.0f;
/** Zuletzt mit SETPOSDG gesendeter Soll (Anzeige bei Ankunft, nicht Bus-Float → kein 309 vs 309,1) */
static float s_goto_commanded_deg = 0.0f;
static uint32_t s_next_pos_poll_ms = 0;
/** 0 = noch keine Nachlaufphase; sonst millis()-Zeitpunkt, ab dem Positions-Polling beendet werden darf */
static uint32_t s_pos_grace_end_ms = 0;

/** Boot: 2 s warten, dann GETREF → optional GETPOSDG wenn referenziert */
static bool s_boot_done = false;
static uint32_t s_boot_earliest_ms = 0;
static uint8_t s_boot_phase = 0; /* 0=Warten, 1=GETREF gesendet, 2=GETPOSDG gesendet, 3=fertig */
/** Nach TEST: GETERR vor erstem GETREF (oder Quittung durch Mitläufer/ACK_ERR), damit kein falsches „Betriebsbereit“. */
static bool s_startup_err_known = false;

/** Nach GETPOSDG-Antwort: Soll = Ist (Einschalten, Boot, Homing) */
static bool s_align_target_after_pos_read = false;
/** Nach SETREF/Homing: bei nächstem GETREF=referenziert einmal GETPOSDG anfordern */
static bool s_request_pos_after_homing = false;

/** Lange Telegramme: ACK_GETACCBINS/GETCALBINS/… (viele Semikolon-getrennte Zahlen) — 320 war zu knapp */
static char s_rx[1536];
static size_t s_rx_len = 0;
/** Puffer voll: bis zum nächsten „#“ verwerfen (sonst Mittendrin-Bytes → Müll im Log) */
static bool s_rx_resync_until_hash = false;

/** Zuletzt aus ACK_GETREF (für UI / Taster / LED) */
static bool s_slave_referenced = false;
/** Zuletzt aus ACK_GETPOSDG / notify_pos (UI-Grad inkl. 360°-Anzeige) */
static float s_last_pos_ui_deg = 0.0f;
/** GETREF alle ROTOR_PERIODIC_GETREF_MS nach Boot, solange kein Homing-Polling */
static uint32_t s_next_periodic_getref_ms = 0;
/** GETERR zyklisch (Fehlercode / Betriebsbereit) */
static uint32_t s_next_geterr_ms = 0;

/** SETPOSCC: zuletzt gewünschter Busgrad — Versand erst in rotor_rs485_loop bei freiem Pending (kein TX vor ACK/GETPOS). */
static bool s_setposcc_queued = false;
static float s_setposcc_queued_deg = 0.0f;

/** HW-Taster: Soll = Ist — SETPOSDG wiederholen bis ACK (Bus busy / kein ACK). */
static bool s_hw_snap_retarget_active = false;
static float s_hw_snap_retarget_deg = 0.0f;

/** SETPOSDG vom Bus (USB/PC) geschnüffelt — kein s_poll_pos, aber Fahrt aktiv (HW-Stop nötig). */
static bool s_remote_setpos_motion = false;
static float s_remote_setpos_target_deg = 0.0f;
static uint32_t s_remote_setpos_grace_end_ms = 0;

/** SETPWM: zuletzt gesendeter Wert (Timeout-Retry) */
static uint8_t s_setpwm_value = 0;

/** Nach Boot: GETANTOFF1→2→3 (Versätze vom Rotor); phase 0 = GET1 noch nicht gesendet */
static bool s_antenna_boot_pending = true;
static uint8_t s_antenna_boot_phase = 0;
/** Nach erfolgreicher GETANTOFF-Kette: GETANGLE1→2→3 (Öffnungswinkel); phase 0 = inaktiv */
static bool s_angle_boot_pending = false;
static uint8_t s_angle_boot_phase = 0;

/** Zuletzt #FremdMaster:RotorID:… gesehen (Millis); 0 = noch keiner */
static uint32_t s_last_foreign_master_to_slave_ms = 0;
/** Zuletzt gesehener Master (SRC), der unseren Slave (DST=Rotor-ID) angesprochen hat — Ziel für SETPOSCC im Mitläufer-Modus */
static uint8_t s_foreign_master_src_id = 0;

/** Zuletzt ein gültiges Telegramm mit SRC = Slave-ID (ACK/NAK/ERR …) — Verbindungs-Watchdog */
static uint32_t s_last_slave_rx_ms = 0;
static bool s_have_slave_rx_ever = false;
/** Start Uhr „keine Slave-Antwort“: erster GETREF (Boot) oder erster Befehl Richtung Slave (PC) */
static uint32_t s_conn_watch_start_ms = 0;
/** GETERR-Recovery nach Fehler 10 (Intervall wie ROTOR_POLL_GAP_MS) */
static uint32_t s_next_conn_recovery_getref_ms = 0;

/** Nächster Wetter-GET (nur ohne PC, Stillstand); 0=ANEMO, 1=TEMPA, 2=WINDDIR */
static uint32_t s_next_weather_ms = 0;
static uint8_t s_weather_phase = 0;
static uint32_t s_next_motortemp_ms = 0;

/** Direkt nach Einschalten: 3× TEST (Ping); ohne ACK nach 3 Versuchen → Fehler 10 */
static bool s_boot_test_done = false;
static uint8_t s_boot_test_timeout_count = 0;

static void abort_angle_boot(void)
{
    s_angle_boot_pending = false;
    s_angle_boot_phase = 0;
}

static void abort_antenna_boot(void)
{
    s_antenna_boot_pending = false;
    s_antenna_boot_phase = 0;
    abort_angle_boot();
}

static void notify_target(float deg);
static void flush_deferred_position_ui(void);
static void try_flush_setposcc(void);

/** Bei Fehler stoppt der Slave — kein weiteres GETPOSDG-Polling / keine ausstehende Pos-Sync bis Recovery */
static void stop_fault_motion_polling(void)
{
    s_poll_pos = false;
    s_pos_grace_end_ms = 0;
    s_align_target_after_pos_read = false;
    s_remote_setpos_motion = false;
    s_remote_setpos_grace_end_ms = 0;
    s_hw_snap_retarget_active = false;
    s_poll_ref = false;
    s_request_pos_after_homing = false;
}

void rotor_rs485_set_master_id(uint8_t id) { s_master_id = id; }
void rotor_rs485_set_slave_id(uint8_t id) { s_slave_id = id; }

void rotor_rs485_set_ref_callback(rotor_rs485_ref_cb_t cb) { s_ref_cb = cb; }
void rotor_rs485_set_position_callback(rotor_rs485_pos_cb_t cb) { s_pos_cb = cb; }
void rotor_rs485_set_target_callback(rotor_rs485_target_cb_t cb) { s_target_cb = cb; }

void rotor_rs485_idle_tasks(void)
{
    flush_deferred_position_ui();
    if (s_pending_antenna_offset_notify) {
        s_pending_antenna_offset_notify = false;
        rotor_app_antenna_offset_changed();
    }
    if (s_pending_config_changed_from_bus) {
        s_pending_config_changed_from_bus = false;
        rotor_app_config_changed_from_bus();
    }
}

float rotor_rs485_get_last_wind_kmh(void)
{
    return s_last_wind_kmh;
}

float rotor_rs485_get_last_tempa_c(void)
{
    return s_last_tempa_c;
}

float rotor_rs485_get_last_tempm_c(void)
{
    return s_last_tempm_c;
}

float rotor_rs485_get_last_wind_dir_deg(void)
{
    return s_last_wind_dir_deg;
}

uint8_t rotor_rs485_weather_ui_take_mask(void)
{
    const uint8_t m = s_weather_dirty_mask;
    s_weather_dirty_mask = 0;
    return m;
}

bool rotor_rs485_is_boot_done(void) { return s_boot_done; }

bool rotor_rs485_is_startup_error_checked(void) { return s_startup_err_known; }

bool rotor_rs485_is_referenced(void) { return s_slave_referenced; }

bool rotor_rs485_is_moving(void) { return s_poll_pos || s_poll_ref; }

bool rotor_rs485_is_homing(void) { return s_poll_ref && !s_slave_referenced; }

bool rotor_rs485_is_position_polling(void) { return s_poll_pos; }

bool rotor_rs485_is_remote_setpos_motion(void) { return s_remote_setpos_motion; }

float rotor_rs485_get_last_position_deg(void) { return s_last_pos_ui_deg; }

bool rotor_rs485_is_foreign_pc_listen_mode(void)
{
    if (s_last_foreign_master_to_slave_ms == 0) {
        return false;
    }
    return (millis() - s_last_foreign_master_to_slave_ms) < ROTOR_PC_FOREIGN_SILENCE_MS;
}

/** SETPOSCC an fremden Master: nur wenn Mitläufer-Fenster aktiv — oder lokale Positionsfahrt (GETPOSDG-Polling),
 * sonst verfällt das 3s-Silence-Fenster ohne PC-Telegramm zum Slave und die Vorschau stoppt mitten in der Fahrt. */
static bool cc_preview_to_foreign_allowed(void)
{
    if (rotor_rs485_is_foreign_pc_listen_mode()) {
        return true;
    }
    return s_foreign_master_src_id != 0 && s_poll_pos;
}

static void clear_pending()
{
    s_pending = Pending::None;
    /* SETPOSCC (Encoder-Vorschau): sonst während GETPOSDG-Polling nie flush — PC sieht Soll nicht live. */
    try_flush_setposcc();
}

static void set_pending(Pending p)
{
    s_pending = p;
    s_pending_since_ms = millis();
}

static bool pending_timed_out()
{
    if (s_pending == Pending::None) {
        return false;
    }
    return (millis() - s_pending_since_ms) >= ROTOR_RS485_ACK_TIMEOUT_MS;
}

static float normalize_deg_0_360(float d)
{
    float x = fmodf(d, 360.0f);
    if (x < 0.0f) {
        x += 360.0f;
    }
    return x;
}

/**
 * Anzeige: Bus meldet oft 360,00° — normalize_deg_0_360 liefert 0° (gleiche Lage).
 * Dann soll die UI 360° zeigen, nicht 0° (Homing-Endlage / Log: ACK …360,00).
 * Referenziert: manche Slaves melden 0,0…0,5° statt 360° an der Endlage (Restfehler) —
 * sonst zeigt Ist/Soll nach Start fälschlich z.B. 0,2° statt 360°.
 */
static float bus_deg_for_ui(float deg_norm, float deg_raw)
{
    if (deg_norm < 0.01f && deg_raw >= 359.5f) {
        return 360.0f;
    }
    if (s_slave_referenced && deg_raw >= 0.0f && deg_raw <= 0.5f && deg_norm <= 0.5f) {
        return 360.0f;
    }
    return deg_norm;
}

/** Kleinster Winkelabstand 0…180° */
static float min_angle_diff(float a_deg, float b_deg)
{
    float d = fabsf(normalize_deg_0_360(a_deg) - normalize_deg_0_360(b_deg));
    if (d > 180.0f) {
        d = 360.0f - d;
    }
    return d;
}

/**
 * Entspricht rs485_protocol.py: NUM_RE = [-+]?\d+(?:[.,]\d+)?
 * Letzte nicht-überlappende Zahl im gesamten PARAMS (wie re.findall, letztes Element).
 * Kein scaled100 — CS = SRC + DST + float(letzte Zahl), wie RotorTcpBridge.
 */
static const char *match_number_end(const char *p)
{
    const char *q = p;
    if (*q == '+' || *q == '-') {
        q++;
    }
    if (*q == '\0' || !isdigit((unsigned char)*q)) {
        return nullptr;
    }
    while (isdigit((unsigned char)*q)) {
        q++;
    }
    if (*q == '.' || *q == ',') {
        if (isdigit((unsigned char)q[1])) {
            q++;
            while (isdigit((unsigned char)*q)) {
                q++;
            }
        }
    }
    return q;
}

static float last_number_in_params_python(const char *params)
{
    if (!params || !*params) {
        return 0.0f;
    }
    float last = 0.0f;
    bool any = false;
    const char *p = params;
    while (*p) {
        const char *end = match_number_end(p);
        if (end) {
            char buf[48];
            size_t n = (size_t)(end - p);
            if (n >= sizeof(buf)) {
                p = end;
                continue;
            }
            memcpy(buf, p, n);
            buf[n] = '\0';
            for (size_t i = 0; i < n; i++) {
                if (buf[i] == ',') {
                    buf[i] = '.';
                }
            }
            last = strtof(buf, nullptr);
            any = true;
            p = end;
        } else {
            p++;
        }
    }
    return any ? last : 0.0f;
}

/** Wie rs485_protocol._fmt_cs: Ganzzahl ohne Komma, sonst max. 2 Nachkommastellen, trailing zeros weg */
static void format_cs_python(char *out, size_t out_sz, float cs)
{
    float r = roundf(cs);
    if (fabsf(cs - r) < 0.005f) {
        snprintf(out, out_sz, "%d", (int)lroundf(r));
        return;
    }
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%.2f", cs);
    for (char *q = tmp; *q; ++q) {
        if (*q == '.') {
            *q = ',';
        }
    }
    size_t len = strlen(tmp);
    while (len > 0 && tmp[len - 1] == '0') {
        len--;
    }
    if (len > 0 && tmp[len - 1] == ',') {
        len--;
    }
    tmp[len] = '\0';
    snprintf(out, out_sz, "%s", tmp);
}

/** Bus senden: bei hartem Fehler gesperrt — Fehler 10 (Timeout) erlaubt GETREF-Recovery */
static bool bus_send_allowed_by_fault(void)
{
    const int e = rotor_error_app_get_error_code();
    return e == 0 || e == 10;
}

static void send_line_to(uint8_t dst, const char *cmd, const char *params_str)
{
    if (!bus_send_allowed_by_fault()) {
        return;
    }
    float last = last_number_in_params_python(params_str);
    float cs_val = (float)s_master_id + (float)dst + last;
    char cs[32];
    format_cs_python(cs, sizeof(cs), cs_val);
    char line[192];
    snprintf(line, sizeof(line), "#%u:%u:%s:%s:%s$", (unsigned)s_master_id, (unsigned)dst, cmd, params_str, cs);
    serial_bridge::hw_send(reinterpret_cast<const uint8_t *>(line), strlen(line));
}

static void send_line(const char *cmd, const char *params_str)
{
    send_line_to(s_slave_id, cmd, params_str);
}

static void send_request(const char *cmd, const char *params_str, Pending p)
{
    if (!bus_send_allowed_by_fault()) {
        return;
    }
    send_line(cmd, params_str);
    set_pending(p);
}

static void format_deg_param(char *buf, size_t sz, float deg)
{
    snprintf(buf, sz, "%.2f", deg);
    for (char *q = buf; *q; ++q) {
        if (*q == '.') {
            *q = ',';
        }
    }
}

void rotor_rs485_send_getref(void)
{
    if (s_pending != Pending::None) {
        return;
    }
    send_request("GETREF", "0", Pending::GetRef);
}

void rotor_rs485_send_setref_homing(void)
{
    if (s_pending != Pending::None) {
        return;
    }
    send_request("SETREF", "1", Pending::SetRef);
    s_poll_ref = true;
    s_request_pos_after_homing = true;
}

bool rotor_rs485_send_stop(void)
{
    if (s_pending == Pending::SetPosDg || s_pending == Pending::SetRef) {
        return false;
    }
    if (s_pending == Pending::GetPosDg || s_pending == Pending::GetRef ||
        s_pending == Pending::GetAnemo || s_pending == Pending::GetTempA ||
        s_pending == Pending::GetWindDir || s_pending == Pending::GetTempM) {
        clear_pending();
    }
    if (s_pending != Pending::None) {
        return false;
    }
    s_poll_pos = false;
    s_pos_grace_end_ms = 0;
    s_poll_ref = false;
    s_request_pos_after_homing = false;
    s_remote_setpos_motion = false;
    s_remote_setpos_grace_end_ms = 0;
    send_request("STOP", "0", Pending::Stop);
    return true;
}

void rotor_rs485_send_getposdg(void)
{
    if (s_pending != Pending::None) {
        return;
    }
    send_request("GETPOSDG", "0", Pending::GetPosDg);
}

void rotor_rs485_send_setaselect(uint8_t antenna_1_to_3)
{
    if (antenna_1_to_3 < 1u || antenna_1_to_3 > 3u) {
        return;
    }
    char p[8];
    snprintf(p, sizeof(p), "%u", (unsigned)antenna_1_to_3);
    send_line_to(ROTOR_RS485_BROADCAST_ID, "SETASELECT", p);
}

bool rotor_rs485_send_set_pwm_limit(uint8_t pct)
{
    if (pct > 100u) {
        pct = 100u;
    }
    /* Während Positionsfahrt: GETPOSDG-Polling kurz unterbrechen — SETPWM muss durch (Geschwindigkeit ändern). */
    if (s_pending == Pending::GetPosDg) {
        clear_pending();
    }
    if (s_pending == Pending::GetAnemo || s_pending == Pending::GetTempA ||
        s_pending == Pending::GetWindDir || s_pending == Pending::GetTempM) {
        clear_pending();
    }
    if (s_pending == Pending::GetAntOff1 || s_pending == Pending::GetAntOff2 ||
        s_pending == Pending::GetAntOff3) {
        clear_pending();
        abort_antenna_boot();
    }
    if (s_pending != Pending::None) {
        return false;
    }
    char p[16];
    snprintf(p, sizeof(p), "%u", (unsigned)pct);
    s_setpwm_value = pct;
    send_request("SETPWM", p, Pending::SetPwm);
    return true;
}

bool rotor_rs485_goto_degrees(float deg)
{
    /* SETPOSDG darf nicht unterbrochen werden. Ein ausstehendes GETPOSDG (Positions-Polling)
     * blockiert sonst jeden Encoder-Klick: Bus ist „dauernd beschäftigt“, SETPOS wird verworfen
     * oder stark verzögert — Anzeige und Soll passen nicht mehr zusammen. */
    if (s_pending == Pending::SetPosDg) {
        return false;
    }
    if (s_pending == Pending::GetPosDg) {
        clear_pending();
    }
    if (s_pending == Pending::GetAnemo || s_pending == Pending::GetTempA ||
        s_pending == Pending::GetWindDir || s_pending == Pending::GetTempM) {
        clear_pending();
    }
    if (s_pending == Pending::GetAntOff1 || s_pending == Pending::GetAntOff2 ||
        s_pending == Pending::GetAntOff3) {
        clear_pending();
        abort_antenna_boot();
    }
    if (s_pending != Pending::None) {
        return false;
    }
    float n = normalize_deg_0_360(deg);
    char p[40];
    format_deg_param(p, sizeof(p), n);
    s_target_deg = n;
    s_goto_commanded_deg = n;
    s_poll_pos = true;
    s_pos_grace_end_ms = 0;
    s_next_pos_poll_ms = 0;
    send_request("SETPOSDG", p, Pending::SetPosDg);
    /* Callback: Bus-Ist/Soll (normalisiert), UI rechnet Antennenversatz um */
    notify_target(n);
    return true;
}

static void try_flush_setposcc(void)
{
    if (!s_setposcc_queued || !bus_send_allowed_by_fault()) {
        return;
    }
    if (!cc_preview_to_foreign_allowed()) {
        s_setposcc_queued = false;
        return;
    }
    if (s_foreign_master_src_id == 0) {
        return;
    }
    char p[40];
    format_deg_param(p, sizeof(p), s_setposcc_queued_deg);
    send_line_to(s_foreign_master_src_id, "SETPOSCC", p);
    s_setposcc_queued = false;
}

void rotor_rs485_send_setposcc_degrees(float deg)
{
    if (!bus_send_allowed_by_fault()) {
        return;
    }
    if (!cc_preview_to_foreign_allowed()) {
        return;
    }
    s_setposcc_queued_deg = normalize_deg_0_360(deg);
    s_setposcc_queued = true;
}

static void try_hw_snap_retarget(void)
{
    if (!s_hw_snap_retarget_active) {
        return;
    }
    (void)rotor_rs485_goto_degrees(s_hw_snap_retarget_deg);
}

void rotor_rs485_hw_snap_retarget_request(float deg)
{
    s_hw_snap_retarget_active = true;
    s_hw_snap_retarget_deg = deg;
    try_hw_snap_retarget();
}

static void notify_ref(bool ref_ok)
{
    s_slave_referenced = ref_ok;
    if (s_ref_cb) {
        s_ref_cb(ref_ok);
    }
}

static void notify_pos(float deg)
{
    /* Ohne Referenz liefert GETPOSDG oft 0° o. Ä. — PC-Polling würde Ist-Anzeige/Arc falsch treiben. */
    if (!s_slave_referenced) {
        return;
    }
    s_last_pos_ui_deg = deg;
    s_pos_ui_deferred = true;
}

static void flush_deferred_position_ui(void)
{
    if (!s_pos_ui_deferred || !s_pos_cb) {
        return;
    }
    s_pos_ui_deferred = false;
    s_pos_cb(s_last_pos_ui_deg);
}

static void notify_target(float deg)
{
    if (s_target_cb) {
        s_target_cb(deg);
    }
}

/** Erster Zahlenwert im PARAMS nach ACK-Tag (bis zum nächsten ':') */
static bool parse_ack_first_param_float(const char *line, const char *ack_tag, float *out)
{
    const char *p = strstr(line, ack_tag);
    if (!p) {
        return false;
    }
    p += strlen(ack_tag);
    if (*p == '\0') {
        return false;
    }
    char valbuf[32];
    size_t i = 0;
    while (*p && *p != ':' && i + 1 < sizeof(valbuf)) {
        valbuf[i++] = (*p == ',') ? '.' : *p;
        p++;
    }
    valbuf[i] = '\0';
    if (i == 0) {
        return false;
    }
    *out = strtof(valbuf, nullptr);
    return true;
}

static bool parse_ack_getanemo(const char *line)
{
    float v = 0.0f;
    if (!parse_ack_first_param_float(line, ":ACK_GETANEMO:", &v)) {
        return false;
    }
    if (s_pending == Pending::GetAnemo) {
        clear_pending();
    }
    s_last_wind_kmh = v;
    s_weather_dirty_mask |= 1u;
    return true;
}

static bool parse_ack_gettempa(const char *line)
{
    float v = 0.0f;
    if (!parse_ack_first_param_float(line, ":ACK_GETTEMPA:", &v)) {
        return false;
    }
    if (s_pending == Pending::GetTempA) {
        clear_pending();
    }
    s_last_tempa_c = v;
    s_weather_dirty_mask |= 2u;
    return true;
}

/** Doku: ACK_WINDDIR (nicht ACK_GETWINDDIR) */
static bool parse_ack_winddir(const char *line)
{
    float v = 0.0f;
    if (!parse_ack_first_param_float(line, ":ACK_WINDDIR:", &v)) {
        return false;
    }
    if (s_pending == Pending::GetWindDir) {
        clear_pending();
    }
    s_last_wind_dir_deg = normalize_deg_0_360(v);
    s_weather_dirty_mask |= 4u;
    return true;
}

static bool parse_ack_gettempm(const char *line)
{
    float v = 0.0f;
    if (!parse_ack_first_param_float(line, ":ACK_GETTEMPM:", &v)) {
        return false;
    }
    if (s_pending == Pending::GetTempM) {
        clear_pending();
    }
    s_last_tempm_c = v;
    s_weather_dirty_mask |= 8u;
    return true;
}

/** #SRC:DST:… */
static bool line_src_dst(const char *line, unsigned *src_out, unsigned *dst_out)
{
    unsigned src = 0;
    unsigned dst = 0;
    if (!line || line[0] != '#' || sscanf(line, "#%u:%u:", &src, &dst) != 2) {
        return false;
    }
    *src_out = src;
    *dst_out = dst;
    return true;
}

/** Parameter nach :SETASELECT: (1…3) */
static bool parse_setaselect_antenna_1_to_3(const char *line, unsigned *out_v)
{
    const char *p = strstr(line, ":SETASELECT:");
    if (!p) {
        return false;
    }
    p += strlen(":SETASELECT:");
    unsigned v = 0;
    if (sscanf(p, "%u", &v) != 1) {
        return false;
    }
    if (v < 1u || v > 3u) {
        return false;
    }
    *out_v = v;
    return true;
}

/** Erster Parameter nach :SETREF: (Homing starten bei ≠ 0) — Befehlszeile Richtung Slave */
static bool parse_setref_command_param(const char *line, int *out_v)
{
    const char *p = strstr(line, ":SETREF:");
    if (!p) {
        return false;
    }
    p += strlen(":SETREF:");
    char tmp[16];
    size_t n = 0;
    while (*p && *p != ':' && *p != '$' && n + 1 < sizeof(tmp)) {
        tmp[n++] = (*p == ',') ? '.' : *p;
        p++;
    }
    tmp[n] = '\0';
    if (n == 0) {
        return false;
    }
    *out_v = (int)(strtof(tmp, nullptr) + 0.5f);
    return true;
}

/** Erster Parameter nach :SETPOSDG: (Grad, Komma erlaubt) — nur echte Befehlszeile, nicht ACK_SETPOSDG */
static bool parse_setposdg_command_deg(const char *line, float *out_deg)
{
    const char *p = strstr(line, ":SETPOSDG:");
    if (!p) {
        return false;
    }
    p += strlen(":SETPOSDG:");
    char tmp[32];
    size_t n = 0;
    while (*p && *p != ':' && *p != '$' && n + 1 < sizeof(tmp)) {
        tmp[n++] = (*p == ',') ? '.' : *p;
        p++;
    }
    tmp[n] = '\0';
    if (n == 0) {
        return false;
    }
    *out_deg = normalize_deg_0_360(strtof(tmp, nullptr));
    return true;
}

/**
 * Grad-String aus Telegramm (z. B. "353,80" in valbuf als "353.80") ohne strtof-Rauschen.
 * Binary-float von 353,8 ist oft 353,7999… — nach bus_to_display + floor (fmt_de) zeigt Ist fälschlich 353,7.
 */
static float parse_rs485_deg_valbuf_to_float(const char *valbuf)
{
    if (valbuf == nullptr || valbuf[0] == '\0') {
        return 0.0f;
    }
    const char *p = valbuf;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    long hi = 0;
    bool any_digit = false;
    while (*p >= '0' && *p <= '9') {
        any_digit = true;
        hi = hi * 10 + (*p - '0');
        if (hi > 1000000L) {
            break;
        }
        ++p;
    }
    if (!any_digit) {
        return strtof(valbuf, nullptr);
    }
    unsigned frac_num = 0;
    unsigned frac_den = 1;
    if ((*p == '.' || *p == ',') && p[1] >= '0' && p[1] <= '9') {
        ++p;
        while (*p >= '0' && *p <= '9' && frac_den < 100000U) {
            frac_num = frac_num * 10U + static_cast<unsigned>(*p - '0');
            frac_den *= 10U;
            ++p;
        }
    }
    const double d = static_cast<double>(hi) + static_cast<double>(frac_num) / static_cast<double>(frac_den);
    return static_cast<float>(d);
}

/** Fremd-Master (USB): SETANTOFFn an Slave — Versatz übernehmen */
static void sniff_setantoff_to_slave(const char *line)
{
    static const struct {
        const char *tag;
        int idx;
    } tags[] = {
        {":SETANTOFF1:", 1},
        {":SETANTOFF2:", 2},
        {":SETANTOFF3:", 3},
    };
    for (size_t i = 0; i < 3; i++) {
        const char *p = strstr(line, tags[i].tag);
        if (!p) {
            continue;
        }
        p += strlen(tags[i].tag);
        char tmp[32];
        size_t n = 0;
        while (*p && *p != ':' && *p != '$' && n + 1 < sizeof(tmp)) {
            tmp[n++] = (*p == ',') ? '.' : *p;
            p++;
        }
        tmp[n] = '\0';
        const float v = normalize_deg_0_360(strtof(tmp, nullptr));
        pwm_config_set_antoff_deg(tags[i].idx, v);
        s_pending_antenna_offset_notify = true;
        return;
    }
}

/** Fremd-Master: SETANGLEn an Slave — Öffnungswinkel übernehmen */
static void sniff_setangle_to_slave(const char *line)
{
    static const struct {
        const char *tag;
        int idx;
    } tags[] = {
        {":SETANGLE1:", 1},
        {":SETANGLE2:", 2},
        {":SETANGLE3:", 3},
    };
    for (size_t i = 0; i < 3; i++) {
        const char *p = strstr(line, tags[i].tag);
        if (!p) {
            continue;
        }
        p += strlen(tags[i].tag);
        char tmp[32];
        size_t n = 0;
        while (*p && *p != ':' && *p != '$' && n + 1 < sizeof(tmp)) {
            tmp[n++] = (*p == ',') ? '.' : *p;
            p++;
        }
        tmp[n] = '\0';
        const float v = strtof(tmp, nullptr);
        if (v == v) {
            pwm_config_set_opening_deg(tags[i].idx, v);
        }
        return;
    }
}

/** PARAMS und CS zwischen :TAG: … :CS$ (wie andere Buszeilen) */
static bool extract_tag_params_cs(const char *line, const char *tag, char *param_out, size_t po_sz, float *cs_out)
{
    const char *cmd_start = strstr(line, tag);
    if (!cmd_start) {
        return false;
    }
    const char *params_start = cmd_start + strlen(tag);
    const char *dollar = strchr(line, '$');
    if (!dollar || params_start >= dollar) {
        return false;
    }
    const char *last_colon = dollar;
    while (last_colon > params_start && last_colon[-1] != ':') {
        last_colon--;
    }
    if (last_colon <= params_start) {
        return false;
    }
    /* last_colon zeigt auf das erste Zeichen von CS; davor steht ':' zwischen PARAMS und CS */
    const char *colon_before_cs = last_colon - 1;
    if (colon_before_cs < params_start || *colon_before_cs != ':') {
        return false;
    }
    size_t plen = (size_t)(colon_before_cs - params_start);
    if (plen >= po_sz) {
        plen = po_sz - 1;
    }
    memcpy(param_out, params_start, plen);
    param_out[plen] = '\0';
    char csbuf[32];
    size_t i = 0;
    const char *q = last_colon;
    while (q < dollar && i + 1 < sizeof(csbuf)) {
        csbuf[i++] = (*q == ',') ? '.' : *q;
        q++;
    }
    csbuf[i] = '\0';
    *cs_out = strtof(csbuf, nullptr);
    return true;
}

static bool config_cs_ok(unsigned src, unsigned dst, const char *params, float cs_rx)
{
    const float expected =
        (float)src + (float)dst + last_number_in_params_python(params);
    return fabsf(cs_rx - expected) < 0.12f;
}

static void config_reply_ack_u8(uint8_t to_src, const char *ack, uint8_t v)
{
    char p[8];
    snprintf(p, sizeof(p), "%u", (unsigned)v);
    send_line_to(to_src, ack, p);
}

static void config_reply_ack_u16(uint8_t to_src, const char *ack, uint16_t v)
{
    char p[8];
    snprintf(p, sizeof(p), "%u", (unsigned)v);
    send_line_to(to_src, ack, p);
}

/** Letzte Zahl in PARAMS fest 0 (CS trotz Ziffern im Namen) */
static void config_reply_ack_label(uint8_t to_src, const char *ack, const char *label)
{
    char p[56];
    snprintf(p, sizeof(p), "%s;0", label ? label : "");
    send_line_to(to_src, ack, p);
}

static void config_reply_nak(uint8_t to_src, const char *nak, int code)
{
    char p[8];
    snprintf(p, sizeof(p), "%d", code);
    send_line_to(to_src, nak, p);
}

/**
 * Broadcast DST=255: neue Controller-Master-ID setzen, wenn die bisherige ID unbekannt ist.
 * Tags: SETCONIDF (vorgesehen für Erstkonfiguration) oder SETCONTID (gleiche Semantik).
 * CS = SRC + 255 + letzte Zahl in PARAMS.
 */
static void handle_broadcast_set_controller_master_id(const char *line, unsigned src)
{
    const char *tag = nullptr;
    const char *ack = "ACK_SETCONTID";
    if (strstr(line, ":SETCONIDF:")) {
        tag = ":SETCONIDF:";
        ack = "ACK_SETCONIDF";
    } else if (strstr(line, ":SETCONTID:")) {
        tag = ":SETCONTID:";
    } else {
        return;
    }
    char par[128];
    float cs = 0.0f;
    if (!extract_tag_params_cs(line, tag, par, sizeof(par), &cs) ||
        !config_cs_ok(src, ROTOR_RS485_BROADCAST_ID, par, cs)) {
        config_reply_nak(src, "NAK_SETCONTID", 2);
        return;
    }
    unsigned v = 0;
    if (sscanf(par, "%u", &v) != 1 || v < 1u || v > 254u) {
        config_reply_nak(src, "NAK_SETCONTID", 1);
        return;
    }
    pwm_config_set_master_id((uint8_t)v);
    pwm_config_save();
    rotor_rs485_set_master_id((uint8_t)v);
    s_pending_config_changed_from_bus = true;
    char p[8];
    snprintf(p, sizeof(p), "%u", (unsigned)v);
    send_line_to(src, ack, p);
}

/**
 * Konfiguration wie config.json: Ziel DST = master_id (Controller).
 * SET → Speichern + UI-Update deferred; GET → sofort ACK.
 */
static bool handle_local_config_command(const char *line, unsigned src, unsigned dst)
{
    if (dst != (unsigned)s_master_id) {
        return false;
    }
    char par[128];
    float cs = 0.0f;

#define CFG_TRY_TAG(t) \
    extract_tag_params_cs(line, (t), par, sizeof(par), &cs) && config_cs_ok(src, dst, par, cs)

    if (strstr(line, ":GETCONRID:") || strstr(line, ":GETTCONRID:")) {
        const char *tag = strstr(line, ":GETTCONRID:") ? ":GETTCONRID:" : ":GETCONRID:";
        if (!extract_tag_params_cs(line, tag, par, sizeof(par), &cs) || !config_cs_ok(src, dst, par, cs)) {
            config_reply_nak(src, "NAK_GETCONRID", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONRID", pwm_config_get_rotor_id());
        return true;
    }

    if (strstr(line, ":SETCONRID:")) {
        if (!CFG_TRY_TAG(":SETCONRID:")) {
            config_reply_nak(src, "NAK_SETCONRID", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v < 1u || v > 254u) {
            config_reply_nak(src, "NAK_SETCONRID", 1);
            return true;
        }
        pwm_config_set_rotor_id((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONRID", (uint8_t)v);
        return true;
    }

    if (strstr(line, ":GETCONTID:")) {
        if (!CFG_TRY_TAG(":GETCONTID:")) {
            config_reply_nak(src, "NAK_GETCONTID", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONTID", pwm_config_get_master_id());
        return true;
    }

    if (strstr(line, ":SETCONTID:")) {
        if (!CFG_TRY_TAG(":SETCONTID:")) {
            config_reply_nak(src, "NAK_SETCONTID", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v < 1u || v > 254u) {
            config_reply_nak(src, "NAK_SETCONTID", 1);
            return true;
        }
        pwm_config_set_master_id((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONTID", (uint8_t)v);
        return true;
    }

    if (strstr(line, ":GETCONSPWM:")) {
        if (!CFG_TRY_TAG(":GETCONSPWM:")) {
            config_reply_nak(src, "NAK_GETCONSPWM", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONSPWM", pwm_config_get_slow());
        return true;
    }

    if (strstr(line, ":SETCONSPWM:")) {
        if (!CFG_TRY_TAG(":SETCONSPWM:")) {
            config_reply_nak(src, "NAK_SETCONSPWM", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v > 100u) {
            config_reply_nak(src, "NAK_SETCONSPWM", 1);
            return true;
        }
        pwm_config_set_slow((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONSPWM", (uint8_t)v);
        return true;
    }

    if (strstr(line, ":GETCONFPWM:")) {
        if (!CFG_TRY_TAG(":GETCONFPWM:")) {
            config_reply_nak(src, "NAK_GETCONFPWM", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONFPWM", pwm_config_get_fast());
        return true;
    }

    if (strstr(line, ":SETCONFPWM:")) {
        if (!CFG_TRY_TAG(":SETCONFPWM:")) {
            config_reply_nak(src, "NAK_SETCONFPWM", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v > 100u) {
            config_reply_nak(src, "NAK_SETCONFPWM", 1);
            return true;
        }
        pwm_config_set_fast((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONFPWM", (uint8_t)v);
        return true;
    }

    if (strstr(line, ":GETCONFRQ:")) {
        if (!CFG_TRY_TAG(":GETCONFRQ:")) {
            config_reply_nak(src, "NAK_GETCONFRQ", 2);
            return true;
        }
        config_reply_ack_u16(src, "ACK_GETCONFRQ", pwm_config_get_touch_beep_freq_hz());
        return true;
    }

    if (strstr(line, ":SETCONFRQ:")) {
        if (!CFG_TRY_TAG(":SETCONFRQ:")) {
            config_reply_nak(src, "NAK_SETCONFRQ", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v < 200u || v > 4000u) {
            config_reply_nak(src, "NAK_SETCONFRQ", 1);
            return true;
        }
        pwm_config_set_touch_beep_freq_hz((uint16_t)v);
        pwm_config_save();
        config_reply_ack_u16(src, "ACK_SETCONFRQ", pwm_config_get_touch_beep_freq_hz());
        return true;
    }

    if (strstr(line, ":GETLSL:")) {
        if (!CFG_TRY_TAG(":GETLSL:")) {
            config_reply_nak(src, "NAK_GETLSL", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETLSL", pwm_config_get_touch_beep_vol());
        return true;
    }

    if (strstr(line, ":SETLSL:")) {
        if (!CFG_TRY_TAG(":SETLSL:")) {
            config_reply_nak(src, "NAK_SETLSL", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v > 50u) {
            config_reply_nak(src, "NAK_SETLSL", 1);
            return true;
        }
        pwm_config_set_touch_beep_vol((uint8_t)v);
        pwm_config_save();
        config_reply_ack_u8(src, "ACK_SETLSL", pwm_config_get_touch_beep_vol());
        return true;
    }

    if (strstr(line, ":GETCONANO:")) {
        if (!CFG_TRY_TAG(":GETCONANO:")) {
            config_reply_nak(src, "NAK_GETCONANO", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONANO", pwm_config_get_anemometer());
        return true;
    }

    if (strstr(line, ":SETCONANO:")) {
        if (!CFG_TRY_TAG(":SETCONANO:")) {
            config_reply_nak(src, "NAK_SETCONANO", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v > 1u) {
            config_reply_nak(src, "NAK_SETCONANO", 1);
            return true;
        }
        pwm_config_set_anemometer((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONANO", pwm_config_get_anemometer());
        return true;
    }

    if (strstr(line, ":GETCONDELTA:")) {
        if (!CFG_TRY_TAG(":GETCONDELTA:")) {
            config_reply_nak(src, "NAK_GETCONDELTA", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONDELTA", pwm_config_get_encoder_delta_tenths());
        return true;
    }

    if (strstr(line, ":SETCONDELTA:")) {
        if (!CFG_TRY_TAG(":SETCONDELTA:")) {
            config_reply_nak(src, "NAK_SETCONDELTA", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || (v != 1u && v != 10u)) {
            config_reply_nak(src, "NAK_SETCONDELTA", 1);
            return true;
        }
        pwm_config_set_encoder_delta_tenths((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONDELTA", pwm_config_get_encoder_delta_tenths());
        return true;
    }

    if (strstr(line, ":GETCONCHA:")) {
        if (!CFG_TRY_TAG(":GETCONCHA:")) {
            config_reply_nak(src, "NAK_GETCONCHA", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONCHA", pwm_config_get_concha());
        return true;
    }

    if (strstr(line, ":SETCONCHA:")) {
        if (!CFG_TRY_TAG(":SETCONCHA:")) {
            config_reply_nak(src, "NAK_SETCONCHA", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v > 1u) {
            config_reply_nak(src, "NAK_SETCONCHA", 1);
            return true;
        }
        pwm_config_set_concha((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONCHA", pwm_config_get_concha());
        return true;
    }

    if (strstr(line, ":GETCONLEDP:")) {
        if (!CFG_TRY_TAG(":GETCONLEDP:")) {
            config_reply_nak(src, "NAK_GETCONLEDP", 2);
            return true;
        }
        config_reply_ack_u8(src, "ACK_GETCONLEDP", pwm_config_get_led_ring_brightness_pct());
        return true;
    }

    if (strstr(line, ":SETCONLEDP:")) {
        if (!CFG_TRY_TAG(":SETCONLEDP:")) {
            config_reply_nak(src, "NAK_SETCONLEDP", 2);
            return true;
        }
        unsigned v = 0;
        if (sscanf(par, "%u", &v) != 1 || v > 100u) {
            config_reply_nak(src, "NAK_SETCONLEDP", 1);
            return true;
        }
        pwm_config_set_led_ring_brightness_pct((uint8_t)v);
        pwm_config_save();
        s_pending_config_changed_from_bus = true;
        config_reply_ack_u8(src, "ACK_SETCONLEDP", pwm_config_get_led_ring_brightness_pct());
        return true;
    }

    static const struct {
        const char *get_tag;
        const char *set_tag;
        const char *ack_get;
        const char *nak_get;
        const char *ack_set;
        const char *nak_set;
        int idx;
    } ant[] = {
        {":GETCONANTNAME1:", ":SETCONANTNAME1:", "ACK_GETCONANTNAME1", "NAK_GETCONANTNAME1",
         "ACK_SETCONANTNAME1", "NAK_SETCONANTNAME1", 1},
        {":GETCONANTNAME2:", ":SETCONANTNAME2:", "ACK_GETCONANTNAME2", "NAK_GETCONANTNAME2",
         "ACK_SETCONANTNAME2", "NAK_SETCONANTNAME2", 2},
        {":GETCONANTNAME3:", ":SETCONANTNAME3:", "ACK_GETCONANTNAME3", "NAK_GETCONANTNAME3",
         "ACK_SETCONANTNAME3", "NAK_SETCONANTNAME3", 3},
    };

    for (size_t i = 0; i < 3; i++) {
        if (strstr(line, ant[i].get_tag)) {
            if (!CFG_TRY_TAG(ant[i].get_tag)) {
                config_reply_nak(src, ant[i].nak_get, 2);
                return true;
            }
            config_reply_ack_label(src, ant[i].ack_get, pwm_config_get_antenna_label(ant[i].idx));
            return true;
        }
    }
    for (size_t i = 0; i < 3; i++) {
        if (strstr(line, ant[i].set_tag)) {
            if (!CFG_TRY_TAG(ant[i].set_tag)) {
                config_reply_nak(src, ant[i].nak_set, 2);
                return true;
            }
            if (strchr(par, ':')) {
                config_reply_nak(src, ant[i].nak_set, 1);
                return true;
            }
            pwm_config_set_antenna_label(ant[i].idx, par);
            pwm_config_save();
            s_pending_config_changed_from_bus = true;
            config_reply_ack_label(src, ant[i].ack_set, pwm_config_get_antenna_label(ant[i].idx));
            return true;
        }
    }

#undef CFG_TRY_TAG
    return false;
}

static void dispatch_bus_command_to_slave(const char *line, unsigned src)
{
    /* SETASELECT oft als #SRC:SlaveID:SETASELECT:n:CS$ (nicht nur Broadcast 255) — sonst sieht der Controller
     * den Antennenwechsel von der PC-Software während der Fahrt nicht. */
    {
        unsigned ant = 0;
        if (parse_setaselect_antenna_1_to_3(line, &ant)) {
            rotor_app_apply_remote_antenna_selection((uint8_t)ant);
            return;
        }
    }

    /* PC-Client (anderer Master): SETREF:1 → gleiche Homing-Flags wie rotor_rs485_send_setref_homing()
     * (LED, Meldetext „Referenziere“, NeoPixel-Lauflicht, GETREF bis referenziert). */
    if (src != (unsigned)s_master_id) {
        int ref_cmd = 0;
        if (parse_setref_command_param(line, &ref_cmd) && ref_cmd != 0) {
            s_poll_ref = true;
            s_request_pos_after_homing = true;
            s_next_ref_poll_ms = millis() + ROTOR_POLL_GAP_MS;
        }
    }

    float deg = 0.0f;
    if (parse_setposdg_command_deg(line, &deg)) {
        /* Nur fremde Master (z. B. PC): Soll-Anzeige + Remote-Fahrt. Eigenes SETPOSDG (SRC =
         * s_master_id): Ziel setzt rotor_rs485_goto_degrees / Encoder; dieselbe Zeile taucht oft
         * nochmals über USB-Spiegel oder RX auf — ein verzögertes *älteres* SETPOSDG würde sonst
         * taget_dg zurückdrehen (Ist schon am neuen Winkel, Soll steht wieder auf alt). */
        if (src != (unsigned)s_master_id) {
            s_remote_setpos_target_deg = deg;
            s_remote_setpos_motion = true;
            s_remote_setpos_grace_end_ms = 0;
            notify_target(deg);
        }
    }
    sniff_setantoff_to_slave(line);
    sniff_setangle_to_slave(line);
}

static void on_ack_timeout()
{
    if (!pending_timed_out()) {
        return;
    }
    /* Fremd-Master bedient den Rotor: eigene GET-*-Pending nicht endlos wiederholen (Kollision).
     * Ausnahme: lokale Positionsfahrt / Homing — sonst kein eigenes GETPOSDG/GETREF und Ist nur aus fremden ACKs. */
    if (rotor_rs485_is_foreign_pc_listen_mode()) {
        switch (s_pending) {
        case Pending::GetRef:
            if (s_poll_ref) {
                break;
            }
            clear_pending();
            return;
        case Pending::GetPosDg:
            if (s_poll_pos) {
                break;
            }
            clear_pending();
            return;
        case Pending::GetErr:
        case Pending::GetAnemo:
        case Pending::GetTempA:
        case Pending::GetWindDir:
        case Pending::GetTempM:
        case Pending::GetAngle1:
        case Pending::GetAngle2:
        case Pending::GetAngle3:
            clear_pending();
            return;
        case Pending::Test:
            clear_pending();
            s_boot_test_done = true;
            return;
        default:
            break;
        }
    }
    /* Kein clear_pending: dasselbe Telegramm erneut (Kollision / fehlendes ACK). */
    switch (s_pending) {
    case Pending::GetRef:
        send_request("GETREF", "0", Pending::GetRef);
        break;
    case Pending::GetPosDg:
        send_request("GETPOSDG", "0", Pending::GetPosDg);
        break;
    case Pending::SetPosDg: {
        char p[40];
        format_deg_param(p, sizeof(p), normalize_deg_0_360(s_goto_commanded_deg));
        send_request("SETPOSDG", p, Pending::SetPosDg);
        break;
    }
    case Pending::SetRef:
        send_request("SETREF", "1", Pending::SetRef);
        break;
    case Pending::Stop:
        send_request("STOP", "0", Pending::Stop);
        break;
    case Pending::GetErr:
        send_request("GETERR", "0", Pending::GetErr);
        break;
    case Pending::SetPwm: {
        char p[16];
        snprintf(p, sizeof(p), "%u", (unsigned)s_setpwm_value);
        send_request("SETPWM", p, Pending::SetPwm);
        break;
    }
    case Pending::GetAntOff1:
        send_request("GETANTOFF1", "1", Pending::GetAntOff1);
        break;
    case Pending::GetAntOff2:
        send_request("GETANTOFF2", "1", Pending::GetAntOff2);
        break;
    case Pending::GetAntOff3:
        send_request("GETANTOFF3", "1", Pending::GetAntOff3);
        break;
    case Pending::GetAngle1:
        send_request("GETANGLE1", "1", Pending::GetAngle1);
        break;
    case Pending::GetAngle2:
        send_request("GETANGLE2", "1", Pending::GetAngle2);
        break;
    case Pending::GetAngle3:
        send_request("GETANGLE3", "1", Pending::GetAngle3);
        break;
    case Pending::GetAnemo:
        send_request("GETANEMO", "0", Pending::GetAnemo);
        break;
    case Pending::GetTempA:
        send_request("GETTEMPA", "0", Pending::GetTempA);
        break;
    case Pending::GetWindDir:
        send_request("GETWINDDIR", "0", Pending::GetWindDir);
        break;
    case Pending::GetTempM:
        send_request("GETTEMPM", "0", Pending::GetTempM);
        break;
    case Pending::Test:
        s_boot_test_timeout_count++;
        if (s_boot_test_timeout_count >= 3) {
            clear_pending();
            s_boot_test_done = true;
            rotor_error_app_set_error_code(10);
            stop_fault_motion_polling();
            s_next_conn_recovery_getref_ms = millis() - ROTOR_POLL_GAP_MS;
        } else {
            send_request("TEST", "0", Pending::Test);
        }
        break;
    case Pending::None:
    default:
        clear_pending();
        break;
    }
}

/** Quittung auf SETPOSDG — danach erst GETPOSDG (nur wenn ACK an unsere Master-ID, nicht fremder Master gleicher Bus) */
static bool parse_ack_setposdg_result(const char *line)
{
    unsigned src = 0;
    unsigned dst = 0;
    if (!line_src_dst(line, &src, &dst)) {
        return false;
    }
    const char *p = strstr(line, ":ACK_SETPOSDG:");
    if (!p) {
        return false;
    }
    const bool for_us = (dst == (unsigned)s_master_id);
    if (for_us && s_pending == Pending::SetPosDg) {
        clear_pending();
        s_hw_snap_retarget_active = false;
    }
    if (for_us && s_poll_pos && !rotor_rs485_is_foreign_pc_listen_mode()) {
        send_request("GETPOSDG", "0", Pending::GetPosDg);
    }
    return true;
}

static bool parse_ack_setref_result(const char *line)
{
    const char *p = strstr(line, ":ACK_SETREF:");
    if (!p) {
        return false;
    }
    if (s_pending == Pending::SetRef) {
        clear_pending();
    }
    if (s_poll_ref) {
        s_next_ref_poll_ms = millis() + ROTOR_POLL_GAP_MS;
    }
    return true;
}

static bool parse_ack_test(const char *line)
{
    if (!strstr(line, ":ACK_TEST:")) {
        return false;
    }
    if (s_pending == Pending::Test) {
        clear_pending();
    }
    if (!s_boot_test_done) {
        s_boot_test_done = true;
    }
    return true;
}

static bool parse_ack_stop(const char *line)
{
    if (!strstr(line, ":ACK_STOP:")) {
        return false;
    }
    if (s_pending == Pending::Stop) {
        clear_pending();
    }
    s_remote_setpos_motion = false;
    s_remote_setpos_grace_end_ms = 0;
    return true;
}

static bool parse_ack_setpwm(const char *line)
{
    if (!strstr(line, ":ACK_SETPWM:")) {
        return false;
    }
    if (s_pending == Pending::SetPwm) {
        clear_pending();
    }
    return true;
}

static float parse_ack_antoff_value_after_tag(const char *line, const char *ack_tag)
{
    const char *p = strstr(line, ack_tag);
    if (!p) {
        return NAN;
    }
    p += strlen(ack_tag);
    char tmp[32];
    size_t n = 0;
    while (*p && *p != ':' && *p != '$' && n + 1 < sizeof(tmp)) {
        tmp[n++] = (*p == ',') ? '.' : *p;
        p++;
    }
    tmp[n] = '\0';
    if (n == 0) {
        return NAN;
    }
    return strtof(tmp, nullptr);
}

static bool parse_ack_getantoff1(const char *line)
{
    if (!strstr(line, ":ACK_GETANTOFF1:")) {
        return false;
    }
    const float v = parse_ack_antoff_value_after_tag(line, ":ACK_GETANTOFF1:");
    if (v == v) {
        pwm_config_set_antoff_deg(1, normalize_deg_0_360(v));
    }
    const bool was = (s_pending == Pending::GetAntOff1);
    if (was) {
        clear_pending();
    }
    if (s_antenna_boot_pending && s_antenna_boot_phase == 1 && was) {
        send_request("GETANTOFF2", "1", Pending::GetAntOff2);
        s_antenna_boot_phase = 2;
    }
    return true;
}

static bool parse_ack_getantoff2(const char *line)
{
    if (!strstr(line, ":ACK_GETANTOFF2:")) {
        return false;
    }
    const float v = parse_ack_antoff_value_after_tag(line, ":ACK_GETANTOFF2:");
    if (v == v) {
        pwm_config_set_antoff_deg(2, normalize_deg_0_360(v));
    }
    const bool was = (s_pending == Pending::GetAntOff2);
    if (was) {
        clear_pending();
    }
    if (s_antenna_boot_pending && s_antenna_boot_phase == 2 && was) {
        send_request("GETANTOFF3", "1", Pending::GetAntOff3);
        s_antenna_boot_phase = 3;
    }
    return true;
}

static bool parse_ack_getantoff3(const char *line)
{
    if (!strstr(line, ":ACK_GETANTOFF3:")) {
        return false;
    }
    const float v = parse_ack_antoff_value_after_tag(line, ":ACK_GETANTOFF3:");
    if (v == v) {
        pwm_config_set_antoff_deg(3, normalize_deg_0_360(v));
    }
    const bool was = (s_pending == Pending::GetAntOff3);
    if (was) {
        clear_pending();
    }
    if (s_antenna_boot_pending && s_antenna_boot_phase == 3 && was) {
        s_antenna_boot_pending = false;
        s_antenna_boot_phase = 0;
        s_pending_antenna_offset_notify = true;
        s_angle_boot_pending = true;
        s_angle_boot_phase = 1;
        send_request("GETANGLE1", "1", Pending::GetAngle1);
    }
    return true;
}

static bool parse_ack_getangle1(const char *line)
{
    if (!strstr(line, ":ACK_GETANGLE1:")) {
        return false;
    }
    const float v = parse_ack_antoff_value_after_tag(line, ":ACK_GETANGLE1:");
    if (v == v) {
        pwm_config_set_opening_deg(1, v);
    }
    const bool was = (s_pending == Pending::GetAngle1);
    if (was) {
        clear_pending();
    }
    if (s_angle_boot_pending && s_angle_boot_phase == 1 && was) {
        send_request("GETANGLE2", "1", Pending::GetAngle2);
        s_angle_boot_phase = 2;
    }
    return true;
}

static bool parse_ack_getangle2(const char *line)
{
    if (!strstr(line, ":ACK_GETANGLE2:")) {
        return false;
    }
    const float v = parse_ack_antoff_value_after_tag(line, ":ACK_GETANGLE2:");
    if (v == v) {
        pwm_config_set_opening_deg(2, v);
    }
    const bool was = (s_pending == Pending::GetAngle2);
    if (was) {
        clear_pending();
    }
    if (s_angle_boot_pending && s_angle_boot_phase == 2 && was) {
        send_request("GETANGLE3", "1", Pending::GetAngle3);
        s_angle_boot_phase = 3;
    }
    return true;
}

static bool parse_ack_getangle3(const char *line)
{
    if (!strstr(line, ":ACK_GETANGLE3:")) {
        return false;
    }
    const float v = parse_ack_antoff_value_after_tag(line, ":ACK_GETANGLE3:");
    if (v == v) {
        pwm_config_set_opening_deg(3, v);
    }
    const bool was = (s_pending == Pending::GetAngle3);
    if (was) {
        clear_pending();
    }
    if (s_angle_boot_pending && s_angle_boot_phase == 3 && was) {
        s_angle_boot_pending = false;
        s_angle_boot_phase = 0;
    }
    return true;
}

static bool parse_ack_getref(const char *line)
{
    const char *p = strstr(line, ":ACK_GETREF:");
    if (!p) {
        return false;
    }
    p += strlen(":ACK_GETREF:");
    if (*p == '\0') {
        return false;
    }
    char valbuf[24];
    size_t i = 0;
    while (*p && *p != ':' && i + 1 < sizeof(valbuf)) {
        valbuf[i++] = (*p == ',') ? '.' : *p;
        p++;
    }
    valbuf[i] = '\0';
    int v = (int)(strtof(valbuf, nullptr) + 0.5f);
    bool ref_ok = (v != 0);
    const bool was_referenced = s_slave_referenced;
    notify_ref(ref_ok);
    const bool became_referenced = ref_ok && !was_referenced;

    if (s_pending == Pending::GetRef) {
        clear_pending();
    }

    /* Boot vor Homing-Logik: GETREF → ggf. GETPOSDG */
    if (!s_boot_done && s_boot_phase == 1) {
        if (ref_ok) {
            s_align_target_after_pos_read = true;
            send_request("GETPOSDG", "0", Pending::GetPosDg);
            s_boot_phase = 2;
        } else {
            s_boot_done = true;
            s_boot_phase = 3;
        }
        return true;
    }

    if (ref_ok && s_request_pos_after_homing) {
        s_request_pos_after_homing = false;
        s_poll_ref = false;
        /* Sonst kein GETPOSDG — Ist bleibt „-“, Soll nicht angeglichen (nur Controller, kein PC) */
        s_align_target_after_pos_read = true;
        send_request("GETPOSDG", "0", Pending::GetPosDg);
        return true;
    }

    /*
     * Referenz neu erlangt (z. B. Slave war weg, Homing ohne Flag-Pfad): Ist immer frisch holen,
     * sonst bleibt s_last_pos_ui_deg / UI auf altem Wert.
     */
    if (became_referenced && s_pending == Pending::None) {
        s_poll_ref = false;
        s_align_target_after_pos_read = true;
        send_request("GETPOSDG", "0", Pending::GetPosDg);
        return true;
    }

    if (ref_ok) {
        s_poll_ref = false;
    } else if (s_poll_ref) {
        s_next_ref_poll_ms = millis() + ROTOR_POLL_GAP_MS;
    }

    return true;
}

static bool parse_ack_getposdg(const char *line)
{
    unsigned ack_src = 0;
    unsigned ack_dst = 0;
    if (!line_src_dst(line, &ack_src, &ack_dst)) {
        return false;
    }
    const char *p = strstr(line, ":ACK_GETPOSDG:");
    if (!p) {
        return false;
    }
    p += strlen(":ACK_GETPOSDG:");
    if (*p == '\0') {
        return false;
    }
    char valbuf[32];
    size_t i = 0;
    while (*p && *p != ':' && i + 1 < sizeof(valbuf)) {
        valbuf[i++] = (*p == ',') ? '.' : *p;
        p++;
    }
    valbuf[i] = '\0';
    const float deg_raw = parse_rs485_deg_valbuf_to_float(valbuf);
    const float deg = normalize_deg_0_360(deg_raw);
    const float deg_ui = bus_deg_for_ui(deg, deg_raw);
    /* Ist: immer aus Slave-Position (Mitläufer-PC kann GETPOSDG mitschicken — Anzeige bleibt aktuell). */
    notify_pos(deg_ui);

    const bool for_us = (ack_dst == (unsigned)s_master_id);
    if (for_us && s_pending == Pending::GetPosDg) {
        clear_pending();
    }

    if (for_us && !s_boot_done && s_boot_phase == 2) {
        s_boot_done = true;
        s_boot_phase = 3;
    }

    /* Positionsfahrt / Toleranz nur mit Antworten auf unser eigenes GETPOSDG — sonst fremde ACKs stören die State Machine. */
    if (for_us && s_poll_pos) {
        if (min_angle_diff(deg, s_target_deg) <= ROTOR_POS_TOL_DEG) {
            if (s_pos_grace_end_ms == 0) {
                s_pos_grace_end_ms = millis() + ROTOR_POLL_POS_GRACE_MS;
            }
            if ((int32_t)(millis() - s_pos_grace_end_ms) >= 0) {
                s_poll_pos = false;
                s_pos_grace_end_ms = 0;
                /* Intern Bus-Ist; Soll-Anzeige = gesendeter SETPOSDG */
                s_target_deg = deg;
                notify_target(s_goto_commanded_deg);
            } else {
                s_next_pos_poll_ms = millis() + ROTOR_POLL_GAP_MS;
            }
        } else {
            s_pos_grace_end_ms = 0;
            s_next_pos_poll_ms = millis() + ROTOR_POLL_GAP_MS;
        }
    }

    /* PC-/USB-Fahrt: gleiche Toleranz wie lokales Polling, kein s_poll_pos */
    if (s_remote_setpos_motion && !s_poll_pos) {
        if (min_angle_diff(deg, s_remote_setpos_target_deg) <= ROTOR_POS_TOL_DEG) {
            if (s_remote_setpos_grace_end_ms == 0) {
                s_remote_setpos_grace_end_ms = millis() + ROTOR_POLL_POS_GRACE_MS;
            }
            if ((int32_t)(millis() - s_remote_setpos_grace_end_ms) >= 0) {
                s_remote_setpos_motion = false;
                s_remote_setpos_grace_end_ms = 0;
            }
        } else {
            s_remote_setpos_grace_end_ms = 0;
        }
    }

    if (for_us && s_align_target_after_pos_read) {
        s_align_target_after_pos_read = false;
        s_target_deg = deg;
        notify_target(deg_ui);
    }
    return true;
}

/** GETERR → ACK_ERR: Code oder 0 (Info/RotorController_RS485.html) */
static bool parse_ack_err(const char *line)
{
    const char *p = strstr(line, ":ACK_ERR:");
    if (!p) {
        return false;
    }
    p += strlen(":ACK_ERR:");
    char valbuf[16];
    size_t i = 0;
    while (*p && *p != ':' && i + 1 < sizeof(valbuf)) {
        valbuf[i++] = (*p == ',') ? '.' : *p;
        p++;
    }
    valbuf[i] = '\0';
    const int code = atoi(valbuf);
    const bool pending_geterr = (s_pending == Pending::GetErr);
    if (!s_startup_err_known) {
        if (pending_geterr || (!s_boot_done && s_boot_phase == 0)) {
            s_startup_err_known = true;
        }
    }
    if (code != 0) {
        rotor_error_app_set_error_code(code);
        stop_fault_motion_polling();
    } else {
        /* Verbindungstimeout (10) nur hier aufheben — nicht bei beliebiger Slave-Zeile (sonst GETREF=1 → „Betriebsbereit“ vor ACK_ERR). */
        rotor_error_app_set_error_code(0);
    }
    if (pending_geterr) {
        clear_pending();
    }
    return true;
}

/** NAK: ausstehende Anfrage abbrechen */
static bool parse_nak_and_clear(const char *line)
{
    if (strstr(line, ":NAK_GETREF:") && s_pending == Pending::GetRef) {
        clear_pending();
        s_request_pos_after_homing = false;
        if (!s_boot_done && s_boot_phase == 1) {
            s_boot_done = true;
            s_boot_phase = 3;
        }
        return true;
    }
    if (strstr(line, ":NAK_GETPOSDG:") && s_pending == Pending::GetPosDg) {
        unsigned nsrc = 0;
        unsigned ndst = 0;
        if (!line_src_dst(line, &nsrc, &ndst) || ndst != (unsigned)s_master_id) {
            return false;
        }
        clear_pending();
        s_align_target_after_pos_read = false;
        if (!s_boot_done && s_boot_phase == 2) {
            s_boot_done = true;
            s_boot_phase = 3;
        }
        return true;
    }
    if (strstr(line, ":NAK_SETPOSDG:") && s_pending == Pending::SetPosDg) {
        unsigned nsrc = 0;
        unsigned ndst = 0;
        if (!line_src_dst(line, &nsrc, &ndst) || ndst != (unsigned)s_master_id) {
            return false;
        }
        clear_pending();
        s_poll_pos = false;
        s_pos_grace_end_ms = 0;
        return true;
    }
    if (strstr(line, ":NAK_SETREF:") && s_pending == Pending::SetRef) {
        clear_pending();
        s_poll_ref = false;
        s_request_pos_after_homing = false;
        return true;
    }
    if (strstr(line, ":NAK_TEST:") && s_pending == Pending::Test) {
        clear_pending();
        if (!s_boot_test_done) {
            s_boot_test_done = true;
        }
        return true;
    }
    if (strstr(line, ":NAK_STOP:") && s_pending == Pending::Stop) {
        clear_pending();
        return true;
    }
    if (strstr(line, ":NAK_SETPWM:") && s_pending == Pending::SetPwm) {
        clear_pending();
        return true;
    }
    if (strstr(line, ":NAK_GETANTOFF1:") && s_pending == Pending::GetAntOff1) {
        clear_pending();
        abort_antenna_boot();
        return true;
    }
    if (strstr(line, ":NAK_GETANTOFF2:") && s_pending == Pending::GetAntOff2) {
        clear_pending();
        abort_antenna_boot();
        return true;
    }
    if (strstr(line, ":NAK_GETANTOFF3:") && s_pending == Pending::GetAntOff3) {
        clear_pending();
        abort_antenna_boot();
        return true;
    }
    if (strstr(line, ":NAK_GETANGLE1:") && s_pending == Pending::GetAngle1) {
        clear_pending();
        abort_angle_boot();
        return true;
    }
    if (strstr(line, ":NAK_GETANGLE2:") && s_pending == Pending::GetAngle2) {
        clear_pending();
        abort_angle_boot();
        return true;
    }
    if (strstr(line, ":NAK_GETANGLE3:") && s_pending == Pending::GetAngle3) {
        clear_pending();
        abort_angle_boot();
        return true;
    }
    if (strstr(line, ":NAK_GETERR:") && s_pending == Pending::GetErr) {
        clear_pending();
        if (!s_startup_err_known) {
            s_startup_err_known = true;
        }
        return true;
    }
    if (strstr(line, ":NAK_GETANEMO:") && s_pending == Pending::GetAnemo) {
        clear_pending();
        return true;
    }
    if (strstr(line, ":NAK_GETTEMPA:") && s_pending == Pending::GetTempA) {
        clear_pending();
        return true;
    }
    if (strstr(line, ":NAK_WINDDIR:") && s_pending == Pending::GetWindDir) {
        clear_pending();
        return true;
    }
    if (strstr(line, ":NAK_GETTEMPM:") && s_pending == Pending::GetTempM) {
        clear_pending();
        return true;
    }
    return false;
}

/** Async ERR (nicht ACK_ERR) — Fehlercode + Pending lösen, Boot abbrechen wenn nötig */
static bool parse_slave_err(const char *line)
{
    const char *p = strstr(line, ":ERR:");
    if (!p) {
        return false;
    }
    p += 5;
    char valbuf[16];
    size_t i = 0;
    while (*p && *p != ':' && i + 1 < sizeof(valbuf)) {
        valbuf[i++] = (*p == ',') ? '.' : *p;
        p++;
    }
    valbuf[i] = '\0';
    if (i > 0U) {
        rotor_error_app_set_error_code(atoi(valbuf));
    }
    if (s_pending == Pending::GetAntOff1 || s_pending == Pending::GetAntOff2 ||
        s_pending == Pending::GetAntOff3) {
        clear_pending();
        abort_antenna_boot();
        return true;
    }
    if (s_pending == Pending::GetAngle1 || s_pending == Pending::GetAngle2 ||
        s_pending == Pending::GetAngle3) {
        clear_pending();
        abort_angle_boot();
        return true;
    }
    if (s_pending == Pending::GetPosDg || s_pending == Pending::SetPosDg || s_pending == Pending::Stop ||
        s_pending == Pending::SetPwm) {
        clear_pending();
        s_align_target_after_pos_read = false;
        if (s_boot_phase == 2) {
            s_boot_done = true;
            s_boot_phase = 3;
        }
        s_poll_pos = false;
        s_pos_grace_end_ms = 0;
        s_remote_setpos_motion = false;
        s_remote_setpos_grace_end_ms = 0;
        return true;
    }
    return false;
}

static void process_complete_line(const char *line, size_t len)
{
    (void)len;
    if (len < 5) {
        return;
    }
    unsigned src = 0;
    unsigned dst = 0;
    if (!line_src_dst(line, &src, &dst)) {
        return;
    }

    /* Anderer Master spricht unseren Rotor an → Mitläufer-Modus (GET-Polling aus loop aussetzen). */
    if (dst == (unsigned)s_slave_id && src != (unsigned)s_master_id) {
        s_last_foreign_master_to_slave_ms = millis();
        s_foreign_master_src_id = (uint8_t)src;
    }

    /* Befehl an den Slave (beliebiger Master): Start Timeout-Uhr „noch keine Slave-Antwort“ (z. B. nur PC) */
    if (dst == (unsigned)s_slave_id && src != (unsigned)s_slave_id) {
        if (s_conn_watch_start_ms == 0) {
            s_conn_watch_start_ms = millis();
        }
    }

    /* Broadcast DST=255: SETCONIDF / SETCONTID → neue master_id (config.json), wenn Controller-ID unbekannt */
    if (dst == ROTOR_RS485_BROADCAST_ID) {
        if (strstr(line, ":SETCONIDF:") || strstr(line, ":SETCONTID:")) {
            handle_broadcast_set_controller_master_id(line, src);
            return;
        }
        /* SETASELECT: auch bei SRC = eigene Master-ID (USB-Loopback vom PC; kein Echo von hw_send).
         * Fremder Master am Broadcast: Mitläufer-Modus setzen (sonst fehlt dst==Slave bei 255). */
        if (strstr(line, ":SETASELECT:")) {
            unsigned v = 0;
            if (parse_setaselect_antenna_1_to_3(line, &v)) {
                if (src != (unsigned)s_master_id) {
                    s_last_foreign_master_to_slave_ms = millis();
                    s_foreign_master_src_id = (uint8_t)src;
                }
                rotor_app_apply_remote_antenna_selection((uint8_t)v);
            }
        }
        return;
    }

    if (src == (unsigned)s_slave_id) {
        s_last_slave_rx_ms = millis();
        s_have_slave_rx_ever = true;
        if (parse_slave_err(line)) {
            return;
        }
        if (parse_nak_and_clear(line)) {
            return;
        }
        /* Reihenfolge: spezifischere ACK-Typen zuerst */
        if (parse_ack_setposdg_result(line)) {
            return;
        }
        if (parse_ack_getposdg(line)) {
            return;
        }
        if (parse_ack_setref_result(line)) {
            return;
        }
        if (parse_ack_test(line)) {
            return;
        }
        if (parse_ack_stop(line)) {
            return;
        }
        if (parse_ack_setpwm(line)) {
            return;
        }
        if (parse_ack_getantoff1(line)) {
            return;
        }
        if (parse_ack_getantoff2(line)) {
            return;
        }
        if (parse_ack_getantoff3(line)) {
            return;
        }
        if (parse_ack_getangle1(line)) {
            return;
        }
        if (parse_ack_getangle2(line)) {
            return;
        }
        if (parse_ack_getangle3(line)) {
            return;
        }
        if (parse_ack_err(line)) {
            return;
        }
        if (parse_ack_getanemo(line)) {
            return;
        }
        if (parse_ack_gettempa(line)) {
            return;
        }
        if (parse_ack_gettempm(line)) {
            return;
        }
        if (parse_ack_winddir(line)) {
            return;
        }
        if (parse_ack_getref(line)) {
            return;
        }
        return;
    }

    /* PC → Controller (master_id): Konfiguration wie config.json; Antwort per hw_send → RS485 + USB */
    if (dst == (unsigned)s_master_id) {
        if (handle_local_config_command(line, src, dst)) {
            return;
        }
    }

    if (dst == (unsigned)s_slave_id) {
        dispatch_bus_command_to_slave(line, src);
    }
}

static void process_rx_byte(int c)
{
    if (s_rx_resync_until_hash) {
        if (c == '#') {
            s_rx_resync_until_hash = false;
            s_rx_len = 0;
            s_rx[s_rx_len++] = (char)c;
        }
        return;
    }
    if (c == '#') {
        s_rx_len = 0;
    }
    if (s_rx_len < sizeof(s_rx) - 1) {
        s_rx[s_rx_len++] = (char)c;
    } else {
        s_rx_resync_until_hash = true;
        s_rx_len = 0;
        return;
    }
    if (c == '$') {
        s_rx[s_rx_len] = '\0';
        process_complete_line(s_rx, s_rx_len);
        s_rx_len = 0;
    }
}

void rotor_rs485_rx_bytes(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    for (size_t i = 0; i < len; i++) {
        process_rx_byte(static_cast<int>(data[i]) & 0xff);
    }
}

void rotor_rs485_init(void)
{
    s_boot_test_done = false;
    s_boot_test_timeout_count = 0;
    s_last_slave_rx_ms = 0;
    s_have_slave_rx_ever = false;
    s_conn_watch_start_ms = 0;
    s_next_conn_recovery_getref_ms = 0;
    s_boot_earliest_ms = millis() + ROTOR_BOOT_QUERY_DELAY_MS;
    s_boot_done = false;
    s_boot_phase = 0;
    s_startup_err_known = false;
    s_next_periodic_getref_ms = millis() + ROTOR_PERIODIC_GETREF_MS;
    s_next_geterr_ms = millis() + ROTOR_POLL_GETERR_MS;
    s_hw_snap_retarget_active = false;
    s_antenna_boot_pending = true;
    s_antenna_boot_phase = 0;
    s_next_weather_ms = millis() + 2000u;
    s_weather_phase = 0;
    s_next_motortemp_ms = millis() + 2000u;
    s_setposcc_queued = false;
}

/** Erstes Paket nach Einschalten: TEST (Ping), 3 Versuche ohne ACK → Fehler 10 */
static void try_boot_test(void)
{
    if (s_boot_test_done) {
        return;
    }
    const int e = rotor_error_app_get_error_code();
    if (e != 0 && e != 10) {
        return;
    }
    if (s_pending != Pending::None) {
        return;
    }
    if (s_boot_test_timeout_count > 0) {
        return;
    }
    send_request("TEST", "0", Pending::Test);
}

/** Einmal GETERR nach TEST und Wartezeit, bevor das erste GETREF die Referenz setzt (Fehler vor „Bereit“). */
static void try_boot_startup_geterr(void)
{
    if (s_startup_err_known || s_boot_done) {
        return;
    }
    if (!s_boot_test_done || s_pending != Pending::None) {
        return;
    }
    if ((int32_t)(millis() - s_boot_earliest_ms) < 0) {
        return;
    }
    if (s_boot_phase != 0) {
        return;
    }
    send_request("GETERR", "0", Pending::GetErr);
}

static void try_boot_getref()
{
    if (s_boot_done || s_pending != Pending::None) {
        return;
    }
    if (!s_boot_test_done) {
        return;
    }
    if (!s_startup_err_known) {
        return;
    }
    if ((int32_t)(millis() - s_boot_earliest_ms) < 0) {
        return;
    }
    if (s_boot_phase != 0) {
        return;
    }
    send_request("GETREF", "0", Pending::GetRef);
    s_boot_phase = 1;
    if (s_conn_watch_start_ms == 0) {
        s_conn_watch_start_ms = millis();
    }
}

/** Erstes GETANTOFF1 nach Boot (Kette ACK → GET2 → GET3 in den Parsern). */
static void try_antenna_boot_first_get(void)
{
    if (!s_boot_done || !s_antenna_boot_pending || s_antenna_boot_phase != 0) {
        return;
    }
    if (s_poll_pos || s_poll_ref) {
        return;
    }
    send_request("GETANTOFF1", "1", Pending::GetAntOff1);
    s_antenna_boot_phase = 1;
}

/** GETANEMO / GETTEMPA / GETWINDDIR nur ohne Fremd-PC, Stillstand (kein Homing-/Positions-Polling).
 * anemometer=0: nur GETTEMPA (Außentemp im Tab Rotor_Info, kein Wind-Tab). */
static void try_weather_poll(void)
{
    if (!s_boot_done || s_poll_pos || s_poll_ref) {
        return;
    }
    if (s_antenna_boot_pending && s_antenna_boot_phase != 0) {
        return;
    }
    if (s_angle_boot_pending && s_angle_boot_phase != 0) {
        return;
    }
    if ((int32_t)(millis() - s_next_weather_ms) < 0) {
        return;
    }
    if (pwm_config_get_anemometer() == 0) {
        send_request("GETTEMPA", "0", Pending::GetTempA);
        s_next_weather_ms = millis() + ROTOR_WEATHER_STAGGER_MS;
        return;
    }
    switch (s_weather_phase % 3u) {
    case 0:
        send_request("GETANEMO", "0", Pending::GetAnemo);
        break;
    case 1:
        send_request("GETTEMPA", "0", Pending::GetTempA);
        break;
    default:
        send_request("GETWINDDIR", "0", Pending::GetWindDir);
        break;
    }
    s_weather_phase = (uint8_t)((s_weather_phase + 1u) % 3u);
    s_next_weather_ms = millis() + ROTOR_WEATHER_STAGGER_MS;
}

/** GETTEMPM alle ROTOR_MOTOR_TEMP_POLL_MS; gleiche Stillstands-/Boot-Sperren wie Wetter. */
static void try_motor_temp_poll(void)
{
    if (!s_boot_done || s_poll_pos || s_poll_ref) {
        return;
    }
    if (s_antenna_boot_pending && s_antenna_boot_phase != 0) {
        return;
    }
    if (s_angle_boot_pending && s_angle_boot_phase != 0) {
        return;
    }
    if ((int32_t)(millis() - s_next_motortemp_ms) < 0) {
        return;
    }
    send_request("GETTEMPM", "0", Pending::GetTempM);
    s_next_motortemp_ms = millis() + ROTOR_MOTOR_TEMP_POLL_MS;
}

void rotor_rs485_loop(void)
{
    const int err = rotor_error_app_get_error_code();
    /* Harte Fehler (≠10): kein Polling — Verbindungstimeout (10): GETREF-Recovery weiter unten */
    if (err != 0 && err != 10) {
        return;
    }

    if (err != 10 && s_boot_test_done) {
        const uint32_t now = millis();
        bool conn_timeout = false;
        if (s_have_slave_rx_ever) {
            if ((uint32_t)(now - s_last_slave_rx_ms) >= ROTOR_CONN_LOST_TIMEOUT_MS) {
                conn_timeout = true;
            }
        } else if (s_conn_watch_start_ms != 0 &&
                   (uint32_t)(now - s_conn_watch_start_ms) >= ROTOR_CONN_LOST_TIMEOUT_MS) {
            conn_timeout = true;
        }
        if (conn_timeout) {
            clear_pending();
            rotor_error_app_set_error_code(10);
            stop_fault_motion_polling();
            s_foreign_master_src_id = 0;
            s_next_conn_recovery_getref_ms = millis() - ROTOR_POLL_GAP_MS;
            return;
        }
    }

    on_ack_timeout();

    /* Vor pending-Return: wartet auf freien Bus (z. B. GETANGLE1…3 nach Boot) — sonst kein erneuter SETPOSDG-Versuch. */
    try_hw_snap_retarget();

    /* SETPOSCC nutzt kein Pending — muss auch laufen während GETPOSDG/anderes ACK aussteht, sonst kein
     * Vorschau-Telegramm beim Encoder während Positionsfahrt oder direkt danach. */
    try_flush_setposcc();

    if (s_pending != Pending::None) {
        return;
    }

    try_boot_test();

    if (s_pending != Pending::None) {
        return;
    }

    /* Fehler 10: GETERR (nicht GETREF) — Slave kann ref=1 melden, obwohl GETERR ≠ 0 (kein Flackern „Betriebsbereit“). */
    if (rotor_error_app_get_error_code() == 10) {
        if (!rotor_rs485_is_foreign_pc_listen_mode()) {
            if ((int32_t)(millis() - s_next_conn_recovery_getref_ms) >= 0) {
                send_request("GETERR", "0", Pending::GetErr);
                s_next_conn_recovery_getref_ms = millis() + ROTOR_POLL_GAP_MS;
            }
        }
        return;
    }

    if (s_pending != Pending::None) {
        return;
    }

    if (rotor_rs485_is_foreign_pc_listen_mode()) {
        if (s_boot_test_done && !s_startup_err_known) {
            s_startup_err_known = true;
        }
        /* Ohne Ausnahme: kein eigenes GETPOSDG bei lokaler Fahrt → Ist nur über fremde Master-ACKs, wirkt wie Nachlaufen. */
        if (!s_poll_pos && !s_poll_ref) {
            return;
        }
    }

    try_boot_startup_geterr();

    if (s_pending != Pending::None) {
        return;
    }

    try_boot_getref();

    if (s_pending != Pending::None) {
        return;
    }

    try_antenna_boot_first_get();

    if (s_pending != Pending::None) {
        return;
    }

    if (s_poll_pos) {
        if ((int32_t)(millis() - s_next_pos_poll_ms) >= 0) {
            send_request("GETPOSDG", "0", Pending::GetPosDg);
        }
        return;
    }

    if (s_poll_ref) {
        if ((int32_t)(millis() - s_next_ref_poll_ms) >= 0) {
            send_request("GETREF", "0", Pending::GetRef);
        }
        return;
    }

    if (s_boot_done && (int32_t)(millis() - s_next_periodic_getref_ms) >= 0) {
        send_request("GETREF", "0", Pending::GetRef);
        s_next_periodic_getref_ms = millis() + ROTOR_PERIODIC_GETREF_MS;
        return;
    }

    if (s_boot_done && (int32_t)(millis() - s_next_geterr_ms) >= 0) {
        send_request("GETERR", "0", Pending::GetErr);
        s_next_geterr_ms = millis() + ROTOR_POLL_GETERR_MS;
        return;
    }

    try_weather_poll();
    if (s_pending == Pending::None) {
        try_motor_temp_poll();
    }
}
