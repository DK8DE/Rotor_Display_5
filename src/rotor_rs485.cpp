/**
 * RS485-Telegramme: #SRC:DST:CMD:PARAMS:CS$
 * CS = SRC + DST + letzte Zahl in PARAMS (float), wie RotorTcpBridge/rs485_protocol.py — kein scaled100.
 * CS-Format wie _fmt_cs: ganzzahlig ohne ",00", sonst bis zu zwei Nachkommastellen (Komma).
 *
 * Bus-Regel: Pro Anfrage ein ausstehendes Telegramm; nächste Anfrage erst nach ACK (oder Timeout).
 */

#include "rotor_rs485.h"

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "serial_bridge.h"

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

enum class Pending : uint8_t { None, GetRef, GetPosDg, SetPosDg, SetRef, Stop };

static uint8_t s_master_id = ROTOR_RS485_MASTER_ID;
static uint8_t s_slave_id = ROTOR_RS485_SLAVE_ID;

static rotor_rs485_ref_cb_t s_ref_cb = nullptr;
static rotor_rs485_pos_cb_t s_pos_cb = nullptr;
static rotor_rs485_target_cb_t s_target_cb = nullptr;

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

/** Nach GETPOSDG-Antwort: Soll = Ist (Einschalten, Boot, Homing) */
static bool s_align_target_after_pos_read = false;
/** Nach SETREF/Homing: bei nächstem GETREF=referenziert einmal GETPOSDG anfordern */
static bool s_request_pos_after_homing = false;

static char s_rx[320];
static size_t s_rx_len = 0;

/** Zuletzt aus ACK_GETREF (für UI / Taster / LED) */
static bool s_slave_referenced = false;
/** Zuletzt aus ACK_GETPOSDG / notify_pos (UI-Grad inkl. 360°-Anzeige) */
static float s_last_pos_ui_deg = 0.0f;
/** GETREF alle ROTOR_PERIODIC_GETREF_MS nach Boot, solange kein Homing-Polling */
static uint32_t s_next_periodic_getref_ms = 0;

/** HW-Taster: Soll = Ist — SETPOSDG wiederholen bis ACK (Bus busy / kein ACK). */
static bool s_hw_snap_retarget_active = false;
static float s_hw_snap_retarget_deg = 0.0f;

static void notify_target(float deg);

void rotor_rs485_set_master_id(uint8_t id) { s_master_id = id; }
void rotor_rs485_set_slave_id(uint8_t id) { s_slave_id = id; }

void rotor_rs485_set_ref_callback(rotor_rs485_ref_cb_t cb) { s_ref_cb = cb; }
void rotor_rs485_set_position_callback(rotor_rs485_pos_cb_t cb) { s_pos_cb = cb; }
void rotor_rs485_set_target_callback(rotor_rs485_target_cb_t cb) { s_target_cb = cb; }

bool rotor_rs485_is_referenced(void) { return s_slave_referenced; }

bool rotor_rs485_is_moving(void) { return s_poll_pos || s_poll_ref; }

bool rotor_rs485_is_homing(void) { return s_poll_ref && !s_slave_referenced; }

bool rotor_rs485_is_position_polling(void) { return s_poll_pos; }

float rotor_rs485_get_last_position_deg(void) { return s_last_pos_ui_deg; }

static void clear_pending()
{
    s_pending = Pending::None;
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
 */
static float bus_deg_for_ui(float deg_norm, float deg_raw)
{
    if (deg_norm < 0.01f && deg_raw >= 359.5f) {
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

static void send_line(const char *cmd, const char *params_str)
{
    float last = last_number_in_params_python(params_str);
    float cs_val = (float)s_master_id + (float)s_slave_id + last;
    char cs[32];
    format_cs_python(cs, sizeof(cs), cs_val);
    char line[192];
    snprintf(line, sizeof(line), "#%u:%u:%s:%s:%s$", (unsigned)s_master_id, (unsigned)s_slave_id, cmd, params_str, cs);
    serial_bridge::hw_send(reinterpret_cast<const uint8_t *>(line), strlen(line));
}

static void send_request(const char *cmd, const char *params_str, Pending p)
{
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
    if (s_pending == Pending::GetPosDg || s_pending == Pending::GetRef) {
        clear_pending();
    }
    if (s_pending != Pending::None) {
        return false;
    }
    s_poll_pos = false;
    s_pos_grace_end_ms = 0;
    s_poll_ref = false;
    s_request_pos_after_homing = false;
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
    /* taget_dg sofort wie übergebener Soll (Ist bei „Taster: Ziel = hier“); gleicher Winkel wie Bus nach normalize */
    notify_target(deg);
    return true;
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
    s_last_pos_ui_deg = deg;
    if (s_pos_cb) {
        s_pos_cb(deg);
    }
}

static void notify_target(float deg)
{
    if (s_target_cb) {
        s_target_cb(deg);
    }
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

static void dispatch_bus_command_to_slave(const char *line)
{
    float deg = 0.0f;
    if (parse_setposdg_command_deg(line, &deg)) {
        notify_target(deg);
    }
}

static void on_ack_timeout()
{
    if (!pending_timed_out()) {
        return;
    }
    Pending was = s_pending;
    clear_pending();

    /* Boot: GETREF erneut versuchen solange Phase 1 und noch kein Abschluss */
    if (!s_boot_done && s_boot_phase == 1 && was == Pending::GetRef) {
        send_request("GETREF", "0", Pending::GetRef);
    }
}

/** Quittung auf SETPOSDG — danach erst GETPOSDG */
static bool parse_ack_setposdg_result(const char *line)
{
    const char *p = strstr(line, ":ACK_SETPOSDG:");
    if (!p) {
        return false;
    }
    if (s_pending == Pending::SetPosDg) {
        clear_pending();
        s_hw_snap_retarget_active = false;
    }
    if (s_poll_pos) {
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

static bool parse_ack_stop(const char *line)
{
    if (!strstr(line, ":ACK_STOP:")) {
        return false;
    }
    if (s_pending == Pending::Stop) {
        clear_pending();
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

    /* Homing fertig (referenziert): Ist abfragen, danach Soll = Ist */
    if (ref_ok && s_request_pos_after_homing) {
        s_request_pos_after_homing = false;
        s_poll_ref = false;
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
    const float deg_raw = strtof(valbuf, nullptr);
    const float deg = normalize_deg_0_360(deg_raw);
    const float deg_ui = bus_deg_for_ui(deg, deg_raw);
    notify_pos(deg_ui);

    if (s_pending == Pending::GetPosDg) {
        clear_pending();
    }

    if (!s_boot_done && s_boot_phase == 2) {
        s_boot_done = true;
        s_boot_phase = 3;
    }

    if (s_poll_pos) {
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

    if (s_align_target_after_pos_read) {
        s_align_target_after_pos_read = false;
        s_target_deg = deg;
        notify_target(deg_ui);
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
        clear_pending();
        s_align_target_after_pos_read = false;
        if (!s_boot_done && s_boot_phase == 2) {
            s_boot_done = true;
            s_boot_phase = 3;
        }
        return true;
    }
    if (strstr(line, ":NAK_SETPOSDG:") && s_pending == Pending::SetPosDg) {
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
    if (strstr(line, ":NAK_STOP:") && s_pending == Pending::Stop) {
        clear_pending();
        return true;
    }
    return false;
}

/** ERR vom Slave — Pending lösen, Boot abbrechen wenn nötig */
static bool parse_slave_err(const char *line)
{
    if (!strstr(line, ":ERR:")) {
        return false;
    }
    if (s_pending == Pending::GetPosDg || s_pending == Pending::SetPosDg || s_pending == Pending::Stop) {
        clear_pending();
        s_align_target_after_pos_read = false;
        if (s_boot_phase == 2) {
            s_boot_done = true;
            s_boot_phase = 3;
        }
        s_poll_pos = false;
        s_pos_grace_end_ms = 0;
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

    if (src == (unsigned)s_slave_id) {
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
        if (parse_ack_stop(line)) {
            return;
        }
        if (parse_ack_getref(line)) {
            return;
        }
        return;
    }

    if (dst == (unsigned)s_slave_id) {
        dispatch_bus_command_to_slave(line);
    }
}

static void process_rx_byte(int c)
{
    if (c == '#') {
        s_rx_len = 0;
    }
    if (s_rx_len < sizeof(s_rx) - 1) {
        s_rx[s_rx_len++] = (char)c;
    } else {
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
    s_boot_earliest_ms = millis() + ROTOR_BOOT_QUERY_DELAY_MS;
    s_boot_done = false;
    s_boot_phase = 0;
    s_next_periodic_getref_ms = millis() + ROTOR_PERIODIC_GETREF_MS;
    s_hw_snap_retarget_active = false;
}

static void try_boot_getref()
{
    if (s_boot_done || s_pending != Pending::None) {
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
}

void rotor_rs485_loop(void)
{
    on_ack_timeout();

    if (s_pending != Pending::None) {
        return;
    }

    /* Vor GETPOSDG-Polling: SETPOSDG für HW-Snap erneut versuchen, wenn Bus eben noch belegt war. */
    try_hw_snap_retarget();

    if (s_pending != Pending::None) {
        return;
    }

    try_boot_getref();

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
    }
}
