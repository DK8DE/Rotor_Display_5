/**
 * config.json auf FFat (data/ → uploadfs): PWM, Antennen-Bezeichner, Versätze, letzte Antenne.
 */

#include "pwm_config.h"

#include <FFat.h>
#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t s_slow = 50;
static uint8_t s_fast = 100;

static char s_ant_label[3][48];
static uint8_t s_last_antenna = 1;
static float s_antoff_deg[3] = { 0.0f, 0.0f, 0.0f };

static int parse_int_after_key(const char *json, const char *key)
{
    const char *p = strstr(json, key);
    if (!p) {
        return -1;
    }
    p = strchr(p, ':');
    if (!p) {
        return -1;
    }
    ++p;
    while (*p && *p != '-' && (*p < '0' || *p > '9')) {
        ++p;
    }
    return atoi(p);
}

static float parse_float_after_key(const char *json, const char *key)
{
    const char *p = strstr(json, key);
    if (!p) {
        return NAN;
    }
    p = strchr(p, ':');
    if (!p) {
        return NAN;
    }
    ++p;
    while (*p && (*p == ' ' || *p == '\t')) {
        ++p;
    }
    char *end = nullptr;
    double v = strtod(p, &end);
    if (end == p) {
        return NAN;
    }
    return (float)v;
}

/** "key": "value" — value ohne escapte Anführungszeichen in value */
static bool parse_string_quoted(const char *json, const char *key, char *out, size_t out_sz)
{
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) {
        return false;
    }
    p = strchr(p + 1, ':');
    if (!p) {
        return false;
    }
    ++p;
    while (*p && (*p == ' ' || *p == '\t')) {
        ++p;
    }
    if (*p != '"') {
        return false;
    }
    ++p;
    if (*p == '"') {
        out[0] = '\0';
        return true;
    }
    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_sz) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return true;
}

static void escape_json_string(const char *in, char *out, size_t out_sz)
{
    size_t o = 0;
    if (!in || !out || out_sz < 4) {
        return;
    }
    while (*in && o + 2 < out_sz) {
        if (*in == '"' || *in == '\\') {
            if (o + 3 >= out_sz) {
                break;
            }
            out[o++] = '\\';
        }
        out[o++] = *in++;
    }
    out[o] = '\0';
}

void pwm_config_load_defaults(void)
{
    s_slow = 50;
    s_fast = 100;
    s_last_antenna = 1;
    strncpy(s_ant_label[0], "KW Beam", sizeof(s_ant_label[0]) - 1);
    strncpy(s_ant_label[1], "2m / 70cm", sizeof(s_ant_label[1]) - 1);
    strncpy(s_ant_label[2], "23 cm", sizeof(s_ant_label[2]) - 1);
    s_ant_label[0][sizeof(s_ant_label[0]) - 1] = '\0';
    s_ant_label[1][sizeof(s_ant_label[1]) - 1] = '\0';
    s_ant_label[2][sizeof(s_ant_label[2]) - 1] = '\0';
    s_antoff_deg[0] = s_antoff_deg[1] = s_antoff_deg[2] = 0.0f;
}

void pwm_config_load(void)
{
    pwm_config_load_defaults();
    File f = FFat.open("/config.json", "r");
    if (!f) {
        pwm_config_save();
        return;
    }
    char buf[768];
    size_t n = f.readBytes(buf, sizeof(buf) - 1);
    f.close();
    buf[n] = '\0';
    int a = parse_int_after_key(buf, "slow_pwm");
    int b = parse_int_after_key(buf, "fast_pwm");
    if (a >= 0 && a <= 100) {
        s_slow = (uint8_t)a;
    }
    if (b >= 0 && b <= 100) {
        s_fast = (uint8_t)b;
    }
    int la = parse_int_after_key(buf, "last_antenna");
    if (la >= 1 && la <= 3) {
        s_last_antenna = (uint8_t)la;
    }
    char tmp[48];
    if (parse_string_quoted(buf, "antenna_1_label", tmp, sizeof(tmp))) {
        strncpy(s_ant_label[0], tmp, sizeof(s_ant_label[0]) - 1);
        s_ant_label[0][sizeof(s_ant_label[0]) - 1] = '\0';
    }
    if (parse_string_quoted(buf, "antenna_2_label", tmp, sizeof(tmp))) {
        strncpy(s_ant_label[1], tmp, sizeof(s_ant_label[1]) - 1);
        s_ant_label[1][sizeof(s_ant_label[1]) - 1] = '\0';
    }
    if (parse_string_quoted(buf, "antenna_3_label", tmp, sizeof(tmp))) {
        strncpy(s_ant_label[2], tmp, sizeof(s_ant_label[2]) - 1);
        s_ant_label[2][sizeof(s_ant_label[2]) - 1] = '\0';
    }
    for (int i = 0; i < 3; i++) {
        char key[24];
        snprintf(key, sizeof(key), "antoff%u_deg", (unsigned)(i + 1));
        float v = parse_float_after_key(buf, key);
        if (!isnan(v)) {
            s_antoff_deg[i] = v;
        }
    }
}

void pwm_config_save(void)
{
    File f = FFat.open("/config.json", "w");
    if (!f) {
        return;
    }
    char e1[96], e2[96], e3[96];
    escape_json_string(s_ant_label[0], e1, sizeof(e1));
    escape_json_string(s_ant_label[1], e2, sizeof(e2));
    escape_json_string(s_ant_label[2], e3, sizeof(e3));
    char line[768];
    snprintf(line, sizeof(line),
             "{\n"
             "  \"slow_pwm\": %u,\n"
             "  \"fast_pwm\": %u,\n"
             "  \"antenna_1_label\": \"%s\",\n"
             "  \"antenna_2_label\": \"%s\",\n"
             "  \"antenna_3_label\": \"%s\",\n"
             "  \"last_antenna\": %u,\n"
             "  \"antoff1_deg\": %.2f,\n"
             "  \"antoff2_deg\": %.2f,\n"
             "  \"antoff3_deg\": %.2f\n"
             "}\n",
             (unsigned)s_slow, (unsigned)s_fast, e1, e2, e3, (unsigned)s_last_antenna,
             (double)s_antoff_deg[0], (double)s_antoff_deg[1], (double)s_antoff_deg[2]);
    f.print(line);
    f.close();
}

uint8_t pwm_config_get_slow(void)
{
    return s_slow;
}

uint8_t pwm_config_get_fast(void)
{
    return s_fast;
}

uint8_t pwm_config_get_last_antenna(void)
{
    return s_last_antenna;
}

void pwm_config_set_last_antenna(uint8_t n)
{
    if (n >= 1 && n <= 3) {
        s_last_antenna = n;
    }
}

const char *pwm_config_get_antenna_label(int idx)
{
    if (idx < 1 || idx > 3) {
        return "";
    }
    return s_ant_label[idx - 1];
}

void pwm_config_set_antenna_label(int idx, const char *s)
{
    if (idx < 1 || idx > 3 || !s) {
        return;
    }
    strncpy(s_ant_label[idx - 1], s, sizeof(s_ant_label[0]) - 1);
    s_ant_label[idx - 1][sizeof(s_ant_label[0]) - 1] = '\0';
}

float pwm_config_get_antoff_deg(int idx)
{
    if (idx < 1 || idx > 3) {
        return 0.0f;
    }
    return s_antoff_deg[idx - 1];
}

void pwm_config_set_antoff_deg(int idx, float deg)
{
    if (idx < 1 || idx > 3) {
        return;
    }
    s_antoff_deg[idx - 1] = deg;
}
