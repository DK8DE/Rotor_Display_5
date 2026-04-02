/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <algorithm>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "ESP_Knob.h"

static const char *TAG = "ESP_Knob";

ESP_Knob::ESP_Knob(int gpio_encoder_a, int gpio_encoder_b):
    _knob_handle(NULL),
    _direction(0),
    _gpio_encoder_a(gpio_encoder_a),
    _gpio_encoder_b(gpio_encoder_b),
    _accel_percent(0),
    _accel_threshold_us(70000),
    _accel_idle_reset_us(200000),
    _accel_max_mult(4),
    _accel_last_event_us(0),
    _accel_last_direction(0),
    _event_data({this, NULL})
{
}

ESP_Knob::~ESP_Knob()
{
    if (_knob_handle != NULL) {
        iot_knob_delete(_knob_handle);
    }
}

void ESP_Knob::invertDirection(void)
{
    _direction = !_direction;
}

void ESP_Knob::begin()
{
    const knob_config_t knob_cfg = {
        .default_direction = (uint8_t)_direction,
        .gpio_encoder_a = (uint8_t)_gpio_encoder_a,
        .gpio_encoder_b = (uint8_t)_gpio_encoder_b,
    };
    _knob_handle = iot_knob_create(&knob_cfg);
    if (_knob_handle == NULL) {
        ESP_LOGE(TAG, "Error create knob");
    }
}

void ESP_Knob::del()
{
    if (_knob_handle != NULL) {
        iot_knob_delete(_knob_handle);
        _knob_handle = NULL;
    }
}

knob_event_t ESP_Knob::getEvent()
{
    return iot_knob_get_event(_knob_handle);
}

int ESP_Knob::getCountValue()
{
    return iot_knob_get_count_value(_knob_handle);
}

void ESP_Knob::clearCountValue()
{
    iot_knob_clear_count_value(_knob_handle);
}

void ESP_Knob::setEventUserDate(void *usr_data)
{
    _event_data.usr_data = usr_data;
}

void ESP_Knob::setAcceleration(uint8_t accel_percent, uint32_t threshold_us, uint8_t max_multiplier)
{
    _accel_percent = std::min(accel_percent, static_cast<uint8_t>(100));
    _accel_threshold_us = (threshold_us > 0U) ? threshold_us : 70000U;
    if (max_multiplier < 1U) {
        max_multiplier = 1;
    }
    if (max_multiplier > 16U) {
        max_multiplier = 16;
    }
    _accel_max_mult = max_multiplier;
}

void ESP_Knob::setAccelerationIdleResetUs(uint32_t idle_us)
{
    _accel_idle_reset_us = (idle_us > 0U) ? idle_us : 200000U;
}

int ESP_Knob::computeAccelSteps(int8_t direction)
{
    if (_accel_percent == 0U || _accel_max_mult <= 1U) {
        _accel_last_direction = direction;
        _accel_last_event_us = esp_timer_get_time();
        return 1;
    }

    const uint64_t now = esp_timer_get_time();
    int steps = 1;

    if (_accel_last_direction != 0 && _accel_last_direction != direction) {
        _accel_last_direction = direction;
        _accel_last_event_us = now;
        return 1;
    }

    if (_accel_last_event_us != 0ULL) {
        const uint64_t dt = now - _accel_last_event_us;
        if (dt > static_cast<uint64_t>(_accel_idle_reset_us)) {
            steps = 1;
        } else if (dt < static_cast<uint64_t>(_accel_threshold_us)) {
            const uint64_t span = static_cast<uint64_t>(_accel_threshold_us) - dt;
            const uint64_t numer = span * static_cast<uint64_t>(_accel_max_mult - 1U);
            uint32_t extra = static_cast<uint32_t>(numer / static_cast<uint64_t>(_accel_threshold_us));
            extra = (extra * static_cast<uint32_t>(_accel_percent)) / 100U;
            steps = 1 + static_cast<int>(extra);
            if (steps > static_cast<int>(_accel_max_mult)) {
                steps = static_cast<int>(_accel_max_mult);
            }
        }
    }

    _accel_last_direction = direction;
    _accel_last_event_us = now;
    return steps;
}

void ESP_Knob::attachLeftEventCallback(std::function<void(int, void *)> callback)
{
    esp_err_t err = iot_knob_register_cb(_knob_handle, KNOB_LEFT, onEventCallback, &_event_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error register knob left callback");
    }
    _left_event_cb = callback;
}

void ESP_Knob::detachLeftEventCallback()
{
    esp_err_t err = iot_knob_unregister_cb(_knob_handle, KNOB_LEFT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error unregister knob left callback");
    }
}

void ESP_Knob::attachRightEventCallback(std::function<void(int, void *)> callback)
{
    esp_err_t err = iot_knob_register_cb(_knob_handle, KNOB_RIGHT, onEventCallback, &_event_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error register knob right callback");
    }
    _right_event_cb = callback;
}

void ESP_Knob::detachRightEventCallback()
{
    esp_err_t err = iot_knob_unregister_cb(_knob_handle, KNOB_RIGHT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error unregister knob right callback");
    }
}

void ESP_Knob::attachHighLimitEventCallback(std::function<void(int, void *)> callback)
{
    esp_err_t err = iot_knob_register_cb(_knob_handle, KNOB_H_LIM, onEventCallback, &_event_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error register knob high limit callback");
    }
    _hight_limit_event_cb = callback;
}

void ESP_Knob::detachHighLimitEventCallback()
{
    esp_err_t err = iot_knob_unregister_cb(_knob_handle, KNOB_H_LIM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error unregister knob high limit callback");
    }
}

void ESP_Knob::attachLowLimitEventCallback(std::function<void(int, void *)> callback)
{
    esp_err_t err = iot_knob_register_cb(_knob_handle, KNOB_L_LIM, onEventCallback, &_event_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error register knob low limit callback");
    }
    _low_limit_event_cb = callback;
}

void ESP_Knob::detachLowLimitEventCallback()
{
    esp_err_t err = iot_knob_unregister_cb(_knob_handle, KNOB_L_LIM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error unregister knob low limit callback");
    }
}

void ESP_Knob::attachZeroEventCallback(std::function<void(int, void *)> callback)
{
    esp_err_t err = iot_knob_register_cb(_knob_handle, KNOB_ZERO, onEventCallback, &_event_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error register knob zero callback");
    }
    _zero_event_cb = callback;
}

void ESP_Knob::detachZeroEventCallback()
{
    esp_err_t err = iot_knob_unregister_cb(_knob_handle, KNOB_ZERO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error unregister knob zero callback");
    }
}

void ESP_Knob::onEventCallback(void *arg, void *data)
{
    event_callback_data_t *event_data = (event_callback_data_t *)data;
    ESP_Knob *knob = (ESP_Knob *)event_data->knob;

    switch (knob->getEvent()) {
    case KNOB_LEFT: {
        const int n = knob->computeAccelSteps(-1);
        const int cv = knob->getCountValue();
        if (knob->_left_event_cb) {
            for (int i = 0; i < n; ++i) {
                knob->_left_event_cb(cv, event_data->usr_data);
            }
        }
        break;
    }
    case KNOB_RIGHT: {
        const int n = knob->computeAccelSteps(1);
        const int cv = knob->getCountValue();
        if (knob->_right_event_cb) {
            for (int i = 0; i < n; ++i) {
                knob->_right_event_cb(cv, event_data->usr_data);
            }
        }
        break;
    }
    case KNOB_H_LIM:
        knob->_hight_limit_event_cb(knob->getCountValue(), event_data->usr_data);
        break;
    case KNOB_L_LIM:
        knob->_low_limit_event_cb(knob->getCountValue(), event_data->usr_data);
        break;
    case KNOB_ZERO:
        knob->_zero_event_cb(knob->getCountValue(), event_data->usr_data);
        break;
    default:
        break;
    }
}
