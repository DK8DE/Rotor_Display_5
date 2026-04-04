/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <functional>
#include <stdint.h>
#include "base/iot_knob.h"

typedef struct {
    void *knob;
    void *usr_data;
} event_callback_data_t;

class ESP_Knob {
public:

    /**
     * @brief Construct a new knob object
     *
     * @param gpio_encoder_a Encoder Pin A
     * @param gpio_encoder_b Encoder Pin B
     */
    ESP_Knob(int gpio_encoder_a, int gpio_encoder_b);

    /**
     * @brief Destroy the knob object
     *
     */
    ~ESP_Knob(void);

    /**
     * @brief Invert the roatae direction of the knob
     *
     * @note  This function will change the direction of count value increment
     * @note  This function should be called before `begin()`
     *
     */
    void invertDirection(void);

    /**
     * @brief create a knob
     *
     */
    void begin(void);

    /**
     * @brief Delete a knob
     *
     */
    void del(void);

    /**
     * @brief Get knob event
     *
     * @return knob_event_t Knob event
     */
    knob_event_t getEvent(void);

    /**
     * @brief Get knob count value
     *
     */
    int getCountValue(void);

    /**
     * @brief Clear knob cout value to zero
     *
     */
    void clearCountValue(void);

    /**
     * @brief Set the user date for all events
     *
     * @param usr_data
     */
    void setEventUserDate(void *usr_data);

    /**
     * @brief Attach the knob left callback function
     *
     * @param callback Callback function
     * @param usr_data user data
     */
    void attachLeftEventCallback(std::function<void(int, void *)> callback);

    /**
     * @brief Detach the knob right callback function
     *
     */
    void detachLeftEventCallback(void);

    /**
     * @brief Attach the knob right callback function
     *
     * @param callback Callback function
     * @param usr_data user data
     */
    void attachRightEventCallback(std::function<void(int, void *)> callback);

    /**
     * @brief Detach the knob right callback function
     *
     */
    void detachRightEventCallback(void);

    /**
     * @brief Attach the knob count reaches maximum limit callback function
     *
     * @note  High limit is set to `1000` default
     *
     * @param callback Callback function
     * @param usr_data user data
     */
    void attachHighLimitEventCallback(std::function<void(int, void *)> callback);

    /**
     * @brief Detach the knob count reaches maximum limit callback function
     *
     */
    void detachHighLimitEventCallback(void);

    /**
     * @brief Attach the knob count reaches minimum limit callback function
     *
     * @note  Low limit is set to `-1000` default
     *
     * @param callback Callback function
     * @param usr_data user data
     */
    void attachLowLimitEventCallback(std::function<void(int, void *)> callback);

    /**
     * @brief Detach the knob count reaches minimum limit callback function
     *
     */
    void detachLowLimitEventCallback(void);

    /**
     * @brief Attach the knob count reaches zero callback function
     *
     * @param callback Callback function
     * @param usr_data user data
     */
    void attachZeroEventCallback(std::function<void(int, void *)> callback);

    /**
     * @brief Detach the knob count reaches zero callback function
     *
     */
    void detachZeroEventCallback(void);

    /**
     * @brief Beschleunigung bei schnellem Drehen (kurze Zeit zwischen zwei Rasten, gleiche Richtung).
     * Langsam (Abstand ≥ threshold_us oder nach Pause): genau 1× Callback pro Raste.
     * Schnell: bis zu max_multiplier Aufrufe pro physischer Raste — quadratische Kennlinie:
     * je kürzer das Zeitfenster zwischen den Rasten, desto stärker der Zusatz (wie UltraEncoder).
     *
     * @param accel_percent 0 = aus, 1–100 Stärke (100 = volle Nutzung bis max_multiplier)
     * @param threshold_us Obergrenze in µs: liegt dt darunter, gilt die Raste als „schnell“ (z. B. 50 ms = 50000)
     * @param max_multiplier Obergrenze 2–32 (z. B. 24 → bis zu 24 virtuelle Schritte pro Raste bei sehr schnellem Drehen)
     */
    void setAcceleration(uint8_t accel_percent, uint32_t threshold_us, uint8_t max_multiplier);

    /**
     * @brief Kennlinie: quadratisch (Standard, aggressiver bei sehr schnellem Drehen) oder linear (flacher Anstieg).
     */
    void setAccelerationCurveQuadratic(bool quadratic);

    /**
     * @brief Nach längerer Pause gilt die nächste Raste wieder als „langsam“ (1×). Standard 200 ms.
     */
    void setAccelerationIdleResetUs(uint32_t idle_us);

private:
    static void onEventCallback(void *arg, void *data);
    int computeAccelSteps(int8_t direction);

    knob_handle_t _knob_handle; /**< Knob handle */

    int _direction;             /*!< Count increase direction */
    int _gpio_encoder_a;        /*!< Encoder Pin A */
    int _gpio_encoder_b;        /*!< Encoder Pin B */

    uint8_t _accel_percent;           /*!< 0 = Beschleunigung aus */
    uint32_t _accel_threshold_us;     /*!< Unterhalb: schnelle Folge */
    uint32_t _accel_idle_reset_us;    /*!< Darüber: neue langsame Raste */
    uint8_t _accel_max_mult;          /*!< 2–32 */
    bool _accel_quadratic_curve;     /*!< true: (dt/th)^2, false: linear */
    uint64_t _accel_last_event_us;
    int8_t _accel_last_direction;     /*!< -1 links, +1 rechts, 0 = noch kein Event */

    event_callback_data_t _event_data;
    std::function<void(int, void *)> _left_event_cb;    /*!< Callback function for knob left event */
    std::function<void(int, void *)> _right_event_cb;    /*!< Callback function for knob left event */
    std::function<void(int, void *)> _hight_limit_event_cb;    /*!< Callback function for knob left event */
    std::function<void(int, void *)> _low_limit_event_cb;    /*!< Callback function for knob left event */
    std::function<void(int, void *)> _zero_event_cb;    /*!< Callback function for knob left event */
};
