/**
 * USB-Serial (CDC) <-> UART-Transparent mit RS485-Richtungssteuerung.
 * Modular gehalten; main ruft nur begin() und poll() auf.
 */
#pragma once

#include <Arduino.h>

namespace serial_bridge {

/** UART-Baud (USB und HW identisch). */
void set_baud(uint32_t baud);

/** UART2 auf RX/TX-Pins, RS485-Dir-Pin; USB-Serial muss bereits initialisiert sein. */
void begin();

/** In loop() sehr oft aufrufen (non-blocking). */
void poll();

} // namespace serial_bridge
