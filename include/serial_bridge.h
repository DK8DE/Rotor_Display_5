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

/**
 * Gemeinsamer Mutex für Zugriff auf Serial2 (RS485), falls mehrere Module senden/empfangen.
 * Rotor-Protokoll und USB-Brücke nutzen dieselbe UART.
 */
void uart_lock();
void uart_unlock();

/**
 * RS485 senden (DE/RE umschalten, schreiben, wieder Empfang).
 * Dieselben Bytes werden auf USB-Serial (Serial) ausgegeben, damit SETPOSDG u. a.
 * vom Display aus auch am PC-Terminal sichtbar sind (HW-RX echo’t lokales TX nicht).
 */
void hw_send(const uint8_t *data, size_t len);

} // namespace serial_bridge
