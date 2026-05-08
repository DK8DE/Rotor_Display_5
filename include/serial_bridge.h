/**
 * USB-Serial (CDC) <-> UART-Transparent mit RS485-Richtungssteuerung.
 * Modular gehalten; main ruft nur begin() und poll() auf.
 */
#pragma once

#include <Arduino.h>

namespace serial_bridge {

enum class BridgeMode : uint8_t {
    LocalMaster = 0,
    PcProxyMaster = 1,
};

/** UART-Baud (USB und HW identisch). */
void set_baud(uint32_t baud);

/** UART2 auf RX/TX-Pins, RS485-Dir-Pin; USB-Serial muss bereits initialisiert sein. */
void begin();

/** Wartung der Bridge-Modus-Zeitfenster; die Datenpfade laufen in eigenen Tasks. */
void poll();

/** Umschalten zwischen lokalem Master und PC-Proxy-Master. */
void set_mode(BridgeMode mode);
BridgeMode get_mode();

/**
 * Gemeinsamer Mutex für direkten Zugriff auf Serial2 (RS485).
 * Normaler Versand soll über hw_send()/hw_send_priority() laufen.
 */
void uart_lock();
void uart_unlock();

/**
 * RS485 senden. Intern wird in die zentrale RS485-TX-Queue gelegt; ein einzelner
 * TX-Task übernimmt Bus-Idle, DE/RE-Umschaltung und UART-Write.
 */
void hw_send(const uint8_t *data, size_t len);

/**
 * Priorisierter RS485-Versand (für SETPOSCC): kürzere Idle-Wartezeit als hw_send(),
 * damit Vorschau-Sollwerte bei laufendem Fremd-GETPOSDG nicht verzögert werden.
 */
void hw_send_priority(const uint8_t *data, size_t len);

} // namespace serial_bridge
