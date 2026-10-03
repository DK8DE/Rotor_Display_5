/**
 * USB-Serial (CDC) <-> UART-Transparent (RS485, selbstschaltender Transceiver — kein DE/RE).
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

/** UART2 auf RX/TX-Pins; USB-Serial muss bereits initialisiert sein. */
void begin();

/** Wartung der Bridge-Modus-Zeitfenster; die Datenpfade laufen in eigenen Tasks. */
void poll();

/** Umschalten zwischen lokalem Master und PC-Proxy-Master. */
void set_mode(BridgeMode mode);
BridgeMode get_mode();

/**
 * Controller Remote USB: kein UART-TX zum RS485; Frames nur USB↔Parser.
 * Initialwert aus pwm_config; SETCONREMOTE aktualisiert zur Laufzeit.
 */
void set_remote_usb(bool on);
bool remote_usb_active();
/** True wenn Remote-USB aktiv und PC-Link kürzlich Bytes/Frames geliefert hat. */
bool remote_usb_pc_seen();
/** Remote-USB: PC-Session noch „online“ (vor Link-Down), auch bei kurzer Sendepause. */
bool remote_usb_link_session();
/** Remote-USB aktiv und kein PC-Link — Display wartet, kein RS485/Boot. */
bool remote_usb_waiting_for_pc();

/**
 * Gemeinsamer Mutex für direkten Zugriff auf Serial2 (RS485).
 * Normaler Versand soll über hw_send()/hw_send_priority() laufen.
 */
void uart_lock();
void uart_unlock();

/**
 * RS485 senden. Intern wird in die zentrale RS485-TX-Queue gelegt; ein einzelner
 * TX-Task übernimmt Bus-Idle und UART-Write (kein DE/RE — Transceiver schaltet selbst).
 */
void hw_send(const uint8_t *data, size_t len);

/**
 * Priorisierter RS485-Versand (für SETPOSCC): kürzere Idle-Wartezeit als hw_send(),
 * damit Vorschau-Sollwerte bei laufendem Fremd-GETPOSDG nicht verzögert werden.
 */
void hw_send_priority(const uint8_t *data, size_t len);

} // namespace serial_bridge
