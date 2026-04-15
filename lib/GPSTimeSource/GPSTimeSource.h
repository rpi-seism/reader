#pragma once
// =============================================================================
// GPSTimeSource.h
// GPS + PPS disciplined time source for rpi-seism firmware (PlatformIO).
//
// Timing model
// ------------
//   Without PPS (nmea-only mode, pps_pin = -1):
//     UTC from NMEA sentence only.  Typical jitter ±2–10 ms.
//
//   With PPS (pps_pin >= 0):
//     ISR captures micros() on the rising PPS edge.  The NMEA sentence that
//     follows (~50–200 ms later) labels that edge with an integer UTC second.
//     Result: unix_time = utc_sec_at_pps + (micros() - pps_micros) / 1e6
//     Typical jitter ±50–200 µs (GPIO interrupt latency on Linux-class MCUs;
//     on bare-metal RP2040/STM32 typically ±1–5 µs).
//
// Usage
// -----
//   GPSTimeSource gps(Serial1, PPS_PIN);
//
//   setup() {
//     gps.begin(9600);
//   }
//
//   loop() {
//     gps.update();          // call as frequently as possible
//     if (gps.isLocked()) {
//       auto ts = gps.now();
//       // ts.unix_sec  → uint32_t seconds since 1970-01-01 UTC
//       // ts.unix_usec → uint32_t microseconds within that second
//       // ts.valid     → bool (false = no fix or stale PPS)
//     }
//   }
//
// Supported platforms (Arduino framework via PlatformIO)
//   AVR (Uno/Nano/Mega), RP2040, STM32, ESP32, Teensy
// =============================================================================

#include <Arduino.h>

#ifndef IRAM_ATTR
  #define IRAM_ATTR  // Define as empty if not on ESP32
#endif

class GPSTimeSource {
public:
    // -------------------------------------------------------------------------
    // Public types
    // -------------------------------------------------------------------------
    struct Timestamp {
        uint32_t unix_sec;   ///< Seconds since Unix epoch (UTC, NOT GPS time)
        uint32_t unix_usec;  ///< Microseconds within that second [0, 999999]
        bool     valid;      ///< False when GPS has no fix or PPS is stale
    };

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * @param serial   HardwareSerial connected to GPS NMEA output (TX of GPS).
     * @param pps_pin  BCM/board pin wired to GPS PPS output, or -1 for NMEA-only.
     *
     * @note  pps_pin must support external interrupts on your board.
     *        On AVR Uno/Nano only pins 2 and 3 support interrupts.
     *        On RP2040, STM32, ESP32, Teensy all GPIO pins support interrupts.
     */
    GPSTimeSource(HardwareSerial& serial, int pps_pin = -1);

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /**
     * Initialise GPS serial and attach PPS interrupt (if pps_pin was set).
     * Call once from setup().
     */
    void begin(uint32_t baud = 9600);

    /**
     * Drive the NMEA parser.  Must be called as often as possible from loop()
     * or a high-priority task.  Reads all bytes currently in the serial buffer
     * and processes any complete sentences.  Non-blocking.
     */
    void update();

    // -------------------------------------------------------------------------
    // State queries
    // -------------------------------------------------------------------------

    /**
     * Returns true once at least one valid NMEA fix sentence has been parsed
     * AND (in PPS mode) at least one PPS edge has been received.
     */
    bool isLocked() const { return _locked; }

    /**
     * Returns the current GPS-disciplined timestamp.
     * In PPS mode:   unix_usec is computed from elapsed micros() since last edge.
     * In NMEA-only:  unix_usec is always 0 (whole-second resolution).
     * Falls back to valid=false if PPS is stale (no edge for > STALE_PPS_US µs).
     */
    Timestamp now() const;

    /**
     * Diagnostic: count of PPS pulses received since begin().
     */
    uint32_t ppsCount() const { return _pps_count; }

    /**
     * Diagnostic: count of valid NMEA sentences parsed since begin().
     */
    uint32_t sentenceCount() const { return _sentence_count; }

private:
    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------
    static constexpr uint32_t NMEA_BUF_LEN  = 128;

    // If no PPS edge arrives within this window, now() marks itself invalid.
    // 2 seconds = one missed pulse plus margin.
    static constexpr uint32_t STALE_PPS_US  = 2000000UL;

    // Minimum plausible Unix timestamp (2020-01-01 00:00:00 UTC).
    // Rejects partially-initialised zero values from the GPS module on boot.
    static constexpr uint32_t MIN_UNIX_SEC  = 1577836800UL;

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    HardwareSerial& _serial;
    int             _pps_pin;

    // -------------------------------------------------------------------------
    // PPS state  (volatile: written in ISR, read in main context)
    // -------------------------------------------------------------------------
    volatile uint32_t _pps_micros;   ///< micros() captured on last PPS rising edge
    volatile bool     _pps_fired;    ///< True after first PPS edge
    volatile uint32_t _pps_count;

    // -------------------------------------------------------------------------
    // GPS epoch state  (set from NMEA, always integer seconds)
    // -------------------------------------------------------------------------
    uint32_t _utc_sec_at_pps;  ///< Unix second that corresponds to _pps_micros
    bool     _locked;
    uint32_t _sentence_count;

    // -------------------------------------------------------------------------
    // NMEA parsing
    // -------------------------------------------------------------------------
    char    _buf[NMEA_BUF_LEN];
    uint8_t _buf_idx;
    bool    _in_sentence;

    void _processChar(char c);
    void _dispatchSentence();

    bool _parseGPRMC(const char* s);  ///< $GPRMC (speed/course/date — has date field)
    bool _parseGPZDA(const char* s);  ///< $GPZDA (time + date, ideal for time-only use)

    /** Validate NMEA checksum (*HH at end). */
    bool _validChecksum(const char* s) const;

    /** Parse two ASCII decimal digits; returns -1 on non-digit. */
    static int _d2(const char* p);

    /** Skip n comma-separated fields and return pointer to start of field n+1. */
    static const char* _skipFields(const char* p, uint8_t n);

    /**
     * Convert UTC date/time components to a Unix timestamp.
     * Uses the Julian Day Number algorithm — handles all years, leap years
     * and century boundaries correctly with only integer arithmetic.
     *
     * @param year   Full year, e.g. 2025
     * @param month  1–12
     * @param day    1–31
     * @param hour   0–23 (UTC)
     * @param min    0–59 (UTC)
     * @param sec    0–60 (UTC, 60 = leap second, treated as 59)
     */
    static uint32_t _utcToUnix(uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t min, uint8_t sec);

    void _commitTime(uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour,  uint8_t min,   uint8_t sec);

    // -------------------------------------------------------------------------
    // ISR trampoline
    // attachInterrupt() requires a plain C function pointer; we store one
    // static instance pointer to route the ISR back to the right object.
    // Only one GPSTimeSource may be active at a time (typical deployment).
    // -------------------------------------------------------------------------
    static GPSTimeSource* _instance;
    static void IRAM_ATTR _ppsISR();   // IRAM_ATTR is a no-op on non-ESP32
};