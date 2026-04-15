// =============================================================================
// GPSTimeSource.cpp
// =============================================================================
#include "GPSTimeSource.h"

// ---------------------------------------------------------------------------
// Static member definitions
// ---------------------------------------------------------------------------
GPSTimeSource* GPSTimeSource::_instance = nullptr;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
GPSTimeSource::GPSTimeSource(HardwareSerial& serial, int pps_pin)
    : _serial(serial)
    , _pps_pin(pps_pin)
    , _pps_micros(0)
    , _pps_fired(false)
    , _pps_count(0)
    , _utc_sec_at_pps(0)
    , _locked(false)
    , _sentence_count(0)
    , _buf_idx(0)
    , _in_sentence(false)
{
    _instance = this;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void GPSTimeSource::begin(uint32_t baud)
{
    _serial.begin(baud);

    if (_pps_pin >= 0) {
        pinMode(_pps_pin, INPUT);
        // Rising edge = start of new UTC second
        attachInterrupt(digitalPinToInterrupt(_pps_pin), _ppsISR, RISING);
    }
}

void GPSTimeSource::update()
{
    while (_serial.available()) {
        _processChar(static_cast<char>(_serial.read()));
    }
}

// ---------------------------------------------------------------------------
// Public: now()
// ---------------------------------------------------------------------------
GPSTimeSource::Timestamp GPSTimeSource::now() const
{
    Timestamp ts{0, 0, false};

    if (!_locked) return ts;

    // PPS mode: compute sub-second offset from last PPS edge
    if (_pps_pin >= 0) {
        if (!_pps_fired) return ts;

        // Snapshot volatile atomically (8-bit MCUs need cli/sei around 32-bit
        // reads; on 32-bit MCUs the assignment is naturally atomic).
        noInterrupts();
        uint32_t pps_snap    = _pps_micros;
        uint32_t pps_count   = _pps_count;
        interrupts();
        (void)pps_count; // used only as liveness check

        uint32_t elapsed_us = micros() - pps_snap;  // wraps correctly (uint32)

        // Stale guard: if no PPS has fired for >2 s something is wrong
        if (elapsed_us > STALE_PPS_US) return ts;

        ts.unix_sec  = _utc_sec_at_pps + elapsed_us / 1000000UL;
        ts.unix_usec = elapsed_us % 1000000UL;
    }
    else {
        // NMEA-only: whole-second resolution
        ts.unix_sec  = _utc_sec_at_pps;
        ts.unix_usec = 0;
    }

    ts.valid = (ts.unix_sec >= MIN_UNIX_SEC);
    return ts;
}

// ---------------------------------------------------------------------------
// Private: NMEA character-level state machine
// ---------------------------------------------------------------------------
void GPSTimeSource::_processChar(char c)
{
    if (c == '$') {
        _in_sentence = true;
        _buf_idx     = 0;
        _buf[0]      = '\0';
        return;
    }

    if (!_in_sentence) return;

    // Discard carriage return; newline = end of sentence
    if (c == '\r') return;

    if (c == '\n') {
        _in_sentence     = false;
        _buf[_buf_idx]   = '\0';
        _dispatchSentence();
        return;
    }

    if (_buf_idx < NMEA_BUF_LEN - 1) {
        _buf[_buf_idx++] = c;
    }
    // If buffer overflows just discard the sentence — it's malformed
    else {
        _in_sentence = false;
    }
}

void GPSTimeSource::_dispatchSentence()
{
    if (_buf_idx < 6) return;           // too short to be valid
    if (!_validChecksum(_buf)) return;  // corrupt sentence

    bool parsed = false;

    if      (strncmp(_buf, "GPRMC", 5) == 0 ||
             strncmp(_buf, "GNRMC", 5) == 0) parsed = _parseGPRMC(_buf);
    else if (strncmp(_buf, "GPZDA", 5) == 0 ||
             strncmp(_buf, "GNZDA", 5) == 0) parsed = _parseGPZDA(_buf);

    if (parsed) ++_sentence_count;
}

// ---------------------------------------------------------------------------
// Private: NMEA sentence parsers
// ---------------------------------------------------------------------------

// $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,x.x,a*HH
//          f0        f1 f2      f3 f4       f5 f6   f7   f8   f9
bool GPSTimeSource::_parseGPRMC(const char* s)
{
    // f0 = HHMMSS.ss
    const char* p = _skipFields(s, 1);
    if (!p) return false;

    int hh = _d2(p);
    int mm = _d2(p + 2);
    int ss = _d2(p + 4);
    if (hh < 0 || mm < 0 || ss < 0) return false;

    // f1 = status: 'A' = active (valid fix), 'V' = void
    p = _skipFields(s, 2);
    if (!p || *p != 'A') return false;

    // f8 = DDMMYY
    p = _skipFields(s, 9);
    if (!p) return false;

    int day   = _d2(p);
    int month = _d2(p + 2);
    int year  = _d2(p + 4);
    if (day < 0 || month < 0 || year < 0) return false;
    if (year < 70) year += 2000; else year += 1900;  // handle Y2K

    _commitTime(static_cast<uint16_t>(year),
                static_cast<uint8_t>(month),
                static_cast<uint8_t>(day),
                static_cast<uint8_t>(hh),
                static_cast<uint8_t>(mm),
                static_cast<uint8_t>(ss));
    return true;
}

// $GPZDA,HHMMSS.ss,DD,MM,YYYY,ZH,ZM*HH  (UTC time and date, no position)
bool GPSTimeSource::_parseGPZDA(const char* s)
{
    // f0 = HHMMSS.ss
    const char* p = _skipFields(s, 1);
    if (!p) return false;

    int hh = _d2(p);
    int mm = _d2(p + 2);
    int ss = _d2(p + 4);
    if (hh < 0 || mm < 0 || ss < 0) return false;

    // f1 = DD
    p = _skipFields(s, 2);
    if (!p) return false;
    int day = _d2(p);

    // f2 = MM
    p = _skipFields(s, 3);
    if (!p) return false;
    int month = _d2(p);

    // f3 = YYYY
    p = _skipFields(s, 4);
    if (!p) return false;
    int year = 0;
    for (uint8_t i = 0; i < 4 && isdigit(p[i]); ++i) year = year * 10 + (p[i] - '0');

    if (day < 0 || month < 0 || year < 1970) return false;

    _commitTime(static_cast<uint16_t>(year),
                static_cast<uint8_t>(month),
                static_cast<uint8_t>(day),
                static_cast<uint8_t>(hh),
                static_cast<uint8_t>(mm),
                static_cast<uint8_t>(ss));
    return true;
}

// ---------------------------------------------------------------------------
// Private: commit a parsed UTC time as the epoch for the last PPS edge
// ---------------------------------------------------------------------------
void GPSTimeSource::_commitTime(uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour,  uint8_t min,   uint8_t sec)
{
    uint32_t unix_sec = _utcToUnix(year, month, day, hour, min, sec);
    if (unix_sec < MIN_UNIX_SEC) return;  // reject implausible times

    if (_pps_pin >= 0) {
        // PPS mode: the NMEA sentence arrives *after* the PPS edge it describes.
        // Convention (u-blox NEO-6M / NEO-M8N / SiRF IV):
        //   PPS fires at second N  →  NMEA arrives ~50–200 ms later carrying time N.
        // Therefore this unix_sec is exactly the epoch of the captured _pps_micros.
        _utc_sec_at_pps = unix_sec;
        _locked = _pps_fired;  // only lock once we have both PPS and NMEA
    }
    else {
        // NMEA-only mode: the sentence itself is the best timestamp we have.
        // Store it as the current whole second.
        _utc_sec_at_pps = unix_sec;
        _locked = true;
    }
}

// ---------------------------------------------------------------------------
// Private: ISR
// ---------------------------------------------------------------------------
void IRAM_ATTR GPSTimeSource::_ppsISR()
{
    if (!_instance) return;
    _instance->_pps_micros = micros();
    _instance->_pps_fired  = true;
    ++_instance->_pps_count;
}

// ---------------------------------------------------------------------------
// Private: NMEA utilities
// ---------------------------------------------------------------------------
bool GPSTimeSource::_validChecksum(const char* s) const
{
    // Sentence stored without leading '$'; find '*' delimiter
    const char* star = strchr(s, '*');
    if (!star || star[1] == '\0' || star[2] == '\0') return false;

    uint8_t expected = 0;
    for (const char* p = s; p != star; ++p) expected ^= static_cast<uint8_t>(*p);

    // Parse two hex digits after '*'
    auto hexVal = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };
    int hi = hexVal(star[1]);
    int lo = hexVal(star[2]);
    if (hi < 0 || lo < 0) return false;

    return expected == static_cast<uint8_t>((hi << 4) | lo);
}

int GPSTimeSource::_d2(const char* p)
{
    if (!p || !isdigit(p[0]) || !isdigit(p[1])) return -1;
    return (p[0] - '0') * 10 + (p[1] - '0');
}

const char* GPSTimeSource::_skipFields(const char* p, uint8_t n)
{
    for (uint8_t i = 0; i < n; ++i) {
        p = strchr(p, ',');
        if (!p) return nullptr;
        ++p;  // move past the comma
    }
    return p;
}

// ---------------------------------------------------------------------------
// Private: UTC to Unix timestamp (Julian Day Number algorithm)
//
// Reference: https://en.wikipedia.org/wiki/Julian_day#Converting_Gregorian_calendar_date_to_Julian_Day_Number
//
// Pure integer arithmetic — no floating point, no large lookup tables.
// Correctly handles all years, leap years, and century boundaries.
// ---------------------------------------------------------------------------
uint32_t GPSTimeSource::_utcToUnix(uint16_t year, uint8_t month, uint8_t day,
                                    uint8_t hour,  uint8_t min,   uint8_t sec)
{
    // Cap leap second to 59 (Unix time does not represent leap seconds)
    if (sec > 59) sec = 59;

    // Compute Julian Day Number for this date
    int32_t a   = (14 - month) / 12;
    int32_t y   = static_cast<int32_t>(year) + 4800 - a;
    int32_t m   = month + 12 * a - 3;
    int32_t jdn = day
                + (153 * m + 2) / 5
                + 365 * y
                + y / 4
                - y / 100
                + y / 400
                - 32045;

    // Unix epoch = JDN 2440588 (1970-01-01)
    const int32_t UNIX_EPOCH_JDN = 2440588L;
    uint32_t days_since_epoch = static_cast<uint32_t>(jdn - UNIX_EPOCH_JDN);

    return days_since_epoch * 86400UL
         + static_cast<uint32_t>(hour) * 3600UL
         + static_cast<uint32_t>(min)  * 60UL
         + sec;
}