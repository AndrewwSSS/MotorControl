#pragma once

// A stand-in for the Arduino core, just big enough for the library's headers to
// compile and run on the host under `pio test -e native`. Only the native env
// puts this directory on the include path; boards build against their real core.
//
// Nothing here touches hardware, so a test controls everything a sketch would
// get from the outside world:
//
//   time    stands still until the test moves it (fake::advanceUs), or moves by
//           a fixed amount on every micros() call (fake::setAutoAdvance) so that
//           busy-wait loops such as homing's drive() make progress
//   writes  every digitalWrite is recorded with the time it happened
//   reads   digitalRead returns what the test scripted, per pin or through a
//           handler that can follow the axis (a switch closing at some step)
//   Serial  collects whatever is printed, log output included
//
// Known gap: unsigned long is 64 bits here and 32 bits on the boards, so the
// micros() rollover cannot be reproduced. Tests keep time well clear of it.

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

#define LOW  0
#define HIGH 1

#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2

typedef uint8_t byte;
typedef bool    boolean;

// On AVR, F() keeps a string in flash and tags it with this type so Print knows
// to read it from there. On the host a plain pointer does, but the type is kept
// so overloads resolve exactly as they do on a board.
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))


class Print {
public:
    virtual ~Print() = default;

    virtual size_t write(uint8_t c) = 0;

    size_t write(const char* s) {
        size_t n = 0;
        while (*s) n += write((uint8_t)*s++);
        return n;
    }

    size_t print(const char* s)                { return write(s); }
    size_t print(const __FlashStringHelper* s) { return write(reinterpret_cast<const char*>(s)); }
    size_t print(const std::string& s)         { return write(s.c_str()); }
    size_t print(char c)                       { return write((uint8_t)c); }

    // As on Arduino, an unsigned char (uint8_t) prints as a number, not a letter.
    size_t print(unsigned char n) { return print((unsigned long)n); }
    size_t print(int n)           { return print((long)n); }
    size_t print(unsigned int n)  { return print((unsigned long)n); }
    size_t print(long n)          { return printf("%ld", n); }
    size_t print(unsigned long n) { return printf("%lu", n); }
    size_t print(double d)        { return printf("%.2f", d); }

    size_t println() { return write("\r\n"); }

    template <typename T>
    size_t println(const T& value) {
        size_t n = print(value);
        return n + println();
    }

private:
    template <typename T>
    size_t printf(const char* format, T value) {
        char buf[32];
        snprintf(buf, sizeof buf, format, value);
        return write(buf);
    }
};


class FakeSerial : public Print {
public:
    void begin(unsigned long) {}
    explicit operator bool() const { return true; }
    int available() const { return 0; }
    int read() { return -1; }

    size_t write(uint8_t c) override {
        output += (char)c;
        return 1;
    }

    std::string output;
};

inline FakeSerial Serial;


namespace fake {

struct PinWrite {
    uint8_t       pin;
    uint8_t       level;
    unsigned long atUs;
};

// Called for every digitalRead when set; overrides the per-pin levels.
typedef int (*ReadHandler)(uint8_t pin);

static constexpr int PIN_COUNT = 256;

struct State {
    unsigned long         nowUs;
    unsigned long         autoAdvanceUs;
    uint8_t               mode[PIN_COUNT];
    uint8_t               level[PIN_COUNT];   // last written
    uint8_t               input[PIN_COUNT];   // what digitalRead returns
    std::vector<PinWrite> writes;
    ReadHandler           readHandler;
};

inline State state;

// Back to power-on: time 0, no writes, every input reading HIGH (a pulled-up
// pin with nothing pulling it down), Serial empty. Call from setUp().
inline void reset() {
    state.nowUs = 0;
    state.autoAdvanceUs = 0;
    memset(state.mode, INPUT, sizeof state.mode);
    memset(state.level, LOW, sizeof state.level);
    memset(state.input, HIGH, sizeof state.input);
    state.writes.clear();
    state.readHandler = nullptr;
    Serial.output.clear();
}

inline void advanceUs(unsigned long us) { state.nowUs += us; }
inline void advanceMs(unsigned long ms) { state.nowUs += ms * 1000UL; }

// Every micros() call moves time on by us after reading it. 0 turns it off.
inline void setAutoAdvance(unsigned long us) { state.autoAdvanceUs = us; }

inline void setInput(uint8_t pin, uint8_t level) { state.input[pin] = level; }
inline void onRead(ReadHandler handler) { state.readHandler = handler; }

inline uint8_t modeOf(uint8_t pin)  { return state.mode[pin]; }
inline uint8_t levelOf(uint8_t pin) { return state.level[pin]; }

inline const std::vector<PinWrite>& writes() { return state.writes; }

// LOW -> HIGH edges written to pin, i.e. step pulses on a STEP pin.
inline size_t risingEdges(uint8_t pin) {
    size_t n = 0;
    uint8_t last = LOW;
    for (const PinWrite& w : state.writes) {
        if (w.pin != pin) continue;
        if (last == LOW && w.level == HIGH) n++;
        last = w.level;
    }
    return n;
}

// When each of those edges happened, for checking pulse spacing.
inline std::vector<unsigned long> risingEdgeTimes(uint8_t pin) {
    std::vector<unsigned long> times;
    uint8_t last = LOW;
    for (const PinWrite& w : state.writes) {
        if (w.pin != pin) continue;
        if (last == LOW && w.level == HIGH) times.push_back(w.atUs);
        last = w.level;
    }
    return times;
}

inline void clearWrites() { state.writes.clear(); }

inline const std::string& serialOutput() { return Serial.output; }
inline void clearSerial() { Serial.output.clear(); }

}


inline unsigned long micros() {
    unsigned long now = fake::state.nowUs;
    fake::state.nowUs += fake::state.autoAdvanceUs;
    return now;
}

inline unsigned long millis() {
    return fake::state.nowUs / 1000UL;
}

inline void delayMicroseconds(unsigned int us) { fake::advanceUs(us); }
inline void delay(unsigned long ms) { fake::advanceMs(ms); }

inline void pinMode(uint8_t pin, uint8_t mode) {
    fake::state.mode[pin] = mode;
}

inline void digitalWrite(uint8_t pin, uint8_t level) {
    fake::state.level[pin] = level ? HIGH : LOW;
    fake::state.writes.push_back({pin, fake::state.level[pin], fake::state.nowUs});
}

inline int digitalRead(uint8_t pin) {
    if (fake::state.readHandler) return fake::state.readHandler(pin);
    return fake::state.input[pin];
}
