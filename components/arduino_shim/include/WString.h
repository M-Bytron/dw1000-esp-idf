/*
 * Minimal Arduino `String` shim for ESP-IDF.
 *
 * Only implements the members used by the arduino-dw1000 library
 * (DW1000::setData(const String&) / DW1000::getData(String&)). The ranging
 * application itself does not use String.
 */
#ifndef ARDUINO_SHIM_WSTRING_H
#define ARDUINO_SHIM_WSTRING_H

#include <string.h>
#include <stdint.h>

class String {
public:
    String() : _len(0) { _buf[0] = '\0'; }
    String(const char *s) { init(s); }

    unsigned int length() const { return _len; }

    void getBytes(unsigned char *buf, unsigned int bufsize) const {
        if (bufsize == 0) {
            return;
        }
        unsigned int n = (_len < bufsize - 1) ? _len : (bufsize - 1);
        memcpy(buf, _buf, n);
        buf[n] = '\0';
    }

    void remove(unsigned int index) {
        if (index < _len) {
            _len = index;
            _buf[_len] = '\0';
        }
    }

    String &operator=(const char *s) {
        init(s);
        return *this;
    }

    String &operator+=(char c) {
        if (_len + 1 < CAPACITY) {
            _buf[_len++] = c;
            _buf[_len] = '\0';
        }
        return *this;
    }

    const char *c_str() const { return _buf; }

private:
    static const unsigned int CAPACITY = 128;
    char _buf[CAPACITY];
    unsigned int _len;

    void init(const char *s) {
        if (s == NULL) {
            s = "";
        }
        _len = strlen(s);
        if (_len >= CAPACITY) {
            _len = CAPACITY - 1;
        }
        memcpy(_buf, s, _len);
        _buf[_len] = '\0';
    }
};

#endif /* ARDUINO_SHIM_WSTRING_H */
