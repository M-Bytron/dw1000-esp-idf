/*
 * Arduino-compat shim for ESP-IDF
 *
 * Implements the tiny subset of the Arduino API that the arduino-dw1000
 * library needs, backed by native ESP-IDF drivers (GPIO, SPI master,
 * esp_timer, FreeRTOS). This is a minimal shim - not a full Arduino core.
 */
#ifndef ARDUINO_SHIM_ARDUINO_H
#define ARDUINO_SHIM_ARDUINO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "WString.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- basic Arduino type aliases ---- */
typedef uint8_t byte;
typedef bool    boolean;

/* ---- digital levels ---- */
#define HIGH 0x1
#define LOW  0x0

/* ---- pin modes ---- */
#define INPUT             0x00
#define OUTPUT            0x01
#define INPUT_PULLUP      0x02
#define INPUT_PULLDOWN    0x03
#define OUTPUT_OPEN_DRAIN 0x04

/* ---- interrupt trigger modes ---- */
#define CHANGE  1
#define FALLING 2
#define RISING  3

/* ---- bit manipulation helpers ---- */
#define lowByte(w)  ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit)                (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)                 ((value) |= (1UL << (bit)))
#define bitClear(value, bit)               ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue)     ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b)                             (1UL << (b))

/* F() macro: on ESP32 strings live in RAM, so this is a no-op */
#define F(string_literal) (string_literal)

/* ---- GPIO ---- */
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int  digitalRead(uint8_t pin);

/* ---- time ---- */
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
uint32_t millis(void);
uint32_t micros(void);

/* ---- global interrupt control (no-op, see Arduino.cpp) ---- */
void interrupts(void);
void noInterrupts(void);

/* ---- external interrupt (deferred to a FreeRTOS task) ---- */
#define digitalPinToInterrupt(p) (p)
void attachInterrupt(uint8_t pin, void (*handler)(void), int mode);
void detachInterrupt(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_SHIM_ARDUINO_H */
