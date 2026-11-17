// Minimal Arduino mock for host testing of MPR
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "Wire.h"
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define RISING 3
#define NOT_AN_INTERRUPT -1
#define digitalPinToInterrupt(p) ((p) == 99 ? NOT_AN_INTERRUPT : (p))   // pin 99: no interrupt support
extern uint32_t g_now_us;
inline uint32_t micros() { return g_now_us; }
// millis() keeps counting across a wrap of the 32 bit micros() (it wraps only after 49 days)
inline uint32_t millis() {
  static uint32_t last = 0; static uint64_t epoch = 0;
  if (g_now_us < last) { epoch += (1ULL << 32); }
  last = g_now_us;
  return (uint32_t)((epoch + g_now_us) / 1000ULL);
}
void delay(uint32_t ms);
void pinMode(int p, int m);
void digitalWrite(int p, int v);
int  digitalRead(int p);
void attachInterrupt(int pin, void (*fn)(), int mode);
void detachInterrupt(int pin);
inline void noInterrupts() {}
inline void interrupts() {}
