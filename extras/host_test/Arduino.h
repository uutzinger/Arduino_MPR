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
#define digitalPinToInterrupt(p) (p)
extern uint32_t g_now_us;
inline uint32_t micros() { return g_now_us; }
inline uint32_t millis() { return g_now_us / 1000UL; }
void delay(uint32_t ms);
void pinMode(int p, int m);
void digitalWrite(int p, int v);
int  digitalRead(int p);
void attachInterrupt(int pin, void (*fn)(), int mode);
void detachInterrupt(int pin);
inline void noInterrupts() {}
inline void interrupts() {}
