// Host test for MPR: simulated sensor, mock clock, mock pins.
#include "Arduino.h"
#include "MPR.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <initializer_list>
TwoWire Wire;

uint32_t g_now_us = 1000;

// ---------------- simulated sensor and pins
struct Sim {
  bool     present = true;        // ACKs
  bool     eoc_works = true;      // EOC pin pulses at end of conversion
  bool     converting = false;
  uint32_t done_us = 0, conv_us = 4500;
  uint8_t  flags = 0x40;          // status bits besides busy
  uint32_t counts = 419430;
  bool     busy_stuck = false;    // never completes
  int      cmds = 0, ignored = 0;  // ignored: commands sent while busy
  int      eoc_pin = 10, rst_pin = 11;
  int      eoc_level = 0;
  bool     rst_low = false;
  int      resets = 0;
  bool     ignore_rst = false;
  void (*handler)() = nullptr;
  uint8_t status() const { return flags | ((converting || busy_stuck) ? 0x20 : 0); }
  void tick() {
    if (converting && !busy_stuck && (int32_t)(g_now_us - done_us) >= 0) {
      converting = false;
      if (eoc_works) { eoc_level = 1; if (handler) handler(); }
    }
  }
} sim;

void delay(uint32_t ms) { for (uint32_t i = 0; i < ms * 10; i++) { g_now_us += 100; sim.tick(); } }
void pinMode(int, int) {}
void digitalWrite(int p, int v) {
  if (p == sim.rst_pin) {
    if (v == 0) { sim.rst_low = true; }
    else if (sim.rst_low) { sim.rst_low = false; if (!sim.ignore_rst) { sim.resets++; sim.flags = 0x40; sim.converting = false; sim.busy_stuck = false; } }
  }
}
int  digitalRead(int p) { return p == sim.eoc_pin ? sim.eoc_level : 0; }
void attachInterrupt(int, void (*fn)(), int) { sim.handler = fn; }
void detachInterrupt(int) { sim.handler = nullptr; }

void TwoWire::beginTransmission(uint8_t a) { _addr = a; _ntx = 0; }
size_t TwoWire::write(uint8_t b) { if (_ntx < 8) _tx[_ntx++] = b; return 1; }
uint8_t TwoWire::endTransmission() {
  if (!sim.present) return 2;
  if (_ntx == 3 && _tx[0] == 0xAA && (sim.converting || sim.busy_stuck)) { sim.ignored++; }   // sensor ignores commands while busy
  else if (_ntx == 3 && _tx[0] == 0xAA) { sim.cmds++; sim.eoc_level = 0; sim.converting = true; sim.done_us = g_now_us + sim.conv_us; }
  g_now_us += 300;                     // 3 bytes at 100 kHz
  return 0;
}
uint8_t TwoWire::requestFrom(uint8_t, uint8_t n) {
  _irx = 0; _nrx = 0;
  if (!sim.present) { g_now_us += 100; return 0; }
  uint8_t st = sim.status();
  _rx[0] = st;
  if (n == 4) { _rx[1] = sim.counts >> 16; _rx[2] = sim.counts >> 8; _rx[3] = sim.counts; }
  _nrx = n;
  g_now_us += 100 * (n + 1);
  return n;
}
int TwoWire::available() { return _nrx - _irx; }
int TwoWire::read() { return _irx < _nrx ? _rx[_irx++] : -1; }

// ---------------- test helpers
static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { printf("  FAIL line %d: %s\n", __LINE__, #c); g_fail++; } } while (0)

TwoWire wire;
static void run_ms(MPR &m, uint32_t ms) {
  uint32_t end = g_now_us + ms * 1000UL;
  while ((int32_t)(g_now_us - end) < 0) { g_now_us += 100; sim.tick(); m.update(); }
}
static void reset_sim() { sim = Sim(); g_now_us = 1000; }

typedef MPRBuffer<32> M;

static void test_text() {
  printf("text helpers\n");
  char b[64];
  MPR::statusString(0x41, b, sizeof b);  CHECK(!strcmp(b, "0x41: powered, math saturation"));
  MPR::statusString(0xFF, b, sizeof b);  CHECK(!strcmp(b, "0xFF: bus reads all ones"));
  MPR::statusString(0x00, b, sizeof b);  CHECK(!strcmp(b, "0x00: not powered"));
  MPR::statusString(0x44, b, sizeof b);  CHECK(!strcmp(b, "0x44: powered, memory error"));
  MPR::statusString(0x60, b, sizeof b);  CHECK(!strcmp(b, "0x60: powered, busy"));
  MPR::errorString(0, b, sizeof b);      CHECK(!strcmp(b, "ok"));
  MPR::errorString(MPR::E_POWER | MPR::E_MEM, b, sizeof b); CHECK(!strcmp(b, "not powered, memory error"));
  char s[8]; size_t n = MPR::errorString(0xFFFF, s, sizeof s); CHECK(n == 7 && s[7] == 0);
  CHECK(MPR::errorString(1, nullptr, 0) == 0);
  CHECK(!strcmp(MPR::i2cResultString(2), "NACK on address"));
  CHECK(!strcmp(MPR::i2cResultString(99), "unknown error"));
  for (uint8_t i = 0; i < MPR::ERROR_BITS; i++) { CHECK(strlen(MPR::errorName(i)) > 0 && strlen(MPR::errorHint(i)) > 0); }
}

static void test_conversion() {
  printf("conversion\n");
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG);
  CHECK(m.minCounts() == 419430 && m.maxCounts() == 3774873);
  CHECK(fabsf(m.toPressure(3774873U) - 300.0f) < 0.01f);
  CHECK(fabsf(m.toPressure(419430U)) < 0.01f);
  m.setZero(420000.0f);
  CHECK(fabsf(m.toPressure(420000U)) < 0.001f);
  M p(MprRange::R_0015PG, MprTransfer::A, MprUnit::KPA);
  CHECK(fabsf(p.toPressure(15099494U) - 103.4214f) < 0.05f);     // 15 psi in kPa
  M a(MprRange::R_0100KA, MprTransfer::C, MprUnit::MBAR);
  CHECK(fabsf(a.toPressure(13421772U) - 1000.0f) < 0.5f);        // 100 kPa = 1000 mbar
}

static void test_poll() {
  printf("status polling, fixed interval\n");
  reset_sim();
  M m(MprRange::R_0300YG);
  CHECK(m.begin(wire));
  CHECK(m.start(10000));
  run_ms(m, 1000);
  MprStats st = m.stats();
  printf("  good %lu bad %lu conv %lu..%lu mean %.0f cmds %d\n", (unsigned long)st.good, (unsigned long)st.bad,
         (unsigned long)st.conv_min_us, (unsigned long)st.conv_max_us, st.conv_mean_us, sim.cmds);
  CHECK(st.good >= 95 && st.good <= 101);
  CHECK(st.bad == 0 && m.lastError() == 0);
  CHECK(m.mode() == MprMode::STATUS_POLL);
  MprSample s; uint32_t prev = 0; int n = 0; uint32_t maxdev = 0;
  while (m.pop(s)) { if (n++ && true) { uint32_t d = s.t_us - prev; uint32_t dev = d > 10000 ? d - 10000 : 10000 - d; if (dev > maxdev) maxdev = dev; } prev = s.t_us; CHECK(s.counts == 419430); }
  printf("  interval deviation max %lu us (polling resolution 500 us)\n", (unsigned long)maxdev);
  CHECK(maxdev <= 1000);
}

static void test_eoc() {
  printf("EOC interrupt, back to back\n");
  reset_sim();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  CHECK(m.begin(wire));
  CHECK(sim.resets == 1);
  CHECK(m.start(0));
  CHECK(m.mode() == MprMode::EOC_INTERRUPT);
  run_ms(m, 1000);
  MprStats st = m.stats();
  printf("  good %lu bad %lu conv mean %.0f us, cmds %d, overruns %lu\n", (unsigned long)st.good, (unsigned long)st.bad, st.conv_mean_us, sim.cmds, (unsigned long)m.overruns());
  CHECK(st.good > 150 && st.bad == 0);
  CHECK(st.eoc_timeouts == 0 && st.eoc_early == 0);
  CHECK(st.conv_mean_us >= 4000 && st.conv_mean_us <= 4700);
  CHECK(m.avail() == 32);                 // buffer full, oldest dropped
  CHECK(m.overruns() > 0);
  m.stop();
  CHECK(m.mode() == MprMode::STOPPED);
  CHECK(sim.handler == nullptr);
}

static void test_eoc_missing() {
  printf("EOC wire broken: fall back to polling\n");
  reset_sim();
  sim.eoc_works = false;
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  m.begin(wire); m.start(10000);
  run_ms(m, 500);
  MprStats st = m.stats();
  printf("  good %lu bad %lu eoc_timeouts %lu eoc_low_seen %lu mode %d\n", (unsigned long)st.good, (unsigned long)st.bad, (unsigned long)st.eoc_timeouts, (unsigned long)st.eoc_low_seen, (int)m.mode());
  CHECK(st.good >= 20 && st.bad == 0);
  CHECK(st.eoc_timeouts == 1);
  CHECK(m.mode() == MprMode::STATUS_POLL);
  CHECK(st.eoc_low_seen > 0);
  CHECK(m.lastError() & MPR::E_EOC);
  MprSample s; CHECK(m.pop(s)); CHECK(s.err & MPR::E_EOC);   // warning, sample kept
}

static void test_saturation_and_range() {
  printf("saturation drops samples, range warns\n");
  reset_sim();
  M m(MprRange::R_0300YG);
  m.begin(wire); m.start(10000);
  sim.flags = 0x41;
  run_ms(m, 200);
  MprStats st = m.stats();
  CHECK(st.good == 0 && st.bad > 5);
  CHECK(m.lastError() & MPR::E_SAT);
  CHECK(m.avail() == 0);
  sim.flags = 0x40; sim.counts = 8000000;      // far above max
  run_ms(m, 200);
  st = m.stats();
  CHECK(st.good > 5);
  CHECK(m.lastError() == MPR::E_RANGE);
  MprSample s; CHECK(m.pop(s)); CHECK(s.err == MPR::E_RANGE);
}

struct CbCount { int before = 0, after = 0, bus = 0, failed = 0; bool last_ok = false; };
static CbCount cbc;
static void cbBefore(void *) { cbc.before++; }
static void cbAfter(void *, bool ok) { cbc.after++; cbc.last_ok = ok; }
static bool cbBus(void *) { cbc.bus++; return true; }
static void cbFail(void *) { cbc.failed++; }

static void test_recovery_reset() {
  printf("memory error -> reset recovers\n");
  reset_sim(); cbc = CbCount();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  MprCallbacks cb = {}; cb.onBeforeReset = cbBefore; cb.onAfterReset = cbAfter; cb.onBusRecover = cbBus; cb.onFailed = cbFail;
  m.setRecoveryCallbacks(cb);
  m.begin(wire); m.start(0);
  run_ms(m, 100);
  uint32_t good0 = m.stats().good;
  CHECK(good0 > 5);
  sim.flags = 0x44;                             // memory error, cleared only by reset
  run_ms(m, 500);
  printf("  resets %d before %d after %d last_ok %d good %lu\n", sim.resets, cbc.before, cbc.after, cbc.last_ok, (unsigned long)m.stats().good);
  CHECK(sim.resets >= 2);                       // one in begin(), one in recovery
  CHECK(cbc.before >= 1 && cbc.after >= 1 && cbc.last_ok);
  CHECK(m.stats().good > good0 + 5);            // sampling resumed
  CHECK(m.lastError() == 0);
  CHECK(cbc.failed == 0);
}

static void test_power_on_reset() {
  printf("power-on reset detected\n");
  reset_sim(); cbc = CbCount();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  m.begin(wire); m.start(0);
  run_ms(m, 100);
  sim.flags = 0x00;                             // sensor lost power
  run_ms(m, 50);
  CHECK(m.lastError() & MPR::E_POWER || sim.resets >= 2);
  run_ms(m, 3000);
  CHECK(sim.resets >= 2);                       // reset issued although fewer than RETRY_MAX... (POR path)
}

static void test_bus_dead() {
  printf("sensor unplugged: ladder, then resume\n");
  reset_sim(); cbc = CbCount();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  MprCallbacks cb = {}; cb.onBeforeReset = cbBefore; cb.onAfterReset = cbAfter; cb.onBusRecover = cbBus; cb.onFailed = cbFail;
  m.setRecoveryCallbacks(cb);
  m.begin(wire); m.start(0);
  run_ms(m, 100);
  sim.present = false;
  run_ms(m, 5000);
  MprStats st = m.stats();
  printf("  bad %lu resets %lu bus_restarts %lu cb: before %d bus %d failed %d, err 0x%04X\n", (unsigned long)st.bad, (unsigned long)st.resets, (unsigned long)st.bus_restarts, cbc.before, cbc.bus, cbc.failed, m.lastError());
  CHECK(st.bad > 10);
  CHECK(m.lastError() & MPR::E_CMD);
  CHECK(cbc.before >= 1 && cbc.bus >= 1 && cbc.failed == 1);
  CHECK(st.bad < 200);                          // failed state retries slowly, no spinning
  sim.present = true;
  run_ms(m, 3000);
  CHECK(m.lastError() == 0);
  CHECK(m.avail() > 0);
}

static void test_busy_timeout() {
  printf("sensor stuck busy: time-out, reset\n");
  reset_sim();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  m.begin(wire); m.start(0);
  run_ms(m, 50);
  sim.busy_stuck = true;
  run_ms(m, 3000);
  CHECK(m.stats().err_hist[2] > 0);             // E_TIMEOUT
  CHECK(sim.resets >= 2);
  printf("  lastError 0x%04X resets %d good %lu\n", m.lastError(), sim.resets, (unsigned long)m.stats().good);
  CHECK(m.lastError() == 0);                    // reset cleared busy, sampling resumed
}

static bool g_lock_ok = true; static int g_locks = 0, g_unlocks = 0;
static uint32_t g_lock_adv_us = 0;
static bool lockFn(void *) { if (!g_lock_ok) { g_now_us += g_lock_adv_us; return false; } g_locks++; return true; }
static void unlockFn(void *) { g_unlocks++; }

static void test_bus_lock() {
  printf("bus lock\n");
  reset_sim(); g_lock_ok = true; g_locks = g_unlocks = 0;
  M m(MprRange::R_0300YG);
  m.setBusLock(lockFn, unlockFn, nullptr);
  m.begin(wire); m.start(10000);
  run_ms(m, 200);
  CHECK(g_locks > 0 && g_locks == g_unlocks);
  uint32_t good = m.stats().good;
  g_lock_ok = false;
  run_ms(m, 300);
  MprStats st = m.stats();
  CHECK(st.lock_failed > 0);
  CHECK(st.bad == 0 && st.resets == 0);         // lock failure is not a sensor error
  g_lock_ok = true;
  run_ms(m, 300);
  CHECK(m.stats().good > good);
}

// F01: a denied lock (which itself takes time) must not cause a timeout, even past the conversion deadline
static void test_lock_denied_no_timeout() {
  printf("denied lock past deadline: no timeout\n");
  for (uint32_t adv : {0u, 5000u}) {
    reset_sim(); g_lock_ok = true; g_lock_adv_us = 0;
    M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
    m.setBusLock(lockFn, unlockFn, nullptr);
    m.begin(wire); m.start(0);
    run_ms(m, 50);
    sim.busy_stuck = true;
    run_ms(m, 5);                               // conversion started, will never complete
    int resets = sim.resets;
    g_lock_ok = false; g_lock_adv_us = adv;
    run_ms(m, 200);                             // well past the 20 ms deadline, every lock attempt denied
    MprStats st = m.stats();
    CHECK(st.lock_failed > 0);
    CHECK(st.bad == 0 && st.err_hist[2] == 0);
    CHECK(sim.resets == resets);
    g_lock_ok = true; g_lock_adv_us = 0;
  }
}

// F04: conversion completes while the unlock callback is delayed; duration must not underflow
static bool g_unlock_delay = false;
static bool lockOk(void *) { return true; }
static void unlockDelayed(void *) {
  if (!g_unlock_delay) return;
  g_unlock_delay = false;
  for (int i = 0; i < 100; i++) { g_now_us += 100; sim.tick(); }   // 10 ms scheduling delay, EOC fires meanwhile
}
static void test_delayed_unlock() {
  printf("command return delayed: no underflow\n");
  reset_sim();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  m.setBusLock(lockOk, unlockDelayed, nullptr);
  m.begin(wire); m.start(0);
  g_unlock_delay = true;
  run_ms(m, 100);
  MprStats st = m.stats();
  CHECK(st.good > 0);
  CHECK(st.conv_max_us < 10000);
  CHECK(st.conv_min_us >= MPR_CONV_MIN_US - 500);
}

// F03: an early edge must not become the sample time stamp
static void test_early_edge() {
  printf("early EOC edge discarded\n");
  reset_sim();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  m.begin(wire); m.start(0);
  sim.eoc_works = false;                         // only the spurious edge below, no genuine one
  while (sim.cmds == 0) { g_now_us += 100; sim.tick(); m.update(); }
  uint32_t t0 = g_now_us;
  while ((uint32_t)(g_now_us - t0) < 1000) { g_now_us += 100; sim.tick(); m.update(); }
  sim.handler();                                 // edge ~1 ms after the command
  run_ms(m, 6);                                  // conversion done, before EOC time-out fallback
  MprStats st = m.stats();
  CHECK(st.eoc_spurious == 1);
  CHECK(st.good == 0);                           // early edge was not accepted as the end of conversion
  run_ms(m, 200);                                // fallback polling then delivers valid samples
  st = m.stats();
  CHECK(st.good > 0 && st.conv_min_us >= MPR_CONV_MIN_US);
}

static bool cbBusFix(void *) { cbc.bus++; sim.present = true; return true; }

// F06: failed begin()/reset() must report the failure
static void test_begin_failure_reporting() {
  printf("failed begin reports error and status\n");
  for (int rst = 0; rst < 2; rst++) {
    reset_sim();
    M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, rst ? 11 : -1);
    CHECK(m.begin(wire));
    CHECK(m.lastError() == 0);
    sim.present = false;
    CHECK(!m.begin(wire));
    CHECK(m.lastError() & MPR::E_READ);
    CHECK(m.status() == 0xFF);
    sim.present = true; sim.flags = 0x44; sim.ignore_rst = true;   // memory fault, not cleared by reset
    CHECK(!m.begin(wire));
    CHECK(m.lastError() & MPR::E_MEM);
    sim.flags = 0x40; sim.ignore_rst = false;
    CHECK(m.begin(wire));
    CHECK(m.lastError() == 0);                  // success clears the earlier error
  }
}

// F07: lifecycle calls must not send a command to a busy sensor
static void test_no_command_while_busy() {
  printf("lifecycle calls do not command a busy sensor\n");
  reset_sim();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, -1);
  m.begin(wire); m.start(0);
  while (sim.cmds == 0) { g_now_us += 100; sim.tick(); m.update(); }
  m.start(0);                                    // restart during a conversion
  m.update();
  CHECK(sim.cmds == 1 && sim.ignored == 0);
  run_ms(m, 200);
  m.stop(); m.start(0);                          // stop/start around a conversion
  for (int i = 0; i < 50; i++) { g_now_us += 100; sim.tick(); m.update(); }
  m.stop();
  for (int i = 0; i < 200; i++) { g_now_us += 100; sim.tick(); }   // let it finish
  sim.conv_us = 50000;                           // long conversion: still busy when begin() looks
  while (!sim.converting) { m.start(0); g_now_us += 100; sim.tick(); m.update(); m.stop(); }   // stopped mid conversion
  CHECK(sim.converting);
  CHECK(m.begin(wire) == false);                 // no RST: sensor busy, begin must say so ...
  m.start(0);
  run_ms(m, 100);
  CHECK(sim.ignored == 0);                       // ... and must not command it
  CHECK(m.stats().good > 0);
}

// F08: busy together with a memory fault is reported as memory fault and recovered
static void test_busy_with_fault() {
  printf("busy + memory fault -> reset\n");
  reset_sim();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  m.begin(wire); m.start(0);
  run_ms(m, 50);
  sim.busy_stuck = true; sim.flags = 0x44;
  run_ms(m, 15);                                 // well before the 20 ms conversion time-out
  CHECK(m.stats().err_hist[5] > 0);              // E_MEM counted
  CHECK(sim.resets >= 2);
  run_ms(m, 100);
  CHECK(m.stats().good > 0);
}

// F09: a successful bus recovery must not report FAILED
static void test_bus_recover_success() {
  printf("bus recovery succeeds -> no failed callback\n");
  reset_sim(); cbc = CbCount();
  M m(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, 10, 11);
  MprCallbacks cb = {}; cb.onBusRecover = cbBusFix; cb.onFailed = cbFail;
  m.setRecoveryCallbacks(cb);
  m.begin(wire); m.start(0);
  run_ms(m, 100);
  sim.present = false;
  run_ms(m, 1000);
  CHECK(cbc.bus >= 1);
  uint32_t good = m.stats().good;
  run_ms(m, 3000);
  CHECK(cbc.failed == 0);
  CHECK(m.stats().good > good);
}

int main() {
  test_text(); test_conversion(); test_poll(); test_eoc(); test_eoc_missing();
  test_saturation_and_range(); test_recovery_reset(); test_power_on_reset(); test_bus_dead();
  test_busy_timeout(); test_bus_lock(); test_lock_denied_no_timeout(); test_delayed_unlock(); test_early_edge();
  test_begin_failure_reporting(); test_no_command_while_busy(); test_busy_with_fault(); test_bus_recover_success();
  printf(g_fail ? "\n%d CHECK(S) FAILED\n" : "\nall passed\n", g_fail);
  return g_fail ? 1 : 0;
}
