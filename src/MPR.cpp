// ************************************************************************
// MPR.cpp, see MPR.h and Design.md
// ************************************************************************
#include "MPR.h"
#include <string.h>

static_assert(MPR_EOC_TIMEOUT_MS < MPR_CONV_TIMEOUT_MS, "EOC time-out must be shorter than conversion time-out");

// ---------------------------------------------------------------- tables
namespace {

struct RangeInfo { float pmax; MprUnit unit; };

const RangeInfo RANGES[] = {
  // absolute
  {1.0f, MprUnit::BAR}, {1.6f, MprUnit::BAR}, {2.5f, MprUnit::BAR},          // 0001BA 01.6BA 02.5BA
  {100.0f, MprUnit::KPA}, {160.0f, MprUnit::KPA}, {250.0f, MprUnit::KPA},    // 0100KA 0160KA 0250KA
  {15.0f, MprUnit::PSI}, {25.0f, MprUnit::PSI}, {30.0f, MprUnit::PSI},       // 0015PA 0025PA 0030PA
  // gage
  {60.0f, MprUnit::MBAR}, {100.0f, MprUnit::MBAR}, {160.0f, MprUnit::MBAR},  // 0060MG 0100MG 0160MG
  {250.0f, MprUnit::MBAR}, {400.0f, MprUnit::MBAR}, {600.0f, MprUnit::MBAR}, // 0250MG 0400MG 0600MG
  {1.0f, MprUnit::BAR}, {1.6f, MprUnit::BAR}, {2.5f, MprUnit::BAR},          // 0001BG 01.6BG 02.5BG
  {6.0f, MprUnit::KPA}, {10.0f, MprUnit::KPA}, {16.0f, MprUnit::KPA},        // 0006KG 0010KG 0016KG
  {25.0f, MprUnit::KPA}, {40.0f, MprUnit::KPA}, {60.0f, MprUnit::KPA},       // 0025KG 0040KG 0060KG
  {100.0f, MprUnit::KPA}, {160.0f, MprUnit::KPA}, {250.0f, MprUnit::KPA},    // 0100KG 0160KG 0250KG
  {1.0f, MprUnit::PSI}, {5.0f, MprUnit::PSI}, {15.0f, MprUnit::PSI},         // 0001PG 0005PG 0015PG
  {30.0f, MprUnit::PSI},                                                     // 0030PG
  {300.0f, MprUnit::MMHG},                                                   // 0300YG
};
const uint8_t N_RANGES = sizeof(RANGES) / sizeof(RANGES[0]);

// Pa per unit
float paPerUnit(MprUnit u) {
  switch (u) {
    case MprUnit::PSI:  return 6894.757f;
    case MprUnit::KPA:  return 1000.0f;
    case MprUnit::PA:   return 1.0f;
    case MprUnit::BAR:  return 100000.0f;
    case MprUnit::MBAR: return 100.0f;
    case MprUnit::MMHG: return 133.322387f;
    case MprUnit::INHG: return 3386.389f;
    case MprUnit::ATM:  return 101325.0f;
    case MprUnit::TORR: return 133.322368f;
  }
  return 1.0f;
}

const char *const ERR_NAME[MPR::ERROR_BITS] = {
  "I2C write failed", "I2C read short", "sensor busy timeout", "bus reads all ones or invalid status",
  "not powered", "memory error", "math saturation", "counts out of range", "sensor busy", "EOC signal missing"
};
const char *const ERR_HINT[MPR::ERROR_BITS] = {
  "no ACK at the sensor address: check address, wiring, power and pull-ups",
  "sensor did not return all bytes: check wiring, pull-ups and bus speed",
  "sensor stayed busy: bus problem or sensor needs a reset",
  "SDA stuck high or sensor not driving the bus: check solder joints, VIN, GND, SDA/SCL and pull-ups",
  "sensor reports no power or power-on reset: check VIN and GND, brown-out",
  "internal memory corrupt (damaged sensor or unstable supply); only a reset or power cycle clears it",
  "internal math overflow: pressure beyond range, or sensor damaged if the port is open to air",
  "counts far outside the expected range: wrong range or transfer function configured, or sensor damaged",
  "data read while still busy, reading discarded",
  "EOC pin did not signal end of conversion: check the EOC wire; status polling is used instead"
};
const char *const I2C_RC_NAME[6] = { "ok", "data too long", "NACK on address", "NACK on data", "other error", "timeout" };

// string builder with truncation
struct Out {
  char *b; size_t n; size_t len;
  Out(char *buf, size_t size) : b(buf), n(size), len(0) { if (n) b[0] = '\0'; }
  void add(const char *s) {
    if (!n) return;
    while (*s && len + 1 < n) { b[len++] = *s++; }
    b[len] = '\0';
  }
  void sep() { if (len) add(", "); }
};

// Interrupt trampolines for cores without attachInterruptArg
#if !defined(ARDUINO_ARCH_ESP32)
MPR *s_inst[4] = {nullptr, nullptr, nullptr, nullptr};
void tramp0() { MPR::isr(s_inst[0]); }
void tramp1() { MPR::isr(s_inst[1]); }
void tramp2() { MPR::isr(s_inst[2]); }
void tramp3() { MPR::isr(s_inst[3]); }
void (*const s_tramp[4])() = {tramp0, tramp1, tramp2, tramp3};
#endif

} // namespace

// ---------------------------------------------------------------- construction
MPR::MPR(MprRange range, MprTransfer tf, MprUnit unit, uint8_t i2c_addr, int8_t eoc_pin, int8_t rst_pin)
  : _wire(nullptr), _addr(i2c_addr), _eoc(eoc_pin), _rst(rst_pin), _unit(unit), _native(MprUnit::PSI),
    _pmax(1.0f), _cmin(0), _cmax(1), _zero(0.0f), _cal(1.0f), _scale(1.0f), _fac(1.0f), _have_zero(false),
    _eoc_flag(false), _eoc_us(0), _state(S_STOPPED), _cfg_mode(MprMode::STATUS_POLL),
    _lock_blocked(false), _t_lockfail(0), _fallback(false), _isr_attached(false), _poll_this(false), _need_idle_check(false), _por(false),
    _seen_power(false), _failed(false), _bus_tried(false), _interval_us(0), _next_start(0), _t_cmd(0), _t_poll(0),
    _fallback_ms(0), _last_reset_ms(0), _last_bus_ms(0), _consec(0), _warn(0), _status(0), _i2c_rc(0),
    _last_err(0), _conv_sum(0.0), _conv_n(0), _sink(nullptr), _sink_ctx(nullptr), _lock(nullptr), _unlock(nullptr),
    _lock_ctx(nullptr)
{
  memset(&_st, 0, sizeof(_st));
  _st.conv_min_us = 0xFFFFFFFFUL;
  memset(&_cb, 0, sizeof(_cb));

  uint8_t idx = (uint8_t)range;
  if (idx < N_RANGES) { _pmax = RANGES[idx].pmax; _native = RANGES[idx].unit; }

  switch (tf) {
    case MprTransfer::A: _cmin = 1677722UL; _cmax = 15099494UL; break;   // 10 % .. 90 % of 2^24
    case MprTransfer::B: _cmin =  419430UL; _cmax =  3774873UL; break;   // 2.5 % .. 22.5 %
    case MprTransfer::C: _cmin = 3355443UL; _cmax = 13421772UL; break;   // 20 % .. 80 %
  }
  computeScale();
}

void MPR::computeScale() {
  _scale = (_cmax > _cmin) ? _pmax / (float)(_cmax - _cmin) : 0.0f;
  _fac   = paPerUnit(_native) / paPerUnit(_unit);
}

void MPR::setZero(float counts)                  { _zero = counts; _have_zero = true; }
void MPR::setCal(float k)                        { _cal = k; }
void MPR::setCountsRange(uint32_t mn, uint32_t mx) { _cmin = mn; _cmax = mx; computeScale(); }
void MPR::setRange(float p_max, MprUnit native)  { _pmax = p_max; _native = native; computeScale(); }

float MPR::toPressure(uint32_t counts) const {
  float base = _have_zero ? _zero : (float)_cmin;
  return ((float)counts - base) * _scale * _fac * _cal;
}

// ---------------------------------------------------------------- bus
void MPR::setBusLock(MprBusLock lock, MprBusUnlock unlock, void *ctx) { _lock = lock; _unlock = unlock; _lock_ctx = ctx; }

bool MPR::busLock() {
  if (_lock == nullptr || _lock(_lock_ctx)) { return true; }
  _st.lock_failed++;                    // other users hold the bus: not a sensor error
  _lock_blocked = true;
  _t_lockfail   = micros();
  return false;
}
void MPR::busUnlock() { if (_unlock) { _unlock(_lock_ctx); } }

int MPR::writeCommand() {
  if (!busLock()) { return -2; }
  _wire->beginTransmission(_addr);
  _wire->write((uint8_t)0xAA);
  _wire->write((uint8_t)0x00);
  _wire->write((uint8_t)0x00);
  int rc = _wire->endTransmission();
  if (rc == 0) { _t_cmd = micros(); }     // conversion starts at the STOP condition: stamp before unlock callback / task switch
  busUnlock();
  _i2c_rc = (uint8_t)rc;
  return rc;
}

int MPR::readStatusByte() {
  if (!busLock()) { return -2; }
  int r = -1;
  if (_wire->requestFrom(_addr, (uint8_t)1) == 1) { r = _wire->read(); }
  while (_wire->available()) { _wire->read(); }
  busUnlock();
  return r;
}

int MPR::readResult(uint8_t *buf) {
  if (!busLock()) { return -2; }
  int n = (int)_wire->requestFrom(_addr, (uint8_t)4);
  if (n == 4) { for (uint8_t i = 0; i < 4; i++) { buf[i] = (uint8_t)_wire->read(); } }
  while (_wire->available()) { _wire->read(); }
  busUnlock();
  return n;
}

// ---------------------------------------------------------------- interrupt
// The EOC flag and its time stamp are published by the ISR and consumed by update().
// Short critical sections give the task a coherent (flag, time) pair; no I2C or callback runs inside.
#if defined(ARDUINO_ARCH_ESP32)
  static portMUX_TYPE s_eoc_mux = portMUX_INITIALIZER_UNLOCKED;
  #define MPR_ISR_ENTER()  portENTER_CRITICAL_ISR(&s_eoc_mux)
  #define MPR_ISR_EXIT()   portEXIT_CRITICAL_ISR(&s_eoc_mux)
  #define MPR_TASK_ENTER() portENTER_CRITICAL(&s_eoc_mux)
  #define MPR_TASK_EXIT()  portEXIT_CRITICAL(&s_eoc_mux)
#else
  #define MPR_ISR_ENTER()
  #define MPR_ISR_EXIT()
  #define MPR_TASK_ENTER() noInterrupts()
  #define MPR_TASK_EXIT()  interrupts()
#endif

void IRAM_ATTR MPR::isr(void *arg) {
  MPR *m = static_cast<MPR *>(arg);
  if (m == nullptr) { return; }
  MPR_ISR_ENTER();
  m->_eoc_us   = micros();
  m->_eoc_flag = true;
  MPR_ISR_EXIT();
}

bool MPR::peekEoc(uint32_t &t) {
  MPR_TASK_ENTER();
  bool f = _eoc_flag;
  t = _eoc_us;
  MPR_TASK_EXIT();
  return f;
}

// Clear the flag only if no newer edge replaced the one that was inspected
void MPR::consumeEoc(uint32_t t) {
  MPR_TASK_ENTER();
  if (_eoc_us == t) { _eoc_flag = false; }
  MPR_TASK_EXIT();
}

void MPR::clearEoc() {
  MPR_TASK_ENTER();
  _eoc_flag = false;
  MPR_TASK_EXIT();
}

void MPR::attachEoc() {
  if (_isr_attached || _cfg_mode != MprMode::EOC_INTERRUPT) { return; }
#if defined(ARDUINO_ARCH_ESP32)
  attachInterruptArg(digitalPinToInterrupt(_eoc), MPR::isr, this, RISING);
  _isr_attached = true;
#else
  for (uint8_t i = 0; i < 4; i++) {
    if (s_inst[i] == nullptr) {
      s_inst[i] = this;
      attachInterrupt(digitalPinToInterrupt(_eoc), s_tramp[i], RISING);
      _isr_attached = true;
      return;
    }
  }
  _cfg_mode = MprMode::STATUS_POLL;   // no free interrupt slot
#endif
}

void MPR::detachEoc() {
  if (!_isr_attached) { return; }
  detachInterrupt(digitalPinToInterrupt(_eoc));
#if !defined(ARDUINO_ARCH_ESP32)
  for (uint8_t i = 0; i < 4; i++) { if (s_inst[i] == this) { s_inst[i] = nullptr; } }
#endif
  _isr_attached = false;
}

// ---------------------------------------------------------------- lifecycle
bool MPR::begin(TwoWire *wire, uint32_t clock_hz) {
  if (wire == nullptr) { return false; }
  return begin(*wire, clock_hz);
}

bool MPR::begin(TwoWire &wire, uint32_t clock_hz) {
  stop();
  _wire = &wire;
  if (clock_hz != 0) { _wire->setClock(clock_hz); }

  if (_rst >= 0) { digitalWrite(_rst, HIGH); pinMode(_rst, OUTPUT); digitalWrite(_rst, HIGH); }
  if (_eoc >= 0) { pinMode(_eoc, INPUT); }
  _cfg_mode = (_eoc >= 0 && digitalPinToInterrupt(_eoc) != NOT_AN_INTERRUPT) ? MprMode::EOC_INTERRUPT
                                                                            : MprMode::STATUS_POLL;
  _por = false; _seen_power = false; _failed = false; _bus_tried = false; _consec = 0;
  // _need_idle_check is kept: a conversion may still be running, see below

  bool ok;
  if (_rst >= 0) {
    ok = reset();
  } else {
    delay(MPR_STARTUP_MS);
    int st = readStatusByte();
    ok = false;
    if (st != -2) {                             // lock denied: no sensor outcome, keep previous status / error
      uint16_t e = statusOutcome(st);
      _last_err = e;
      _status   = (st >= 0) ? (uint8_t)st : 0xFF;
      if (st >= 0 && st != 0xFF && !(st & ST_INVALID)) { _need_idle_check = (st & ST_BUSY) != 0; }   // never command a busy sensor
      ok = (e == 0);
      _seen_power = ok;
    }
  }
  return ok;
}

void MPR::end() {
  stop();
  _wire = nullptr;
}

bool MPR::start(uint32_t interval_us) {
  if (_wire == nullptr) { return false; }
  _interval_us = interval_us;
  _consec = 0; _failed = false; _bus_tried = false;
  _last_reset_ms = _last_bus_ms = millis() - MPR_RECOVER_MIN_MS;
  attachEoc();
  if (_state == S_CONVERTING) { return true; }  // already sampling: only the interval changes, the running conversion is not disturbed
  clearEoc(); _fallback = false; _poll_this = false;
  _next_start = micros();
  _state = S_IDLE;                              // a pending busy check (_need_idle_check) is preserved
  return true;
}

void MPR::stop() {
  detachEoc();
  if (_state == S_CONVERTING) { _need_idle_check = true; }   // sensor may still be busy
  _state = S_STOPPED;
}

// Error code describing a status byte seen during initialization / reset (0 = ready, powered, no fault).
// st < 0: no usable answer from the sensor.
uint16_t MPR::statusOutcome(int st) const {
  if (st < 0)                          { return E_READ; }
  if (st == 0xFF || (st & ST_INVALID)) { return E_BUS; }
  uint16_t e = 0;
  if (!(st & ST_POWERED))  { e |= E_POWER; }
  if (st & ST_INTEGRITY)   { e |= E_MEM; }
  if (st & ST_BUSY)        { e |= E_BUSY; }
  return e;
}

bool MPR::reset() {
  uint16_t e;
  bool ok = resetSensor(e);
  if (e != OUTCOME_NONE) { _last_err = e; }     // manual reset: report the outcome
  return ok;
}

// err: outcome of the reset, OUTCOME_NONE if the bus lock never allowed talking to the sensor.
bool MPR::resetSensor(uint16_t &err) {
  err = OUTCOME_NONE;
  if (_rst < 0 || _wire == nullptr) { return false; }
  digitalWrite(_rst, LOW);  delay(MPR_RST_LOW_MS);
  digitalWrite(_rst, HIGH); delay(MPR_RST_WAIT_MS);

  bool ok = false;
  bool busy_seen = false;
  uint32_t t0 = millis();
  do {
    int st = readStatusByte();
    if (st != -2) {
      uint16_t e = statusOutcome(st);
      err = e;
      _status = (st >= 0) ? (uint8_t)st : 0xFF;
      busy_seen = (st >= 0 && st != 0xFF && !(st & ST_INVALID) && (st & ST_BUSY));
      if (e == 0) { ok = true; break; }
    }
    delay(2);
  } while ((millis() - t0) < MPR_RST_TIMEOUT_MS);
  if (err == E_BUSY) { err = E_TIMEOUT; }       // only busy until the end of the wait

  _st.resets++;
  _por = false;
  _seen_power = ok;
  if (ok) { _need_idle_check = false; }         // a failed reset says nothing about the sensor being idle
  else if (busy_seen) { _need_idle_check = true; }
  clearEoc();
  _fallback = false;                 // a stuck sensor also explains a missing EOC edge: try the interrupt again
  if (_state == S_CONVERTING) { _state = S_IDLE; }
  _next_start = micros();
  return ok;
}

MprMode MPR::mode() const {
  if (_state == S_STOPPED) { return MprMode::STOPPED; }
  return (_cfg_mode == MprMode::EOC_INTERRUPT && !_fallback) ? MprMode::EOC_INTERRUPT : MprMode::STATUS_POLL;
}

MprStats MPR::stats() const {
  MprStats s = _st;
  if (_conv_n) { s.conv_mean_us = (float)(_conv_sum / (double)_conv_n); }
  else         { s.conv_min_us = 0; s.conv_mean_us = 0.0f; }
  return s;
}

// ---------------------------------------------------------------- state machine
void MPR::update() {
  if (_state == S_STOPPED || _wire == nullptr) { return; }
  if (_state == S_CONVERTING) { processConverting(micros()); }
  if (_state == S_IDLE)       { tryStart(micros()); }
}

void MPR::tryStart(uint32_t now) {
  if ((int32_t)(now - _next_start) < 0) { return; }

  if (_need_idle_check) {                       // sensor ignores commands while busy
    int st = readStatusByte();
    if (st == -2) { return; }
    if (st < 0)   { sampleBad(E_READ); return; }
    _status = (uint8_t)st;
    if (st == 0xFF || (st & ST_INVALID)) { sampleBad(E_BUS); return; }
    if (st & ST_BUSY)                    { sampleBad(E_TIMEOUT); return; }
    _need_idle_check = false;
  }

  clearEoc();                                   // clear immediately before the command
  _poll_this = false;
  _warn = 0;
  _lock_blocked = false;
  int rc = writeCommand();
  if (rc == -2) { return; }                     // bus busy, try again on next update()
  if (rc != 0)  { _status = 0xFF; sampleBad(E_CMD); return; }

  _t_poll = _t_cmd;                             // _t_cmd was recorded by writeCommand()
  _state  = S_CONVERTING;
  if (_interval_us) {                           // period measured from previous start, no drift
    uint32_t base = _next_start;
    _next_start = base + _interval_us;
    if ((int32_t)(_t_cmd - _next_start) > 0) { _next_start = _t_cmd + _interval_us; }   // late: do not catch up
  } else {
    _next_start = _t_cmd;
  }
}

void MPR::processConverting(uint32_t now) {
  uint32_t el = now - _t_cmd;

  // End of conversion by interrupt
  if (_cfg_mode == MprMode::EOC_INTERRUPT && !_fallback && !_poll_this) {
    uint32_t te;
    if (peekEoc(te)) {
      if ((int32_t)(te - _t_cmd) < (int32_t)MPR_CONV_MIN_US) {
        consumeEoc(te);                         // edge before the earliest possible end of conversion (or before the command): spurious
        _st.eoc_spurious++;                     // keep waiting for a genuine edge; time-out / polling cover its absence
      } else {
        if (finishRead(te)) { consumeEoc(te); } // flag stays set if the bus lock was not available
        return;
      }
    }
    if (el <= MPR_EOC_TIMEOUT_MS * 1000UL) { return; }
    _fallback = true;                           // EOC wire broken or not connected: poll instead
    _fallback_ms = millis();
    _st.eoc_timeouts++;
    _warn |= E_EOC;
    _t_poll = now - MPR_POLL_US;
  }

  // End of conversion by status polling
  if (el >= MPR_CONV_MIN_US && (now - _t_poll) >= MPR_POLL_US) {
    _t_poll = now;
    int st = readStatusByte();
    if (st == -2) {
      // bus lock not available: not a sensor error, no timeout evaluation; retry at the next poll
      return;
    } else if (st < 0) {
      sampleBad(E_READ); return;
    } else {
      _status = (uint8_t)st;
      if (st == 0xFF || (st & ST_INVALID)) { sampleBad(E_BUS); return; }
      uint16_t fatal = decode((uint8_t)st, _cmin) & (E_POWER | E_MEM);   // valid even while busy: do not wait for a busy sensor that reports a fault
      if (fatal) { sampleBad(fatal); return; }
      if (!(st & ST_BUSY)) {
        uint32_t t = micros();
        if (_cfg_mode == MprMode::EOC_INTERRUPT) {          // EOC must be high when data is ready
          if (digitalRead(_eoc)) {
            if (_fallback && (millis() - _fallback_ms) >= MPR_EOC_RETRY_MS) { _fallback = false; }   // try interrupt again
          } else {
            _st.eoc_low_seen++;
            _warn |= E_EOC;
          }
        }
        finishRead(t);
        return;
      }
    }
  }
  // Time-out, but not while other users kept us from the bus
  bool blocked = _lock_blocked && ((uint32_t)(micros() - _t_lockfail) <= MPR_CONV_TIMEOUT_MS * 1000UL);   // fresh time: 'now' may predate _t_lockfail
  if (el > MPR_CONV_TIMEOUT_MS * 1000UL && !blocked) { sampleBad(E_TIMEOUT); }
}

uint16_t MPR::decode(uint8_t st, uint32_t counts) {
  if (st == 0xFF || (st & ST_INVALID)) { return E_BUS; }
  uint16_t e = 0;
  if (!(st & ST_POWERED)) { e |= E_POWER; if (_seen_power) { _por = true; } }
  else                    { _seen_power = true; }
  if (st & ST_BUSY)       { e |= E_BUSY; }
  if (st & ST_INTEGRITY)  { e |= E_MEM; }
  if (st & ST_SATURATION) { e |= E_SAT; }
  uint32_t margin = (_cmax - _cmin) / 20U;
  if ((counts + margin < _cmin) || (counts > _cmax + margin)) { e |= E_RANGE; }
  return e;
}

bool MPR::finishRead(uint32_t t_stamp) {
  uint8_t buf[4];
  int n = readResult(buf);
  if (n == -2) { return false; }
  if (n != 4)  { sampleBad(E_READ); return true; }

  uint32_t counts = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
  _status = buf[0];
  uint16_t e = decode(_status, counts);

  if (e & E_BUSY) {                              // not ready after all: keep waiting, by polling
    uint16_t fatal = e & (E_BUS | E_POWER | E_MEM);   // status-level faults are valid even though the data is not
    if (fatal) { sampleBad(fatal); return true; }
    _last_err = E_BUSY;
    _st.err_hist[8]++;
    if (_cfg_mode == MprMode::EOC_INTERRUPT && !_fallback && !_poll_this) { _st.eoc_early++; }
    _poll_this = true;
    clearEoc();
    return true;
  }

  if (_cfg_mode == MprMode::EOC_INTERRUPT && _fallback) { _warn |= E_EOC; }
  e |= _warn;

  if (e & ~E_WARN) { sampleBad(e); return true; }

  int32_t dt = (int32_t)(t_stamp - _t_cmd);
  if (dt >= 0) {                                 // impossible (negative) durations are not added to the statistics
    uint32_t conv = (uint32_t)dt;
    if (conv < _st.conv_min_us) { _st.conv_min_us = conv; }
    if (conv > _st.conv_max_us) { _st.conv_max_us = conv; }
    _conv_sum += (double)conv;
    _conv_n++;
  }
  _st.good++;
  for (uint8_t b = 0; b < ERROR_BITS; b++) { if (e & (1U << b)) { _st.err_hist[b]++; } }

  _last_err = e;
  _consec   = 0;
  _failed   = false;
  _bus_tried = false;
  _state    = S_IDLE;                            // next conversion starts in this same update() call

  if (_sink) {
    MprSample s;
    s.counts = counts; s.t_us = t_stamp; s.status = _status; s.pad = 0; s.err = (uint16_t)(e & E_WARN);
    _sink(_sink_ctx, s);
  }
  return true;
}

void MPR::sampleBad(uint16_t err) {
  _last_err = err;
  _st.bad++;
  for (uint8_t b = 0; b < ERROR_BITS; b++) { if (err & (1U << b)) { _st.err_hist[b]++; } }
  _consec++;
  if (err & E_TIMEOUT) { _need_idle_check = true; }
  _state = S_IDLE;
  _next_start = micros() + ((_failed || _bus_tried) ? MPR_FAILED_GAP_US : MPR_RETRY_GAP_US);
  recoveryLadder();
}

void MPR::recoveryLadder() {
  uint32_t ms = millis();

  // Step 2: reset (RST pin)
  bool want_reset = _por || (_last_err & E_MEM) || (_consec >= MPR_RETRY_MAX);
  if (want_reset && _rst >= 0 && (ms - _last_reset_ms) >= MPR_RECOVER_MIN_MS) {
    _last_reset_ms = ms;
    if (_cb.onBeforeReset) { _cb.onBeforeReset(_cb.ctx); }
    uint16_t re;
    bool ok = resetSensor(re);
    if (_cb.onAfterReset) { _cb.onAfterReset(_cb.ctx, ok); }
    if (ok) { _consec = 0; return; }
  }

  // Step 3 and 4: bus recovery, then failed state
  if (_consec >= MPR_BUS_RESTART_AFTER && (ms - _last_bus_ms) >= MPR_RECOVER_MIN_MS) {
    _last_bus_ms = ms;
    bool repaired = false;
    if (_cb.onBusRecover) {
      _st.bus_restarts++;
      repaired = _cb.onBusRecover(_cb.ctx);
    }
    if (repaired && !_bus_tried) {
      _bus_tried = true;                        // give the repaired bus a chance: retry slowly, FAILED only if it is still bad
      _next_start = micros() + MPR_FAILED_GAP_US;   // after the next cooldown
    } else if (!_failed) {
      _failed = true;
      if (_cb.onFailed) { _cb.onFailed(_cb.ctx); }
    }
  }
}

// ---------------------------------------------------------------- text helpers
const char *MPR::errorName(uint8_t bit) { return (bit < ERROR_BITS) ? ERR_NAME[bit] : "unknown error"; }
const char *MPR::errorHint(uint8_t bit) { return (bit < ERROR_BITS) ? ERR_HINT[bit] : ""; }

const char *MPR::i2cResultString(uint8_t rc) { return (rc < 6) ? I2C_RC_NAME[rc] : "unknown error"; }

size_t MPR::errorString(uint16_t err, char *buf, size_t n) {
  Out o(buf, n);
  if (err == 0) { o.add("ok"); return o.len; }
  for (uint8_t b = 0; b < ERROR_BITS; b++) {
    if (err & (1U << b)) { o.sep(); o.add(ERR_NAME[b]); }
  }
  return o.len;
}

size_t MPR::statusString(uint8_t st, char *buf, size_t n) {
  Out o(buf, n);
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  char h[7] = { '0', 'x', HEX_DIGITS[st >> 4], HEX_DIGITS[st & 0x0F], ':', ' ', '\0' };
  o.add(h);
  if (st == 0xFF) { o.add("bus reads all ones"); return o.len; }
  o.add((st & ST_POWERED) ? "powered" : "not powered");
  if (st & ST_BUSY)       { o.sep(); o.add("busy"); }
  if (st & ST_INTEGRITY)  { o.sep(); o.add("memory error"); }
  if (st & ST_SATURATION) { o.sep(); o.add("math saturation"); }
  if (st & ST_INVALID)    { o.sep(); o.add("invalid bits set"); }
  return o.len;
}
