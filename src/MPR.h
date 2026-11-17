// ************************************************************************
// MPR.h
//
// Driver for Honeywell MPR / MPRLS series pressure sensors, I2C variant.
//
//  - Non-blocking state machine, call update() from loop() or from a task
//  - End of conversion by EOC interrupt (when EOC pin is wired) or by
//    status byte polling (fallback, automatic)
//  - Optional RST pin: reset and recovery
//  - Shares the I2C bus (TwoWire) with other devices, optional bus lock
//  - No printing in the driver, text helpers for status and errors
//
// See Design.md.
// ************************************************************************
#ifndef MPR_H
#define MPR_H

#include <Arduino.h>
#include <Wire.h>
#include <stddef.h>
#include <stdint.h>
#include <RingBuffer.h>      // UUtzinger_RingBuffer

#ifndef IRAM_ATTR
  #define IRAM_ATTR
#endif

// ---------------------------------------------------------------- I2C constants
#define MPR_I2C_ADDRESS        0x18
#define MPR_I2C_FAST         400000
#define MPR_I2C_REGULAR      100000
#define MPR_I2C_SLOW          50000
#define MPR_I2C_DEFAULTSTRETCH   50   // ms, Wire time-out, sensor typically needs 230 us
#define MPR_I2C_LONGSTRETCH     200

// ---------------------------------------------------------------- timing constants (overridable)
#ifndef MPR_CONV_MIN_US
  #define MPR_CONV_MIN_US       3000UL  // no status poll / EOC accepted earlier than this after the command
#endif
#ifndef MPR_POLL_US
  #define MPR_POLL_US            500UL  // status poll period
#endif
#ifndef MPR_CONV_TIMEOUT_MS
  #define MPR_CONV_TIMEOUT_MS     20UL  // datasheet worst case is about 6.2 ms
#endif
#ifndef MPR_EOC_TIMEOUT_MS
  #define MPR_EOC_TIMEOUT_MS      10UL  // no EOC edge: fall back to status polling (must be < MPR_CONV_TIMEOUT_MS)
#endif
#ifndef MPR_EOC_RETRY_MS
  #define MPR_EOC_RETRY_MS     10000UL  // after a fallback, try the EOC interrupt again after this time
#endif
#ifndef MPR_STARTUP_MS
  #define MPR_STARTUP_MS          10UL  // begin() without RST pin
#endif
#ifndef MPR_RST_LOW_MS
  #define MPR_RST_LOW_MS           5UL
#endif
#ifndef MPR_RST_WAIT_MS
  #define MPR_RST_WAIT_MS          5UL
#endif
#ifndef MPR_RST_TIMEOUT_MS
  #define MPR_RST_TIMEOUT_MS      50UL  // wait for power bit after reset
#endif
#ifndef MPR_RETRY_MAX
  #define MPR_RETRY_MAX             3U  // consecutive bad readings before reset (recovery step 2)
#endif
#ifndef MPR_BUS_RESTART_AFTER
  #define MPR_BUS_RESTART_AFTER    50U  // consecutive bad readings before bus recovery (step 3)
#endif
#ifndef MPR_RECOVER_MIN_MS
  #define MPR_RECOVER_MIN_MS     2000UL // minimum time between resets / bus recoveries
#endif
#ifndef MPR_RETRY_GAP_US
  #define MPR_RETRY_GAP_US       1000UL // pause after a bad reading
#endif
#ifndef MPR_FAILED_GAP_US
  #define MPR_FAILED_GAP_US   1000000UL // retry period in FAILED state
#endif

// ---------------------------------------------------------------- types
// Part number ranges (datasheet ordering table). A = absolute, G = gage. P_MIN is always 0.
enum class MprRange : uint8_t {
  // absolute
  R_0001BA, R_01_6BA, R_02_5BA,
  R_0100KA, R_0160KA, R_0250KA,
  R_0015PA, R_0025PA, R_0030PA,
  // gage
  R_0060MG, R_0100MG, R_0160MG, R_0250MG, R_0400MG, R_0600MG,
  R_0001BG, R_01_6BG, R_02_5BG,
  R_0006KG, R_0010KG, R_0016KG, R_0025KG, R_0040KG, R_0060KG, R_0100KG, R_0160KG, R_0250KG,
  R_0001PG, R_0005PG, R_0015PG, R_0030PG,
  R_0300YG,
  CUSTOM                                   // use setRange()
};
enum class MprTransfer : uint8_t { A, B, C };
enum class MprUnit     : uint8_t { PSI, KPA, PA, BAR, MBAR, MMHG, INHG, ATM, TORR };
enum class MprMode     : uint8_t { STOPPED, EOC_INTERRUPT, STATUS_POLL };

struct MprSample {
  uint32_t counts;      // raw 24 bit output
  uint32_t t_us;        // micros() at EOC interrupt, or at ready detection when polling
  uint8_t  status;      // raw status byte of this reading
  uint8_t  pad;
  uint16_t err;         // warning bits only (E_RANGE, E_EOC)
};
static_assert(sizeof(MprSample) == 12, "MprSample must be 12 bytes");

struct MprStats {
  uint32_t good, bad;
  uint32_t err_hist[10];        // readings with each error bit set (index = bit number)
  uint32_t resets, bus_restarts;
  uint32_t eoc_timeouts;        // EOC edge did not arrive, polling used instead
  uint32_t eoc_early;           // EOC edge arrived but status still busy
  uint32_t eoc_spurious;        // EOC edge discarded: earlier than the minimum conversion time
  uint32_t eoc_low_seen;        // EOC pin low although status said ready
  uint32_t lock_failed;         // bus lock could not be acquired, attempt skipped
  uint32_t conv_min_us, conv_max_us;
  float    conv_mean_us;
};

typedef void (*MprSink)(void *ctx, const MprSample &s);   // receives every good sample
typedef bool (*MprBusLock)(void *ctx);                    // acquire the shared bus; false = could not
typedef void (*MprBusUnlock)(void *ctx);                  // release the shared bus

struct MprCallbacks {
  void (*onBeforeReset)(void *ctx);
  void (*onAfterReset)(void *ctx, bool ok);
  bool (*onBusRecover)(void *ctx);          // restart the shared TwoWire, return true if done
  void (*onFailed)(void *ctx);
  void *ctx;
};

// ---------------------------------------------------------------- driver
class MPR {
public:
  // Error bits, bit number = index into MprStats::err_hist
  enum : uint16_t {
    E_CMD     = 0x0001,   // command write failed
    E_READ    = 0x0002,   // read returned fewer bytes than requested
    E_TIMEOUT = 0x0004,   // still busy after MPR_CONV_TIMEOUT_MS
    E_BUS     = 0x0008,   // status 0xFF or invalid bits set
    E_POWER   = 0x0010,   // not powered / in power-on reset
    E_MEM     = 0x0020,   // memory integrity check failed
    E_SAT     = 0x0040,   // math saturation
    E_RANGE   = 0x0080,   // counts outside MIN..MAX by more than 5 % of span (warning)
    E_BUSY    = 0x0100,   // data read still busy, discarded
    E_EOC     = 0x0200,   // EOC edge missing or EOC low while ready (warning)
    E_WARN    = E_RANGE | E_EOC
  };
  static const uint8_t ERROR_BITS = 10;

  // Status byte bits
  enum : uint8_t {
    ST_POWERED    = 0x40,
    ST_BUSY       = 0x20,
    ST_INTEGRITY  = 0x04,
    ST_SATURATION = 0x01,
    ST_INVALID    = 0x9A   // bits 7,4,3,1 are always 0
  };

  MPR(MprRange range, MprTransfer tf = MprTransfer::B, MprUnit unit = MprUnit::MMHG,
      uint8_t i2c_addr = MPR_I2C_ADDRESS, int8_t eoc_pin = -1, int8_t rst_pin = -1);
  ~MPR() { end(); }

  // Lifecycle. clock_hz = 0: do not touch clock (shared bus). Wire.begin() is the caller's job.
  bool     begin(TwoWire &wire = Wire, uint32_t clock_hz = 0);
  bool     begin(TwoWire *wire, uint32_t clock_hz = 0);
  void     end();
  bool     start(uint32_t interval_us = 0);   // 0 = back to back, else fixed period
  void     stop();
  void     update();                          // non-blocking
  bool     reset();                           // RST pulse, blocks about 15 ms

  // Shared bus
  void     setBusLock(MprBusLock lock, MprBusUnlock unlock, void *ctx);

  // State
  uint8_t  status() const        { return _status; }
  uint16_t lastError() const     { return _last_err; }
  uint8_t  lastI2CResult() const { return _i2c_rc; }
  MprMode  mode() const;
  MprStats stats() const;

  // Text helpers (static, no allocation, no printing). Buffers are always null terminated.
  static const char *errorName(uint8_t bit);
  static const char *errorHint(uint8_t bit);
  static size_t      errorString(uint16_t err, char *buf, size_t n);
  static size_t      statusString(uint8_t status, char *buf, size_t n);
  static const char *i2cResultString(uint8_t rc);

  // Conversion
  float    toPressure(uint32_t counts) const;
  float    toPressure(const MprSample &s) const { return toPressure(s.counts); }
  void     setZero(float counts);
  float    zero() const          { return _have_zero ? _zero : (float)_cmin; }
  void     setCal(float k);
  void     setCountsRange(uint32_t mn, uint32_t mx);
  void     setRange(float p_max, MprUnit native);
  uint32_t minCounts() const     { return _cmin; }
  uint32_t maxCounts() const     { return _cmax; }

  // Hooks
  void     setSink(MprSink fn, void *ctx) { _sink = fn; _sink_ctx = ctx; }
  void     setRecoveryCallbacks(const MprCallbacks &cb) { _cb = cb; }

  // ISR entry, public only for the interrupt trampolines
  static void IRAM_ATTR isr(void *arg);

private:
  MPR(const MPR &) = delete;
  MPR &operator=(const MPR &) = delete;

  enum State : uint8_t { S_STOPPED, S_IDLE, S_CONVERTING };

  // bus
  bool     busLock();
  void     busUnlock();
  int      readStatusByte();                       // >=0 status, -1 short read, -2 lock failed
  int      readResult(uint8_t *buf);               // 4 bytes: 4 ok, 0..3 short, -2 lock failed
  int      writeCommand();                         // rc, -2 lock failed

  // state machine
  void     tryStart(uint32_t now_us);
  void     processConverting(uint32_t now_us);
  bool     finishRead(uint32_t t_stamp_us);      // false: bus lock not available, try again
  uint16_t decode(uint8_t status, uint32_t counts);
  void     sampleBad(uint16_t err);
  void     recoveryLadder();
  void     computeScale();
  void     attachEoc();
  void     detachEoc();
  bool     peekEoc(uint32_t &t);
  void     consumeEoc(uint32_t t);
  void     clearEoc();
  bool     checkIdleStatus(int st);
  void     noteWarn(uint16_t err) { _warn |= err; }

  // configuration
  TwoWire *_wire;
  uint8_t  _addr;
  int8_t   _eoc, _rst;
  MprUnit  _unit, _native;
  float    _pmax;
  uint32_t _cmin, _cmax;
  float    _zero, _cal, _scale, _fac;
  bool     _have_zero;

  // run time
  volatile bool     _eoc_flag;
  volatile uint32_t _eoc_us;
  State    _state;
  MprMode  _cfg_mode;
  bool     _lock_blocked;
  uint32_t _t_lockfail;
  bool     _fallback, _isr_attached, _poll_this, _need_idle_check, _por, _seen_power, _failed;
  uint32_t _interval_us, _next_start, _t_cmd, _t_poll, _fallback_ms;
  uint32_t _last_reset_ms, _last_bus_ms;
  uint32_t _consec;
  uint16_t _warn;
  uint8_t  _status, _i2c_rc;
  uint16_t _last_err;
  double   _conv_sum;
  uint32_t _conv_n;
  MprStats _st;

  MprSink  _sink;   void *_sink_ctx;
  MprBusLock _lock; MprBusUnlock _unlock; void *_lock_ctx;
  MprCallbacks _cb;
};

// ---------------------------------------------------------------- driver with sample ring buffer
// N must be a power of 2. Oldest sample is dropped when the buffer is full.
template <size_t N = 32>
class MPRBuffer : public MPR {
public:
  MPRBuffer(MprRange range, MprTransfer tf = MprTransfer::B, MprUnit unit = MprUnit::MMHG,
            uint8_t i2c_addr = MPR_I2C_ADDRESS, int8_t eoc_pin = -1, int8_t rst_pin = -1)
    : MPR(range, tf, unit, i2c_addr, eoc_pin, rst_pin), _overruns(0) {
    setSink(&MPRBuffer::sinkFn, this);
  }

  uint16_t avail() const { return (uint16_t)_rb.available(); }
  bool     pop(MprSample &s) { return _rb.pop(s) == 1; }
  uint32_t overruns() const { return _overruns; }
  void     clear() { _rb.clear(); }

private:
  static void sinkFn(void *ctx, const MprSample &s) {
    MPRBuffer *self = static_cast<MPRBuffer *>(ctx);
    if (self->_rb.available() >= N) { self->_overruns = self->_overruns + 1; }
    self->_rb.push(s, true);
  }

  RingBuffer<MprSample, N> _rb;
  volatile uint32_t _overruns;
};

#endif // MPR_H
