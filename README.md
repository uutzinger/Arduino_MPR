# UUtzinger_MPR

Arduino driver for Honeywell MPR / MPRLS I2C pressure sensors.

It differs from other implementations:
- non-blocking: `update()` in main loop
- End Of Conversion output can trigger interrupt to read data.
- status polling if EOC is not wired (or stops working)
- reset toggles RST pin
- automatic recovery (retry, sensor reset, bus restart hook).
- I2C bus sharing with other devices
- Data through a timestamped ring buffer.
- Errors reporting

For design details see [Design.md](Design.md).
Examples in `examples/`: 
- Basic
- EocInterrupt (interrupt driven, RST, recovery callbacks)
- Diagnostics (reads several datapoints and reports status and errors)
- SharedBus (several tasks on one I2C bus).

## Installation

Copy the folder to your Arduino `libraries` folder. It needs the library [UUtzinger_RingBuffer](https://github.com/uutzinger/RingBuffer).
Tested with the ESP32 Arduino core (Feather ESP32-S3); the code has no ESP32-only parts other than a few optional speed-ups, but other cores have not been built yet.

## Minimalistic program

```cpp
#include <Wire.h>
#include <MPR.h>

MPRBuffer<32> mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG);   // no EOC, no RST pin

void setup() {
  Wire.begin();                // the main program sets up Wire (pins, clock) first
  mpr.begin(Wire);
  mpr.start(0);                // 0 = back to back, or start(period_us)
}

void loop() {
  mpr.update();                // from loop(), never blocks
  MprSample s;
  while (mpr.pop(s)) { float p = mpr.toPressure(s); }
}
```

## Driver Setup

```cpp
MPRBuffer<N> mpr(range, transfer, unit, i2c_address, eoc_pin, rst_pin);
```

| Argument | Meaning | Default |
|---|---|---|
| `N` (template) | Samples held in the ring buffer. Must be a power of 2. Oldest sample is dropped when full. | 32 |
| `range` | `MprRange`, the pressure range of your part number, see below | required |
| `transfer` | `MprTransfer::A`, `B` or `C`, the transfer function letter of your part number | `B` |
| `unit` | Unit of the values returned by `toPressure()`: `PSI, KPA, PA, BAR, MBAR, MMHG, INHG, ATM, TORR` | `MMHG` |
| `i2c_address` | 7 bit address, `0x18` unless your part number says otherwise | `0x18` |
| `eoc_pin` | Pin connected to EOC, `-1` if not wired | `-1` |
| `rst_pin` | Pin connected to RST, `-1` if not wired | `-1` |

Example: `MPRLS0300YG00001BB` is range `0300YG`, address `0x18` (the digit before the last letter: 1), transfer function `B`:

```cpp
MPRBuffer<32> mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, EOC_PIN, RST_PIN);
```

**Range** names follow the part number (`01.6` and `02.5` are written `01_6`, `02_5`):

| Reference | Ranges |
|---|---|
| Absolute | `R_0001BA R_01_6BA R_02_5BA R_0100KA R_0160KA R_0250KA R_0015PA R_0025PA R_0030PA` |
| Gage, mbar | `R_0060MG R_0100MG R_0160MG R_0250MG R_0400MG R_0600MG` |
| Gage, bar | `R_0001BG R_01_6BG R_02_5BG` |
| Gage, kPa | `R_0006KG R_0010KG R_0016KG R_0025KG R_0040KG R_0060KG R_0100KG R_0160KG R_0250KG` |
| Gage, psi | `R_0001PG R_0005PG R_0015PG R_0030PG` |
| Gage, mmHg | `R_0300YG` |
| Other | `CUSTOM`, then call `setRange(p_max, native_unit)` |

**Transfer function** (output counts as a fraction of 2^24): A = 10 % to 90 %, B = 2.5 % to 22.5 %, C = 20 % to 80 %.

**I2C address**: the sensor family exists with addresses `0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78`, selected by the part number.

**Pins**: the driver sets `pinMode` itself. The RST pin is driven high first thing in `begin()`. Use an interrupt capable pin for EOC; if the pin cannot be used for interrupts the driver uses status polling instead.

## Lifecycle

```cpp
bool begin(TwoWire &wire = Wire, uint32_t clock_hz = 0);   // also: begin(TwoWire *wire, ...)
void end();
bool start(uint32_t interval_us = 0);   // 0 = back to back, else fixed period
void stop();
void update();                          // non-blocking, call often
bool reset();                           // RST pulse, blocks about 15 ms
```

- **`begin()`** configures the pins, resets the sensor (RST pin) or waits for start-up, and checks the status. It returns `false` if the sensor is not ready (no answer, not powered, memory error). Read `mpr.status()` and the error helpers to see why. The driver stays configured, so you can still call `start()` and let the recovery ladder try again.
  - It does **not** call `Wire.begin()`. Set up the bus (pins, clock, time-out) in your program first.
  - `clock_hz = 0` leaves the bus clock alone (recommended, especially on a shared bus). A non-zero value calls `setClock()` and affects every device on that bus.
- **`start(interval_us)`**: `0` runs conversions back to back (about 160 to 200 samples/s). A non-zero value starts a conversion every `interval_us` microseconds, measured from start to start (no drift). If the loop is late the driver does not try to catch up.
- **`update()`** is the only function that talks to the sensor after `begin()`. Call it often from `loop()` or from a task you create. It handles interrupt from EOC. It does at most one short I2C transaction per call and never waits. The sampling rate is only reached if `update()` is called at least every few hundred microseconds to a millisecond; in EOC mode the exact time of each sample is still captured by the interrupt.
- **`stop()`** halts conversions and detaches the interrupt; **`end()`** also forgets the `TwoWire`. `start()` can be called again afterwards.
- **`reset()`** needs an RST pin (returns `false` otherwise). It blocks for about 15 ms (5 ms low, 5 ms wait, then waits for the power bit). The driver also calls it by itself during recovery.

Sampling mode: `mpr.mode()` returns `STOPPED`, `EOC_INTERRUPT` or `STATUS_POLL`. It switches from `EOC_INTERRUPT` to `STATUS_POLL` by itself if the EOC edge does not arrive (`stats().eoc_timeouts`), and tries the interrupt again later.

## Pressure Zero and Range

`toPressure()` returns

```
pressure = (counts - zero) * (P_MAX - P_MIN) / (MAX_COUNTS - MIN_COUNTS) * unit_factor * cal
```

`zero` is the nominal `MIN_COUNTS` of the transfer function unless you set it.

```cpp
void     setZero(float counts);                    // counts at zero pressure (measure with port open to air)
float    zero() const;
void     setCal(float k);                          // multiplies the result, default 1.0
void     setCountsRange(uint32_t mn, uint32_t mx); // override nominal MIN/MAX counts of the transfer function
void     setRange(float p_max, MprUnit native);    // for MprRange::CUSTOM: full scale value and its unit
uint32_t minCounts() const;
uint32_t maxCounts() const;
```

Real sensors have an offset, so measure the zero once the sensor is open to air and still, for example by averaging 100 readings, then store it:

```cpp
double sum = 0; int n = 0;
while (n < 100) {
  mpr.update();
  MprSample s;
  while (mpr.pop(s) && n < 100) { sum += s.counts; n++; }
}
mpr.setZero((float)(sum / n));
```

`setCountsRange()` is for a sensor whose measured counts at zero and at full scale differ from the nominal ones (for example a 0300YG that reads 413,800 at zero instead of 419,430). `setZero()` is usually enough.

## Data Access
A sample has following structure:
```cpp
struct MprSample {
  uint32_t counts;   // raw 24 bit sensor output
  uint32_t t_us;     // time stamp, micros()
  uint8_t  status;   // raw status byte of this reading
  uint8_t  pad;
  uint16_t err;      // warning bits only (see Error Handling)
};
```
Accessing number of samples in buffer, obtaining oldest sample in buffer, clearing buffer and conversion of counts to pressure:

```cpp
uint16_t avail() const;        // samples waiting in the ring buffer
bool     pop(MprSample &s);    // oldest sample, false if the buffer is empty
uint32_t overruns() const;     // samples dropped because the buffer was full
void     clear();              // empty the buffer
float    toPressure(const MprSample &s) const;
float    toPressure(uint32_t counts) const;
```
Example readout:
```cpp
MprSample s;
while (mpr.pop(s)) {
  float p = mpr.toPressure(s);
  Serial.printf("%lu,%.2f\n", (unsigned long)s.t_us, p);
}
```

- **Time stamp.** `t_us` is `micros()` at the end of the conversion. With an EOC interrupt it is the time of the interrupt, which is accurate to a few microseconds. When polling the status it is the time the driver noticed the sensor was ready, which is accurate to about the poll period (500 us) plus the delay until `update()` runs. `micros()` wraps after about 71.6 minutes; compute intervals with unsigned subtraction: `uint32_t dt = s.t_us - previous.t_us;`.
- **Only good readings are stored.** Readings with an error are dropped and counted in `stats()`, so gaps show in the time stamps.
- If you do not read the buffer fast enough the oldest samples are overwritten (`overruns()`).
- Without a buffer, use the class `MPR` directly and receive each good sample through a sink function (see Callbacks).

## Callbacks

Recovery callbacks are all optional. They are called from inside `update()`. They may use `Serial` and `Wire`, but should return quickly.

```cpp
void setRecoveryCallbacks(const MprCallbacks &cb);
```

Callbacks:
- onBeforeReset: before reset is executed
- onAfterReset: after reset was executed
- onBusRecover: should restart I2C
- onFailed: after all recovery attempts failed

```cpp
MprCallbacks cb = {};
cb.onBeforeReset = myBefore;   // void (void *ctx)         about to pulse RST
cb.onAfterReset  = myAfter;    // void (void *ctx, bool ok) sensor back or not; re-apply settings if needed
cb.onBusRecover  = myBus;      // bool (void *ctx)         restart the shared TwoWire (end / begin / clock / time-out), return true if done
cb.onFailed      = myFailed;   // void (void *ctx)         all recovery steps failed
cb.ctx           = nullptr;    // passed to every callback
mpr.setRecoveryCallbacks(cb);
```

Recovery ladder: a bad reading is retried on the next cycle; after 3 bad readings in a row (or a memory error, or a detected power-on reset) the sensor is reset through the RST pin; after 50 bad readings in a row `onBusRecover` is called; if that does not help the driver goes into a failed state and retries once per second. The first good reading ends it. Without an RST pin only the last two steps exist (the sensor has no soft reset).

## Error Handling

The driver reports what happened; your program decides if and when to print it.

```cpp
uint8_t  status() const;        // last raw status byte from the sensor (0xFF = bus did not answer)
uint16_t lastError() const;     // error bits of the last attempt, 0 = fine
uint8_t  lastI2CResult() const; // last Wire.endTransmission() code
MprMode  mode() const;
MprStats stats() const;
```

Error bits (`MPR::E_...`):

| Bit | Meaning |
|---|---|
| `E_CMD` | command write failed (no ACK) |
| `E_READ` | sensor returned too few bytes |
| `E_TIMEOUT` | sensor stayed busy too long |
| `E_BUS` | status `0xFF` or impossible bits: bus problem |
| `E_POWER` | sensor reports no power / power-on reset |
| `E_MEM` | memory integrity error (only a reset clears it) |
| `E_SAT` | math saturation: pressure beyond range |
| `E_BUSY` | data read while the sensor was still busy (discarded) |
| `E_RANGE` | **warning**: counts far outside the expected range, sample is kept |
| `E_EOC` | **warning**: EOC signal missing, polling was used, sample is kept |

Text helpers (static, no allocation, buffers are always null terminated, output is cut off if the buffer is too short):

```cpp
static const char *errorName(uint8_t bit);                            // "math saturation"
static const char *errorHint(uint8_t bit);                            // what to check
static size_t      errorString(uint16_t err, char *buf, size_t n);    // "not powered, memory error"; "ok" if none
static size_t      statusString(uint8_t status, char *buf, size_t n); // "0x41: powered, math saturation"
static const char *i2cResultString(uint8_t rc);                       // "NACK on address"
```

Example: report when the error changes.

```cpp
static uint16_t reported = 0;
uint16_t err = mpr.lastError();
if (err != reported) {
  char st[64], msg[128];
  MPR::statusString(mpr.status(), st, sizeof st);
  MPR::errorString(err, msg, sizeof msg);
  Serial.printf("MPR %s | %s\n", st, msg);
  reported = err;
}
```

`stats()` returns counters: `good`, `bad`, `err_hist[bit]` (how often each error bit occurred), `resets`, `bus_restarts`, `eoc_timeouts`, `eoc_early`, `eoc_low_seen`, `lock_failed`, and the conversion time `conv_min_us / conv_mean_us / conv_max_us`.

## Shared Bus

Several devices on one `TwoWire` work as long as they are used from one task. If they are used from different tasks, give the driver a lock so that its transactions cannot interleave with those of other tasks:

```cpp
void setBusLock(MprBusLock lock, MprBusUnlock unlock, void *ctx);
//  bool lock(void *ctx)    return false if the bus could not be obtained
//  void unlock(void *ctx)
```

The driver calls `lock` before and `unlock` after each single transaction. If `lock` returns `false` it skips that attempt and tries again at the next `update()`; this is not counted as a sensor error (`stats().lock_failed`).
The driver does not provide the mutex: you create one per `TwoWire`, and all other users of that bus must take the same mutex around their transactions. See `examples/SharedBus` for a complete FreeRTOS example. With several MPR sensors on one bus give all of them the same lock functions.

Other rules for sharing: the driver never calls `Wire.begin()` or `Wire.end()`, and changes the clock only if you pass a non-zero `clock_hz` to `begin()`. If the bus has to be restarted, the `onBusRecover` callback does it.

## Timing and other constants

Defaults can be changed at compile time (compiler flag `-DNAME=value`, for example in `platformio.ini` build flags or an Arduino `build.extra_flags`; a `#define` in the sketch is not seen by the library source file):

| Constant | Default | Meaning |
|---|---|---|
| `MPR_CONV_MIN_US` | 3000 | no end-of-conversion accepted / status polled earlier than this after the command |
| `MPR_POLL_US` | 500 | status poll period when not using EOC |
| `MPR_CONV_TIMEOUT_MS` | 20 | conversion time-out (datasheet worst case about 6.2 ms) |
| `MPR_EOC_TIMEOUT_MS` | 10 | no EOC edge within this time: fall back to polling |
| `MPR_EOC_RETRY_MS` | 10000 | try the EOC interrupt again after a fallback |
| `MPR_RST_LOW_MS`, `MPR_RST_WAIT_MS` | 5, 5 | reset pulse and wait |
| `MPR_RETRY_MAX` | 3 | consecutive bad readings before a reset |
| `MPR_BUS_RESTART_AFTER` | 50 | consecutive bad readings before `onBusRecover` |
| `MPR_RECOVER_MIN_MS` | 2000 | minimum time between resets / bus recoveries |

I2C helper constants: `MPR_I2C_FAST` (400 kHz), `MPR_I2C_REGULAR` (100 kHz), `MPR_I2C_SLOW` (50 kHz), `MPR_I2C_DEFAULTSTRETCH` (50 ms Wire time-out), `MPR_I2C_LONGSTRETCH` (200 ms).

## Tests

`extras/host_test/` contains tests that run the driver against a simulated sensor on a PC (no hardware needed). Run them after any change to the driver:

```bash
cd extras/host_test
g++ -std=c++17 -Wall -Wextra -I. -I../../src -I<path to UUtzinger_RingBuffer> \
    host_test.cpp ../../src/MPR.cpp -o /tmp/host_test -lm && /tmp/host_test
```

They do not replace testing with the real sensor: the exact EOC timing, conversion time and minimum reset pulse still have to be checked on the bench (see [Design.md](Design.md)).
