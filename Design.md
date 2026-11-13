# MPR Pressure Sensor I2C Driver: Final Design

Honeywell MPR / MPRLS series, I2C variant. Arduino framework, ESP32-S3 first, portable to other cores.
Class `MPR` and template `MPRBuffer<N>`, files `src/MPR.h` (both classes), `src/MPR.cpp`.

---

## 1. Goals and non-goals

Goals
1. Non-blocking sampling. Interrupt driven when the EOC pin is wired; the same state machine runs on time plus status polling when it is not.
2. Every stored sample carries raw counts, a timestamp and the status byte. Faulty readings are never stored, only counted.
3. Hardware reset through the RST pin when available, with an automatic recovery ladder and optional application callbacks.
4. I2C at 100 kHz or 400 kHz. The application owns the bus; the driver is handed a `TwoWire` instance (reference or pointer) and can share it with other drivers and other devices on the same bus (section 5a).
5. The ISR does nothing but record a timestamp and set a flag.
6. Errors are exposed as raw status plus error bits. The driver provides functions that turn them into human-readable text; it never prints and has no `Serial` dependency.

Non-goals: SPI variant, temperature output (the MPR has none), sleep modes (the sensor returns to standby by itself), RTOS objects (no tasks, queues or notifications are created or required).

---

## 2. Public API

Ring buffer: `UUtzinger_RingBuffer` (`../UUtzinger_RingBuffer/RingBuffer.h`), `RingBuffer<T, N>`. N must be a power of 2, T trivially copyable. It is protected by a per-instance spinlock on ESP32, so `update()` may push while another task pops. The buffer size is a template parameter, so the driver is split into a non-template core (`MPR`) and a buffered wrapper (`MPRBuffer<N>`).

```cpp
enum class MprRange    : uint8_t { /* part-number enum, section 8, plus CUSTOM */ };
enum class MprTransfer : uint8_t { A, B, C };
enum class MprUnit     : uint8_t { PSI, KPA, PA, BAR, MBAR, MMHG, INHG, ATM, TORR };
enum class MprMode     : uint8_t { STOPPED, EOC_INTERRUPT, STATUS_POLL };

struct MprSample {              // trivially copyable, static_assert(sizeof == 12)
  uint32_t counts;              // raw 24 bit output
  uint32_t t_us;                // micros() at EOC interrupt, or at ready detection when polling
  uint8_t  status;              // raw status byte of this reading
  uint8_t  pad;
  uint16_t err;                 // warning bits only (E_RANGE, E_EOC)
};

struct MprStats {
  uint32_t good, bad;
  uint32_t err_hist[10];        // readings with each error bit set (index = bit number)
  uint32_t resets, bus_restarts;
  uint32_t eoc_timeouts;        // EOC edge did not arrive, polling used instead
  uint32_t eoc_early;           // EOC edge arrived but status still busy
  uint32_t eoc_low_seen;        // EOC pin low although status said ready
  uint32_t lock_failed;         // bus lock could not be acquired, attempt skipped
  uint32_t conv_min_us, conv_max_us;
  float    conv_mean_us;
};

typedef void (*MprSink)(void *ctx, const MprSample &s);   // receives every good sample
typedef bool (*MprBusLock)(void *ctx);     // acquire the shared bus; false = could not, skip this attempt
typedef void (*MprBusUnlock)(void *ctx);   // release the shared bus
```

```cpp
class MPR {
public:
  MPR(MprRange range, MprTransfer tf = MprTransfer::B, MprUnit unit = MprUnit::MMHG,
      uint8_t i2c_addr = 0x18, int8_t eoc_pin = -1, int8_t rst_pin = -1);

  // Lifecycle
  bool     begin(TwoWire &wire = Wire, uint32_t clock_hz = 0);   // clock_hz 0 = do not touch clock or time-out (shared bus)
  bool     begin(TwoWire *wire, uint32_t clock_hz = 0);          // same, for a pointer held by the main program
  void     setBusLock(MprBusLock lock, MprBusUnlock unlock, void *ctx);  // optional, section 5a
  void     end();                         // detach ISR, stop sampling
  bool     start(uint32_t interval_us = 0);  // 0 = back to back (max rate), else fixed period in us
  void     stop();                        // disable ISR, no new conversions
  void     update();                      // non-blocking, call from loop() or from your own task
  bool     reset();                       // RST pulse, blocks about 15 ms; false if no RST pin or sensor did not return

  // State, for the application's own reporting
  uint8_t  status() const;                // last raw status byte (0xFF if the bus did not answer)
  uint16_t lastError() const;             // error bits of the last attempt (section 6)
  uint8_t  lastI2CResult() const;         // last Wire.endTransmission() code
  MprMode  mode() const;
  MprStats stats() const;

  // Human-readable text, static, no allocation, no Serial (section 6)
  static const char *errorName(uint8_t bit);                          // "bus reads all ones (0xFF)"
  static const char *errorHint(uint8_t bit);                          // likely cause and what to check
  static size_t      errorString(uint16_t err, char *buf, size_t n);  // "not powered, math saturation"
  static size_t      statusString(uint8_t status, char *buf, size_t n); // "0x41: powered, math saturation"
  static const char *i2cResultString(uint8_t rc);                     // "NACK on address"

  // Conversion
  float    toPressure(uint32_t counts) const;
  float    toPressure(const MprSample &s) const { return toPressure(s.counts); }
  void     setZero(float counts);   float zero() const;
  void     setCal(float k);                          // pressure calibration factor, default 1.0
  void     setCountsRange(uint32_t mn, uint32_t mx); // per-sensor measured min/max counts
  void     setRange(float p_max, MprUnit native);    // for MprRange::CUSTOM

  // Hooks
  void     setSink(MprSink fn, void *ctx);           // used by MPRBuffer
  void     setRecoveryCallbacks(const MprCallbacks &cb);   // section 7
};

template <size_t N = 32>
class MPRBuffer : public MPR {          // owns RingBuffer<MprSample, N>, installs itself as sink
public:
  using MPR::MPR;
  uint16_t avail() const;                 // samples in the buffer
  bool     pop(MprSample &s);             // oldest sample; false if empty
  uint32_t overruns() const;              // samples dropped because the buffer was full
private:
  RingBuffer<MprSample, N> _rb;           // push(s, overwrite = true): oldest is dropped when full
};
```

Notes
- Default ring buffer size is 32 samples (384 bytes per sensor).
- `begin()` never calls `Wire.begin(sda, scl)`. The application sets up the bus (for example an ESP32 `TwoWire(0)` with custom pins). With the default `clock_hz = 0` it does not change clock or time-out either; it only does so when asked (section 5a).
- Pin defaults live in the application, not in the library.
- `MPR` can be used without a buffer by installing a sink with `setSink()`.
- Several sensors can coexist (different address or bus, own EOC pin, own instance state), and other I2C devices can use the same `TwoWire` (section 5a).

Typical application loop:

```cpp
mpr.update();
MprSample s;
while (mpr.pop(s)) { use(mpr.toPressure(s), s.t_us); }

uint16_t err = mpr.lastError();
if (err != lastReported) {                 // application decides when and how to report
  char msg[96];
  MPR::errorString(err, msg, sizeof msg);
  Serial.printf("MPR status %s | %s\n", statusText, msg);
}
```

---

## 3. Modes

Selected in `begin()`:

| Mode | Condition | End-of-conversion detection |
|---|---|---|
| `EOC_INTERRUPT` | EOC pin given and it supports interrupts | RISING-edge ISR sets a per-instance flag |
| `STATUS_POLL` | no EOC pin | wait a fixed minimum time, then poll the status byte |
| fallback | EOC configured, but no edge within `EOC_TIMEOUT_MS` | switch to `STATUS_POLL` for this and following samples, count `eoc_timeouts`, set warning `E_EOC` on every sample delivered this way. After `EOC_RETRY_MS` the interrupt is tried again, but only if EOC reads high when status reports ready (a retry that fails again costs one `EOC_TIMEOUT_MS` delay). `reset()` also clears the fallback |

ISR, per instance (`attachInterruptArg` on ESP32, static trampoline table on other cores):

```cpp
void IRAM_ATTR MPR::isr(void *arg) {
  MPR *m = static_cast<MPR*>(arg);
  m->_eoc_us   = micros();
  m->_eoc_flag = true;               // volatile
}
```

The ISR performs no I2C, no printing and no buffer access.

Edge/race handling
1. Clear the flag immediately before writing the conversion command, never after.
2. Accept an EOC flag only in state `CONVERTING` and only after `CONV_MIN_US` has elapsed since the command (rejects glitches and stale edges).
3. The data read is the final authority: if its status byte still has BUSY set, discard the data, stay in `CONVERTING` and continue by status polling for this sample (`eoc_early`).
4. To be verified on hardware with a logic analyzer: the exact time at which EOC falls again (at command write or at data read). The design does not depend on it, because of rules 1 to 3.

---

## 4. State machine in update()

```
            start()
               |
               v
   +-----> IDLE --(interval elapsed)--> write 0xAA 00 00 ---> CONVERTING
   |                          write failed: E_CMD --------+        |
   |                                                      |        |
   |   CONVERTING                                         |        |
   |     EOC_INTERRUPT: wait for flag                     |        |
   |     STATUS_POLL  : wait CONV_MIN_US, then one 1-byte |        |
   |                    status read per POLL_US           |        |
   |     both: CONV_TIMEOUT_MS exceeded -> E_TIMEOUT -----+        |
   |                                                      |   ready|
   |                          read 4 bytes in one transaction      v
   |                          decode status, check range
   |                          good / warning only -> sink(sample)
   |                          bad -> count, nothing stored
   +--------------------------------------------+                  |
                                                 ERROR <-----------+  -> recovery ladder (section 7)
```

Rules
- `update()` never waits. At most one bus transaction per call, plus the data read when ready.
- The next conversion is started in the same call that read the data (back to back, roughly 160 to 200 samples/s), or after `interval_us` measured from the previous start, so the rate does not drift with I2C time. A caller that is late by more than one interval does not catch up.
- If the bus lock is not available (`lock_failed`), the attempt is skipped and repeated on the next `update()`. An EOC flag stays set until the data has really been read. The conversion time-out does not fire while the lock was denied within the last `CONV_TIMEOUT_MS`, so another device holding the bus cannot cause resets.
- Never send a new command while the last status had BUSY set (the sensor ignores commands while busy). After a time-out read the status first; if still busy go to recovery.
- Timestamp `t_us`: ISR time in interrupt mode; time of the successful ready detection in polling mode.
- Result is read with a single 4-byte transaction: status, data[23:16], data[15:8], data[7:0]. No separate status read in interrupt mode.

Timing constants (all overridable at compile time with a compiler flag such as `-DMPR_CONV_MIN_US=3500`; a `#define` in the sketch is not seen by `MPR.cpp`, which is compiled separately):

```
CONV_MIN_US         3000    // no status poll earlier than this after the command
POLL_US              500    // status poll period
CONV_TIMEOUT_MS       20    // datasheet worst case is about 6.2 ms
EOC_TIMEOUT_MS        10    // no EOC edge: fall back to status polling (must be < CONV_TIMEOUT_MS)
EOC_RETRY_MS       10000    // after a fallback, try the interrupt again after this time (and after every reset())
RETRY_GAP_US        1000    // pause after a bad reading
FAILED_GAP_US    1000000    // retry period in the FAILED state
STARTUP_MS            10    // begin() without RST pin
RST_LOW_MS             5    // reset pulse
RST_WAIT_MS            5    // wait after releasing reset
RETRY_MAX              3
BUS_RESTART_AFTER     50
RECOVER_MIN_MS      2000
```

The driver measures conversion time (command to ready) and reports min/mean/max in `stats()` so `CONV_MIN_US` can be tuned per sensor.

---

## 5. Transactions

| Purpose | I2C |
|---|---|
| Start conversion | write `0xAA 0x00 0x00` |
| Status poll | read 1 byte |
| Result | read 4 bytes: status, data[23:16], data[15:8], data[7:0] |

Constants

```
I2C_ADDRESS            0x18      // other addresses depend on part number
I2C_FAST             400000
I2C_REGULAR          100000
I2C_SLOW              50000
I2C_DEFAULTSTRETCH       50      // ms Wire time-out, sensor typically needs 230 us
I2C_LONGSTRETCH         200
SDA 3, SCL 4                     // ESP32-S3 Feather defaults, only if not predefined
```

- Pull-ups 1 to 50 kOhm (datasheet). The driver cannot check them; a missing or weak pull-up shows as `E_BUS`.
- 400 kHz is supported by the sensor. The slowest device on the bus decides the speed, so the clock is chosen by the application.
- A stuck bus can cost up to the Wire time-out inside one `update()` call. Recovery step 3 exists for this case.

---

## 5a. Shared I2C bus

The main program creates and owns the `TwoWire` instance and hands the same instance to every driver:

```cpp
TwoWire myWire = TwoWire(0);
myWire.begin(SDA, SCL);  myWire.setClock(400000);  myWire.setTimeOut(50);   // main program decides, once

MPRBuffer<32> mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, 0x18, EOC_PIN, RST_PIN);
mpr.begin(myWire);                 // or mpr.begin(&myWire)
OtherSensor other;  other.begin(myWire);
```

Rules the driver follows so that it can coexist with other devices and drivers:
1. **Does not own the bus.** It stores only a pointer to the `TwoWire` it was given. `end()` and recovery never call `Wire.end()` themselves.
2. **Does not change shared settings by default.** `begin(wire)` leaves clock and time-out alone. `begin(wire, clock_hz)` with a non-zero value is an explicit request to set the clock, and is documented as affecting every device on the bus. The clock must be a speed all devices on the bus support; the main program is responsible for that.
3. **Short, self-contained transactions.** Every driver call is a complete write or a complete read (`beginTransmission/endTransmission`, or `requestFrom` followed immediately by reading the bytes). The driver never holds the bus open between calls, never leaves a transaction half done, and never uses a repeated start across `update()` calls. Other drivers can therefore interleave their own transactions between ours.
4. **Non-blocking cost is bounded.** One `update()` call costs at most one command write (3 bytes) or one 4-byte read, so it adds a few hundred microseconds at 100 kHz to the loop. Main programs that need strict timing for other devices can call `update()` at a point of their choosing.
5. **Optional bus lock.** If `update()` and other drivers run in different tasks, the main program supplies `setBusLock(lock, unlock, ctx)` (typically a FreeRTOS mutex shared by all drivers on this bus). The driver calls `lock` before and `unlock` after each transaction (never across several transactions, never from the ISR). If `lock` returns false (bus busy longer than the main program allows), the driver skips this attempt without counting it as a sensor error and tries again on the next `update()`. Without a lock the driver assumes all bus access happens from one task.
6. **No bus access from the ISR.** The EOC interrupt only sets a flag, so it cannot disturb a transaction that another driver has in progress. The data read happens later in `update()`, after the lock is acquired.
7. **Address conflicts.** The I2C address is a constructor argument. The driver does not scan the bus. A device at the same address on the same bus is a main-program error and shows up as `E_BUS` / garbage status.
8. **Bus recovery is the main program's job.** The sensor driver cannot restart a bus other devices depend on. When recovery step 3 is reached, `onBusRecover` is called; the main program restarts the `TwoWire` (and restores clock and time-out) and may notify its other drivers. Without the callback only `E_BUS` is raised. A failure that is clearly caused by another device holding the bus (lock never acquired) is counted in `stats().lock_failed` and does not advance the recovery ladder.
9. **Multiple MPR sensors on one bus** (different addresses) work as separate instances. Conversion starts are independent, each has its own EOC interrupt, and a shared lock serializes their transactions.
10. **More than one `TwoWire`.** Different instances can use different `TwoWire` objects (for example `Wire` and `Wire1`); there is no global state in the driver.

Additional `MprStats` field: `uint32_t lock_failed;`

---

## 6. Status, errors and human-readable text

Status byte (datasheet): bit 6 powered, bit 5 busy, bit 2 memory integrity error, bit 0 math saturation. Bits 7, 4, 3, 1 are always 0.

Error bits (16 bit):

| Bit | Name | Meaning |
|---|---|---|
| 0x0001 | `E_CMD` | command write failed (NACK or bus error) |
| 0x0002 | `E_READ` | read returned fewer bytes than requested |
| 0x0004 | `E_TIMEOUT` | still busy after `CONV_TIMEOUT_MS` |
| 0x0008 | `E_BUS` | status 0xFF, or bits that must be 0 are set: bus reads all ones or garbage |
| 0x0010 | `E_POWER` | status says not powered / in power-on reset |
| 0x0020 | `E_MEM` | memory integrity check failed (only a reset clears it) |
| 0x0040 | `E_SAT` | math saturation: pressure beyond range, or damaged sensor |
| 0x0080 | `E_RANGE` | counts more than 5 % of span outside MIN..MAX (**warning**, sample kept) |
| 0x0100 | `E_BUSY` | data read still busy, data discarded |
| 0x0200 | `E_EOC` | EOC edge missing or EOC low while status ready (**warning** if polling delivered the sample) |

Warning bits (`E_WARN = E_RANGE | E_EOC`) do not invalidate the sample; any other bit does.

Decode order for a reading:
1. `status == 0xFF` or `status & 0x9A` set: `E_BUS`, nothing else decoded.
2. Power bit clear: `E_POWER`. If the power bit was seen set before and is now clear, a power-on reset happened: go straight to recovery step 2.
3. Busy bit set in the data read: `E_BUSY`.
4. Memory bit: `E_MEM`. Go straight to recovery step 2.
5. Saturation bit: `E_SAT`.
6. Counts outside MIN..MAX by more than 5 % of span: `E_RANGE`.

Human-readable reporting (no printing inside the driver). The application reads `status()`, `lastError()`, `lastI2CResult()` and `stats()`, decides when to report (on change, periodically, on clear) and uses these static helpers to get text:

| Function | Example output |
|---|---|
| `errorName(bit)` | `"math saturation"` |
| `errorHint(bit)` | `"pressure beyond range; with the port open to air this suggests a damaged sensor"` |
| `errorString(err, buf, n)` | `"not powered, memory error"` (comma separated names of all set bits, `"ok"` if none) |
| `statusString(status, buf, n)` | `"0x41: powered, math saturation"`; `"0xFF: bus reads all ones"` |
| `i2cResultString(rc)` | `"NACK on address"` (`ok`, `data too long`, `NACK on address`, `NACK on data`, `other error`, `timeout`) |

All functions are bounds-checked, null-terminate the buffer and return the length written.

---

## 7. Reset and recovery

`reset()` requires an RST pin:

```
digitalWrite(RST, LOW);   delay(RST_LOW_MS);       // 5 ms
digitalWrite(RST, HIGH);  delay(RST_WAIT_MS);      // 5 ms (datasheet: first command after 1 ms, first measurement after 2.5 ms)
poll status until power bit set and no memory error, timeout 50 ms -> true
```

The datasheet gives no minimum reset pulse width; 5 ms is conservative and can be shortened after measuring. There is no soft reset command, so without an RST pin only an external power cycle resets the sensor and the ladder ends at step 3.

`begin()` drives RST high first (so a floating pin cannot hold the sensor in reset), then pulses reset if an RST pin exists, otherwise waits `STARTUP_MS`; in both cases it then reads the status and returns false unless the sensor answers with the power bit set and no error.

Recovery ladder, entered from the ERROR state:

| Step | Trigger | Action |
|---|---|---|
| 1 | any single bad reading | count, retry on the next cycle |
| 2 | `RETRY_MAX` consecutive bad readings, or `E_MEM`, or a detected power-on reset | with RST pin: `onBeforeReset`, `reset()`, `onAfterReset`, restart sampling |
| 3 | `BUS_RESTART_AFTER` consecutive bad readings (lock failures do not count) and at least `RECOVER_MIN_MS` since the last restart | `onBusRecover()`. If the callback is missing, keep `E_BUS` set and retry at 1 Hz |
| 4 | still failing | state `FAILED`, retry at 1 Hz, `onFailed()` once; counters visible in `stats()`. Sampling resumes automatically on the first good reading |

Callbacks (all optional; called from `update()`, never from the ISR, never during a bus transaction):

```cpp
struct MprCallbacks {
  void (*onBeforeReset)(void *ctx);              // e.g. pause other users of the bus
  void (*onAfterReset)(void *ctx, bool ok);      // re-apply setZero / setCal if needed
  bool (*onBusRecover)(void *ctx);               // application restarts the shared TwoWire (end / begin / clock / time-out) and informs its other drivers; true if done
  void (*onFailed)(void *ctx);                   // step 4 reached
  void *ctx;
};
```

Without callbacks the driver still performs steps 1 and 2 (if RST exists) and counts everything.

---

## 8. Configuration

All configuration is run-time (constructor arguments). Compile-time macros exist only for timing constants.

### Pressure range (P_MIN is always 0)

Part-number enum (`MprRange::R_<code>`, checked against the datasheet ordering table), mapped by a constexpr table to `{P_MAX, native unit}`. `01.6` / `02.5` are written `01_6` / `02_5`.

| Reference | Codes (P_MAX, native unit) |
|---|---|
| Absolute (`...A`) | `0001BA` 1 bar, `01_6BA` 1.6 bar, `02_5BA` 2.5 bar, `0100KA` 100 kPa, `0160KA` 160 kPa, `0250KA` 250 kPa, `0015PA` 15 psi, `0025PA` 25 psi, `0030PA` 30 psi |
| Gage (`...G`), mbar | `0060MG`, `0100MG`, `0160MG`, `0250MG`, `0400MG`, `0600MG` |
| Gage, bar | `0001BG` 1, `01_6BG` 1.6, `02_5BG` 2.5 |
| Gage, kPa | `0006KG`, `0010KG`, `0016KG`, `0025KG`, `0040KG`, `0060KG`, `0100KG`, `0160KG`, `0250KG` |
| Gage, psi | `0001PG` 1, `0005PG` 5, `0015PG` 15, `0030PG` 30 |
| Gage, mmHg | `0300YG` 300 |
| `CUSTOM` | `setRange(p_max, native)` |

Absolute and gage parts use the same counts-to-pressure math; the reference only changes what zero means physically.

### Transfer function (counts)

| TF | MIN counts | MAX counts | % of 2^24 |
|---|---|---|---|
| A | 0x19999A (1,677,722) | 0xE66666 (15,099,494) | 10 to 90 |
| B | 0x066666 (419,430) | 0x399999 (3,774,873) | 2.5 to 22.5 |
| C | 0x333333 (3,355,443) | 0xCCCCCC (13,421,772) | 20 to 80 |

`setCountsRange(min, max)` overrides the nominal values with a per-sensor measurement (for example a 0300YG that reads 413,800 at zero instead of 419,430).

### Unit conversion

```
p_native = (counts - zero_or_MIN) * (P_MAX - P_MIN) / (MAX - MIN) [+ P_MIN when using MIN]
p_out    = p_native * factor(native -> requested) * cal
```

`zero` is set by `setZero(counts)`: the application measures it (open to air, sensor still) and the driver subtracts it instead of MIN. Without `setZero`, MIN is the zero. `cal` (default 1.0) is `setCal()`.

Conversion factors are floats computed through Pa as the common base:

```
psi 6894.757 Pa   bar 100000 Pa   mbar 100 Pa   kPa 1000 Pa   mmHg/torr 133.3224 Pa   inHg 3386.389 Pa   atm 101325 Pa
```

---

## 9. Pins

```
EOC pin: -1 = none          constructor argument
RST pin: -1 = none          constructor argument
```

- RST: `OUTPUT`, driven HIGH first thing in `begin()`.
- EOC: `INPUT`; the sensor output is push-pull, no pull-up needed.

---

## 10. Files, dependencies, tests

```
src/MPR.h, src/MPR.cpp       core driver, MPRBuffer<N> wrapper, error text helpers
examples/Basic/              status polling, prints pressure
examples/EocInterrupt/       interrupt mode, ring buffer, 100+ samples/s
examples/Diagnostics/        bus check, status, noise, conversion time, EOC check, zeroing, error reporting using the helpers of section 6
examples/SharedBus/          MPR plus another device in a different task, shared TwoWire with one mutex
library.properties           depends=UUtzinger_RingBuffer
```

Bench checks for the first implementation
1. Logic analyzer on SDA, SCL and EOC around a conversion: EOC edge timing (section 3, rule 4).
2. EOC pin configured but disconnected: the driver falls back to status polling and reports `E_EOC`.
3. Unplug the sensor during a run: `E_BUS`, recovery ladder, resumption after reconnect.
4. Pull RST low by hand: power-on-reset detection and automatic re-initialization.
5. Sample time jitter at 100 samples/s in interrupt mode (expected tens of microseconds).
6. Two sensors on one bus (different addresses), each with its own EOC interrupt.
8. Shared bus: another driver (or a test task) hammers the same `TwoWire` from a second task with a shared mutex; the MPR must keep sampling with no `E_BUS`, and `lock_failed` must stay 0 with a generous lock time-out. Repeat without a lock in a single-task program with two different devices interleaved.
9. `begin(wire)` with `clock_hz = 0` leaves a clock set by the main program unchanged (read back with `getClock()`).
7. Error text helpers: every error bit and the status values 0x00, 0x40, 0x41, 0x44, 0x60, 0xFF produce sensible strings; buffers shorter than the text are truncated safely.

---

## 11. Final decisions

1. Range as part-number enum, plus `CUSTOM`.
2. `start(0)` runs back to back, `start(us)` uses a fixed period.
3. Timestamps are stored with every sample; faulty readings are dropped and only counted.
4. Optional recovery callbacks; error reporting is done by the application from `status()`, `lastError()` and `stats()`, using the provided text helpers.
5. No RTOS usage inside the driver; the ISR only sets a flag and a timestamp.
6. Class `MPR`, files `src/MPR.h/.cpp`.
7. Reset: 5 ms low, 5 ms wait; to be refined by measurement.
8. Ring buffer: `UUtzinger_RingBuffer`, `RingBuffer<MprSample, N>`, N a power of 2, default 32, oldest sample dropped on overflow.
9. Shared bus: the main program owns the `TwoWire` and passes it by reference or pointer; the driver changes no shared bus settings unless asked, uses short complete transactions, supports an optional bus lock, and leaves bus restart to the main program (section 5a).

Implementation notes
- `begin()` returns false if the sensor is not ready (no answer, not powered, memory error) but leaves the driver configured, so `start()` and the recovery ladder still work.
- Conversion time in `stats()` is measured from the end of the command write to the EOC edge (interrupt) or to the successful status poll.
- Host tests with a simulated sensor: `extras/host_test/`.

Open items (hardware needed): EOC falling-edge timing, minimum reset pulse width, confirmation of the range table against the datasheet.
