# Changelog

Changes between 1.0.0 and 1.3.0. Items F01 to F12 refer to the findings of the code review ("Astra Code Review.md", 2026-11-16). Each entry gives the change and the reason for it.

## 1.3.0 - 2026-11-17

### Added

- **Builds on AVR (Arduino Uno) and Teensy 4 as well as ESP32.** Verified by compiling the library for `arduino:avr:uno`, `teensy:avr:teensy40` and `esp32:esp32:esp32` / `esp32s3`; the ESP32 build compiles the `portMUX` critical sections and `attachInterruptArg`, the Teensy build the generic four-slot interrupt trampolines. AVR needed RingBuffer 2.1.1 (no `<type_traits>` on avr-gcc), hence the new dependency `UUtzinger_RingBuffer (>=2.1.1)`.
- `examples/BasicPortable`: example that builds on any board (uses only `Serial.print`, no ESP32 APIs).

### Changed

- **`setZero()`, `setCal()`, `setCountsRange()` and `setRange()` validate their input and return `bool`.** Unusable values (NaN or infinity, zero `cal`, negative or out of 24 bit zero, reversed/empty/over 24 bit counts range, non-positive full scale) are rejected and the previous value is kept. *Reason:* a reversed counts range set the scale to zero and could underflow the range-margin arithmetic. Existing code that ignores the return value still compiles.
- The ESP32-specific examples (Basic, Diagnostics, EocInterrupt, SharedBus) stop with a clear `#error` on other boards instead of a list of unrelated compile errors; the README says which examples are portable.

- `start()` limits the interval to `MPR_MAX_INTERVAL_US` (30 min). *Reason:* deadlines are compared as signed 32 bit values, which is only correct for intervals below 2^31 us.
- Compile-time checks (`static_assert`) reject inconsistent timing overrides: `MPR_CONV_MIN_US` not below the EOC time-out or conversion time-out, a zero or too long `MPR_POLL_US`. *Reason:* e.g. `-DMPR_CONV_MIN_US=20000` made every genuine EOC edge unacceptable without any message.

### Tests

- The whole host suite now runs five times with the mock clock starting shortly before the 32 bit `micros()` wrap, so every timing test crosses the wrap. The mock `millis()` keeps counting across the wrap, like the real one.
- New host tests: short data read, EOC fallback followed by the interrupt retry after `MPR_EOC_RETRY_MS`, a pin without interrupt support, exhaustion of the four interrupt slots, setter validation, a very long interval across the wrap.

## 1.2.0 - 2026-11-17

### Changed

- **`MPRBuffer::overruns()` is exact (F10).** The sink uses the new `overwritten` output of `RingBuffer::push()` (UUtzinger_RingBuffer 2.1.0), which is determined in the push's own critical section. *Reason:* the old full check followed by a separate push could count a sample as lost when a consumer popped in between. **Requires UUtzinger_RingBuffer >= 2.1.0** (`depends` updated).

### Tests

- New host test for the overrun count of a full `MPRBuffer`.

## 1.1.0 - 2026-11-17

### Fixed

- **A denied bus lock no longer causes a conversion time-out (F01).**
  A denied lock on the status poll now returns without evaluating the time-out, and the "recently blocked" check uses a fresh `micros()`.
  *Reason:* the check subtracted a time stamp taken later than `now`, which wrapped the unsigned result, so contention on a shared bus (the SharedBus example's lock can wait 5 ms) was reported as `E_TIMEOUT` and could advance the recovery ladder. This contradicted the design rule that lock failures are not sensor errors.

- **Command time stamp is taken at the end of the command (F04).**
  `_t_cmd` is recorded in `writeCommand()` right after `endTransmission()`, before the unlock callback. Negative durations are no longer added to the conversion statistics, and the mean is computed from the number of recorded durations.
  *Reason:* with the old order a task switch or slow unlock callback could let the EOC edge arrive before the baseline, producing a conversion time of about 4.29e9 us.

- **Early or stale EOC edges are rejected (F03).**
  The edge's own time is compared with the command time; edges earlier than `MPR_CONV_MIN_US` (or before the command) are discarded and counted in the new `stats().eoc_spurious`. The driver keeps waiting, with the time-out and polling fallback as backstop.
  *Reason:* the old gate tested the current time, so a spurious early edge was accepted as the sample time stamp whenever `update()` ran late.

- **EOC flag and time are read as a pair (F05).**
  `update()` reads the flag and time stamp in a short critical section (ESP32 spinlock, `noInterrupts()` elsewhere); the flag is cleared only if no newer edge replaced the one inspected.
  *Reason:* `volatile` does not make a 32-bit time stamp read atomic on 8-bit targets, and a repeated edge could replace the time while it was consumed. `stats()` and `lastError()` remain unsynchronized by design; this is now documented (use them from the `update()` task or protect them with your own mutex).

- **Failed `begin()` and `reset()` report why (F06).**
  Every failure sets `lastError()` and `status()` (`E_READ` and `0xFF` for no answer, `E_BUS`, `E_POWER`, `E_MEM`, `E_BUSY`, `E_TIMEOUT`); success clears the error. A denied lock changes neither. Automatic recovery resets no longer overwrite the error that triggered them.
  *Reason:* after unplugging the sensor, `begin()` returned false while `status()` still said powered and `lastError()` said success.

- **Lifecycle calls no longer command a busy sensor (F07).**
  `start()` while sampling only changes the interval and leaves the running conversion alone. `begin()` and a failed `reset()` keep the "check for idle first" requirement; `begin()` without an RST pin now fails if the sensor reports busy.
  *Reason:* the sensor ignores commands while busy, so a restarted conversion would be associated with the old conversion's data and timing. The Diagnostics example triggered this when zeroing.

- **Power and memory faults take priority over BUSY (F08).**
  A busy status that also reports `E_POWER` or `E_MEM` is handled as that fault immediately, in both the EOC and the polling path. Pressure data of a busy reading is still discarded.
  *Reason:* the old code saw only BUSY, waited, and finally reported a time-out, delaying the required reset.

- **A successful bus recovery no longer reports FAILED (F09).**
  The result of `onBusRecover` is used. If it returns true the driver retries at 1 Hz and calls `onFailed()` only if readings are still bad after the next cooldown. If the callback is missing or returns false, the driver goes to FAILED immediately as before.
  *Reason:* `onFailed()` fired in the same call as the recovery, before any communication could show whether the repair worked.

### Added

- `MPR::clearStats()` to reset counters and conversion statistics (F11).
- `MprStats::eoc_spurious`, count of EOC edges discarded as too early (F03).

### Changed

- `MPRBuffer::sinkFn` no longer uses `++` on the volatile overrun counter (avoids the C++20 deprecation warning).
- Documentation (README, Design.md) now states the real `update()` latency contract (F02): normal operation is non-blocking, a completing sample performs read plus next command in one call, and automatic sensor reset and bus recovery can block for up to about 60 ms, at most once per `MPR_RECOVER_MIN_MS`. Conversion time is measured from the STOP condition of the command. `lastI2CResult()` covers command writes only.
- Diagnostics example: uses `clearStats()` so results describe the current run, and reports the EOC check as SKIP when no pin, no interrupt support or no successful reading makes it inconclusive (F11).
- Diagnostics and EocInterrupt examples read the sampling mode after `start()` and print all three modes (F12). Before `start()` the mode is always `STOPPED`.

### Tests

- Host mock sensor now ignores commands while busy, like the real sensor.
- New host tests: denied lock past the deadline (with a lock that takes time), delayed unlock callback, early EOC edge, failed `begin()` reporting, no command while busy across lifecycle calls, busy plus memory fault, successful bus recovery, `clearStats()`. Each new test fails on the 1.0.0 code.
