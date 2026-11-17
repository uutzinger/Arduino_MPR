# TODO

Open items after the 1.1.0 release (F01 to F12 fixes) (see CHANGELOG.md). Nothing here is known to be broken; these are checks and decisions that need hardware, a target toolchain or a design decision.

## Hardware checks (need a sensor and a logic analyzer)

Suggested order: do 1 first, because it can invalidate assumptions in the other items and in the F03 edge gate.

1. **EOC timing.** Capture command, EOC edge and data read on one trace.
   - Confirm EOC is low during conversion and rises at the end (RISING is used).
   - Measure when EOC falls again (at command or at data read). The design does not depend on it, but note it in Design.md.
   - Measure the real minimum and maximum conversion time over the whole supported range. Check that `MPR_CONV_MIN_US` (3000) is below the shortest conversion, otherwise genuine edges are rejected as `eoc_spurious`. The review used the 3000 us default against the datasheet, not a measurement.
   - Check for glitches on EOC (long wire, shared bus). Any `eoc_spurious` count in a normal run is worth investigating.
2. **Reset pulse.** Find the shortest RST low time and the shortest wait after release that always gives a valid status. Then shorten `MPR_RST_LOW_MS` / `MPR_RST_WAIT_MS` if it helps, since this reduces the longest `update()` stall (F02). Keep a safety factor of at least 2.
3. **Real recovery behavior.** Unplug the sensor, short SDA to ground, cut power briefly, hold RST low. Compare against the ladder in Design.md section 7 (reset, then `onBusRecover`, then FAILED). Confirm the sensor really reports power and memory bits as the datasheet says, and that BUSY plus fault combinations (F08) can occur.
4. **Longest update() time.** Measure `update()` duration during a successful reset, a failed reset and normal sampling, and decide whether the bounded stall of about 60 ms is acceptable for your applications. If not, a timed reset state machine (the alternative discussed for F02) becomes necessary.

## Target builds (need toolchains)

- Build all four examples for ESP32-S3. The host mock does not compile the `attachInterruptArg` path or the `portMUX` critical sections added for F05.
- Build the library (not the examples) for at least one non-ESP32 target, for example an AVR or RP2040 board. This exercises the generic four-slot interrupt trampolines and the `noInterrupts()` path. Examples use ESP32-specific APIs; either mark them as such in `library.properties` / README or make them portable.
- Run a two-task test on ESP32: sampling task calls `update()`, a second task reads `stats()` and the ring buffer. Confirm no EOC samples are lost or torn, and decide whether `stats()` needs a mutex wrapper in the SharedBus example.

## Tests still to add (host)

- Timer wrap: run the new tests (lock denial, early edge, delayed unlock) with the clock starting just below 2^32.
- Mock realism: short reads, lock waits that consume time during every transaction, interrupts arriving in the middle of a transaction.
- EOC retry after the 10 s fallback period; invalid interrupt pin (falls back to polling); fifth sensor on generic cores (four slots exhausted); two independent sensors on one or two buses.
- Define and test the maximum supported sampling interval for the signed deadline comparison (`(int32_t)(now - _next_start)`), and pathological compile-time timing overrides (for example `MPR_CONV_MIN_US` above `MPR_EOC_TIMEOUT_MS`; the existing `static_assert` only covers one pair).

## Input validation

- `setCountsRange()` with reversed bounds sets the scale to zero and can underflow the range-margin arithmetic. Decide: reject (return false) or swap. Same for non-finite calibration inputs (`setCal`, `setZero`). Low priority because it needs an invalid configuration.

## Design decisions pending

- **F10 exact overrun count.** Needs `push()` in UUtzinger_RingBuffer to report how many items it overwrote. Worth doing there if overrun accounting matters; otherwise keep the documented approximation.
- **Non-blocking recovery (F02).** Only if item 4 under hardware checks shows the stall is a problem.
- 1.1.0 was released before the hardware checks above. Findings from those checks go into 1.1.x / 1.2.0.
