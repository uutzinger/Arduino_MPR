# TODO

Open items after the 1.3.0 release (see CHANGELOG.md). Nothing here is known to be broken; these are checks and decisions that need hardware, a target toolchain or a design decision.

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

## Target builds

- Done (1.3.0): library builds for Uno (AVR), Teensy 4 and ESP32 / ESP32-S3; the four ESP32 examples build for ESP32-S3.
- Still open, needs hardware: two-task test on ESP32. Sampling task calls `update()`, a second task reads `stats()` and the ring buffer. Confirm no EOC samples are lost or torn, and decide whether `stats()` needs a mutex wrapper in the SharedBus example.
- Not tried: other cores (RP2040, STM32, nRF52, ESP8266). The generic interrupt path is only compile-tested on Teensy 4 and AVR.

## Tests still to add (host)

- Two independent sensors (different addresses or buses, each with its own EOC pin). The mock has one simulated sensor and one interrupt handler; it needs a per-pin / per-address model.
- Interrupts arriving in the middle of a transaction (mock calls the handler only between `update()` calls).
- Hardware-in-the-loop version of the early-edge test once the real minimum conversion time is known.

## Design decisions pending

- **Non-blocking recovery (F02).** Only if item 4 under hardware checks shows the stall is a problem.
- The hardware checks above were not done before the 1.1.0 to 1.3.0 releases. Findings from them go into 1.3.x / 1.4.0.
