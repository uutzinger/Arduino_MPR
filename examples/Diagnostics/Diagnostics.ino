// MPR diagnostics: bus, status, series of readings, offset, noise, conversion time, EOC, statistics.
// Sensor must be open to air and not moving. Send 'd' to repeat, 'z' to zero, 'h' for help.
#include <Wire.h>
#include <math.h>
#include <MPR.h>

#ifndef D10
  #define D10 10
#endif
#ifndef D11
  #define D11 11
#endif

#ifndef SDA
  #define SDA 3
#endif
#ifndef SCL
  #define SCL 4
#endif

constexpr int8_t   EOC_PIN       = D10;     // -1 if not wired
constexpr int8_t   RST_PIN       = D11;     // -1 if not wired
constexpr uint16_t DIAG_SAMPLES  = 200;
constexpr float    NOISE_WARN    = 1.0f;    // mmHg
constexpr float    NOISE_FAIL    = 5.0f;
constexpr uint32_t CONV_WARN_US  = 15000UL;

TwoWire myWire = TwoWire(0);
MPRBuffer<32> mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, MPR_I2C_ADDRESS, EOC_PIN, RST_PIN);

// Collects n good readings (waits at most 10 s). Returns number collected.
uint16_t collect(uint16_t n, double &mean, double &sd, uint32_t &cmin, uint32_t &cmax) {
  uint16_t got = 0;
  double s1 = 0, s2 = 0; uint32_t x0 = 0;
  cmin = 0xFFFFFFFFUL; cmax = 0;
  mpr.clear();
  uint32_t t0 = millis();
  while (got < n && (millis() - t0) < 10000UL) {
    mpr.update();
    MprSample s;
    while (mpr.pop(s) && got < n) {
      if (got == 0) { x0 = s.counts; }
      double d = (double)((int32_t)s.counts - (int32_t)x0);
      s1 += d; s2 += d * d; got++;
      if (s.counts < cmin) { cmin = s.counts; }
      if (s.counts > cmax) { cmax = s.counts; }
    }
  }
  mean = got ? (double)x0 + s1 / got : 0.0;
  sd   = (got > 1) ? sqrt(fmax(0.0, (s2 - s1 * s1 / got) / (got - 1))) : 0.0;
  return got;
}

void printErrors() {
  MprStats st = mpr.stats();
  for (uint8_t b = 0; b < MPR::ERROR_BITS; b++) {
    if (st.err_hist[b]) { Serial.printf("       %lu x %s\n         hint: %s\n", (unsigned long)st.err_hist[b], MPR::errorName(b), MPR::errorHint(b)); }
  }
}

void runDiagnostics() {
  uint8_t fails = 0, warns = 0;
  char msg[96];
  Serial.println("=== MPR diagnostics (open to air, sensor still) ===");

  // 1. Start-up, status
  bool ok = mpr.begin(myWire);
  MPR::statusString(mpr.status(), msg, sizeof(msg));
  if (ok) { Serial.printf("[PASS] Status: %s\n", msg); }
  else    { Serial.printf("[FAIL] Status: %s, I2C: %s\n", msg, MPR::i2cResultString(mpr.lastI2CResult())); fails++; }

  // 2. Readings (statistics cover this run only)
  mpr.clearStats();
  mpr.start(10000);
  MprMode mode = mpr.mode();                    // valid only after start()
  Serial.printf("       mode: %s\n", mode == MprMode::EOC_INTERRUPT ? "EOC interrupt" : mode == MprMode::STATUS_POLL ? "status poll" : "stopped");
  double mean, sd; uint32_t cmin, cmax;
  uint16_t got = collect(DIAG_SAMPLES, mean, sd, cmin, cmax);
  float pct = 100.0f * got / DIAG_SAMPLES;
  if      (got == DIAG_SAMPLES) { Serial.printf("[PASS] Readings: %u of %u\n", got, DIAG_SAMPLES); }
  else if (pct >= 90.0f)        { Serial.printf("[WARN] Readings: only %u of %u\n", got, DIAG_SAMPLES); warns++; }
  else                          { Serial.printf("[FAIL] Readings: only %u of %u\n", got, DIAG_SAMPLES); fails++; }
  printErrors();

  if (got > 0) {
    float perCount = mpr.toPressure((uint32_t)1 + mpr.minCounts()) - mpr.toPressure(mpr.minCounts());
    float off = (float)(mean - mpr.minCounts());
    if (fabsf(off * perCount) <= 6.0f) { Serial.printf("[PASS] Zero offset: mean %.0f counts, %+.2f mmHg\n", mean, off * perCount); }
    else { Serial.printf("[WARN] Zero offset: mean %.0f counts, %+.1f mmHg. Open to air?\n", mean, off * perCount); warns++; }

    if (cmax == cmin && got > 10) { Serial.printf("[FAIL] Counts constant at %lu: output stuck\n", (unsigned long)cmin); fails++; }

    float noise = (float)sd * perCount;
    if      (noise <= NOISE_WARN) { Serial.printf("[PASS] Noise: std %.1f counts (%.2f mmHg)\n", sd, noise); }
    else if (noise <= NOISE_FAIL) { Serial.printf("[WARN] Noise: std %.1f counts (%.2f mmHg) is high\n", sd, noise); warns++; }
    else                          { Serial.printf("[FAIL] Noise: std %.1f counts (%.2f mmHg) is very high\n", sd, noise); fails++; }
  }

  // 3. Conversion time, EOC
  MprStats st = mpr.stats();
  if (st.good) {
    if (st.conv_max_us <= CONV_WARN_US) { Serial.printf("[PASS] Conversion: %lu..%lu us, mean %.0f us\n", (unsigned long)st.conv_min_us, (unsigned long)st.conv_max_us, st.conv_mean_us); }
    else { Serial.printf("[WARN] Conversion up to %lu us, mean %.0f us\n", (unsigned long)st.conv_max_us, st.conv_mean_us); warns++; }
  }
  if (EOC_PIN < 0) { Serial.println("[SKIP] EOC pin not configured"); }
  else if (mode != MprMode::EOC_INTERRUPT) { Serial.println("[SKIP] EOC pin has no interrupt support, status polling is used"); }
  else if (st.good == 0) { Serial.println("[SKIP] EOC: no successful reading, inconclusive"); }
  else {
    if (st.eoc_timeouts == 0 && st.eoc_low_seen == 0) { Serial.println("[PASS] EOC pin signalled every conversion"); }
    else { Serial.printf("[FAIL] EOC: %lu time-outs, %lu times low while ready. Check the EOC wire.\n", (unsigned long)st.eoc_timeouts, (unsigned long)st.eoc_low_seen); fails++; }
  }

  if      (fails) { Serial.printf("=== Result: FAIL (%u failed, %u warnings) ===\n", fails, warns); }
  else if (warns) { Serial.printf("=== Result: PASS with %u warnings ===\n", warns); }
  else            { Serial.println("=== Result: PASS ==="); }
}

void zeroSensor() {
  double mean, sd; uint32_t cmin, cmax;
  mpr.start(10000);
  if (collect(100, mean, sd, cmin, cmax) >= 80) { mpr.setZero((float)mean); Serial.printf("Zero set: %.1f counts, std %.1f counts\n", mean, sd); }
  else { Serial.println("Zero failed: too few good readings"); }
}


void setup() {
  Serial.begin(500000);
  delay(3000);
  myWire.begin(SDA, SCL);
  myWire.setClock(MPR_I2C_REGULAR);
  myWire.setTimeOut(MPR_I2C_DEFAULTSTRETCH);
  runDiagnostics();
  zeroSensor();
  Serial.println("Commands: d = diagnostics, z = zero, h = help");
}

void loop() {
  mpr.update();
  MprSample s;
  while (mpr.pop(s)) {
    static uint32_t last = 0;
    if (millis() - last >= 100) { last = millis(); Serial.printf("pressure_mmHg:%.2f,counts:%lu\n", mpr.toPressure(s), (unsigned long)s.counts); }
  }
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == 'd') { runDiagnostics(); }
    else if (c == 'z') { zeroSensor(); }
    else if (c == 'h') { Serial.println("Commands: d = diagnostics, z = zero, h = help"); }
  }
}
