// MPR with EOC interrupt, RST pin, recovery callbacks and error reporting by the application.
#if !defined(ARDUINO_ARCH_ESP32)
  #error "This example uses ESP32 APIs (TwoWire(n), printf, FreeRTOS). For other boards see BasicPortable."
#endif
#include <Wire.h>
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

constexpr int8_t EOC_PIN = D10;   // Feather D10
constexpr int8_t RST_PIN = D11;   // Feather D11

TwoWire myWire = TwoWire(0);
MPRBuffer<32> mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, MPR_I2C_ADDRESS, EOC_PIN, RST_PIN);

// Recovery hooks, called from mpr.update()
bool onBusRecover(void *) {
  myWire.end();
  delay(10);
  myWire.begin(SDA, SCL);
  myWire.setClock(MPR_I2C_REGULAR);
  myWire.setTimeOut(MPR_I2C_DEFAULTSTRETCH);
  Serial.println("I2C bus restarted");
  return true;
}
void onAfterReset(void *, bool ok) { Serial.printf("MPR reset %s\n", ok ? "ok" : "failed"); }
void onFailed(void *)              { Serial.println("MPR failed, retrying once per second"); }

uint16_t lastReported = 0;
uint32_t lastReportMs = 0;
uint32_t lastStatsMs  = 0;

void setup() {
  Serial.begin(500000);
  myWire.begin(SDA, SCL);
  myWire.setClock(MPR_I2C_REGULAR);
  myWire.setTimeOut(MPR_I2C_DEFAULTSTRETCH);

  MprCallbacks cb = {};
  cb.onBusRecover = onBusRecover;
  cb.onAfterReset = onAfterReset;
  cb.onFailed     = onFailed;
  mpr.setRecoveryCallbacks(cb);

  bool ok = mpr.begin(myWire);          // resets the sensor through RST, waits for the power bit
  mpr.start(0);                         // back to back, up to about 160 readings/s
  MprMode mode = mpr.mode();            // the mode is only known after start()
  Serial.printf("MPR begin: %s, mode %s\n", ok ? "ok" : "NOT ready",
                mode == MprMode::EOC_INTERRUPT ? "EOC interrupt" : mode == MprMode::STATUS_POLL ? "status poll" : "stopped");
}

void loop() {
  mpr.update();

  MprSample s;
  uint8_t n = 0;
  while (mpr.pop(s) && ++n) {
    if (n == 1 && (millis() - lastStatsMs) >= 100) {          // print at 10 Hz
      lastStatsMs = millis();
      Serial.printf("t_us:%lu,pressure_mmHg:%.2f\n", (unsigned long)s.t_us, mpr.toPressure(s));
    }
  }

  // The application decides when to report errors: on change, repeated every 5 s while it lasts.
  uint16_t err = mpr.lastError();
  if (err != lastReported || (err && (millis() - lastReportMs) >= 5000)) {
    char st[64], msg[160];
    MPR::statusString(mpr.status(), st, sizeof(st));
    MPR::errorString(err, msg, sizeof(msg));
    Serial.printf("MPR status %s | error: %s | I2C: %s\n", st, msg, MPR::i2cResultString(mpr.lastI2CResult()));
    for (uint8_t b = 0; b < MPR::ERROR_BITS; b++) {
      if ((err & (1U << b)) && !(MPR::E_WARN & (1U << b))) { Serial.printf("  hint: %s\n", MPR::errorHint(b)); }
    }
    lastReported = err;
    lastReportMs = millis();
  }
}
