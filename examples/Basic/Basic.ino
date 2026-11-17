// Basic MPR example: status polling (no EOC pin), prints pressure in mmHg.
// Sensor: MPRLS0300YG (0..300 mmHg gage), transfer function B, I2C address 0x18.
#if !defined(ARDUINO_ARCH_ESP32)
  #error "This example uses ESP32 APIs (TwoWire(n), printf, FreeRTOS). For other boards see BasicPortable."
#endif
#include <Wire.h>
#include <MPR.h>

#ifndef SDA
  #define SDA 3
#endif
#ifndef SCL
  #define SCL 4
#endif

MPRBuffer<32> mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG);   // no EOC, no RST

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);                 // the main program owns the bus
  Wire.setClock(MPR_I2C_REGULAR);
  Wire.setTimeOut(MPR_I2C_DEFAULTSTRETCH);

  if (!mpr.begin(Wire)) {
    char msg[64];
    MPR::statusString(mpr.status(), msg, sizeof(msg));
    Serial.printf("MPR not ready: %s\n", msg);
  }
  mpr.start(10000);                     // one reading every 10 ms; start(0) = as fast as possible
}

void loop() {
  mpr.update();                         // never blocks

  MprSample s;
  while (mpr.pop(s)) {
    Serial.printf("t_us:%lu,pressure_mmHg:%.2f,counts:%lu\n",
                  (unsigned long)s.t_us, mpr.toPressure(s), (unsigned long)s.counts);
  }
}
