// Portable MPR example for any board (AVR, ARM, ESP32): status polling, no EOC / RST pin.
// Prints pressure in mmHg. The other examples use ESP32-only APIs.
// Sensor: MPRLS0300YG (0..300 mmHg gage), transfer function B, I2C address 0x18.
#include <Wire.h>
#include <MPR.h>

MPRBuffer<8> mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG);   // small buffer for small RAM

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(MPR_I2C_REGULAR);

  if (!mpr.begin(Wire)) {
    Serial.print(F("MPR not ready, status 0x"));
    Serial.println(mpr.status(), HEX);
  }
  mpr.start(10000);                     // one reading every 10 ms; start(0) = as fast as possible
}

void loop() {
  mpr.update();

  MprSample s;
  while (mpr.pop(s)) {
    Serial.print(F("t_us:"));         Serial.print(s.t_us);
    Serial.print(F(",pressure_mmHg:")); Serial.print(mpr.toPressure(s), 2);
    Serial.print(F(",counts:"));      Serial.println(s.counts);
  }
}
