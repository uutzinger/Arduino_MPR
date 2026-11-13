// MPR on an I2C bus shared with other devices that are used from different FreeRTOS tasks (ESP32).
//
// The driver never owns the bus. The main program creates ONE mutex per TwoWire and every user of
// that bus takes it around each of its I2C transactions: the MPR driver (through setBusLock),
// and your own code / other drivers (directly, as in readOtherDevice() below).
// With several MPR sensors on the same bus, give all of them the same lock functions.
//
// If everything runs from one loop() / one task, you do not need any of this.
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

constexpr int8_t  EOC_PIN       = D10;
constexpr int8_t  RST_PIN       = D11;
constexpr uint8_t OTHER_ADDRESS = 0x40;        // some other sensor on the same bus (example only)

TwoWire           myWire = TwoWire(0);
SemaphoreHandle_t busMutex;                    // one mutex per TwoWire
MPRBuffer<32>     mpr(MprRange::R_0300YG, MprTransfer::B, MprUnit::MMHG, MPR_I2C_ADDRESS, EOC_PIN, RST_PIN);

// ---- lock functions handed to the MPR driver ----
// Return false when the bus is not free within a short time: the driver skips this attempt and
// tries again at the next update(). Keep the time short so update() never stalls behind a slow device.
bool lockBus(void *)   { return xSemaphoreTake(busMutex, pdMS_TO_TICKS(5)) == pdTRUE; }
void unlockBus(void *) { xSemaphoreGive(busMutex); }

// ---- another user of the same bus: takes the same mutex around each complete transaction ----
bool readOtherDevice(uint16_t &value) {
  if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(20)) != pdTRUE) { return false; }
  bool ok = false;
  if (myWire.requestFrom(OTHER_ADDRESS, (uint8_t)2) == 2) {
    value = ((uint16_t)myWire.read() << 8) | myWire.read();
    ok = true;
  }
  xSemaphoreGive(busMutex);
  return ok;
}

// Bus recovery hook: restarts the shared bus. Other drivers may need to be told as well.
bool onBusRecover(void *) {
  if (xSemaphoreTake(busMutex, pdMS_TO_TICKS(100)) != pdTRUE) { return false; }
  myWire.end();
  delay(10);
  myWire.begin(SDA, SCL);
  myWire.setClock(MPR_I2C_REGULAR);
  myWire.setTimeOut(MPR_I2C_DEFAULTSTRETCH);
  xSemaphoreGive(busMutex);
  return true;
}

// ---- task 1: MPR sampling ----
void mprTask(void *) {
  for (;;) {
    mpr.update();                              // lock is taken inside, once per transaction
    MprSample s;
    while (mpr.pop(s)) { /* use mpr.toPressure(s), s.t_us */ }
    vTaskDelay(1);
  }
}

// ---- task 2: the other device ----
void otherTask(void *) {
  for (;;) {
    uint16_t v;
    if (readOtherDevice(v)) { /* use v */ }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(500000);
  busMutex = xSemaphoreCreateMutex();

  myWire.begin(SDA, SCL);                      // the main program owns the bus and its settings
  myWire.setClock(MPR_I2C_REGULAR);
  myWire.setTimeOut(MPR_I2C_DEFAULTSTRETCH);

  mpr.setBusLock(lockBus, unlockBus, nullptr);
  MprCallbacks cb = {};
  cb.onBusRecover = onBusRecover;
  mpr.setRecoveryCallbacks(cb);

  mpr.begin(myWire);                           // begin() and reset() also take the lock
  mpr.start(0);

  xTaskCreate(mprTask,   "mpr",   4096, nullptr, 2, nullptr);
  xTaskCreate(otherTask, "other", 4096, nullptr, 1, nullptr);
}

void loop() {
  // Report from the main task. stats().lock_failed counts attempts skipped because the bus was busy.
  static uint32_t last = 0;
  if (millis() - last >= 5000) {
    last = millis();
    MprStats st = mpr.stats();
    char msg[96];
    MPR::errorString(mpr.lastError(), msg, sizeof(msg));
    Serial.printf("good %lu bad %lu lock_failed %lu | %s\n", (unsigned long)st.good, (unsigned long)st.bad,
                  (unsigned long)st.lock_failed, msg);
  }
  delay(100);
}
