/*
  Basic test of the MPR MicroPressure Sensor with transfer function other than default 'A'
  By: Urs Utzinger
  Date: November 2024
*/

#include <Wire.h>
#include <SparkFun_MicroPressure.h>

// Serial
#define BAUD_RATE          500000     // Up to 2,000,000 on ESP32, however more than 500 kBaud might be unreliable

// I2C speed
#define I2C_SPEED           400000     // 400,000, 100,000 50,000
#define I2C_STRETCH             50     //  50, 200 ms, typical is 230 micro seconds
#ifndef SDA
  #define SDA 3
#endif
#ifndef SCL
  #define SCL 4
#endif
TwoWire myWire = TwoWire(0);           // Create a TwoWire instance

// Example for MPRLS0300GY pressure sensor used to measure blood pressure in mmHg
// 300mmHg = 5.80104 PSI is max pressure
#define EOC_PIN -1
#define RST_PIN -1
#define I2C_ADDRESS 0x18
#define MIN_PSI 0
#define MAX_PSI 5.80104
#define TRANSFER_FUNCTION 'B'

SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI, TRANSFER_FUNCTION);
uint32_t lastTime;

void setup() {
  Serial.begin(115200);

  myWire.begin(SDA, SCL);
  myWire.setClock(I2C_SPEED);
  myWire.setTimeOut(I2C_STRETCH);

  if(!mpr.begin(I2C_ADDRESS, myWire))
  {
    Serial.println("Cannot connect to MicroPressure sensor.");
    while(1);
  }
  
  // Optional: set the measured zero pressure value
  // mpr.setZero(419430); 

  // Optional: for manual calibration with a water column e.g. in a plastic tube
  // (you need to select appropriate length of water column to not damage the sensor)
  //   1 mmHg ≈ 13.5951 mmH₂O
  //   1 mmH₂O ≈ 0.00142233 PSI
  // Example calculation:
  //   minCounts =  419430; measured raw counts with 0 mm Water Column
  //   2meterH2OCounts = 3000000; measured raw counts with a 2 meter Water Column
  //   deltaCounts = 2meterH2OCounts - minCounts
  //   calculatedPSI = 2000mm * 0.00142233 PSI/mm 
  //   deltaPSI = calculatedPSI - 0 PSI
  //   calFactor = deltaPSI / deltaCounts
  // mpr.setCalFac(5.80104/3355444);

  lastTime = micros();
}

void loop() {
  uint32_t currentTime= micros();
  if(currentTime - lastTime >= 2*6250) { // 6250 micros = 160Hz is maximum rate
    lastTime = currentTime;
    Serial.print(mpr.readPressure(TORR),3);
    Serial.print(" mmHg ");
    Serial.print(mpr.readPressure(RAW),3);
    Serial.println(" counts");
  } else {
    delay(1);
  }
}