#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define DEFAULT_ADDRESS 0x18
#define MAXIMUM_PSI     25
#define MINIMUM_PSI     0

#define BUSY_FLAG       0x20
#define INTEGRITY_FLAG  0x04
#define MATH_SAT_FLAG   0x01

enum Pressure_Units {
  PSI,
  PA,
  KPA,
  TORR,
  INHG,
  ATM,
  BAR,
  RAW
};

class SparkFun_MicroPressure
{
  public:
    SparkFun_MicroPressure(int8_t eoc_pin=-1, int8_t rst_pin=-1, float minimumPSI=MINIMUM_PSI, float maximumPSI=MAXIMUM_PSI, char transfer_function='A');
    bool begin(uint8_t deviceAddress = DEFAULT_ADDRESS, TwoWire &wirePort = Wire);
    uint8_t readStatus(void);
    float readPressure(Pressure_Units units=PSI);
    void setZero(uint32_t zero);
    void setCalFac(float calFac);
    
  private:
    int8_t _address;
	  int8_t _eoc, _rst;
    float _minPsi, _maxPsi, _deltaPsi, _calFactor;
    uint32_t _zero, _maxCounts, _minCounts, _deltaCounts;
    char _transfer_function;
    
    TwoWire *_i2cPort;
};
