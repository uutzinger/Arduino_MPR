#pragma once
#include <stdint.h>
#include <stddef.h>
class TwoWire {
public:
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t addr);
  size_t write(uint8_t b);
  uint8_t endTransmission();
  uint8_t requestFrom(uint8_t addr, uint8_t n);
  int available();
  int read();
private:
  uint8_t _addr, _tx[8], _ntx = 0, _rx[8], _nrx = 0, _irx = 0;
};
extern TwoWire Wire;
