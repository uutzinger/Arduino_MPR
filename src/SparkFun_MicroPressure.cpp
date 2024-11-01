/*
  This is a library for the Qwiic MicroPressure Sensor, which can read from 0 to 25 PSI.
  By: Alex Wende
  Date: July 2020 
  License: This code is public domain but you buy me a beer if you use this and 
  we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
 */
 
 #include "SparkFun_MicroPressure.h"

/* Constructor and sets default values.
   - (Optional) eoc_pin, End of Conversion indicator. Default: -1 (skip)
   - (Optional) rst_pin, Reset pin for MPR sensor. Default: -1 (skip)
   - minimumPSI, minimum range value of the sensor (in PSI). Default: 0
   - maximumPSI, maximum range value of the sensor (in pSI). Default: 25

   The maximum and minimum PSI are listed in the datasheet of the sensor.
   They are in units of PSI, kPa, bar, or mmHg depending on the sensor type.
   For this software to work properly you need to convert them to PSI.
*/
SparkFun_MicroPressure::SparkFun_MicroPressure(int8_t eoc_pin, int8_t rst_pin, float minimumPSI, float maximumPSI, char transfer_function)
{
  _eoc = eoc_pin;
  _rst = rst_pin;
  _minPsi = minimumPSI;
  _maxPsi = maximumPSI;
  _transfer_function = transfer_function;

  // MPR Sensors can have 3 different types of transfer functions
  // In the data sheet they are called A, B, C
  //   each function has two calibration points; one axis is sensor counts and the other is pressure
  //   counts are given below, min and max pressure is stated in the datasheet for each of the sensor types
  //   min and max corresponds to the counts below
  //
  // maximum possible counts is 2^24 = 16777216
  // A 
  // 10% * 16777216   =  1677722 = min Counts
  // 90% * 16777216   = 15099494 = max Counts
  // B
  //  2.5% * 16777216 =  419430
  // 22.5% * 16777216 = 3774874
  // C
  // 20% * 16777216   =  3355443
  // 80% * 16777216   = 13421773

  switch (_transfer_function)
  {
    case 'A':
      _minCounts =  1677722;
      _maxCounts = 15099494;
      break;
    case 'B':
      _minCounts =  419430;
      _maxCounts = 3774874;
      break;
    case 'C':
      _minCounts =  3355443;
      _maxCounts = 13421773;
      break;
    default:
      _minCounts =  1677722;
      _maxCounts = 15099494;
      break;
  }
  _deltaCounts = _maxCounts - _minCounts;
  _deltaPsi = _maxPsi - _minPsi;
  _calFactor = _deltaPsi / float(_deltaCounts) ;
}

/* Initialize hardware
  - deviceAddress, I2C address of the sensor. Default: 0x18
  - wirePort, sets the I2C bus used for communication. Default: Wire
  
  - Returns 0/1: 0: sensor not found, 1: sensor connected
*/
bool SparkFun_MicroPressure::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
  _address = deviceAddress;
  _i2cPort = &wirePort;
  
  if(_eoc != -1)
  {
    pinMode(_eoc, INPUT);
  }
  
  if(_rst != -1)
  {
    pinMode(_rst, OUTPUT);
	digitalWrite(_rst,LOW);
	delay(5);
	digitalWrite(_rst,HIGH);
	delay(5);
  }

  _i2cPort->beginTransmission(_address);

  uint8_t error = _i2cPort->endTransmission();

  if(error == 0) return true;
  else           return false;
}

/* Read the status byte of the sensor
  - Returns status byte
*/
uint8_t SparkFun_MicroPressure::readStatus(void)
{
  _i2cPort->requestFrom(_address,1);
  return _i2cPort->read();
} 

/* Provide runtime zeroing of the sensor
   zero, is the value in counts at minimum pressure
   zero is automatically calculated based on the transfer function
   however these values can be improved by measuring the zero pressure reading
*/
void SparkFun_MicroPressure::setZero(uint32_t zero)
{
  _minCounts = zero;
  _deltaCounts = _maxCounts - _minCounts;
  _calFactor = _deltaPsi / float(_deltaCounts) ;
}

/* Provide runtime calibration option for he sensor
   calFac, is the calibration factor
   calFac is automatically calculated based on the transfer function and max and min pressure
   However these values can be improved by measuring the sensor at a known pressure,
   for example by measuring the pressure in a water column at a known height or depth
*/
void SparkFun_MicroPressure::setCalFac(float calFac)
{
  _calFactor = calFac;
  _deltaPsi = _calFactor * float(_deltaCounts);
  _maxPsi = _minPsi + _deltaPsi;
}

/* Read the Pressure Sensor Reading
 - (optional) Pressure_Units, can return various pressure units. Default: PSI
   Pressure Units available:
   - PSI: Pounds per Square Inch
	 - PA: Pascals
	 - KPA: Kilopascals
	 - TORR: mm Mercury
	 - INHG: Inch of Mercury
	 - ATM: Atmospheres
	 - BAR: Bars
   - RAW: returns the raw 24-bit pressure reading
*/
float SparkFun_MicroPressure::readPressure(Pressure_Units units)
{
  _i2cPort->beginTransmission(_address);
  _i2cPort->write((uint8_t)0xAA);
  _i2cPort->write((uint8_t)0x00);
  _i2cPort->write((uint8_t)0x00);
  _i2cPort->endTransmission();
  
  // Wait for new pressure reading available
  if(_eoc != -1) // Use GPIO pin if defined
  {
    while(!digitalRead(_eoc))
	{
	  delay(1);
	}
  }
  else // Check status byte if GPIO is not defined
  {
    uint8_t status = readStatus();
    while((status&BUSY_FLAG) && (status!=0xFF))
    {
      delay(1);
      status = readStatus();
    }
  }
  
  _i2cPort->requestFrom(_address,4);
  
  uint8_t status = _i2cPort->read();
  
  //check memory integrity and math saturation bits
  if((status & INTEGRITY_FLAG) || (status & MATH_SAT_FLAG))
  {
    return NAN;
  }
  
  //read 24-bit pressure
  uint32_t reading = 0;
  for(uint8_t i=0;i<3;i++)
  {
    reading |= _i2cPort->read();
    if(i != 2) reading = reading<<8;
  }

  if (units == RAW){
    return float(reading);
  } 
  else {
    //convert from counts to pressure
    float pressure;
    pressure = ( ( float(int32_t(reading) - int32_t(_minCounts)) ) * _calFactor )  + _minPsi;

    if(units == PSI)       return pressure;           //PSI (pounds per square inch)
    else if(units == PA)   return pressure*6894.7573; //Pa (Pascal, Newton per square meter)
    else if(units == KPA)  return pressure*6.89476;   //kPa (kilopascal)
    else if(units == TORR) return pressure*51.7149;   //torr (mmHg)
    else if(units == INHG) return pressure*2.03602;   //inHg (inch of mercury)
    else if(units == ATM)  return pressure*0.06805;   //atm (atmosphere, 1atm = 1.01325bar)
    else if(units == BAR)  return pressure*0.06895;   //bar (1bar = 100,000Pa)
    else                   return pressure;           //PSI
  }
}
