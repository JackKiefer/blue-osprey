#include "barometer.h"

MS5xxx Barometer::baro = MS5xxx();


Barometer::Barometer(TwoWire* wire) : Sensor(KALMAN_PROCESS_NOISE, KALMAN_MEASUREMENT_NOISE, KALMAN_ERROR) {
  baro.setWire(wire);
  altitude = kalmanInit(0);
}

float Barometer::getTemperatureC()
{
  return baro.GetTemp() * 100.0;
}

int Barometer::init() {
  return (baro.connect() <= 0);
}

float Barometer::getPressure() {
  baro.ReadProm();
  baro.Readout();
  kalmanUpdate(&altitude, baro.GetPres());
  return altitude.value;
}

float const SEA_LEVEL_PRESSURE_Pa = 101325.0;
float const EXPONENT = 0.19022256039566293;
float const TO_KELVIN = 273.15;
float const DENOM = 0.0065;
 
float Barometer::getAltitudeAboveSeaLevel() {
  float pressure = getPressure();
  float temp = getTemperatureC();

  if(pressure == NO_DATA) {
    return NO_DATA;
  }

  return (( (pow(SEA_LEVEL_PRESSURE_Pa/pressure,EXPONENT)-1.0)*(temp+TO_KELVIN) )/(DENOM))/1000.0;
}

float Barometer::getAltitudeAboveGround() {
  float pressure = getPressure();

  if(pressure == NO_DATA) {
    return NO_DATA;
  }

  return getAltitudeAboveSeaLevel() - groundLevel;
}


void Barometer::zero() {
  setGroundLevel();
}


void Barometer::setGroundLevel() {
  groundLevel = getAltitudeAboveSeaLevel();
}
