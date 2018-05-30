#include "barometer.h"


Barometer::Barometer() : Sensor(KALMAN_PROCESS_NOISE, KALMAN_MEASUREMENT_NOISE, KALMAN_ERROR) {
  altitude = kalmanInit(0);
}

float getTemperatureC()
{
  return baro.getTemp() * 100.0;
}

int Barometer::init() {
  return (baro.connect() <= 0);
}

float Barometer::getPressure() {
  kalmanUpdate(&altitude, baro.getPres());
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

  return ( (pow(SEA_LEVEL/pressure,EXPONENT)-1.0)*(temp+TO_KELVIN) )/(DENOM);
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
