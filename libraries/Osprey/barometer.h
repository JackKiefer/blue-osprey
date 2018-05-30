#ifndef BAROMETER_H
#define BAROMETER_H

#include <MS5xxx.h>
#include <Wire.h>

#include "constants.h"
#include "sensor.h"

#define KALMAN_PROCESS_NOISE 0.01
#define KALMAN_MEASUREMENT_NOISE 0.25
#define KALMAN_ERROR 1

class Barometer : public virtual Sensor 
{
  public:
    Barometer();
    int init();
    float getPressure();
    float getAltitudeAboveSeaLevel(); //
    float getAltitudeAboveGround(); //
    void zero(); // 
    float getTemperatureC();

  protected:
    static MS5xxx baro(&Wire);

    float groundLevel;
    kalman_t altitude;

    void setGroundLevel();
};

#endif
