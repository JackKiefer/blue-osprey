#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "Adafruit_BNO055.h"

#include <math.h>

#include "constants.h"
#include "kalman.h"
#include "sensor.h"

#define KALMAN_PROCESS_NOISE 0.01
#define KALMAN_MEASUREMENT_NOISE 0.25
#define KALMAN_ERROR 1

class Accelerometer : public virtual Sensor {
  public:
    Accelerometer();
    int init();

    /* Begin externally used funcs */
    float getRoll();
    float getPitch();
    float getHeading();
    float getAccelerationG();
    imu::Vector<3> getAccelerationVec(unsigned long const);
    imu::Vector<3> getVelocityVec();
    float accelNorm(imu::Vector<3> const & v);
    /* end */

    void getAccelOrientation(sensors_vec_t *orientation);
    void getMagOrientation(sensors_vec_t *orientation);

  protected:
    static Adafruit_BNO055 bno;

    imu::Vector<3> oldAccel;
    imu::Vector<3> newAccel;

    imu::Vector<3> lastVel;

    unsigned long oldTime = 0;
    unsigned long newTime = 0;

    double getDt();

    unsigned long const UL_MAX = 4294967295;

    kalman_t roll;
    kalman_t pitch;
    kalman_t heading;
    kalman_t accelerationX;
    kalman_t accelerationY;
    kalman_t accelerationZ;
};

#endif
