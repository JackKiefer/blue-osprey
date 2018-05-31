#include <math.h>
#include "accelerometer.h"

#define PI (3.14159265F);

Adafruit_BNO055 Accelerometer::bno = Adafruit_BNO055(55);

Accelerometer::Accelerometer() : Sensor(KALMAN_PROCESS_NOISE, KALMAN_MEASUREMENT_NOISE, KALMAN_ERROR) {
  roll = kalmanInit(0);
  pitch = kalmanInit(90);
  heading  = kalmanInit(0);
  accelerationX  = kalmanInit(1);
  accelerationY  = kalmanInit(1);
  accelerationZ  = kalmanInit(1);
  oldAccel[0] = 0.0;
  oldAccel[1] = 0.0;
  oldAccel[2] = 0.0;
  newAccel[0] = 0.0;
  newAccel[1] = 0.0;
  newAccel[2] = 0.0;
  lastVel[0] = 0.0;
  lastVel[1] = 0.0;
  lastVel[2] = 0.0;

}

int Accelerometer::init() {
  return bno.begin();
}

float Accelerometer::getRoll() {
  sensors_vec_t orientation;
  getAccelOrientation(&orientation);

  if(!orientation.roll) {
    return NO_DATA;
  }

  kalmanUpdate(&roll, orientation.roll);
  return roll.value;
}

float Accelerometer::getPitch() {
  sensors_vec_t orientation;
  getAccelOrientation(&orientation);

  if(!orientation.pitch) {
    return NO_DATA;
  }

  kalmanUpdate(&pitch, orientation.pitch);
  return pitch.value;
}

float Accelerometer::getHeading() {
  sensors_vec_t orientation;
  getMagOrientation(&orientation);

  if(!orientation.heading) {
    return NO_DATA;
  }

  kalmanUpdate(&heading, orientation.heading);
  return heading.value;
}


imu::Vector<3> Accelerometer::getAccelerationVec(unsigned long const curTime) {
  sensors_event_t event;
  bno.getOspreyEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  kalmanUpdate(&accelerationX, event.acceleration.x);
  kalmanUpdate(&accelerationY, event.acceleration.y);
  kalmanUpdate(&accelerationZ, event.acceleration.z);
  imu::Vector<3> xyz;
  xyz[0] = accelerationX.value;
  xyz[1] = accelerationY.value;
  xyz[2] = accelerationZ.value;


  oldAccel = newAccel;
  newAccel = xyz;

  oldTime = newTime;
  newTime = curTime;

  return xyz;
}

unsigned long Accelerometer::getDt() {
  if (newTime >= oldTime)
  {
    return newTime - oldTime;
  }
  else
  {
    /* newTime < oldTime */
    /* Overflow occurred, that's okay! */
    return (UL_MAX - oldTime) + newTime;
  }
}

float trapezoidalIntegrate(float a0, float a1, float dt)
{
  return (a0 * dt) + ((a1-a0)*dt)/2.0;
}

imu::Vector<3> Accelerometer::getVelocityVec() {
  unsigned long udt = getDt();
  if (udt == 0)
  {
    return lastVel;
  }
  /* Microseconds -> Seconds */
  float dt = dt / 1000000.0;

  imu::Vector<3> vel;
  vel[0] = trapezoidalIntegrate(oldAccel[0],newAccel[0],dt);
  vel[1] = trapezoidalIntegrate(oldAccel[1],newAccel[1],dt);
  vel[2] = trapezoidalIntegrate(oldAccel[2],newAccel[2],dt);

  lastVel = vel;

  return vel;
}

float accelNorm(imu::Vector<3> const & v)
{
  return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

float Accelerometer::getAccelerationG() {
  sensors_event_t event;
  bno.getOspreyEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  return sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2)) * MS2_TO_G;
}

void Accelerometer::getAccelOrientation(sensors_vec_t *orientation) {
  sensors_event_t event;
  bno.getOspreyEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float t_pitch;
  float t_roll;
  float t_heading;
  float signOfZ = event.acceleration.z >= 0 ? 1.0F : -1.0F;

  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
  /* roll is positive and increasing when moving downward                                     */
  /*                                                                                          */
  /*                                 y                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(x^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_roll = event.acceleration.x * event.acceleration.x + event.acceleration.z * event.acceleration.z;
  orientation->roll = (float)atan2(event.acceleration.y, sqrt(t_roll)) * 180 / PI;

  /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
  /* pitch is positive and increasing when moving upwards                                     */
  /*                                                                                          */
  /*                                 x                                                        */
  /*            pitch = atan(-----------------)                                               */
  /*                          sqrt(y^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_pitch = event.acceleration.y * event.acceleration.y + event.acceleration.z * event.acceleration.z;
  orientation->pitch = (float)atan2(event.acceleration.x, signOfZ * sqrt(t_pitch)) * 180 / PI;

}

void Accelerometer::getMagOrientation(sensors_vec_t *orientation) {
  sensors_event_t event;
  bno.getOspreyEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  orientation->heading = (float)atan2(event.magnetic.z, event.magnetic.y) * 180 / PI;
}
