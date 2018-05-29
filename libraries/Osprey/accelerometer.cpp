#include <math.h>
#include "accelerometer.h"

#define PI (3.14159265F);

Adafruit_BNO055 Accelerometer::bno = Adafruit_BNO055(55);

Accelerometer::Accelerometer() : Sensor(KALMAN_PROCESS_NOISE, KALMAN_MEASUREMENT_NOISE, KALMAN_ERROR) {
  roll = kalmanInit(0);
  pitch = kalmanInit(90);
  heading  = kalmanInit(0);
  acceleration  = kalmanInit(1);
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

float Accelerometer::getAcceleration() {
  kalmanUpdate(&acceleration, getRawAcceleration());
  return acceleration.value;
}

float Accelerometer::getRawAcceleration() {
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
