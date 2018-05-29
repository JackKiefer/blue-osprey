#include <Wire.h>
#include <accelerometer.h>
#include <barometer.h>
#include <battery.h>
#include <clock.h>
#include <constants.h>
#include <event.h>
#include <gps.h>
#include <radio.h>
#include <thermometer.h>

#define HEARTBEAT_LED 13
#define HEARTBEAT_INTERVAL 25

namespace Osprey {
  Accelerometer accelerometer;
  Barometer barometer;
  Battery battery;
  Event event;
  Osprey::Clock clock;
  GPS gps;
  Radio radio;
  Thermometer thermometer;

  extern int commandStatus;
  int counter;

  void printJSON();
  void heartbeat();
  void initSensors();
  void printInitError(const char* const message);
  extern void processCommand();
}


using namespace Osprey;

void setup(void) {
  Serial.begin(9600);
  initSensors();
  pinMode(HEARTBEAT_LED, OUTPUT);
  counter = 0;
}

void loop(void) {
  event.check();
  printJSON();
  processCommand();
  heartbeat();

  counter++;
}

void Osprey::printJSON() {
  // The JSON structure is simple enough. Rather than bringing in another
  // library to do a bunch of heavylifting, just construct the string manually.

  Serial.println("{");

  Serial.println("\"roll\": ");
  Serial.println(accelerometer.getRoll());

  Serial.println(", \"pitch\": ");
  Serial.println(accelerometer.getPitch());

  Serial.println(", \"heading\": ");
  Serial.println(accelerometer.getHeading());

  Serial.println(", \"raw_acceleration\": ");
  Serial.println(accelerometer.getRawAcceleration());

  Serial.println(", \"pressure_altitude\": ");
  Serial.println(barometer.getAltitudeAboveSeaLevel());

  Serial.println(", \"temp\": ");
  Serial.println(thermometer.getTemperature());

  Serial.println(", \"id\": ");
  Serial.println(counter);

  Serial.println(", \"delta\": ");
  Serial.println(Osprey::clock.getSeconds());

  Serial.println(", \"iso8601\": \"");
  Serial.println(gps.getIso8601());
  Serial.println("\"");

  Serial.println(", \"agl\": ");
  Serial.println(barometer.getAltitudeAboveGround());

  Serial.println(", \"acceleration\": ");
  Serial.println(accelerometer.getAcceleration());

  Serial.println(", \"latitude\": ");
  Serial.println(gps.getLatitude(), 6);

  Serial.println(", \"longitude\": ");
  Serial.println(gps.getLongitude(), 6);

  Serial.println(", \"speed\": ");
  Serial.println(gps.getSpeed());

  Serial.println(", \"gps_altitude\": ");
  Serial.println(gps.getAltitude());

  Serial.println(", \"gps_quality\": ");
  Serial.println(gps.getQuality());

  Serial.println(", \"command_status\": ");
  Serial.println(commandStatus);

  Serial.println(", \"previous_command\": \"");
  Serial.println(radio.getMostRecentMessage());
  Serial.println("\"");

  Serial.println(", \"pressure_setting\": ");
  Serial.println(barometer.getPressureSetting());

  Serial.println(", \"phase\": ");
  Serial.println(event.getPhase());

  Serial.println(", \"battery\": ");
  Serial.println(battery.getVoltage(), 2);

  Serial.println(", \"apogee_cause\": ");
  Serial.println(event.getApogeeCause());

  Serial.println(", \"armed\": ");
  Serial.println(event.isArmed());

  Serial.println(", \"apogee_fired\": ");
  Serial.println(event.didFire(EVENT_APOGEE));

  Serial.println(", \"main_fired\": ");
  Serial.println(event.didFire(EVENT_MAIN));

  Serial.println(", \"main_alt\": ");
  Serial.println(event.getAltitude(EVENT_MAIN));

  Serial.println(", \"logging\": ");
  Serial.println(radio.isLogging());

  Serial.println("}");
  Serial.println("\r\n");
}

void Osprey::heartbeat() {
//  digitalWrite(HEARTBEAT_LED, HIGH);
  delay(HEARTBEAT_INTERVAL);
//  digitalWrite(HEARTBEAT_LED, LOW);
}

void Osprey::initSensors() {

  if(!accelerometer.init()) {
    printInitError("Failed to intialize accelerometer");
  }
  
  while(true){
  Serial.println("\"rollBITCH\": ");
  Serial.println(accelerometer.getRoll());
  heartbeat();
  }


  if(!barometer.init()) {
    printInitError("Failed to intialize barometer");
  }

  if(!battery.init()) {
    printInitError("Failed to intialize battery");
  }

  if(!Osprey::clock.init()) {
    printInitError("Failed to intialize clock");
  }

  if(!event.init()) {
    printInitError("Failed to intialize events");
  }

  if(!gps.init()) {
    printInitError("Failed to intialize GPS");
  }

  if(!radio.init()) {
    printInitError("Failed to intialize radio");
  }

  if(!thermometer.init()) {
    printInitError("Failed to intialize thermometer");
  }
}

void Osprey::printInitError(const char* const message) {
  while(1) {
    Serial.println(message);
    Serial.println("boiiiiiiii");
    Serial.println("\r\n");
    delay(1000);
  }
}
