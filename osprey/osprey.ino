#include <Wire.h>
#include <accelerometer.h>
#include <barometer.h>
#include <battery.h>
#include <clock.h>
#include <constants.h>
#include <event.h>
#include <logger.h>
#include <gps.h>
#include <radio.h>

#include <SPI.h>
#include <SD.h>

#define chipSelect 4
#define FILENAME_FORMAT "%d.log"

char filename[20];

float initAlt = -1;

void blowUp(const char* const message) {
  while(1) {
    Serial.println(message);
    Serial.println("Check your connections and reset the board");
    Serial.println("\r\n");
    delay(1000);
  }
}

void initSD() {
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    blowUp("Card failed, or not present");
    // don't do anything more:
    return;
  }

  int logNumber = 0;
  // Find the first unused log number
  do {
    sprintf(filename, FILENAME_FORMAT, logNumber);
    logNumber++;
  } while(SD.exists(filename));

  
  Serial.println("card initialized.");
}

#define chipSelect = 4;
#define HEARTBEAT_LED 8
#define HEARTBEAT_INTERVAL 25

namespace Osprey {
  Accelerometer accelerometer;
  Barometer barometer(&Wire);
  Battery battery;
  Event event;
  Osprey::Clock clock;
  GPS gps;
  Radio radio;

  extern int commandStatus;
  int counter;

  void printJSON();
  void heartbeat();
  void initSensors();
  void printInitError(const char* const message);
  extern void processCommand();
}


using namespace Osprey;
unsigned long start;
unsigned long brakeStart;
bool deployed;

#define DEPLOY_TIME_MILLIS 120000


void setup(void) {
  Serial.begin(9600);
  pinMode(HEARTBEAT_LED, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  initSD();
  initSensors();
  deployed = false;
}

void deploy()
{
  Serial.println("Deploying!");
  digitalWrite(13,HIGH);
  delay(1500);
  digitalWrite(13,LOW);
}

void loop(void) {  
  printJSON();
  heartbeat();
}

void Osprey::printJSON() {
  File dataFile = SD.open(filename, FILE_WRITE);
  // The JSON structure is simple enough. Rather than bringing in another
  // library to do a bunch of heavylifting, just construct the string manually.
  dataFile.println("{");

  dataFile.println("\"roll\": ");
  dataFile.println(accelerometer.getRoll());

  dataFile.println(", \"pitch\": ");
  dataFile.println(accelerometer.getPitch());

  dataFile.println(", \"heading\": ");
  dataFile.println(accelerometer.getHeading());

  dataFile.println(", \"acceleration magnitude (g)\": ");
  dataFile.println(accelerometer.getAccelerationG());

  dataFile.println(", \"pressure_altitude\": ");
  dataFile.println(barometer.getAltitudeAboveSeaLevel());

  dataFile.println(", \"temp\": ");
  dataFile.println(barometer.getTemperatureC());
  dataFile.println(", \"time\": ");
  dataFile.println(Osprey::clock.getSeconds());
  dataFile.println(", \"agl\": ");
  float alt = barometer.getAltitudeAboveGround();
  if (initAlt == -1)
  {
    initAlt = alt;
  }
  if (!deployed && alt-initAlt>=1000)
  {
    delay(600);
    deploy();
    deployed = true;
  }
  dataFile.println(alt);
  dataFile.println("}");
  dataFile.println("\r\n");
  dataFile.close();
}

void Osprey::heartbeat() {
//  Serial.println("oiiiiiiiii");
  digitalWrite(HEARTBEAT_LED, HIGH);
  delay(HEARTBEAT_INTERVAL);
  digitalWrite(HEARTBEAT_LED, LOW);
}

void Osprey::initSensors() {

  if(!accelerometer.init()) {
    printInitError("Failed to intialize IMU (BNO055)");
  }
  if(!barometer.init()) {
    printInitError("Failed to intialize barometer (MS5607)");
  }
  if(!Osprey::clock.init()) {
    printInitError("Failed to intialize clock");
  }

}

void Osprey::printInitError(const char* const message) {
  while(1) {
    Serial.println(message);
    Serial.println("Check your connections and reset the board");
    Serial.println("\r\n");
    delay(1000);
  }
}
