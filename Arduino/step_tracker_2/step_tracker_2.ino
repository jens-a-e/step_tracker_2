/**
 * Simple wearable data logger to explore probes on resonance
 * 
 * TODO: - [x] Add Battery level to logged output to monitor the energy consumption
 * Version 0.0.3
 */

#include <RTClib.h>

#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>

#include <SPI.h>
#include <SD.h>

const int chipSelect = 10; // 10 for the logger SD card on the Feather

RTC_PCF8523 rtc;

Adafruit_APDS9960 apds9960; // proximity, light, color, gesture
Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LIS3MDL lis3mdl;   // magnetometer
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30;       // humidity

typedef struct {
  uint32_t time;
  uint8_t proximity;
  uint8_t steps;
  uint8_t _[2]; // make padding explicit
  uint16_t r, g, b, c;
  int32_t mic;
  float temperature, pressure, altitude;
  float magnetic_x, magnetic_y, magnetic_z;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float humidity;
  float batt;
} Record;

extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

#include "lib.h"

// uint8_t proximity;
// uint16_t r, g, b, c;
// float temperature, pressure, altitude;
// float magnetic_x, magnetic_y, magnetic_z;
// float accel_x, accel_y, accel_z;
// int steps;
// float gyro_x, gyro_y, gyro_z;
// float humidity;
// int32_t mic;

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);

  init_serial();

  Serial.println("Feather Sense Sensor Demo");

  // debug_print_record_sizes();

  init_rtc();
  init_storage();

  init_sensors();

  // Insert a comment line to indicate a restart of the board
  File db = SD.open("datalog.txt", FILE_WRITE);
  if (db) {
    db.println("# New record " + date_fmt());
    db.flush();
  }
}

void loop(void) {
  Serial.println("\nFeather Sense Sensor Demo" + date_fmt());
  Serial.println("---------------------------------------------");
  
  Record r = new_record();
  read_record(&r);
  // Log the data to the CSV file
  digitalWrite(LED_BUILTIN, HIGH);
  if(!log_record(&r)) {
    // If there is not an SD card present, flash 3 times and try again
    // TODO: Add a multi stage retry to slowly escalate. Is there a way to know when the SD card is inserted?
    flash(3);
    delay(1000);
  }
  digitalWrite(LED_BUILTIN, LOW);

  // Reset the pedometer after a successful write to make sure to keep steps recorded even when there is no SD card
  lsm6ds33.resetPedometer();
  delay(1000);
}
