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

uint8_t proximity;
uint16_t r, g, b, c;
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
int steps;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;

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

int32_t getPDMwave(int32_t samples);

void flash() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);  
}

void flash(int times) {
  while(times--) {
    flash();
    delay(200);
  }
}


Record new_record() {
  Record r;
  r.time = 0;
  r.proximity = 0;
  r.r = 0;
  r.g = 0;
  r.b = 0;
  r.c = 0;
  r.temperature = 0;
  r.pressure = 0;
  r.altitude = 0;
  r.magnetic_x = 0;
  r.magnetic_y = 0;
  r.magnetic_z = 0;
  r.accel_x = 0;
  r.accel_y = 0;
  r.accel_z = 0;
  r.gyro_x = 0;
  r.gyro_y = 0;
  r.gyro_z = 0;
  r.humidity = 0;
  r.mic = 0;
  r.batt = 0;
  r.steps = 0;
  return r;
}

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Feather Sense Sensor Demo");
  Serial.print("Data raw record byte size: "); Serial.println(sizeof(RawRecord));
  Serial.print("Data record byte size: "); Serial.println(sizeof(Record));
  Serial.print("size of float: "); Serial.println(sizeof(float));
  Serial.print("size of uint8_t: "); Serial.println(sizeof(uint8_t));
  Serial.print("size of uint16_t: "); Serial.println(sizeof(uint16_t));
  Serial.print("size of int32_t: "); Serial.println(sizeof(int32_t));
  Serial.print("Data record byte size: "); Serial.println(0
    + 4    //   int32_t time; 
    + 1    //   uint8_t proximity;
    + 4*2  //   uint16_t r, g, b, c;
    + 3*4  //   float temperature, pressure, altitude;
    + 3*4  //   float magnetic_x, magnetic_y, magnetic_z;
    + 3*4  //   float accel_x, accel_y, accel_z;
    + 3*4  //   float gyro_x, gyro_y, gyro_z;
    + 1*4  //   float humidity;
    + 1*4  //   int32_t mic;
    + 1*4  //   float batt;
    + 1*1  //   uint8_t steps;
  );

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  rtc.start();

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) {
      flash(10);
      while(1);
    }
  }

  // initialize the sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  lsm6ds33.enablePedometer(true); // Magic happends here :)
  // Reset the pedometer in case the reset button has been pressed and the device was powered before.
  lsm6ds33.resetPedometer();

  sht30.begin();
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);

  // Test writing binary records
  SD.remove("bintest.dat");
  File bintest = SD.open("bintest.dat", FILE_WRITE);

  Record r = new_record();
  for (int32_t i = 0; i<100; i++) {
    r.time++;
    r.proximity++;
    r.r++;
    r.g++;
    r.b++;
    r.c++;
    r.temperature++;
    r.pressure++;
    r.altitude++;
    r.magnetic_x++;
    r.magnetic_y++;
    r.magnetic_z++;
    r.accel_x++;
    r.accel_y++;
    r.accel_z++;
    r.gyro_x++;
    r.gyro_y++;
    r.gyro_z++;
    r.humidity++;
    r.mic++;
    r.batt++;
    r.steps++;
    bintest.write((byte*) &r, sizeof(Record));
  }
  bintest.flush();

  // Insert a comment line to indicate a restart of the board
  File db = SD.open("datalog.txt", FILE_WRITE);
  if (db) {
    db.println("# New record " + date_fmt());
    db.flush();
  }
}

/**
 * Code from https://learn.adafruit.com/adafruit-feather-sense/power-management
 */
float read_batt() {
  // Arduino Example Code snippet

  #define VBATPIN A6

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

// A couple of simple functional style log helpers
inline String add_log(String val) {
  return val + ",";
}

inline String end_log(String val) {
  return val + "\r\n";
}

void loop(void) {

  Record r = new_record();

  r.time = rtc.now().unixtime();

  r.proximity = apds9960.readProximity();
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&r.r, &r.g, &r.b, &r.c);

  r.temperature = bmp280.readTemperature();
  r.pressure = bmp280.readPressure();
  r.altitude = bmp280.readAltitude(1013.25);

  lis3mdl.read();
  r.magnetic_x = lis3mdl.x;
  r.magnetic_y = lis3mdl.y;
  r.magnetic_z = lis3mdl.z;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  r.accel_x = accel.acceleration.x;
  r.accel_y = accel.acceleration.y;
  r.accel_z = accel.acceleration.z;
  r.steps = lsm6ds33.readPedometer();


  r.gyro_x = gyro.gyro.x;
  r.gyro_y = gyro.gyro.y;
  r.gyro_z = gyro.gyro.z;

  r.humidity = sht30.readHumidity();

  samplesRead = 0;
  r.mic = getPDMwave(4000);

  float batt = read_batt();
  r.batt = batt;

  // Log the data to the CSV file

  Serial.println("\nFeather Sense Sensor Demo" + date_fmt());
  Serial.println("---------------------------------------------");
  
  File db = SD.open("datalog.txt", FILE_WRITE);
  File dbin = SD.open("datalog.bin", FILE_WRITE);

  // If there is not an SD card present, flash 3 times and try again
  // TODO: Add a multi stage retry to slowly escalate. Is there a way to know when the SD card is inserted?
  if (!db || !dbin) {
    flash(3);
    delay(1000);
    return;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  String date =
  // Timestamp
    add_log(date_fmt())
  // Serial.print("Proximity: ");
  + add_log(String(r.proximity))
  // Serial.print("Red: ");
  + add_log(String(r.r))
  // Serial.print(" Green: ");
  + add_log(String(r.g))
  // Serial.print(" Blue :");
  + add_log(String(r.b))
  // Serial.print(" Clear: ");
  + add_log(String(r.c))
  // Serial.print("Temperature: ");
  + add_log(String(r.temperature))
  // Serial.println(" C");
  // Serial.print("Barometric pressure: ");
  + add_log(String(r.pressure))
  // Serial.print("Altitude: ");
  + add_log(String(r.altitude))
  // Serial.println(" m");
  // Serial.print("Magnetic: ");
  + add_log(String(r.magnetic_x))
  // Serial.print(" ");
  + add_log(String(r.magnetic_y))
  // Serial.print(" ");
  + add_log(String(r.magnetic_z))
  // Serial.println(" uTesla");
  // Serial.print("Acceleration: ");
  + add_log(String(r.accel_x))
  // Serial.print(" ");
  + add_log(String(r.accel_y))
  // Serial.print(" ");
  + add_log(String(r.accel_z))
  // Serial.println(" m/s^2");
  // Serial.print("Gyro: ");
  + add_log(String(r.gyro_x))
  // Serial.print(" ");
  + add_log(String(r.gyro_y))
  // Serial.print(" ");
  + add_log(String(r.gyro_z))
  // Serial.println(" dps");
  // Serial.print("Humidity: ");
  + add_log(String(r.humidity))
  // Serial.println(" %");
  // Serial.print("Mic: ");
  + add_log(String(r.mic))
  + add_log(String(r.batt))
  + end_log(String(r.steps));

  db.print(date);
  db.flush(); //; make it sure it is written to disk

  Serial.print(date);

  dbin.write((byte*) &r, sizeof(Record));
  dbin.flush();

  digitalWrite(LED_BUILTIN, LOW);

  // Reset the pedometer after a successful write to make sure to keep steps recorded even when there is no SD card
  lsm6ds33.resetPedometer();
  delay(1000);
}

String date_fmt() {
  return rtc.now().timestamp(DateTime::TIMESTAMP_FULL);
}

/*****************************************************************/
int32_t getPDMwave(int32_t samples) {
  short minwave = 30000;
  short maxwave = -30000;

  while (samples > 0) {
    if (!samplesRead) {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++) {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    // clear the read count
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
