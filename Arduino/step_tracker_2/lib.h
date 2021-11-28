/**
 * Utility and helper functions for step_tracker_2
 *
 * Copyright (C) 2021  jens alexander ewald <jens@poetic.systems>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 * ------
 * 
 * This project has received funding from the European Union’s Horizon 2020
 * research and innovation programme under the Marie Skłodowska-Curie grant
 * agreement No. 813508.
 */

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

void init_serial() {
  Serial.begin(115200);
  while (!Serial) delay(10);
}

void debug_print_record_sizes() {
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
}

void init_rtc() {
  Serial.println("Initialising RTC...");
  if (! rtc.begin()) {
    Serial.print(" No RTC found! Aborting");
    Serial.flush();
    abort();
  }
  rtc.start();
}

void init_storage() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // Trap in a blinking loop
    while (1) {
      flash(10);
      delay(500);
    }
  }
}

/**
 * Initialise all used sensors
 */
void init_sensors() {
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
}

// A couple of simple functional style log helpers
inline String add_log(String val) {
  return val + ",";
}

inline String end_log(String val) {
  return val + "\r\n";
}

String date_fmt() {
  return rtc.now().timestamp(DateTime::TIMESTAMP_FULL);
}

/**
 * Read current battery level
 * Code from https://learn.adafruit.com/adafruit-feather-sense/power-management
 */
float read_batt() {
  #define VBATPIN A6
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

/**
 * Read data from the sensors into a Record
 */
void read_record(Record* r) {

  r->time = rtc.now().unixtime();

  r->proximity = apds9960.readProximity();
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&r->r, &r->g, &r->b, &r->c);

  r->temperature = bmp280.readTemperature();
  r->pressure = bmp280.readPressure();
  r->altitude = bmp280.readAltitude(1013.25);

  lis3mdl.read();
  r->magnetic_x = lis3mdl.x;
  r->magnetic_y = lis3mdl.y;
  r->magnetic_z = lis3mdl.z;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  r->accel_x = accel.acceleration.x;
  r->accel_y = accel.acceleration.y;
  r->accel_z = accel.acceleration.z;
  r->steps = lsm6ds33.readPedometer();

  r->gyro_x = gyro.gyro.x;
  r->gyro_y = gyro.gyro.y;
  r->gyro_z = gyro.gyro.z;

  r->humidity = sht30.readHumidity();

  samplesRead = 0;
  r->mic = getPDMwave(4000);

  r->batt = read_batt();

}

bool log_record(Record* r) {
  File dbin = SD.open("datalog.bin", FILE_WRITE);
  if (!dbin) {
    return false;
  }
  dbin.write((byte*) r, sizeof(Record));
  dbin.flush();
  return true;
}

bool write_test_record() {
  // Test writing binary records
  SD.remove("bintest.dat");
  File bintest = SD.open("bintest.dat", FILE_WRITE);
  if(!bintest) return false;

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
  return true;
}