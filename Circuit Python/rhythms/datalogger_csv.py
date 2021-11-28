"""data logger csv - logs sensor data from a Feather Sense board to csv

TODO Use the FeatherSense class we made to imrpve readability of the code

Copyright (C) 2021  jens alexander ewald <jens@poetic.systems>

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <https://www.gnu.org/licenses/>.

------

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under the Marie Skłodowska-Curie grant
agreement No. 813508.
"""

import time
import array
import math
import adafruit_sdcard
import storage
import busio
import board
import audiobusio
import adafruit_apds9960.apds9960
import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_lsm6ds.lsm6ds33
from adafruit_lsm6ds import Rate, AccelRange
import adafruit_sht31d
import adafruit_pcf8523
import digitalio
import neopixel
from analogio import AnalogIn

led = digitalio.DigitalInOut(board.RED_LED)
led.switch_to_output()

# Battery
vbat_voltage = AnalogIn(board.VOLTAGE_MONITOR)

def get_voltage(pin):
    return (pin.value * 3.3) / 65536 * 2

# SD Card
SD_CS_PIN = board.D10  # Pin 10 for Feather Boards
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
sd_cs = digitalio.DigitalInOut(SD_CS_PIN)
sdcard = adafruit_sdcard.SDCard(spi, sd_cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")  # Mount the SD Card under /sd

# Boot switch
switch = digitalio.DigitalInOut(board.SWITCH)
switch.direction = digitalio.Direction.INPUT
switch.pull = digitalio.Pull.UP

# Sensors
i2c = board.I2C()

apds9960 = adafruit_apds9960.apds9960.APDS9960(i2c)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
lis3mdl = adafruit_lis3mdl.LIS3MDL(i2c)

lsm6ds33 = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c)

sht31d = adafruit_sht31d.SHT31D(i2c)

microphone = audiobusio.PDMIn(
    board.MICROPHONE_CLOCK, board.MICROPHONE_DATA, sample_rate=16000, bit_depth=16
)

neo = neopixel.NeoPixel(board.NEOPIXEL, 1, pixel_order=neopixel.RGB)

# The Real Time Clock
rtc = adafruit_pcf8523.PCF8523(i2c)

def normalized_rms(values):
    minbuf = int(sum(values) / len(values))
    return int(
        math.sqrt(
            sum(float(sample - minbuf) * (sample - minbuf) for sample in values)
            / len(values)
        )
    )

# Configure the Sensors
apds9960.enable_proximity = True
apds9960.enable_color = True

# Set this to sea level pressure in hectoPascals at your location
# for accurate altitude reading.
bmp280.sea_level_pressure = 1013.25

lsm6ds33.accelerometer_range = AccelRange.RANGE_2G
lsm6ds33.accelerometer_data_rate = Rate.RATE_26_HZ
# no gyro used for step detection
lsm6ds33.gyro_data_rate = Rate.RATE_SHUTDOWN
lsm6ds33.pedometer_enable = True

def reset_pedometer():
    lsm6ds33.pedometer_enable = False
    lsm6ds33.pedometer_enable = True

def timestamp():
    t = rtc.datetime
    return "{}-{}-{}T{}:{}:{}".format(
        t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec
    )

LOG_FILE = "/sd/data.csv"

def logfile():
    return open(LOG_FILE, "at")

# Prepare log file
with logfile() as file:
    file.write("# New record {}\r\n".format(timestamp()))
    file.flush()

try:
    while True:
        # Turn off the onbaord neopixel
        neo[0] = (0, 0, 0)

        now = timestamp()

        battery_voltage = get_voltage(vbat_voltage)

        samples = array.array("H", [0] * 160)
        microphone.record(samples, len(samples))

        # print("\nTime", now)
        # print("Feather Sense Sensor Demo")
        # print("---------------------------------------------")
        # print("Proximity:", apds9960.proximity)
        # print("Red: {}, Green: {}, Blue: {}, Clear: {}".format(*apds9960.color_data))
        # print("Temperature: {:.1f} C".format(bmp280.temperature))
        # print("Barometric pressure:", bmp280.pressure)
        # print("Altitude: {:.1f} m".format(bmp280.altitude))
        # print("Magnetic: {:.3f} {:.3f} {:.3f} uTesla".format(*lis3mdl.magnetic))
        # print("Acceleration: {:.2f} {:.2f} {:.2f} m/s^2".format(*lsm6ds33.acceleration))
        # print("Gyro: {:.2f} {:.2f} {:.2f} dps".format(*lsm6ds33.gyro))
        # print("Humidity: {:.1f} %".format(sht31d.relative_humidity))
        # print("Sound level:", normalized_rms(samples))
        # print("Batt: {}".format(battery_voltage))
        # print("Steps: {}".format(lsm6ds33.pedometer_steps))

        data = []

        data.append(now)
        data.append(apds9960.proximity)
        data.append("{},{},{},{}".format(*apds9960.color_data))
        data.append(bmp280.temperature)
        data.append(bmp280.pressure)
        data.append(bmp280.altitude)
        data.append("{:.2f},{:.2f},{:.2f}".format(*lis3mdl.magnetic))
        data.append("{:.2f},{:.2f},{:.2f}".format(*lsm6ds33.acceleration))
        data.append("{:.2f},{:.2f},{:.2f}".format(*lsm6ds33.gyro))
        data.append(sht31d.relative_humidity)
        data.append(normalized_rms(samples))
        data.append(battery_voltage)
        data.append(lsm6ds33.pedometer_steps)

        reset_pedometer()

        # Write the data capture to file
        with logfile() as file:
            led.value = True
            file.write(",".join(str(e) for e in data) + "\r\n")
            file.flush()
            led.value = False

        time.sleep(1)

# Trap the system in a blinking state when an error occured
except OSError as e:
    delay = 0.5
    # 28 means out of space
    if e.args[0] == 28:
        delay = 0.25
    while True:
        led.value = not led.value
        time.sleep(delay)
