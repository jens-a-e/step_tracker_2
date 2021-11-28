"""
Sensor demo for Adafruit Feather Sense. Prints data from each of the sensors.

Copied from:
https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/main/Adafruit_Feather_Sense/code.py
"""

import time
import array
import math
import board
import audiobusio
import adafruit_apds9960.apds9960
import adafruit_bmp280
import adafruit_lis3mdl
import adafruit_lsm6ds.lsm6ds33
from adafruit_lsm6ds import Rate, AccelRange
import adafruit_sht31d
import adafruit_pcf8523

import neopixel

i2c = board.I2C()

apds9960 = adafruit_apds9960.apds9960.APDS9960(i2c)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
lis3mdl = adafruit_lis3mdl.LIS3MDL(i2c)

lsm6ds33 = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c)
lsm6ds33.accelerometer_range = AccelRange.RANGE_2G
lsm6ds33.accelerometer_data_rate = Rate.RATE_26_HZ
# no gyro used for step detection
lsm6ds33.gyro_data_rate = Rate.RATE_SHUTDOWN
lsm6ds33.pedometer_enable = True

sht31d = adafruit_sht31d.SHT31D(i2c)
microphone = audiobusio.PDMIn(
    board.MICROPHONE_CLOCK, board.MICROPHONE_DATA, sample_rate=16000, bit_depth=16
)

neo = neopixel.NeoPixel(board.NEOPIXEL, 1, pixel_order=neopixel.RGB)

# The Real Time Clock
i2c = board.I2C()
rtc = adafruit_pcf8523.PCF8523(i2c)


def normalized_rms(values):
    minbuf = int(sum(values) / len(values))
    return int(
        math.sqrt(
            sum(float(sample - minbuf) * (sample - minbuf) for sample in values)
            / len(values)
        )
    )


apds9960.enable_proximity = True
apds9960.enable_color = True

# Set this to sea level pressure in hectoPascals at your location
# for accurate altitude reading.
bmp280.sea_level_pressure = 1013.25

while True:
    neo[0] = (0, 0, 0)

    t = rtc.datetime
    now = "{}-{}-{}T{}:{}:{}".format(
        t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec
    )

    samples = array.array("H", [0] * 160)
    microphone.record(samples, len(samples))
    print("\nTime", now)
    print("Feather Sense Sensor Demo")
    print("---------------------------------------------")
    print("Proximity:", apds9960.proximity)
    print("Red: {}, Green: {}, Blue: {}, Clear: {}".format(*apds9960.color_data))
    print("Temperature: {:.1f} C".format(bmp280.temperature))
    print("Barometric pressure:", bmp280.pressure)
    print("Altitude: {:.1f} m".format(bmp280.altitude))
    print("Magnetic: {:.3f} {:.3f} {:.3f} uTesla".format(*lis3mdl.magnetic))
    print("Acceleration: {:.2f} {:.2f} {:.2f} m/s^2".format(*lsm6ds33.acceleration))
    print("Gyro: {:.2f} {:.2f} {:.2f} dps".format(*lsm6ds33.gyro))
    print("Steps: {}", lsm6ds33.pedometer_steps)
    print("Humidity: {:.1f} %".format(sht31d.relative_humidity))
    print("Sound level:", normalized_rms(samples))

    time.sleep(1)
