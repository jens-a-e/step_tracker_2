"""The FeatherSense class wraps the board API for better handling

Largely based on the original Adafruit example by 2020 Kattni Rembor for Adafruit Industries

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
from clock import RTC

def normalized_rms(values):
    minbuf = int(sum(values) / len(values))
    return int(
        math.sqrt(
            sum(float(sample - minbuf) * (sample - minbuf)
                for sample in values)
            / len(values)
        )
    )


class FeatherSense:
    # Busses
    i2c = board.I2C()

    # The attached clock
    clock = RTC()

    led = digitalio.DigitalInOut(board.RED_LED)

    # Battery
    vbat_voltage = AnalogIn(board.VOLTAGE_MONITOR)

    def get_voltage(self):
        return (self.vbat_voltage.value * 3.3) / 65536 * 2

    # Boot switch
    switch = digitalio.DigitalInOut(board.SWITCH)
    switch.direction = digitalio.Direction.INPUT
    switch.pull = digitalio.Pull.UP

    # Sensors

    apds9960 = adafruit_apds9960.apds9960.APDS9960(i2c)
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)
    lis3mdl = adafruit_lis3mdl.LIS3MDL(i2c)

    lsm6ds33 = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c)

    sht31d = adafruit_sht31d.SHT31D(i2c)

    microphone = audiobusio.PDMIn(
        board.MICROPHONE_CLOCK, board.MICROPHONE_DATA, sample_rate=16000, bit_depth=16
    )

    neo = neopixel.NeoPixel(board.NEOPIXEL, 1, pixel_order=neopixel.RGB)


    def __init__(self) -> None:
        super().__init__()
        self._setup()

    def _setup(self):
        # Configure the Sensors
        self.apds9960.enable_proximity = True
        self.apds9960.enable_color = True

        # Set this to sea level pressure in hectoPascals at your location
        # for accurate altitude reading.
        self.bmp280.sea_level_pressure = 1013.25

        self.lsm6ds33.accelerometer_range = AccelRange.RANGE_2G
        self.lsm6ds33.accelerometer_data_rate = Rate.RATE_26_HZ
        # no gyro used for step detection
        self.lsm6ds33.gyro_data_rate = Rate.RATE_SHUTDOWN
        self.lsm6ds33.pedometer_enable = True

        self.led.switch_to_output()

    def reset_pedometer(self):
        self.lsm6ds33.pedometer_enable = False
        self.lsm6ds33.pedometer_enable = True

    def all_sensors(self):
        now = self.timestamp()

        battery_voltage = self.get_voltage()

        samples = array.array("H", [0] * 160)
        self.microphone.record(samples, len(samples))

        data = []

        data.append(now)
        data.append(self.apds9960.proximity)
        data.append("{},{},{},{}".format(*self.apds9960.color_data))
        data.append(self.bmp280.temperature)
        data.append(self.bmp280.pressure)
        data.append(self.bmp280.altitude)
        data.append("{:.2f},{:.2f},{:.2f}".format(*self.lis3mdl.magnetic))
        data.append("{:.2f},{:.2f},{:.2f}".format(*self.lsm6ds33.acceleration))
        data.append("{:.2f},{:.2f},{:.2f}".format(*self.lsm6ds33.gyro))
        data.append(self.sht31d.relative_humidity)
        data.append(normalized_rms(samples))
        data.append(battery_voltage)
        data.append(self.lsm6ds33.pedometer_steps)

        self.reset_pedometer()
        return data

    def print_sensors(self):
        data = self.all_sensors()

        print("\nFeather Sense Sensor Demo")
        print("---------------------------------------------")
        print("Time", data[0])
        print("Proximity:", data[1])
        print("RGB+Clear:", data[2])
        print("Temperature:", data[3], "C")
        print("Barometric pressure:", data[4])
        print("Altitude:", data[5], "m")
        print("Magnetic", data[6], "uTesla")
        print("Acceleration:", data[7], "m/s^2")
        print("Gyro:", data[8], "dps")
        print("Humidity:", data[9], " %")
        print("Sound level:", data[10])
        print("Batt:", data[11])
        print("Steps:", data[12])
