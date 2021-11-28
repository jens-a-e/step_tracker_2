"""deep sleep testbed for nrf52 chips

Unfortunately the `alarm` module is not available on the nRF52x boards for
Circuit Python

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

import alarm
import time

from adafruit_pcf8523 import PCF8523

# The Real Time Clock
rtc = adafruit_pcf8523.PCF8523(i2c)

def timestamp():
    t = rtc.datetime
    return "{}-{}-{} {}:{}:{}".format(
        t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec
    )


print("Waking up")

# Set an alarm for 60 seconds from now.
time_alarm = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + 1)

# Deep sleep until the alarm goes off. Then restart the program.
alarm.exit_and_deep_sleep_until_alarms(time_alarm)