"""clock - Simple clock module wrapping adafruit_pcf8523

It adds a convenient `timestamp` method to the clock.

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

import adafruit_pcf8523

# The Real Time Clock
class RTC(adafruit_pcf8523.PCF8523):
    FORMAT = "{}-{}-{} {}:{}:{}"
    @property
    def timestamp(self):
      t = self.datetime
      return self.FORMAT.format(
          t.tm_year, t.tm_mon, t.tm_mday,
          t.tm_hour, t.tm_min, t.tm_sec
      )