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