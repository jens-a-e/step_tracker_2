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