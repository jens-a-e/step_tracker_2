"""
Simple Circtui Python blink example

Used to test code upload and basic board functionality.
Any blink example in the embedded space should be considered public domain :)

"""

import digitalio
import board
from time import sleep

led = digitalio.DigitalInOut(board.D3)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = True
    sleep(0.1)
    led.value = False
    sleep(1)