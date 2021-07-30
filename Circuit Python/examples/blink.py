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