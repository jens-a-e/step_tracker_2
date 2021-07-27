# import board
# import digitalio
# import storage

# switch = digitalio.DigitalInOut(board.SWITCH)
# switch.direction = digitalio.Direction.INPUT
# switch.pull = digitalio.Pull.UP

# # If the USER_SWITCH is connected to ground with a wire
# # CircuitPython can write to the drive
# storage.remount("/", not switch.value)