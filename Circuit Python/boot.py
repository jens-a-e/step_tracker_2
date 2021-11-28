"""Typical boot.py file for Circuit Python projects

The prupose of this particular boot.py is to enable the on-chip storage
available in nRF52x boards.

This would allow us to use roughly 1MB for data logging without having to add an
external storage like an sd card.

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
import board
import digitalio
import storage

"""USER_SWITCH setup"""
# switch = digitalio.DigitalInOut(board.SWITCH)
# switch.direction = digitalio.Direction.INPUT
# switch.pull = digitalio.Pull.UP

"""Mount the storage based on the USER_SWITCH: pressed switch is not enable.
If the USER_SWITCH is connected to ground with a wire CircuitPython can write
to the drive.
"""
# storage.remount("/", not switch.value)