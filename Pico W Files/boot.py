"""
boot.py file for Pico data logging example. If pin GPx is not connected to GND when
the pico starts up, make the filesystem writeable by CircuitPython.
"""
import board
import digitalio
import storage

# Use rotary encoder push button GPIO pin as an input to make the system writeable. Normally high, sink to GND when pushing.
# Hold the pin while booting (ie: make it False) to make the pico boot as writeable by the PC.
# Otherwise the pico will boot and have full read/write control over itself to save setpoints.
write_pin = digitalio.DigitalInOut(board.GP12) 
write_pin.direction = digitalio.Direction.INPUT
write_pin.pull = digitalio.Pull.UP

# If write pin is not connected to ground on start-up, CircuitPython can write to CIRCUITPY filesystem.
if write_pin.value:
    storage.remount("/", readonly=False)
