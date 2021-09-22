import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
import adafruit_rfm9x

import sys
sys.path.insert(0, "/home/pi/Gamepad")
import Gamepad

def millis():
    return int(round(time.time() * 1000))

def clamp(val, low, high):
    return low if val < low else high if val > high else val

def axis_to_byte(axis):
    assert axis >= -1 and axis <= 1
    return 127 if axis == 0 else clamp(int((axis + 1) * 127.5), 0, 255)

def bit_list_to_byte(arr):
    assert len(arr) <= 8
    for i in range(len(arr), 8): arr.append(False)
    byte = 0
    for bit in arr:
        byte = (byte << 1) | bit
    return byte

led = DigitalInOut(board.D17)
led.direction = Direction.OUTPUT

while True:
    try:
        lastTime = millis()

        # Configure LoRa
        CS = DigitalInOut(board.CE1)
        RESET = DigitalInOut(board.D25)
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
        rfm9x.enable_crc = True
        rfm9x.tx_power = 23
        rfm9x.signal_bandwidth = 250000
        rfm9x.spreading_factor = 7
        rfm9x.coding_rate = 5

        gamepad = Gamepad.Xbox360()
        print("Gamepad connected")

        gamepad.startBackgroundUpdates()

        while True:
            if not gamepad.isConnected():
                raise IOError("Gamepad disconnected")
            led.value = True
            left_x = axis_to_byte(gamepad.axis("LEFT-X"))
            left_y = axis_to_byte(gamepad.axis("LEFT-Y"))
            right_x = axis_to_byte(gamepad.axis("RIGHT-X"))
            buttons = bit_list_to_byte([gamepad.isPressed(button) for button in ["LB", "RB", "A", "B", "X", "Y", "BACK", "START"]])
            packet = bytes([35, left_x, left_y, right_x, buttons, 36])
            print(millis() - lastTime, "ms")
            lastTime = millis()
            rfm9x.send(packet)
    except OSError:
        print("No controller, trying again...")
        led.value = False