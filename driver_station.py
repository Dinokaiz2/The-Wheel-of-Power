import time
import busio
from digitalio import DigitalInOut, Direction, Pull
import board
import adafruit_rfm9x

import sys
sys.path.insert(0, "/home/pi/Gamepad")
import Gamepad

while True:
    try:
        millis = lambda: int(round(time.time() * 1000))
        lastTime = millis()

        # Configure LoRa
        CS = DigitalInOut(board.CE1)
        RESET = DigitalInOut(board.D25)
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
        rfm9x.enable_crc = True
        rfm9x.tx_power = 23
        # rfm9x.signal_bandwidth = 250000
        # rfm9x.spreading_factor = 7
        # rfm9x.coding_rate = 5

        leftX = 0
        leftY = 0
        rightX = 0
        leftBumper = 0
        rightBumper = 0
        dpadLeft = 0
        dpadRight = 0
        dpadUp = 0
        dpadDown = 0
        aButton = 0
        bButton = 0
        xButton = 0
        yButton = 0

        gamepad = Gamepad.Xbox360()
        print('Gamepad connected')

        gamepad.startBackgroundUpdates()
         
        while True:
            leftX = gamepad.axis("LEFT-X")
            leftY = gamepad.axis("LEFT-Y")
            rightX = gamepad.axis("RIGHT-X")
            leftBumper = gamepad.isPressed("LB")
            rightBumper = gamepad.isPressed("RB")
            aButton = gamepad.isPressed("A")
            bButton = gamepad.isPressed("B")
            xButton = gamepad.isPressed("X")
            yButton = gamepad.isPressed("Y")
            packet = "z%0.2f;%0.2f;%0.2f;%d;%d;%d;%d;%d;%d" % (leftX, leftY, rightX, leftBumper, rightBumper, aButton, bButton, xButton, yButton)
            print(packet + ",", millis() - lastTime, "ms")
            lastTime = millis()
            rfm9x.send(bytes(packet, "utf-8"))
    except:
        print("No controller, trying again...")
