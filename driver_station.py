import threading
import time

import inputs # https://pypi.org/project/inputs/
import busio # https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
import adafruit_rfm9x # https://github.com/adafruit/Adafruit_CircuitPython_RFM9x

# https://github.com/adafruit/circuitpython
import digitalio
import board

connected = False

left_x = left_y = right_x = right_y = 127
a_button = b_button = x_button = y_button = 0
left_bumper = right_bumper = left_trigger = right_trigger = 0
dpad_left = dpad_right = dpad_up = dpad_down = 0
start_button = xbox_button = back_button = 0

def update_gamepad():
    global connected
    global left_x, left_y, right_x, right_y
    global a_button, b_button, x_button, y_button
    global left_bumper, right_bumper, left_trigger, right_trigger
    global dpad_left, dpad_right, dpad_up, dpad_down
    global start_button, xbox_button, back_button

    while True:
        events = ()
        try:
            events = inputs.get_gamepad()
            connected = True
        except OSError:
            connected = False
        for event in events:
            if event.code == "ABS_X": left_x = axis_to_byte(event.state)
            elif event.code == "ABS_Y": left_y = axis_to_byte(event.state)
            elif event.code == "ABS_RX": right_x = axis_to_byte(event.state)
            elif event.code == "ABS_RY": right_y = axis_to_byte(event.state)

            elif event.code == "BTN_SOUTH": a_button = event.state
            elif event.code == "BTN_EAST": b_button = event.state
            elif event.code == "BTN_WEST": y_button = event.state # x is north and y is west
            elif event.code == "BTN_NORTH": x_button = event.state

            elif event.code == "BTN_TL": left_bumper = event.state
            elif event.code == "BTN_TR": right_bumper = event.state
            elif event.code == "ABS_Z": left_trigger = trigger_to_bit(event.state)
            elif event.code == "ABS_RZ": right_trigger = trigger_to_bit(event.state)

            elif event.code == "ABS_HAT0X":
                dpad_left = 1 if event.state == -1 else 0
                dpad_right = 1 if event.state == 1 else 0
            elif event.code == "ABS_HAT0Y":
                dpad_up = 1 if event.state == -1 else 0
                dpad_down = 1 if event.state == 1 else 0

            elif event.code == "BTN_SELECT": back_button = event.state
            elif event.code == "BTN_MODE": xbox_button = event.state
            elif event.code == "BTN_START": start_button = event.state

def axis_to_byte(axis):
    return constrain(round(interpolate(axis, -32768, 32768, 0, 255)), 0, 255)

def trigger_to_bit(trigger):
    return 1 if trigger > 512 else 0

def interpolate(val, in_min, in_max, out_min, out_max):
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def constrain(val, min_val, max_val):
    return min(max(min_val, val), max_val)

def millis():
    return int(round(time.time() * 1000))

def bit_list_to_byte(bits):
    assert len(bits) <= 8
    for i in range(len(bits), 8): bits.append(0)
    byte = 0
    for bit in reversed(bits): byte = (byte << 1) | bit
    return byte

if __name__ == "__main__":
    gamepad_thread = threading.Thread(target=update_gamepad, daemon=True)
    gamepad_thread.start()

    led = digitalio.DigitalInOut(board.D17)
    led.direction = digitalio.Direction.OUTPUT

    lora_cs = digitalio.DigitalInOut(board.CE1)
    lora_reset = digitalio.DigitalInOut(board.D25)
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

    rfm9x = adafruit_rfm9x.RFM9x(spi, lora_cs, lora_reset, 915.0)
    rfm9x.enable_crc = True
    rfm9x.tx_power = 23
    rfm9x.signal_bandwidth = 250000
    rfm9x.spreading_factor = 7
    rfm9x.coding_rate = 5

    prev_ms = millis()
    while 1:
        led.value = connected
        if connected:
            buttons1 = bit_list_to_byte([
                a_button, b_button, x_button, y_button,
                left_bumper, right_bumper, left_trigger, right_trigger
            ])
            buttons2 = bit_list_to_byte([
                dpad_left, dpad_right, dpad_up, dpad_down,
                start_button, back_button, xbox_button
            ])
            packet = bytes([35, left_x, left_y, right_x, buttons1, buttons2, 36])
            rfm9x.send(packet)

            print(millis() - prev_ms, "ms")
            prev_ms = millis()
        else:
            print("Gamepad not connected. Retrying...")