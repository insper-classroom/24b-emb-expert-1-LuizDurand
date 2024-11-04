import serial
import uinput
import time
#rfcomm0
#ttyACM0
ser = serial.Serial('/dev/rfcomm1', 115200)

# Create new mouse device
device = uinput.Device([
    uinput.BTN_LEFT,
    uinput.BTN_RIGHT,
    uinput.REL_X,
    uinput.REL_Y,
    uinput.KEY_W,
    uinput.KEY_A,
    uinput.KEY_S,
    uinput.KEY_D,
    uinput.KEY_SPACE,
    uinput.KEY_ESC,
    uinput.KEY_Q,
    uinput.KEY_E,

])


def parse_data(data):
    axis = data[0]  # 0 for X, 1 for Y
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    print(f"Received data: {data}")
    print(f"axis: {axis}, value: {value}")
    return axis, value


def move_mouse(axis, value):
    if axis == 0:    # X-axis
        # device.emit(uinput.REL_X, value)
        if value > 0:
            device.emit(uinput.KEY_D, 1)
            # time.sleep(0.1)
            # device.emit(uinput.KEY_D, 0)
        elif value < 0:
            device.emit(uinput.KEY_A, 1)
            # time.sleep(0.1)
            # device.emit(uinput.KEY_A, 0)
        elif value == 0:
            device.emit(uinput.KEY_D, 0)
            device.emit(uinput.KEY_A, 0)
    elif axis == 1:  # Y-axis
        # device.emit(uinput.REL_Y, value)
        if value > 0:
            device.emit(uinput.KEY_S, 1)
            # time.sleep(0.1)
            # device.emit(uinput.KEY_S, 0)
        elif value < 0:
            device.emit(uinput.KEY_W, 1)
            # time.sleep(0.1)
            # device.emit(uinput.KEY_W, 0)
        elif value == 0:
            device.emit(uinput.KEY_W, 0)
            device.emit(uinput.KEY_S, 0)

    elif axis == 2:
        if value == 0:
            device.emit(uinput.KEY_SPACE, 0)
            device.emit(uinput.BTN_LEFT, 0)
            device.emit(uinput.BTN_RIGHT, 0)
            device.emit(uinput.KEY_Q, 0)
        elif value == 1:
            device.emit(uinput.KEY_SPACE, 1)
            device.emit(uinput.BTN_LEFT, 0)
            device.emit(uinput.BTN_RIGHT, 0)
            device.emit(uinput.KEY_Q, 0)
        elif value == 10:
            device.emit(uinput.BTN_LEFT, 1)
            device.emit(uinput.BTN_RIGHT, 0)
            device.emit(uinput.KEY_Q, 0)
            device.emit(uinput.KEY_SPACE, 0)
        elif value == 15:
            device.emit(uinput.BTN_RIGHT, 1)
            device.emit(uinput.BTN_LEFT, 0)
            device.emit(uinput.KEY_Q, 0)
            device.emit(uinput.KEY_SPACE, 0)
        elif value == 4:
            device.emit(uinput.KEY_Q, 1)
            device.emit(uinput.BTN_LEFT, 0)
            device.emit(uinput.BTN_RIGHT, 0)
            device.emit(uinput.KEY_SPACE, 0)
    # elif axis == 3:
    #     if value > 0:
    #         device.emit(uinput.BTN_LEFT, 1)
    #         # time.sleep(0.1)
    #         device.emit(uinput.BTN_LEFT, 0)
    # elif axis == 4:
    #     if value > 0:
    #         device.emit(uinput.BTN_RIGHT, 1)
    #         # time.sleep(0.1)
    #         device.emit(uinput.BTN_RIGHT, 0)
    # elif axis == 5:
    #     if value > 0:
    #         device.emit(uinput.KEY_Q, 1)
    #         # time.sleep(0.1)
    #         device.emit(uinput.KEY_Q, 0)
    elif axis == 6:    # X-axis
        device.emit(uinput.REL_X, value)
    elif axis == 7:  # Y-axis
        device.emit(uinput.REL_Y, value)
    elif axis == 8:
        if value == 1:
            device.emit(uinput.BTN_LEFT, 1)
            time.sleep(0.1)
            device.emit(uinput.BTN_LEFT, 0)


try:
    # sync package
    while True:
        print('Waiting for sync package...')
        while True:
            data = ser.read(1)
            if data == b'\xff':
                break
            else:
                print(f"Received error: {data}")

        # Read 4 bytes from UART
        data = ser.read(3)
        #print(data)
        axis, value = parse_data(data)
        move_mouse(axis, value)

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()
        