import sys

import serial

if __name__ == '__main__':

    fd = serial.Serial('/dev/ttyUSB0', baudrate=4800, timeout=None)

    while True:
        try:
            fd = serial.Serial('/dev/ttyUSB0', baudrate=4800, )
            fd.flushInput()
            data = str(fd.readline())
            print(data)
        except serial.serialutil.SerialException as e:
            # fd = serial.Serial('/dev/ttyUSB0', baudrate=4800)
            print("Device Disconnected | Waiting for Reconnection")
            fd.close()

        except KeyboardInterrupt:
            print("Interrupted | Exiting Execution")
            sys.exit()



    # data = fd.readline()
    # print(ser_bytes)
