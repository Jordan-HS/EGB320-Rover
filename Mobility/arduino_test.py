#!/usr/bin/env python3
import serial
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    ser.flush()
    while True:
        # if ser.in_waiting > 0:
        line = ser.readline()
        print(line)