#!/usr/bin/env python3
import serial
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyS0', 9600, 8, 'N', 1, timeout=5)
    ser.flush()
    while True:
        # if ser.in_waiting > 0:
        line = ser.readline()
        print(line)