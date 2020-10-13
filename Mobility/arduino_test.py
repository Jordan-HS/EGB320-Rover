#!/usr/bin/env python3
import serial

ser = serial.Serial('/dev/ttyS0', 9600, 8, 'N', 1, timeout=5)
ser.flush()

ser.write(str(0).encode('utf-8'))
while True:
    # if ser.in_waiting > 0:
    line = ser.readline()
    print(line)