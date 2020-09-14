import sys
import RPi.GPIO as GPIO
import tm1637
import socket
import os
import time

# Get the ip address
gw = os.popen("ip -4 route show default").read().split()
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect((gw[2], 0))
ip_address = s.getsockname()[0]
ip_split = ip_address.split(".")

Display = tm1637.TM1637(23,24,tm1637.BRIGHT_TYPICAL)
Display.Clear()
Display.SetBrightnes(1)

while(True):
	for number in ip_split:
		if len(number) == 1:
			digits = [ -1, -1, -1, int(number[0]) ]
		elif len(number) == 2:
			digits = [ -1, -1, int(number[0]), int(number[1]) ]
		elif len(number) == 3:
			digits = [ -1, int(number[0]), int(number[1]), int(number[2]) ] 

		Display.Show(digits)
		time.sleep(1)
