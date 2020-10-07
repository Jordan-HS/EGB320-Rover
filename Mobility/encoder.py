#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import threading

panelNum = 1
pinA = 23
pinB = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

error = 0
counts = 0

Encoder_A, Enconder_B = GPIO.input(pinA), GPIO.input(pinB)
Encoder_B_old = GPIO.input(pinB)


def reset(addr, tags, stuff, source):
	global counts
	global error
	counts = 0
	error = 0


def encodercount(term):
    global counts
    global Encoder_A
    global Encoder_B
    global Encoder_B_old
    global error

    Encoder_A, Encoder_B = GPIO.input(pinA), GPIO.input(pinB)

    if (((Encoder_A,Encoder_B_old) == (1,0)) or ((Encoder_A,Encoder_B_old) == (0,1))):
        counts += 1
    elif ((Encoder_A,Encoder_B_old) == (1,1)) or ((Encoder_A,Encoder_B_old) == (0,0)):
        counts -= 1
    else:
        error += 1

    Encoder_B_old = Encoder_B

GPIO.add_event_detect(pinA, GPIO.BOTH, callback=encodercount)
GPIO.add_event_detect(pinB, GPIO.BOTH, callback=encodercount)

try:
	while True:
		print(counts)
		time.sleep(.1)

except KeyboardInterrupt:
    print("closing")
