#!/usr/bin/python

import time
import RPi.GPIO as GPIO
import math
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

# First encoder
pinE1A = 23
pinE1B = 24

# Second encoder
pinE2A = 25
pinE2B = 8

GPIO.setmode(GPIO.BCM)
GPIO.setup(pinE1A, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(pinE1B, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(pinE2A, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(pinE2B, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

error = 0
count_E1 = 0
count_E2 = 0

Encoder_E1A, Encoder_E1B = GPIO.input(pinE1A), GPIO.input(pinE1B)
Encoder_E1B_old = GPIO.input(pinE1B)

Encoder_E2A, Encoder_E2B = GPIO.input(pinE2A), GPIO.input(pinE2B)
Encoder_E2B_old = GPIO.input(pinE2B)

def encodercount_E1(term):
    global count_E1
    global Encoder_E1A
    global Encoder_E1B
    global Encoder_E1B_old
    global error

    Encoder_E1A,Encoder_E1B = GPIO.input(pinE1A),GPIO.input(pinE1B)

    if((Encoder_E1A,Encoder_E1B_old) == (1,0)) or((Encoder_E1A,Encoder_E1B_old) == (0,1)):
        count_E1 += 1
    elif ((Encoder_E1A,Encoder_E1B_old) == (1,1)) or((Encoder_E1A,Encoder_E1B_old) == (0,0)):
        count_E1 -= 1
    else:
        error += 1

    Encoder_E1B_old = Encoder_E1B

def encodercount_E2(term):
    global count_E2
    global Encoder_E2A
    global Encoder_E2B
    global Encoder_E2B_old
    global error

    Encoder_E2A,Encoder_E2B = GPIO.input(pinE2A),GPIO.input(pinE2B)

    if((Encoder_E2A,Encoder_E2B_old) == (1,0)) or((Encoder_E2A,Encoder_E2B_old) == (0,1)):
        count_E2 += 1
    elif ((Encoder_E2A,Encoder_E2B_old) == (1,1)) or((Encoder_E2A,Encoder_E2B_old) == (0,0)):
        count_E2 -= 1
    else:
        error += 1

    Encoder_E2B_old = Encoder_E2B

GPIO.add_event_detect(pinE1A, GPIO.BOTH, callback=encodercount_E1)
GPIO.add_event_detect(pinE1B, GPIO.BOTH, callback=encodercount_E1)
GPIO.add_event_detect(pinE2A, GPIO.BOTH, callback=encodercount_E2)
GPIO.add_event_detect(pinE2B, GPIO.BOTH, callback=encodercount_E2)

def turnLeft(magnitude):
    board.motor_movement([board.M1], board.CCW, duty)
    board.motor_movement([board.M2], board.CCW, duty)

def turnRight(magnitude):
    board.motor_movement([board.M1], board.CW, duty)
    board.motor_movement([board.M2], board.CW, duty)

def forward(magnitude):
    board.motor_movement([board.M1], board.CCW, duty)
    board.motor_movement([board.M2], board.CW, duty)

def backwards(magnitude):
    board.motor_movement([board.M1], board.CW, duty)
    board.motor_movement([board.M2], board.CCW, duty)

def board_detect(board):
    l = board.detecte()
    print("Board list conform:")
    print(l)


''' print last operate status, users can use this variable to determine the result of a function call. '''


def print_board_status():
    if board.last_operate_status == board.STA_OK:
        print("board status: everything ok")
    elif board.last_operate_status == board.STA_ERR:
        print("board status: unexpected error")
    elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
        print("board status: device not detected")
    elif board.last_operate_status == board.STA_ERR_PARAMETER:
        print("board status: parameter error, last operate no effective")
    elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
        print("board status: unsupport board framware version")

def motorSetup():
    board = Board(1, 0x10) 

    board_detect(board)

    while board.begin() != board.STA_OK:    # Board begin and check board status
        print_board_status()
        print("board begin faild")
        time.sleep(2)
    print("board begin success")

    # Set selected DC motor encoder enable
    board.set_encoder_enable(board.ALL)
    # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8
    board.set_encoder_reduction_ratio(board.ALL, 40)

    # Set DC motor pwm frequency to 1000HZ
    board.set_moter_pwm_frequency(1000)

    return board

board = motorSetup()
duty = 20
ang = 0
r = 0.018559
r2 = 0.137
forward(duty)
time.sleep(3)

while ang < 90:
    turnRight(duty)

    ang2 = ( ( ( (abs(count_E1)/1200)*2*math.pi*r) + ( (abs(count_E2)/1200)*2*math.pi*r) ) / (r2*math.pi) ) * 180 
    if (count_E1 < 0) and (count_E2 > 0):
        ang += ang2
    elif (count_E1 > 0) and (count_E2 < 0):
        ang -= ang2
    print(ang)

forward(duty)
time.sleep(3)

while ang < 180:
    turnRight(duty)

    ang2 = ( ( ( (abs(count_E1)/1200)*2*math.pi*r) + ( (abs(count_E2)/1200)*2*math.pi*r) ) / (r2*math.pi) ) * 180 
    if (count_E1 < 0) and (count_E2 > 0):
        ang += ang2
    elif (count_E1 > 0) and (count_E2 < 0):
        ang -= ang2

forward(duty)
time.sleep(3)

while ang < 270:
    turnRight(duty)

    ang2 = ( ( ( (abs(count_E1)/1200)*2*math.pi*r) + ( (abs(count_E2)/1200)*2*math.pi*r) ) / (r2*math.pi) ) * 180 
    if (count_E1 < 0) and (count_E2 > 0):
        ang += ang2
    elif (count_E1 > 0) and (count_E2 < 0):
        ang -= ang2

forward(duty)
time.sleep(3)   
print("Angle: {:.2f}".format(ang))
count_E1 = 0
count_E2 = 0



print("stop all motor")
board.motor_stop(board.ALL)   # stop all DC motor
print_board_status()