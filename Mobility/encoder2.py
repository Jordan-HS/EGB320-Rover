#!/usr/bin/python

import time
import RPi.GPIO as GPIO
import math
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

pinA = 23
pinB = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

error = 0
counts = 0

Encoder_A,Enconder_B = GPIO.input(pinA),GPIO.input(pinB)
Encoder_B_old = GPIO.input(pinB)

def reset(addr,tags,stuff,source):
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

    Encoder_A,Encoder_B = GPIO.input(pinA),GPIO.input(pinB)

    if((Encoder_A,Encoder_B_old) == (1,0)) or((Encoder_A,Encoder_B_old) == (0,1)):
        counts += 1

    elif ((Encoder_A,Encoder_B_old) == (1,1)) or((Encoder_A,Encoder_B_old) == (0,0)):
        counts -= 1

    else:
        error += 1

    Encoder_B_old = Encoder_B

GPIO.add_event_detect(pinA, GPIO.BOTH, callback=encodercount)
GPIO.add_event_detect(pinB, GPIO.BOTH, callback=encodercount)

def forward(magnitude):
    board.motor_movement([board.M1], board.CCW, duty)
    board.motor_movement([board.M2], board.CW, duty)

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
r = 0.01925
start = time.time()
while time.time() - start < 12:
    forward(duty)
    
    time.sleep(.1)

distance = counts/1200 * 2*math.pi*r
print("distance: {:.2f}cm".format(distance*100))

print("stop all motor")
board.motor_stop(board.ALL)   # stop all DC motor
print_board_status()