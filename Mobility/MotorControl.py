#!/usr/bin/python

import time
import RPi.GPIO as GPIO
import math
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
import threading

GPIO.setmode(GPIO.BCM)

class EncoderCounter(threading.Thread):
    def __init__(self):
      threading.Thread.__init__(self)
      # First encoder
      self.pinE1A = 23
      self.pinE1B = 24

      # Second encoder
      self.pinE2A = 25
      self.pinE2B = 8

      GPIO.setup(self.pinE1A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
      GPIO.setup(self.pinE1B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
      GPIO.setup(self.pinE2A, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
      GPIO.setup(self.pinE2B, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

      GPIO.add_event_detect(self.pinE1A, GPIO.BOTH, callback=self.encodercount_E1)
      GPIO.add_event_detect(self.pinE1B, GPIO.BOTH, callback=self.encodercount_E1)
      GPIO.add_event_detect(self.pinE2A, GPIO.BOTH, callback=self.encodercount_E2)
      GPIO.add_event_detect(self.pinE2B, GPIO.BOTH, callback=self.encodercount_E2)

      self.error = 0
      self.count_E1 = 0
      self.count_E2 = 0

      self.Encoder_E1A, self.Encoder_E1B = GPIO.input(self.pinE1A), GPIO.input(self.pinE1B)
      self.Encoder_E1B_old = GPIO.input(self.pinE1B)

      self.Encoder_E2A, self.Encoder_E2B = GPIO.input(self.pinE2A), GPIO.input(self.pinE2B)
      self.Encoder_E2B_old = GPIO.input(self.pinE2B)


    def encodercount_E1(self, term):
        # global count_E1
        # global Encoder_E1A
        # global Encoder_E1B
        # global Encoder_E1B_old
        # global error

        self.Encoder_E1A, self.Encoder_E1B = GPIO.input(self.pinE1A), GPIO.input(self.pinE1B)

        if((self.Encoder_E1A, self.Encoder_E1B_old) == (1, 0)) or ((self.Encoder_E1A, self.Encoder_E1B_old) == (0, 1)):
            self.count_E1 += 1
        elif ((self.Encoder_E1A, self.Encoder_E1B_old) == (1, 1)) or ((self.Encoder_E1A, self.Encoder_E1B_old) == (0, 0)):
            self.count_E1 -= 1
        else:
            self.error += 1

        self.Encoder_E1B_old = self.Encoder_E1B


    def encodercount_E2(self, term):
        # global count_E2
        # global Encoder_E2A
        # global Encoder_E2B
        # global Encoder_E2B_old
        # global error

        self.Encoder_E2A, self.Encoder_E2B = GPIO.input(self.pinE2A), GPIO.input(self.pinE2B)

        if((self.Encoder_E2A, self.Encoder_E2B_old) == (1, 0)) or ((self.Encoder_E2A, self.Encoder_E2B_old) == (0, 1)):
            self.count_E2 += 1
        elif ((self.Encoder_E2A, self.Encoder_E2B_old) == (1, 1)) or ((self.Encoder_E2A, self.Encoder_E2B_old) == (0, 0)):
            self.count_E2 -= 1
        else:
            self.error += 1

        self.Encoder_E2B_old = self.Encoder_E2B





def turnLeft(board, magnitude):
    board.motor_movement([board.M1], board.CW, duty)
    board.motor_movement([board.M2], board.CCW, duty)


def turnRight(board, magnitude):
    board.motor_movement([board.M1], board.CCW, duty)
    board.motor_movement([board.M2], board.CW, duty)


def forward(board, magnitude):
    board.motor_movement([board.M1], board.CCW, duty)
    board.motor_movement([board.M2], board.CCW, duty)


def stop(board):
    board.motor_stop(board.ALL)


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

enc = EncoderCounter()
board = motorSetup()
duty = 20
# ang = 0
# r = 0.018559
# r2 = 0.137
start = time.time()
forward(board, duty)
while time.time() - start < 3:
    print(enc.count_E2)

print("stop all motor")
board.motor_stop(board.ALL)   # stop all DC motor
print_board_status()
