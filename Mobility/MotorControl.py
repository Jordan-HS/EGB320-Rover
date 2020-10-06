# -*- coding:utf-8 -*-

'''
  # DC_Motor_Demo.py
  #
  # Connect board with raspberryPi.
  # Make board power and motor connection correct.
  # Run this demo.
  #
  # Motor 1 will move slow to fast, orientation clockwise, 
  # motor 2 will move fast to slow, orientation count-clockwise, 
  # then fast to stop. loop in few seconds.
  # Motor speed will print on terminal
  #
  # test motor: https://www.dfrobot.com/product-634.html
  #
  # Copyright   [DFRobot](http://www.dfrobot.com), 2016
  # Copyright   GNU Lesser General Public License
  #
  # version  V1.0
  # date  2019-3-26
'''

import time
import math
import keyboard

from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

   # Select bus 1, set address to 0x10


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
    board.set_encoder_reduction_ratio(board.ALL, 35)

    # Set DC motor pwm frequency to 1000HZ
    board.set_moter_pwm_frequency(1000)

    return board

def getSpeed(board, duty):

    speed = board.get_encoder_speed(board.ALL)
    r = 0.01925
    if speed[0] != 0 and speed[1] != 0:
        vel = ((2*math.pi*r)/60)*((speed[0]+speed[0])/2)
    else:
        vel = 0

    return vel

if __name__ == "__main__":

    board = motorSetup()

    duty = 20
    
    Break = False
    while not Break:
        try:
            if keyboard.is_pressed('W'):
                forward(duty)
            elif keyboard.is_pressed('s'):
                backwards(duty)
            elif keyboard.is_pressed('a'):
                turnLeft(duty)
            elif keyboard.is_pressed('d'):
                turnRight(duty)
            else:
                board.motor_stop(board.ALL)
            

            print(getSpeed(board, duty))
        except(KeyboardInterrupt):
            print("stop all motor")
            board.motor_stop(board.ALL)   # stop all DC motor
            print_board_status()
            Break = True
               
