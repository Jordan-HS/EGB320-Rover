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
import numpy as np

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
    
    # start = time.time()
    # avg_vel = None
    # dt = 2
    # while time.time() - start < dt:
    #     if avg_vel is None:
    #         avg_vel = getSpeed()
        
    #     avg_vel = (avg_vel+getSpeed())/2

    # return (avg_vel*dt)-0.015



def backwards(magnitude):
    board.motor_movement([board.M1], board.CW, duty)
    board.motor_movement([board.M2], board.CCW, duty)



def getDistance(oldTime):
    speed = getSpeed()
    current_time = time.time()
    # if speed > 0:
    distance = global_distance + (speed * (current_time-oldTime))
    # else:
    #     distance = global_distance
    oldTime = current_time
    return distance



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

def getSpeed():

    speed = board.get_encoder_speed(board.ALL)
    r = 0.01925
    if speed[0] != 0 and speed[1] != 0:
        vel = ((2*math.pi*r)/60)*((speed[0]+speed[0])/2)
    else:
        vel = 0

    return round(vel,2)
board = motorSetup()
global_distance = 0
duty = 20
target = 0.2
oldTime = time.time()
r = 0.01925
if __name__ == "__main__":
    
    Break = False
    
    speeds = np.array([])
    dt = np.array([])
    oldtime = 0
    
    time.sleep(2)
    start = time.time()
    t = 2
    while time.time()-start<t:
        try:
            # speed = board.get_encoder_speed(board.ALL)
            forward(duty)
            
            speed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
            # print("duty: %d, M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(duty, speed[0], speed[1]))
            current = r * ((speed[0]+speed[1])/2) * 0.10472
            # print("speed: {:.3f}".format(current))
            if oldtime == 0:
                # dt = np.append(dt, time.time()-start)
                speeds = np.append(speeds, current)
                oldtime=time.time()
            else:
                # dt = np.append(dt, time.time()-oldtime)
                speeds = np.append(speeds, current)
                oldtime = time.time()

            if current < target:
                duty += 0.02
            elif current > target:
                duty -= 0.02
            print("current: {:.5f}\t Duty: {:.2f}".format(current, duty))
            # if keyboard.is_pressed('W'):
            #     dist = forward(duty)
            #     global_distance += dist
            # elif keyboard.is_pressed('s'):
            #     backwards(duty)
            # elif keyboard.is_pressed('a'):
            #     turnLeft(duty)
            # elif keyboard.is_pressed('d'):
            #     turnRight(duty)
            # elif keyboard.is_pressed('p'):
            #     print(global_distance*100)
            # else:
            #     board.motor_stop(board.ALL)
            # global_distance = getDistance(oldTime)
 
        except(KeyboardInterrupt):
            print("stop all motor")
            board.motor_stop(board.ALL)   # stop all DC motor
            print_board_status()
            Break = True
               
    # distances = np.multiply(speeds, dt)
    val = round(len(speeds)/2 * 10)
    tot_dist = np.mean(speeds[val:]) * t
    # print(dt)
    # print(np.sum(dt))
    # print(speeds)
    print("distance travelled: {:.2f}cm".format(tot_dist*100))
    
    print("stop all motor")
    board.motor_stop(board.ALL)   # stop all DC motor
    print_board_status()
    Break = True