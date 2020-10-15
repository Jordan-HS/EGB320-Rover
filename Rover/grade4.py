from math import radians, degrees
import potentialField
import time
import numpy as np
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
import motorControl
import math
from cv_vision import current_observation
import RPi.GPIO as GPIO
import os
import sys
import cv2
clear = lambda: os.system('clear')

display = False

class Rover():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.bearing = 0
        self.ref_x = 0
        self.ref_y = 0
        self.ref_bearing = 0
        self.current_movement = ""
        self.last_movement = ""
        self.at_sample = False

    def updateCurrentPos(self):
        motorControl.sendCommand(self.current_movement)
        self.x, self.y, self.bearing = motorControl.updatePosition(self)

    def move(self, movement, magnitude=None):
        ## Convert magnitude to duty

        motorControl.move(movement, magnitude)
        self.current_movement = movement

    def decision(self, samplesRB, landerRB, obstaclesRB, rocksRB):
        if samplesRB is not None:
            sample = samplesRB[0]
            
            if sample[0] > 0.15:
                speed = 200
                accuracy = 10
            else:
                speed = 120
                accuracy = 3

            if sample[0] < 0.113 or self.at_sample:
                self.move("forward", 0)
                self.at_sample = True
                return

            if math.radians(-accuracy) < sample[1] < math.radians(accuracy):
                self.move("forward", speed)
            elif sample[1] < math.radians(-accuracy):
                self.move("right", speed)
            elif sample[1] > math.radians(accuracy):
                self.move("left", speed)

            
        return

def splitObservation(observation):
    samplesRB = []
    rocksRB = []
    obstaclesRB = []
    rocksRB = []
    landerRB = []
    for obj in observation:
        if obj[1] == "SAMP":
            samplesRB.append([obj[3], obj[2]])
        elif obj[1] == "ROC":
            rocksRB.append([obj[3], obj[2]])
        elif obj[1] == "SAT":
            obstaclesRB.append([obj[3], obj[2]])
        elif obj[1] == "LAND":
            landerRB.append([obj[3], obj[2]])

    # If empty make None
    if len(samplesRB) < 1:
        samplesRB = None
    if len(rocksRB) < 1:
        rocksRB = None
    if len(obstaclesRB) < 1:
        obstaclesRB = None
    if len(landerRB) < 1:
        landerRB = None
    return samplesRB, landerRB, obstaclesRB, rocksRB

try:
    rover = Rover()
    observation = current_observation()
    rover.current_movement = "stop"
    rover.updateCurrentPos()
    time.sleep(2)
    while True:
        observation, img = current_observation()
        
        samplesRB, landerRB, obstaclesRB, rocksRB = splitObservation(observation)

        print("Range: {}   Bearing: {} {}".format(samplesRB[0][0], samplesRB[0][1], rover.at_sample))
        # clear()

        # Update rover global positio
        rover.updateCurrentPos()

        rover.decision(samplesRB, landerRB, obstaclesRB, rocksRB)

        if display:
            cv2.imshow("View", img)
            cv2.waitKey(0)

except KeyboardInterrupt:
    motorControl.closePins()
    print("done")