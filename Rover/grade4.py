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
clear = lambda: os.system('clear')

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
            accuracy = 10

            if sample[0] < 0.11:
                self.move("forward", 0)
                return

            if math.radians(-accuracy) < sample[1] < math.radians(accuracy):
                self.move("forward", 200)
            elif sample[1] < math.radians(-accuracy):
                self.move("right", 200)
            elif sample[1] > math.radians(accuracy):
                self.move("left", 200)

            
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
    while True:
        observation = current_observation()
        
        samplesRB, landerRB, obstaclesRB, rocksRB = splitObservation(observation)

        print("Range: {}   Bearing: {}".format(samplesRB[0][0], samplesRB[0][1]))
        # clear()

        # Update rover global positio
        rover.updateCurrentPos()

        rover.decision(samplesRB, landerRB, obstaclesRB, rocksRB)

except KeyboardInterrupt:
    motorControl.closePins()
    print("done")