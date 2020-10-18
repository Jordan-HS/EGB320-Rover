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
import collectionControl
import closecollection
import opencollection
import tiltdowncollection
import tiltupcollection
import holdSample
import liftrock

clear = lambda: os.system('clear')

display = False

Sample_demo = False

Rock_demo = False

obstacle_avoidance = True

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
        self.at_target = False
        self.done = False
        self.memory = None
        self.start_timer = 0
        self.on_lander = True
        self.sample_collected = False
        self.has_ball = False


    def move(self, movement, magnitude=None):
        ## Convert magnitude to duty

        motorControl.move(movement, magnitude)
        self.current_movement = movement

    def decision(self, samplesRB, landerRB, obstaclesRB, rocksRB):
        if self.on_lander:
            holdSample.hold()
            self.move("forward", "normal")

            if time.time() - self.start_timer > 2.5:
                self.on_lander = False
                self.start_time = time.time()

        elif samplesRB is not None and not self.on_lander and not self.has_ball:
            sample = samplesRB[0]
            
            if sample[0] > 0.25:
                speed = "normal"
                accuracy = 10
            else:
                speed = "slow"
                accuracy = 3

            if self.at_target:
                tiltdowncollection.down()
                time.sleep(1)    
                closecollection.close()
                time.sleep(1)
                holdSample.hold()
                self.has_ball = True
            else:
                opencollection.open()
                holdSample.hold()
                
            print(sample[0])

            if sample[0] < 0.135 or self.at_target:
                self.move("stop", "stop")
                self.at_target = True
                return

            if math.radians(-accuracy) < sample[1] < math.radians(accuracy):
                self.move("forward", speed)
            elif sample[1] < math.radians(-accuracy):
                self.move("right", speed)
            elif sample[1] > math.radians(accuracy):
                self.move("left", speed)

        elif self.has_ball and landerRB is not None and not self.sample_collected:
            lander = landerRB[0]
            speed = "normal"
            accuracy = 10
            print(lander[0])
            if lander[0] < 0.5:
                time.sleep(2)
                opencollection.open()
                self.sample_collected = True
            else:
                closecollection.close()
                holdSample.hold()

            if math.radians(-accuracy) < lander[1] < math.radians(accuracy):
                self.move("forward", speed)
            elif lander[1] < math.radians(-accuracy):
                self.move("right", speed)
            elif lander[1] > math.radians(accuracy):
                self.move("left", speed)

        elif self.sample_collected and rocksRB is not None:
            # Look for a rock to flip
            rock = rocksRB[0]
            if rock[0] > 0.25:
                speed = "normal"
                accuracy = 10 
            else:
                speed = "slow"
                accuracy = 3        

            if self.at_target:
                tiltdowncollection.down()
                time.sleep(1)    
                liftrock.lift()
                self.done = True
            else:
                closecollection.close()
                tiltdowncollection.down()      

            if rock[0] < 0.135 or self.at_target:
                self.move("stop", "stop")
                self.at_target = True
                return

            if math.radians(-accuracy) < rock[1] < math.radians(accuracy):
                self.move("forward", speed)
            elif rock[1] < math.radians(-accuracy):
                self.move("right", speed)
            elif rock[1] > math.radians(accuracy):
                self.move("left", speed)
        else:
            self.move("right", "normal")
            return

    def determinePos(self, distance, angle):
        theta = self.bearing + angle

        x_dist = distance * math.cos(theta)
        y_dist = distance * math.sin(theta)

        x = round(self.x + x_dist, 2)
        y = round(self.y + y_dist, 2)

        return x, y

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
    count_disp = 0
    rover = Rover()
    observation = current_observation()
    # rover.current_movement = "stop"
    # rover.updateCurrentPos()
    print("Booted")
    time.sleep(2)
    rover.start_timer = time.time()
    while not rover.done:
        observation, img = current_observation()
        
        samplesRB, landerRB, obstaclesRB, rocksRB = splitObservation(observation)


        # if Sample_demo and samplesRB is not None:
        #     print("Range: {}   Bearing: {} {}".format(samplesRB[0][0], math.degrees(samplesRB[0][1]), rover.at_target))
        # elif Rock_demo and rocksRB is not None:
        #     print("Range: {}   Bearing: {} {}".format(rocksRB[0][0], math.degrees(rocksRB[0][1]), rover.at_target))
        # elif obstacle_avoidance:
        #     # ob_x, ob_y = rover.determinePos(rocksRB[0][0], rocksRB[0][1])
        #     print("x: {}  y:{}  obs at: [{}, {}]".format(rover.x, rover.y, 0, 0))
        # clear()

        # Update rover global positio
        # rover.updateCurrentPos()

        rover.decision(samplesRB, landerRB, obstaclesRB, rocksRB)
        # count_disp += 1
        # if display:
        #     if count_disp == 10:
        #         cv2.imshow("View", img)
        #         count_disp = 0
        #         cv2.waitKey(0)

except KeyboardInterrupt:
    motorControl.close()
    print("done")