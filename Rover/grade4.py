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
import led

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

    # def updateCurrentPos(self):
    #     motorControl.sendCommand(self.current_movement)
    #     self.x, self.y, self.bearing = motorControl.updatePosition(self)

    def move(self, movement, magnitude=None):
        ## Convert magnitude to duty

        motorControl.move(movement, magnitude)
        self.current_movement = movement

    def decision(self, samplesRB, landerRB, obstaclesRB, rocksRB):
        if Sample_demo:
            
            if samplesRB is not None:
                led.led_state("collecting")
                sample = samplesRB[0]
                
                if sample[0] > 0.25:
                    speed = "normal"
                    accuracy = 10
                else:
                    speed = "slow"
                    accuracy = 3

                # print(speed)

                if self.at_target:
                    tiltdowncollection.down()
                    time.sleep(1)    
                    closecollection.close()
                    time.sleep(1)
                    holdSample.hold()
                    self.done = True
                    led.led_state("SampleReturn")
                else:
                    opencollection.open()
                    holdSample.hold()
                    

                if sample[0] < 0.114 or self.at_target:
                    self.move("forward", "stop")
                    self.at_target = True
                    return

                if math.radians(-accuracy) < sample[1] < math.radians(accuracy):
                    self.move("forward", speed)
                elif sample[1] < math.radians(-accuracy):
                    self.move("right", speed)
                elif sample[1] > math.radians(accuracy):
                    self.move("left", speed)

            else:
                led.led_state("searching")
                self.move("right", "normal")
                return
        elif Rock_demo:
            if rocksRB is not None:
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

                print(rock[0])

                if rock[0] < 0.145 or self.at_target:
                    time.sleep(0.5)
                    self.move("forward", "stop")
                    self.at_target = True
                    return

                if math.radians(-accuracy) < rock[1] < math.radians(accuracy):
                    self.move("forward", speed)
                elif rock[1] < math.radians(-accuracy):
                    self.move("right", speed)
                elif rock[1] > math.radians(accuracy):
                    self.move("left", speed)
            else:
                opencollection.open()
                self.move("right", "normal")
            return
                

        elif obstacle_avoidance:
            opencollection.open()
            tiltupcollection.up()
            angle_scale = 1
            if obstaclesRB is not None:
                obstacle = obstaclesRB[0]

                if obstacle[0] < 0.2:
                    angle_scale = 2.5 
                obs_x, obs_y = self.determinePos(obstacle[0], obstacle[1])
                # print("seem")

                U = potentialField.getForce([0, 0], [0.5, 0], [[obs_x, obs_y]])
                # print(obs)
                self.memory = [obs_x, obs_y]
                self.start_timer = time.time()
            elif self.memory is not None and time.time() - self.start_timer < 1:
                obs_x, obs_y = self.determinePos(self.memory[0], self.memory[1])
                # print("Going off memory")
                U = potentialField.getForce([0, 0], [0.5, 0], [[obs_x, obs_y]])

            else:
                U = [0.1, 0]
                obs_x, obs_y = self.determinePos(U[0], U[1])
            target_angle = motorControl.WrapToPi(math.atan2(U[1], U[0]) * angle_scale)
            # target_angle, target_mag = getForce(self)
            accuracy = 5

            # print("target angle: {:.2f}   current angle: {:.2f}   x: {:.2f}  y: {:.2f}".format(target_angle, self.bearing, self.x, self.y))
            

            if abs(math.atan2(obs_y, obs_x)) > 90:
                self.move("back", "normal")
            elif math.isclose(self.bearing, target_angle, abs_tol=math.radians(accuracy)):
                self.move("forward", "normal")
            elif abs(self.bearing - target_angle) < math.pi:
                if self.bearing - target_angle < 0:
                    self.move("left", "normal")
                elif self.bearing - target_angle > 0:
                    self.move("right", "normal")   
            elif abs(self.bearing - target_angle) > math.pi:
                if self.bearing - target_angle < 0:
                    self.move("right", "normal")
                elif self.bearing - target_angle > 0:
                    self.move("left", "normal")   

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
    led.close()
    print("done")