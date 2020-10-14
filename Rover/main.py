# Import the QUT rover bot library
# from roverbot_lib import *
# import setup
from math import radians, degrees
from potentialField import getForce, calculateMovement, show
import time
import numpy as np
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
import motorControl
import math
from cv_vision import current_observation

# GPIO LED output
LED_out = False

# HUD output
HUD = True

# Potential fields view
POT = False

# # Initialise the simulation
# robotParameters, sceneParameters = setup.init_sim()

if HUD:
    import os
    import sys
    clear = lambda: os.system('clear')

if LED_out:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(26, GPIO.OUT) # RED
    GPIO.setup(16, GPIO.OUT) # GREEN
    GPIO.setup(13, GPIO.OUT) # YELLOW


class Rover:
    def __init__(self):
        self.lander = [0,0]
        self.rocks = None
        self.samples = None
        self.obstacles = None
        self.target = None
        self.unseen = None
        self.POT = POT
        self.target_type = ""
        self.target_speed = 1
        self.x = 0
        self.y = 0
        self.ref_x = 0
        self.ref_y = 0
        self.bearing = 0
        self.ref_bearing = 0
        self.current_action = ""
        self.current_movement = ""
        self.last_movement = None
        self.initial = True
        self.save_bearing = 0
        # self.lunarBotSim = lunarBotSim
        # self.board = motorControl.motorSetup()

    def updateCurrentPos(self):
        self.x, self.y, self.bearing = motorControl.updatePosition(self)

    def move(self, movement, magnitude=None):
        ## Convert magnitude to duty

        motorControl.move(movement, magnitude)
        self.current_movement = movement
        # motorControl.sendCommand(movement)

    def decision(self, samplesRB, landerRB, obstaclesRB, rocksRB):
        ### Survering scene ###    
        if self.current_action == "Surveying landing site":
            self.map(samplesRB, landerRB, obstaclesRB, rocksRB)
            if not math.isclose(self.save_bearing, self.bearing, abs_tol=math.radians(10)):
                self.move("left", 250)
            else:
                self.current_action = "Pick sample"
                self.initial_bearing = 0
                self.target = None
                self.target_type = ""
            return

        ### Do this first to gather information ###
        if self.initial:
            ### Initial state - Move off lander ###
            if not self.sampleCollected() and self.checkOnLander():
                self.current_action = "On lander - Moving off"
                self.move("forward", 150)
                return

            ### Off lander - Survey scene ###
            if (not self.sampleCollected() and not self.checkOnLander()):
                self.current_action = "Surveying landing site"
                self.save_bearing = self.bearing - math.radians(11)
                self.initial = False
                return

        ### Holding sample - Approaching lander ramp###
        if self.sampleCollected() and self.current_action == "Holding sample - Approaching lander ramp" and self.target_type != "lander approach":            
            # Find closest lander ramp and set as target
            self.target = [0, -0.4]
            lander_ramp_pos = [[0, -0.4], [0, 0.4], [0.4, 0], [-0.4, 0]]
            for lander_ramp in lander_ramp_pos:
                if self.distanceToObject(lander_ramp) < self.distanceToObject(self.target):
                    self.target = lander_ramp
            self.target_type = "lander approach"
            self.target_speed = 1
            return

        ### Need to find sample - Search unknowns ###
        if (self.samples is None or len(self.samples) == 0) and self.target is None and self.current_action == "Pick sample":
            # find closest unknown 
            dist = 10
            for unseen in self.unseen:
                if self.distanceToObject(unseen) < dist:
                    self.target = unseen
                    self.target_type = "Searching unknown"
                    self.target_speed = 1
                    self.POT = True
                    dist = self.distanceToObject(unseen)
            return

        ### Searching unknown - Found a new sample ###
        if self.target is not None and self.target_type == "Searching unknown" and (samplesRB is not None and len(samplesRB) > 0):
            self.current_action = "Surveying landing site"
            self.save_bearing = self.bearing - math.radians(11)
            return

        ### Pick a target sample ###
        if self.samples is not None and self.target is None and self.current_action == "Pick sample":
            dist = 10
            for sample in self.samples:
                if self.distanceToObject(sample) < dist:
                    self.target = sample
                    self.target_type = "sample"
                    self.target_speed = 1
                    dist = self.distanceToObject(sample)
            return
        
        ### At target sample - Pick up ###
        # if self.target_type == "sample" and self.target_distance is not None and self.target_distance < 0.04:
        #     self.lunarBotSim.CollectSample()
        #     self.samples.remove(self.target)
        #     self.target = None
        #     self.target_type = ""
        #     self.current_action = "Holding sample - Approaching lander ramp"
        #     return

        ### Close range sample collection ###
        if self.target is not None and self.target_type == "sample" and self.distanceToObject(self.target) < 0.3:
            if samplesRB is None:
                if self.bearingToObject(self.target) < self.bearing:
                    self.move("left", 250)
                    return
                else: 
                    self.move("right", 250)
                    return
            else:
                # Target closest sample
                dist = 10
                target = None
                for sample in samplesRB:
                    if sample[0] < dist:
                        target = sample
                        dist = sample[0]
                target_angle = target[1]
                target_mag = 0.2
                # target_angle, target_mag = getForce(self, closeRange=target)
                self.current_action = "Close range targeting {} \nAngle:{:.2f} \tMag:{:.2f} \tDistance:{:.2f}\nGlobal pos:{}".format(self.target_type, math.degrees(target_angle), target_mag, target[0], self.target)

                if target[0] <= 0.04:
                    # self.lunarBotSim.CollectSample()
                    self.samples.remove(self.target)
                    self.target = None
                    self.target_type = ""
                    self.current_action = "Holding sample - Approaching lander ramp"
                    return

                if math.isclose(target_angle, 0, abs_tol=math.radians(5)):
                    self.move("forward", target_mag/3)
                elif target_angle < -math.radians(5):
                    self.move("right", target_mag*2)
                elif target_angle > math.radians(5):
                    self.move("left", target_mag*2)      
                return
            
        ### At lander ramp - climp lander ###
        if self.target_type == "lander approach" and self.distanceToObject(self.target) < 0.12:
            # Drive up lander to drop off sample
            self.target_type = "drop off"
            self.target = [self.target[0]/4, self.target[1]/4] # basically 0,0  but scaled to 0.1 for offset
            self.target_speed = 1.5
            return

        ### Drop ball ###
        if self.target_type == "drop off" and self.distanceToObject(self.target) < 0.08:
            # self.lunarBotSim.DropSample()
            self.current_action = "Continue mission - Moving to other side of lander"
            # Clear the target
            self.target = None          
            self.target_type = ""
            return

        ### More samples to collect ### 
        if self.current_action == "Continue mission - Moving to other side of lander":
            if self.checkOnLander():
                self.move("forward", 1)
            else:
                self.current_action = "Surveying landing site"
                self.save_bearing = self.bearing - math.radians(11)
            return

        ### Move towards target using potential fields ### 
        if self.target is not None:
            target_angle, target_mag = getForce(self)
            accuracy = 5
            lowSpeedBoost = 1

            target_mag = target_mag*100
            
            # Display potential field graph
            if self.POT:
                show(self)
                self.POT = False

            if target_mag > 2:
                accuracy = 15

            if target_mag < 0.75:
                lowSpeedBoost = 2.5

            if math.isclose(self.bearing, target_angle, abs_tol=math.radians(accuracy)):
                self.move("forward", target_mag*self.target_speed)
            elif abs(self.bearing - target_angle) < math.pi:
                if self.bearing - target_angle < 0:
                    self.move("left", target_mag*self.target_speed*lowSpeedBoost)
                elif self.bearing - target_angle > 0:
                    self.move("right", target_mag*self.target_speed*lowSpeedBoost)   
            elif abs(self.bearing - target_angle) > math.pi:
                if self.bearing - target_angle < 0:
                    self.move("right", target_mag*self.target_speed*lowSpeedBoost)
                elif self.bearing - target_angle > 0:
                    self.move("left", target_mag*self.target_speed*lowSpeedBoost)   
            
            self.current_action = "Targeting {} \nAngle:{:.2f} \tMag:{:.2f} \tDistance:{:.2f}\nGlobal pos:{}\t Movement: {}".format(self.target_type, math.degrees(target_angle), target_mag, self.distanceToObject(self.target), self.target, self.current_movement)
            return

        ### stop moving ###
        if self.current_action == "Stopped":
            self.move("stop")

    def checkOnLander(self):
        if (-0.3 < self.x < 0.3) and (-0.3 < self.y < 0.3):
            return True
        return False 

    def map(self, samplesRB, landerRB, obstaclesRB, rocksRB):
        if samplesRB is not None:
            for sample in samplesRB:
                distance = sample[0] + 0.1  # Add 0.1 for distance to centre of rover
                angle = sample[1]

                x, y = self.determinePos(distance, angle)

                if self.samples is None:
                    self.samples = [[x, y]]
                    return
                else:
                    for index, sample_in_mem in enumerate(self.samples):
                        # If the sample is already in memory then refine the position
                        if math.isclose(sample_in_mem[0], x, abs_tol=0.1) and math.isclose(sample_in_mem[1], y, abs_tol=0.1):
                            self.samples[index][0] = round((self.samples[index][0] + x)/2, 2)
                            self.samples[index][1] = round((self.samples[index][1] + y)/2, 2)
                            return

                    # Add the new sample
                    self.samples.append([x, y])

        if rocksRB is not None:
            for rock in rocksRB:
                distance = rock[0] + 0.1
                angle = rock[1]

                x, y = self.determinePos(distance, angle)

                if self.rocks is None:
                    self.rocks = [[x, y]]
                    return
                else:
                    for index, rock_in_mem in enumerate(self.rocks):
                        if math.isclose(rock_in_mem[0], x, abs_tol=0.3) and math.isclose(rock_in_mem[1], y, abs_tol=0.3):
                            self.rocks[index][0] = round((self.rocks[index][0] + x)/2, 2)
                            self.rocks[index][1] = round((self.rocks[index][1] + y)/2, 2)
                            return

                    # Add new obstacle
                    self.rocks.append([x, y])

        if obstaclesRB is not None:
            for obstacle in obstaclesRB:
                distance = obstacle[0] + 0.1
                angle = obstacle[1]

                x, y = self.determinePos(distance, angle)

                self.addUnseen(distance, angle)

                if self.obstacles is None:
                    self.obstacles = [[x, y]]
                    return
                else:
                    for index, obs_in_mem in enumerate(self.obstacles):
                        if math.isclose(obs_in_mem[0], x, abs_tol=0.3) and math.isclose(obs_in_mem[1], y, abs_tol=0.3):
                            self.obstacles[index][0] = round((self.obstacles[index][0] + x)/2, 2)
                            self.obstacles[index][1] = round((self.obstacles[index][1] + y)/2, 2)
                            return

                    # Add a new obstacle
                    self.obstacles.append([x, y])

    def addUnseen(self, distance, angle):
        further_distance = distance + 0.5

        x, y = self.determinePos(further_distance, angle)

        if self.unseen is None:
            self.unseen = [[x, y]]
            return
        else:
            for index, unseen_in_mem in enumerate(self.unseen):
                if math.isclose(unseen_in_mem[0], x, abs_tol=0.5) and math.isclose(unseen_in_mem[1], y, abs_tol=0.5):
                    self.unseen[index][0] = round((self.unseen[index][0] + x)/2, 2)
                    self.unseen[index][1] = round((self.unseen[index][1] + y)/2, 2)
                    return

            self.unseen.append([x, y])

    def determinePos(self, distance, angle):
        theta = self.bearing + angle

        x_dist = distance * math.cos(theta)
        y_dist = distance * math.sin(theta)

        x = round(self.x + x_dist, 2)
        y = round(self.y + y_dist, 2)

        return x, y

    def distanceToObject(self, vector):
        x_dist = abs(self.x - vector[0])
        y_dist = abs(self.y - vector[1])
        return math.sqrt(x_dist**2 + y_dist**2)

    def bearingToObject(self, vector):
        x_dist = vector[0] - self.x
        y_dist = vector[1] - self.y
        return math.atan2(y_dist, x_dist) + self.bearing

    def sampleCollected(self):
        return False

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
    return samplesRB, landerRB, obstaclesRB, rocksRB

motorControl.sendCommand("clear")
try:
    # Create VREP RoverBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
    # lunarBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
    # lunarBotSim.StartSimulator()

    # memory stuff
    # time.sleep(2)
    rover = Rover()
    observation = current_observation()
    time.sleep(2)

    while (True):
        # Get Detected Objects
        # samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()
        # start = time.time()
        observation = current_observation()
        # print("loop time: {}".format(time.time()-start))

        
        samplesRB, landerRB, obstaclesRB, rocksRB = splitObservation(observation)

        # Update rover global positio
        rover.updateCurrentPos()

        rover.decision(samplesRB, landerRB, obstaclesRB, rocksRB)
        motorControl.sendCommand(rover.current_movement)
        
        
             
        ## Display HUD
        if HUD:
            clear()
            print("{}\n\n\n"
                  "Robot x:{:.2f}  Robot y:{:.2f}   Bearing:{:.2f}\n"
                  "Samples at:{}\n"
                  "Rocks at:{}\n"
                  "Obstacles at:{}\n"
                  "Unseen areas: {}".format(rover.current_action, rover.x, rover.y, math.degrees(rover.bearing), rover.samples,rover.rocks, rover.obstacles, rover.unseen))

        # print("loop time: {}".format(time.time()-start))
        # start = time.time()
        # Update Ball Position
        # lunarBotSim.UpdateObjectPositions()
        # rover.lunarBotSim.UpdateVREPRobot()
        # rover.lunarBotSim.UpdateObjectPositions()
        # rover.lunarBotSim.UpdateSample()

except KeyboardInterrupt:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    # lunarBotSim.StopSimulator()
    print("done")

