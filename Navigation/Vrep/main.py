# Import the QUT rover bot library
from roverbot_lib import *
import setup
from math import radians, degrees
from potentialField import getForce, calculateMovement
import time
import numpy as np

# GPIO LED output
LED_out = False

# HUD output
HUD = True

# Potential fields view
POT = False

# Initialise the simulation
robotParameters, sceneParameters = setup.init_sim()

if HUD:
    import os
    import sys
    clear = lambda: os.system('cls')

if LED_out:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(26, GPIO.OUT) # RED
    GPIO.setup(16, GPIO.OUT) # GREEN
    GPIO.setup(13, GPIO.OUT) # YELLOW

class Rover:
    def __init__(self, lunarBotSim):
        self.lander = [0,0]
        self.rocks = None
        self.samples = None
        self.obstacles = None
        self.target = None
        self.target_type = ""
        self.x = 0
        self.y = 0
        self.bearing = 0
        self.current_action = ""
        self.initial = True
        self.initial_bearing = 0
        self.lunarBotSim = lunarBotSim

    def updateCurrentPos(self):
        self.lunarBotSim.GetObjectPositions()
        self.x = self.lunarBotSim.robotPose[0]
        self.y = self.lunarBotSim.robotPose[1]
        self.bearing = self.lunarBotSim.robotPose[5]

    def move(self, movement, magnitude=None):
        if movement == "forward":
            self.lunarBotSim.SetTargetVelocities(magnitude, 0)
        elif movement == "left":
            self.lunarBotSim.SetTargetVelocities(0, magnitude)
        elif movement == "right":
            self.lunarBotSim.SetTargetVelocities(0, -magnitude)
        elif movement == "stop":
            self.lunarBotSim.SetTargetVelocities(0, 0)

    def decision(self, samplesRB, landerRB, obstaclesRB, rocksRB):
        ### Survering scene ###    
        if self.current_action == "Surveying landing site":
            self.map(samplesRB, landerRB, obstaclesRB, rocksRB)
            self.surveySite()
            return

        ### Do this first to gather information ###
        if self.initial:
            ### Initial state - Move off lander ###
            if not self.lunarBotSim.SampleCollected() and self.checkOnLander():
                self.current_action = "On lander - Moving off"
                self.move("forward", 0.5)
                return

            ### Off lander - Survey scene ###
            if (not self.lunarBotSim.SampleCollected() and not self.checkOnLander()):
                self.current_action = "Surveying landing site"
                self.initial_bearing = self.bearing - math.radians(11)
                self.initial = False
                return

        ### Holding sample ###
        if self.lunarBotSim.SampleCollected() and self.target_type != "lander":
            self.current_action = "Holding sample - Navigating back to lander"
            self.target = [0, -0.4]
            self.target_type = "lander"
            return

        ### Pick a target sample ###
        if self.samples is not None and self.target is None:
            dist = 10
            for sample in self.samples:
                if self.distanceToObject(sample) < dist:
                    self.target = sample
                    self.target_type = "sample"
                    dist = self.distanceToObject(sample)
            return

        ### Move towards target using potential fields ### 
        if self.target is not None:
            target_angle, target_mag = getForce(self)
            self.current_action = "Targeting {} \nAngle:{:.2f} \tMag:{:.2f} \tDistance:{:.2f}\nGlobal pos:{}".format(self.target_type, math.degrees(target_angle), target_mag, self.distanceToObject(self.target), self.target)

            if math.isclose(self.bearing, target_angle, abs_tol=math.radians(5)):
                self.move("forward", target_mag)
            elif self.bearing < target_angle:
                self.move("left", target_mag*2)
            else:
                self.move("right", target_mag*2)      
            return

        ### At target sample - Pick up ###
        if self.target_type == "sample" and self.distanceToObject(self.target) < 0.12:
            self.lunarBotSim.CollectSample()
            

        ### stop moving ###
        if self.current_action == "Stopped":
            self.move("stop")

    def checkOnLander(self):
        if (-0.4 < self.x < 0.4) and (-0.4 < self.y < 0.4):
            return True
        return False 

    def surveySite(self):
        # continue moving if not finished surveying site
        if not math.isclose(self.initial_bearing, self.bearing, abs_tol=math.radians(10)):
            self.move("left", 1)
        else:
            self.current_action = "Stopped"
            self.initial_bearing = 0

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
                        if math.isclose(sample_in_mem[0], x, abs_tol=0.05) and math.isclose(sample_in_mem[1], y, abs_tol=0.05):
                            self.samples[index][0] = round((self.samples[index][0] + x)/2, 2)
                            self.samples[index][1] = round((self.samples[index][1] + y)/2, 2)
                            return

                    # Add the new sample
                    self.samples.append([x, y])

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

try:
    # Create VREP RoverBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
    lunarBotSim = VREP_RoverRobot('192.168.1.111', robotParameters, sceneParameters)
    lunarBotSim.StartSimulator()

    # memory stuff
    time.sleep(1)
    rover = Rover(lunarBotSim)

    while (True):
        # Get Detected Objects
        samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()
        
        # Update rover global position
        rover.updateCurrentPos()

        rover.decision(samplesRB, landerRB, obstaclesRB, rocksRB)
             
        ## Display HUD
        if HUD:
            clear()
            print("{}\n\n\n"
                  "Robot x:{:.2f}  Robot y:{:.2f}   Bearing:{:.2f}\n"
                  "Samples at:{}\n"
                  "Rocks at:{}\n"
                  "Obstacles at:{}\n".format(rover.current_action, rover.x, rover.y, math.degrees(rover.bearing), rover.samples,rover.rocks, rover.obstacles))

        # Update Ball Position
        lunarBotSim.UpdateObjectPositions()

except KeyboardInterrupt as e:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    lunarBotSim.StopSimulator()


