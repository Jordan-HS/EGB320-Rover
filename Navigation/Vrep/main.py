# Import the QUT rover bot library
from roverbot_lib import *
import setup
from math import radians, degrees
from potentialField import getForce, calculateMovement
import RPi.GPIO as GPIO
import time

# Initialise the simulation
robotParameters, sceneParameters = setup.init_sim()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(26, GPIO.OUT) # RED
GPIO.setup(16, GPIO.OUT) # GREEN
GPIO.setup(13, GPIO.OUT) # YELLOW

try:
    # Create VREP RoverBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
    lunarBotSim = VREP_RoverRobot('172.19.3.123', robotParameters, sceneParameters)
    lunarBotSim.StartSimulator()
    force_memory = [None]

    while (True):
        delta_x = 0
        delta_y = 0
        targetVel = [0,0]
        # Get Detected Objects
        samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()

        if force_memory[0] is not None:
            if time.time() - force_memory[0][2] > 5:
                force_memory = [None]

        # Check to see if any obstacles are within the camera's FOV
        if obstaclesRB is not None:
            # loop through each obstacle detected using Pythonian way
            for obstacle in obstaclesRB:
                obstacleRange = obstacle[0]
                obstacleBearing = obstacle[1]

                # Obstacle avoidance
                if force_memory[0] is not None:
                            delta_x += force_memory[0][0]
                            delta_y += force_memory[0][1]
                delta_x, delta_y = getForce("obstacle", obstacleRange, obstacleBearing, [delta_x, delta_y])
                force_memory[0] = ([delta_x, delta_y, time.time()])


        ### NO OBJECTS SEEN - SEARCH FOR OBJECT ###
        if samplesRB is None and obstaclesRB is None and rocksRB is None and not lunarBotSim.SampleCollected():
            radial_vel = 1.2    # rotate on the spot to search
            forward_vel = 0.1
            # set red LED
            GPIO.output(26, GPIO.HIGH)
            GPIO.output(16, GPIO.LOW)
            GPIO.output(13, GPIO.LOW)

        ### SAMPLE SEEN - NAVIGATE TOWARDS SAMPLE ###
        elif not lunarBotSim.SampleCollected():
            # Check to see if the sample is within the camera's FOV
            if samplesRB is not None:
                # loop through each sample detected using Pythonian way
                for sample in samplesRB:
                    sampleRange = sample[0]
                    sampleBearing = sample[1]

                    # check if sample is in range and there isnt one currently held
                    if sampleRange < 0.03 and not lunarBotSim.SampleCollected():
                        lunarBotSim.CollectSample()
                    elif not lunarBotSim.SampleCollected():
                        if force_memory[0] is not None:
                            delta_x += force_memory[0][0]
                            delta_y += force_memory[0][1]
                        delta_x, delta_y = getForce("sample", sampleRange, sampleBearing, [delta_x, delta_y])
                    # set Yellow LED
                    GPIO.output(26, GPIO.LOW)
                    GPIO.output(16, GPIO.LOW)
                    GPIO.output(13, GPIO.HIGH)

            # Check to see if any obstacles are within the camera's FOV
            if rocksRB is not None:
                # loop through each obstacle detected using Pythonian way
                for obstacle in obstaclesRB:
                    obstacleRange = obstacle[0]
                    obstacleBearing = obstacle[1]

            radial_vel, forward_vel = calculateMovement(delta_x, delta_y)


        ### SAMPLE COLLECTED - NAVIGATE TOWARDS DROP OFF ###
        elif lunarBotSim.SampleCollected():
            # set green LED
            GPIO.output(26, GPIO.LOW)
            GPIO.output(16, GPIO.HIGH)
            GPIO.output(13, GPIO.LOW)
            if landerRB is None:
                radial_vel = 1.2    # rotate on the spot to search
                forward_vel = 0.1
            else:
                landerRange = landerRB[0]
                landerBearing = landerRB[1]

                if landerRange < 0.035:
                    lunarBotSim.DropSample()
                else:
                    # Calculate force to drop off
                    if force_memory[0] is not None:
                            delta_x += force_memory[0][0]
                            delta_y += force_memory[0][1]
                    delta_x, delta_y = getForce("lander", landerRange, landerBearing, [delta_x, delta_y])
                    radial_vel, forward_vel = calculateMovement(delta_x, delta_y)
            

        # Get Detected Wall Points
        wallPoints = lunarBotSim.GetDetectedWallPoints()

        # [forward vel m/s, rotational vel rads/s]
        
        lunarBotSim.SetTargetVelocities(forward_vel/1.5, radial_vel/2)

        # Update Ball Position
        lunarBotSim.UpdateObjectPositions()

except KeyboardInterrupt as e:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    lunarBotSim.StopSimulator()
