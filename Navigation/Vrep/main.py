# Import the QUT rover bot library
from roverbot_lib import *
import setup
from math import radians, degrees
from potentialField import getForce, calculateMovement
import time

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

try:
    # Create VREP RoverBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
    lunarBotSim = VREP_RoverRobot('192.168.1.111', robotParameters, sceneParameters)
    lunarBotSim.StartSimulator()

    # memory stuff
    rover_mem = Memory()

    # Init position as (0, 0)
    current_pos = [0, 0]

    while (True):
        delta_x = 0
        delta_y = 0
        radial_vel = 0
        forward_vel = 0
        # Get Detected Objects
        samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()

        if force_memory[0] is not None:
            if time.time() - force_memory[0][2] > 10:
                force_memory = [None]
            else:
                force_memory[0][0] = force_memory[0][0] * (((10 - (time.time() - force_memory[0][2]))/13))
                force_memory[0][1] = force_memory[0][1] * (((10 - (time.time() - force_memory[0][2]))/13))

        
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


        ### INITIAL STATE - MOVING OFF LANDER ###
        if not lunarBotSim.SampleCollected() and landerRB is not None and landerRB[0] < 0.4:
            current_state = "INITIAL STATE - MOVING OFF LANDER"

            forward_vel = 0.5
        ### NO OBJECTS SEEN - SEARCH FOR OBJECT ###
        elif samplesRB is None and obstaclesRB is None and rocksRB is None and not lunarBotSim.SampleCollected():
            current_state = "NO OBJECTS SEEN - SEARCH FOR OBJECT"
            if force_memory[0] is not None:
                radial_vel, forward_vel = calculateMovement(force_memory[0][0], force_memory[0][1])
            else:
                radial_vel = 0
                forward_vel = 0

            radial_vel += 2    # rotate on the spot to search
            forward_vel += 0

            if LED_out:
                # set red LED
                GPIO.output(26, GPIO.HIGH)
                GPIO.output(16, GPIO.LOW)
                GPIO.output(13, GPIO.LOW)

        ### SAMPLE SEEN - NAVIGATE TOWARDS SAMPLE ###
        elif not lunarBotSim.SampleCollected():
            current_state = "SAMPLE SEEN - NAVIGATE TOWARDS SAMPLE"
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

                    if LED_out:
                        # set Yellow LED
                        GPIO.output(26, GPIO.LOW)
                        GPIO.output(16, GPIO.LOW)
                        GPIO.output(13, GPIO.HIGH)

            if rocksRB is not None:
                for rock in rocksRB:
                    rockRange = rock[0]
                    rockBearing = rock[1]
                
                    # Obstacle avoidance
                    if force_memory[0] is not None:
                                delta_x += force_memory[0][0]
                                delta_y += force_memory[0][1]

                    if rockRange < (0.03+0.072):
                        radial_vel = 0
                        forward_vel = 0
                        break
                    else:
                        delta_x, delta_y = getForce("rock", rockRange, rockBearing, [delta_x, delta_y])

            # Check to see if any obstacles are within the camera's FOV
            # if rocksRB is not None:
            #     # loop through each obstacle detected using Pythonian way
            #     for obstacle in obstaclesRB:
            #         obstacleRange = obstacle[0]
            #         obstacleBearing = obstacle[1]

            radial_vel, forward_vel = calculateMovement(delta_x, delta_y)


        ### SAMPLE COLLECTED - NAVIGATE TOWARDS DROP OFF ###
        elif lunarBotSim.SampleCollected():
            current_state = "SAMPLE COLLECTED - NAVIGATE TOWARDS DROP OFF"
            if LED_out:
                # set green LED
                GPIO.output(26, GPIO.LOW)
                GPIO.output(16, GPIO.HIGH)
                GPIO.output(13, GPIO.LOW)
            if landerRB is None:
                if force_memory[0] is not None:
                    radial_vel, forward_vel = calculateMovement(force_memory[0][0], force_memory[0][1])
                else:
                    radial_vel = 0
                    forward_vel = 0

                radial_vel += 1.2    # rotate on the spot to search
                forward_vel += 0.1
            else:
                landerRange = landerRB[0]
                landerBearing = landerRB[1]

                if landerRange < 0.035:
                    lunarBotSim.DropSample()
                else:
                    delta_x, delta_y = getForce("lander", landerRange, landerBearing, [delta_x, delta_y])

                    # Calculate force to drop off
                    if force_memory[0] is not None:
                            delta_x += force_memory[0][0]
                            delta_y += force_memory[0][1]
                    radial_vel, forward_vel = calculateMovement(delta_x, delta_y)
            

        ## Display HUD
        if HUD:
            clear()
            HUD_string = ("#### {} ####\n"
                          "Forward Vel: {}          lander: {}\n"
                          "Radial Vel: {}".format(current_state, forward_vel, landerRB, radial_vel))
            print(HUD_string)


        # Get Detected Wall Points
        wallPoints = lunarBotSim.GetDetectedWallPoints()

        # [forward vel m/s, rotational vel rads/s]
        
        lunarBotSim.SetTargetVelocities(forward_vel/2, radial_vel/2)

        # Update memory
        rover_mem.update(samplesRB, landerRB, obstaclesRB, rocksRB, forward_vel, radial_vel)

        # Update Ball Position
        lunarBotSim.UpdateObjectPositions()

except KeyboardInterrupt as e:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    lunarBotSim.StopSimulator()


class Memory:
    def __init__(self):
        lander = [None]
        rocks = [None]
        sample = [None]
        obstacle = [None]
        last_time = 0
        last_move = 0

    def update(self, samplesRB, landerRB, obstaclesRB, rocksRB):
        if samplesRB is None:
            # 
