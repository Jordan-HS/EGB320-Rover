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
    rover = Brain()

    # Init position as (0, 0)
    current_pos = [0, 0]

    while (True):
        delta_x = 0
        delta_y = 0
        radial_vel = 0
        forward_vel = 0
        # Get Detected Objects
        samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()

        # Get current bearing
        bearing = rover.bearing

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
                movement, magnitude = calculateMovement(force_memory[0][0], force_memory[0][1])
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

            movement, magnitude = calculateMovement(delta_x, delta_y)


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
                    movement, magnitude = calculateMovement(force_memory[0][0], force_memory[0][1])
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
                    movement, magnitude = calculateMovement(delta_x, delta_y)
            

        ## Display HUD
        if HUD:
            clear()
            HUD_string = ("#### {} ####\n"
                          "Forward Vel: {}          lander: {}\n"
                          "Radial Vel: {}".format(current_state, forward_vel, landerRB, radial_vel))
            print(HUD_string)


        # Get Detected Wall Points
        wallPoints = lunarBotSim.GetDetectedWallPoints()

        # Move the robot
        moveBot(movement, magnitude)
        
        # Update memory
        rover_mem.update(samplesRB, landerRB, obstaclesRB, rocksRB, movement, magnitude)

        # Update Ball Position
        lunarBotSim.UpdateObjectPositions()

except KeyboardInterrupt as e:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    lunarBotSim.StopSimulator()

def moveBot(movement, magnitude):
    if movement == "forward":
        lunarBotSim.SetTargetVelocities(magnitude, 0)
    elif movement == "left":
        lunarBotSim.SetTargetVelocities(0, -magnitude)
    elif movement == "right":
        lunarBotSim.SetTargetVelocities(0, magnitude)


class Brain:
    def __init__(self):
        lander = [None]
        rocks = [None]
        samples = [None]
        obstacles = [None]
        bearing = 0
        last_time = 0
        speed = 0
        x = 0
        y = 0

    def update(self, samplesRB, landerRB, obstaclesRB, rocksRB, movement, magnitude):
        if samplesRB is None:
            # make an estimation
            samples = updateObjectsPosition
        else:
            samples = checkInMemory(samplesRB, samples)

            # update with current pos

        if landerRB is None:
             # make an estimation
        else:
            # update with current pos
            lander = checkInMemory(landerRB, lander)

        if obstaclesRB is None:
             # make an estimation
        else:
            # update with current pos
            obstacles = checkInMemory(obstaclesRB, obstacles)

        if rocksRB is None:
             # make an estimation
        else:
            # update with current pos
            rocks = checkInMemory(rocksRB, rocks)

        # Update estimation of current pos and bearing

    def checkInMemory(self, objects_seen, objects_memory):
        if objects_memory is None:
                # First samples seen
                for object_seen in objects_seen:
                    objects_memory.append(object_seen)
            
            # Check that the sample being seen isnt already in memory
            for object_seen in objects_seen:
                for object_memory in objects_memory:
                    if (math.isclose(object_seen[0], objects_seen[0], abs_tol=0.01) 
                        and math.isclose(object_seen[1], objects_seen[1], abs_tol=math.pi/12)):
                        objects_seen = object_seen

        return objects_memory

    def updateObjectsPosition(self, objects):


