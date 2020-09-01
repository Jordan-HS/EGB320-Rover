# Import the QUT rover bot library
from roverbot_lib import *
import setup
from math import radians, degrees
from potentialField import getForce, calculateMovement

# Initialise the simulation
robotParameters, sceneParameters = setup.init_sim()

try:
    # Create VREP RoverBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
    lunarBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
    lunarBotSim.StartSimulator()

    while (True):
        targetVel = [0,0]
        # Get Detected Objects
        samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()

        # Check to see if the sample is within the camera's FOV
        if samplesRB is not None:
            # loop through each sample detected using Pythonian way
            for sample in samplesRB:
                sampleRange = sample[0]
                sampleBearing = sample[1]

                # check if sample is in range and there isnt one currently held
                if sampleRange < 0.035 and not lunarBotSim.SampleCollected():
                    lunarBotSim.CollectSample()
                elif not lunarBotSim.SampleCollected():
                    delta_X, delta_y = getForce("sample", sampleRange, sampleBearing)
                elif lunarBotSim.SampleCollected():
                    forward_vel = -1
                    radial_vel = 0

                

        # Check to see if any obstacles are within the camera's FOV
        if obstaclesRB is not None:
            # loop through each obstacle detected using Pythonian way
            for obstacle in obstaclesRB:
                obstacleRange = obstacle[0]
                obstacleBearing = obstacle[1]

        # Check to see if any obstacles are within the camera's FOV
        if rocksRB is not None:
            # loop through each obstacle detected using Pythonian way
            for obstacle in obstaclesRB:
                obstacleRange = obstacle[0]
                obstacleBearing = obstacle[1]

        # Get Detected Wall Points
        wallPoints = lunarBotSim.GetDetectedWallPoints()
        # if wallPoints is None:
        #     print("To close to the wall")
        # else:
        #     print("\nDetected Wall Points")
        #     # print the range and bearing to each wall point in the list
        #     for point in wallPoints:
        #         print("\tWall Point (range, bearing): %0.4f, %0.4f"%(point[0], point[1]))


        # State: No objects in the scene
        if samplesRB is None and obstaclesRB is None and rocksRB is None:
            radial_vel = 1.2    # rotate on the spot to search
            forward_vel = 0.1
        elif not lunarBotSim.SampleCollected():
            # [forward vel m/s, rotational vel rads/s]
            radial_vel, forward_vel = calculateMovement(delta_X, delta_y)
        lunarBotSim.SetTargetVelocities(forward_vel, radial_vel)


        # Update Ball Position
        lunarBotSim.UpdateObjectPositions()

except KeyboardInterrupt as e:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    lunarBotSim.StopSimulator()