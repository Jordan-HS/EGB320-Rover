# Import the QUT rover bot library
from roverbot_lib import *
import setup
from math import radians, degrees

# Initialise the simulation
robotParameters, sceneParameters = setup.init_sim()

try:
    # Create VREP RoverBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
    lunarBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
    lunarBotSim.StartSimulator()

    while (True):
        # move the robot at a forward velocity of 0.1m/s with a rotational velocity of 0.5 rad/s.
        lunarBotSim.SetTargetVelocities(0.1, 0.5)

        # Get Detected Objects
        samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()

        # Check to see if the sample is within the camera's FOV
        if samplesRB is not None:
            # loop through each sample detected using Pythonian way
            for sample in samplesRB:
                sampleRange = sample[0]
                sampleBearing = sample[1]
                print("\nSample dist: " + str(sampleRange))
                print("Sample bearing: " + str(degrees(sampleBearing)))

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


        #do something here with the robot
        targetVel = [0,0]
        lunarBotSim.SetTargetVelocities(targetVel[0], targetVel[1])


        # Update Ball Position
        lunarBotSim.UpdateObjectPositions()

except KeyboardInterrupt as e:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    lunarBotSim.StopSimulator()