#!/usr/bin/python


# import the soccer bot module - this will include math, time, numpy (as np) and vrep python modules
from roverbot_lib import *

#import any other required python modules


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()

sceneParameters.obstacle0_StartingPosition = [-0.45, 0.5]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
# sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene

sceneParameters.sample0_StartingPosition = [0.5, 0]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
# sceneParameters.sample0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.sample1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.sample2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


# sceneParameters.rock0_StartingPosition = -1  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.rock0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.rock1_StartingPosition = None   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
sceneParameters.rock2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'	# specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the kicker/dribbler in metres
robotParameters.cameraHeightFromFloor = 0.15 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxBallDetectionDistance = 1 # the maximum distance away that you can detect the ball in metres
robotParameters.maxLanderDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres

# Dribbler Parameters
robotParameters.collectorQuality = 1 # specifies how good your sample collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
robotParameters.autoCollectSample = True #specifies whether the simulator automatically collects samples if near the collector 
robotParameters.maxCollectDistance = 0.03 #specificies the operating distance of the automatic collector function. Sample needs to be less than this distance to the collector


# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the VREP Simulator so don't have to Stop it manually when pressing CTRL+C
	try:

		# Create VREP SoccerBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
		lunarBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
		lunarBotSim.StartSimulator()


		while True:
			# move the robot at a forward velocity of 0.1m/s with a rotational velocity of 0.5 rad/s.
			lunarBotSim.SetTargetVelocities(0.1, 0.5)

			# Get Detected Objects
			samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()

			# Check to see if the sample is within the camera's FOV
			if samplesRB != None:
				# loop through each sample detected using Pythonian way
				for sample in samplesRB:
					sampleRange = samplesRB[0]
					sampleBearing = samplesRB[1]

			# Check to see if any obstacles are within the camera's FOV
			if obstaclesRB != None:
				# loop through each obstacle detected using Pythonian way
				for obstacle in obstaclesRB:
					obstacleRange = obstacle[0]
					obstacleBearing = obstacle[1]

			# Check to see if any obstacles are within the camera's FOV
			if rocksRB != None:
				# loop through each obstacle detected using Pythonian way
				for obstacle in obstaclesRB:
					obstacleRange = obstacle[0]
					obstacleBearing = obstacle[1]

			# Get Detected Wall Points
			wallPoints = lunarBotSim.GetDetectedWallPoints()
			if wallPoints == None:
				print("To close to the wall")
			else:
				print("\nDetected Wall Points")
				# print the range and bearing to each wall point in the list
				for point in wallPoints:
					print("\tWall Point (range, bearing): %0.4f, %0.4f"%(point[0], point[1]))


			#do something here with the robot
			targetVel = [0,0]
			lunarBotSim.SetTargetVelocities(targetVel[0], targetVel[1])


			# Update Ball Position
			lunarBotSim.UpdateObjectPositions()

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		lunarBotSim.StopSimulator()



