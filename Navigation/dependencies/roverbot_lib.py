#import any required python modules
import vrep
import time
import math
import numpy as np
import sys
from enum import IntEnum


################################
####### SOCCER BOT CLASS #######
################################



class VREP_RoverRobot(object):
	"""docstring for VREP_SoccerBot"""
	
	####################################
	####### VREP SOCCER BOT INIT #######
	####################################

	def __init__(self, vrep_server_ip, robotParameters, sceneParameters):
		# Robot Parameters
		self.robotParameters = robotParameters
		self.leftWheelBias = 0
		self.rightWheelBias = 0

		# Scene Paramaters
		self.sceneParameters = sceneParameters

		# VREP Simulator Client ID
		self.clientID = None

		# VREP Object Handle Variables
		self.robotHandle = None
		self.cameraHandle = None
		self.leftMotorHandle = None 		# left and right used for differential drive
		self.rightMotorHandle = None
		self.v60MotorHandle = None 			# 60, 180, 300 used for omni drive
		self.v180MotorHandle = None
		self.v300MotorHandle = None
		self.sampleHandles = [None, None, None]
		self.obstacleHandles = [None, None, None]
		self.rockHandles = [None, None, None]
		self.landerHandle = None


		# Wheel Bias
		if self.robotParameters.driveSystemQuality != 1:
			# randomly generate a left and right wheel bias
			self.leftWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)
			self.rightWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)

		# Obstacle Parameters
		self.obstacleSize = 0.18 # diameter of obstacle

		# sample Parameters
		self.sampleSize = 0.05 # diameter of sample

		# Variables to hold object positions
		self.robotPose = None
		self.cameraPose = None
		self.samplePositions = [None, None, None]
		self.landerPosition = None
		self.obstaclePositions = [None, None, None]
		self.rockPositions = [None, None, None]

		# Variable to hold whether the sample has been joined to the robot
		self.sampleConnectedToRobot = False

		# Attempt to Open Connection to VREP API Server
		self.OpenConnectionToVREP(vrep_server_ip)

		# Attempt To Get VREP Object Handles
		self.GetVREPObjectHandles()

		# Send Robot Parameters to VREP and set the scene
		self.UpdateVREPRobot()
		self.SetScene()

		



	########################################
	####### SOCCER BOT API FUNCTIONS #######
	########################################
	# THESE ARE THE FUNCTIONS YOU SHOULD CALL.
	# ALL OTHER FUNCTIONS ARE HELPER FUNCTIONS.

	# Starts the VREP Simulator. 
	# The VREP Simulator can also be started manually by pressing the Play Button in VREP.
	def StartSimulator(self):
		print('Attempting to Start the Simulator')
		if vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot_wait) != 0:
			print('An error occurred while trying to start the simulator via the Python API. Terminating Program!')
			print('Comment out calls to StartSimulator() and start the simulator manully by pressing the Play button in VREP.')
			sys.exit(-1)
		else:
			print('Successfully started the VREP Simulator.')

		# Setup streaming modes to each object
		vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_streaming)
		vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_streaming)
		vrep.simxGetObjectPosition(self.clientID, self.cameraHandle, -1, vrep.simx_opmode_streaming)
		vrep.simxGetObjectPosition(self.clientID, self.landerHandle, -1, vrep.simx_opmode_streaming)
		
		res,resolution,image=vrep.simxGetVisionSensorImage(self.clientID,self.cameraHandle,0,vrep.simx_opmode_streaming)


		for handle in self.obstacleHandles:
			vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_streaming)

		for handle in self.sampleHandles:
			vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_streaming)

		for handle in self.rockHandles:
			vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_streaming)

		time.sleep(1)

		#initialise local copy of object positions
		self.GetObjectPositions()


	# Stops the VREP Simulator. 
	# The VREP Simulator can also be stopped manually by pressing the Stop Button in VREP.
	def StopSimulator(self):
		print('Attempting to Stop the Simulator')
		if vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait) != 0:
			print('Could not stop the simulator. You can stop the simulator manually by pressing the Stop button in VREP.')
		else:
			print('Successfully stoped the VREP Simulator.')

		# Stop streaming modes to each object
		vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_discontinue)
		vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_discontinue)
		vrep.simxGetObjectPosition(self.clientID, self.cameraHandle, -1, vrep.simx_opmode_discontinue)
		vrep.simxGetObjectPosition(self.clientID, self.landerHandle, -1, vrep.simx_opmode_discontinue)

		for handle in self.obstacleHandles:
			vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_discontinue)

		for handle in self.sampleHandles:
			vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_discontinue)

		for handle in self.rockHandles:
			vrep.simxGetObjectPosition(self.clientID, handle, -1, vrep.simx_opmode_discontinue)

	# Gets the Range and Bearing to All Detected Objects.
	# returns:
	#	sampleRangeBearing - range and bearing to the samples with respect to the camera, will return None if the object is not detected
	#	landerRangeBearing - range and bearing to the landerwith respect to the camera, will return None if the object is not detected
	#	obstaclesRangeBearing - range and bearing to the obstacles with respect to the camera, will return None if the object is not detected
	def GetDetectedObjects(self):
		# Variables used to return range and bearing to the objects
		sampleRangeBearing = None
		landerRangeBearing = None
		obstaclesRangeBearing = None
		rocksRangeBearing = None

		# Make sure the camera's pose is not none
		if self.cameraPose != None:

			#check which objects are currently in FOV using object detection sensor within VREP sim
			retCode,objectsDetected,_,_,_ = vrep.simxCallScriptFunction(self.clientID, 'Robot', vrep.sim_scripttype_childscript, 'getObjectsInView',[],[],[],bytearray(),vrep.simx_opmode_blocking)
		
			if objectsDetected != []:

				# check to see if sample is in field of view
				for index, samplePosition in enumerate(self.samplePositions):
				
					if samplePosition != None:
											
						#if detected calculate range and bearing from camera pose location
						if objectsDetected[lunar_object.sample0 + index] == True  and self.PointInsideArena(samplePosition):
							_range, _bearing = self.GetRBInCameraFOV(samplePosition)
													
							# make sampleRangeBearing into empty lists, if currently set to None
							if sampleRangeBearing == None:
								sampleRangeBearing = []

							# check range is not to far away
							if _range < self.robotParameters.maxsampleDetectionDistance:
								sampleRangeBearing.append([_range, _bearing])
					
					
				# check to see which obstacles are within the field of view
				for index, obstaclePosition in enumerate(self.obstaclePositions):
					if obstaclePosition != None:

						# check to see if the current obstacle is in the FOV and within the field. If so add to detected obstacle range bearing list
						if objectsDetected[lunar_object.obstacle0 + index] == True and self.PointInsideArena(obstaclePosition):
							_range, _bearing = self.GetRBInCameraFOV(obstaclePosition)

							# make obstaclesRangeBearing into empty lists, if currently set to None
							if obstaclesRangeBearing == None:
								obstaclesRangeBearing = []

							if _range <  self.robotParameters.maxObstacleDetectionDistance:
								obstaclesRangeBearing.append([_range, _bearing])

				# check to see which rocks are within the field of view
				for index, rockPosition in enumerate(self.rockPositions):
					if rockPosition != None:

						# check to see if the current obstacle is in the FOV and within the field. If so add to detected obstacle range bearing list
						if objectsDetected[lunar_object.rock0 + index] == True and self.PointInsideArena(rockPosition):
							_range, _bearing = self.GetRBInCameraFOV(rockPosition)

							# make obstaclesRangeBearing into empty lists, if currently set to None
							if rocksRangeBearing == None:
								rocksRangeBearing = []

							if _range <  self.robotParameters.maxRockDetectionDistance:
								rocksRangeBearing.append([_range, _bearing])


				# check to see if yellow lander is in field of view
				if self.landerPosition != None:
					
					#if detected calculate range and bearing from camera pose location
					if objectsDetected[lunar_object.lander] == True:
						_range, _bearing = self.GetRBInCameraFOV(self.landerPosition)
						
						# check range is not to far away
						if _range < self.robotParameters.maxLanderDetectionDistance:
							landerRangeBearing = [_range, _bearing]


		return sampleRangeBearing, landerRangeBearing, obstaclesRangeBearing, rocksRangeBearing


	def GetCameraImage(self):

		if self.cameraHandle == None:
			None, None
	
		res,resolution,image=vrep.simxGetVisionSensorImage(self.clientID,self.cameraHandle,0,vrep.simx_opmode_buffer)
		
		if res==vrep.simx_return_ok:
			return resolution, image    
		else:
			return None, None
			# res=vrep.simxSetVisionSensorImage(clientID,v1,image,0,vrep.simx_opmode_oneshot)
	
	# Gets the Range and Bearing to the wall(s)
	# returns:
	#	None - if there are no valid wall points (i.e. the robot is right up against a wall and facing it)
	#	A list of [range, bearing] arrays. There will either be 1, 2, or 3 [range, bearing] arrays depending on the situation
	#		will return 1 range-bearing array if the robot is close to a wall but not directly facing it and one edge of the camera's view limit is up against the wall, while the other can see part of the field
	#		will return 2 range-bearing array if the robot can see the wall but is not facing a corner
	#		will return 3 range-bearing array if the robot can see the wall and is facing into a corner
	def GetDetectedWallPoints(self):
		wallPoints = None

		if self.cameraPose == None:
			return None
		
		cameraPose2D = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]

		# Get range and bearing to the valid points at limit of camera's view (no occlusions)
		wallPoints = self.CameraViewLimitsRangeAndBearing(cameraPose2D)
		if wallPoints == None:
			# return None to indicate to close to wall or because some maths error and didn't get 1 or 2 valid intersection points 
			# (hopefully a maths error doesn't occur and believe all cases have been taken care of)
			return None

		# See if a corner is within the field of view (no occlusions)
		cornerRangeBearing = self.FieldCornerRangeBearing(cameraPose2D)
		if cornerRangeBearing == []:
			return wallPoints

		wallPoints.append(cornerRangeBearing)
		return wallPoints
		

	# Set Target Velocities
	# inputs:
	#	x - the velocity of the robot in the forward direction (in m/s)
	#	theta_dt - the rotational velocity of the robot (in rad/s)
	def SetTargetVelocities(self, x_dot, theta_dot):
		
		# Need to set based on drive system type
		if self.robotParameters.driveType == 'differential':
			# ensure wheel base and wheel radius are set as these are not allowed to be changed
			self.robotParameters.wheelBase = 0.15
			self.robotParameters.wheelRadius = 0.04
		
			# determine minimum wheel speed based on minimumLinear and maximumLinear speed
			minWheelSpeed = self.robotParameters.minimumLinearSpeed / self.robotParameters.wheelRadius
			maxWheelSpeed = self.robotParameters.maximumLinearSpeed / self.robotParameters.wheelRadius

			# calculate left and right wheel speeds in rad/s
			leftWheelSpeed = (x_dot - 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.leftWheelBias
			rightWheelSpeed = (x_dot + 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.rightWheelBias

			# add gaussian noise to the wheel speed
			if self.robotParameters.driveSystemQuality != 1:
				leftWheelSpeed = np.random.normal(leftWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)
				rightWheelSpeed = np.random.normal(rightWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)

			# ensure wheel speeds are not greater than maximum wheel speed
			leftWheelSpeed = min(leftWheelSpeed, maxWheelSpeed)
			rightWheelSpeed = min(rightWheelSpeed, maxWheelSpeed)

			# set wheel speeds to 0 if less than the minimum wheel speed
			if abs(leftWheelSpeed) < minWheelSpeed:
				leftWheelSpeed = 0
			if abs(rightWheelSpeed) < minWheelSpeed:
				rightWheelSpeed = 0

			# set motor speeds
			errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotorHandle, leftWheelSpeed, vrep.simx_opmode_oneshot_wait)
			errorCode = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotorHandle, rightWheelSpeed, vrep.simx_opmode_oneshot_wait) 
			if errorCode != 0:
				print('Failed to set left and/or right motor speed. Error code %d'%errorCode)

		

	# Returns true if the sample is within the collector
	# returns:
	#	true - if sample is in the collector
	def SampleCollected(self):
		return self.sampleConnectedToRobot

	
	def DropSample(self):
		if self.sampleConnectedToRobot:
	 			vrep.simxCallScriptFunction(self.clientID, 'Robot', vrep.sim_scripttype_childscript, 'JoinRobotAndSample',[0],[0.0],[],bytearray(),vrep.simx_opmode_blocking)
	 			self.sampleConnectedToRobot = False

	# Use this to force a physical connection between sample and rover
	# Ideally use this if no collector has been added to your robot model
	# if two samples are within the distance it will attempt to collect both
	# Outputs:
	#		Success - returns True if a sample was collected or false if not
	def CollectSample(self):

		for _, samplePosition in enumerate(self.samplePositions):

			sampleDist = self.CollectorToSampleDistance(samplePosition)

			if sampleDist != None and sampleDist < self.robotParameters.maxCollectDistance and self.sampleConnectedToRobot == False:
				# make physical connection between sample and robot to simulate collector
				vrep.simxCallScriptFunction(self.clientID, 'Robot', vrep.sim_scripttype_childscript, 'JoinRobotAndSample',[1],[self.robotParameters.maxCollectDistance],[],bytearray(),vrep.simx_opmode_blocking)
				self.sampleConnectedToRobot = True
				
		
		return self.sampleConnectedToRobot




	# Update Object Positions - call this in every loop of your navigation code (or at the frequency your vision system runs at). 
	# This is required to get correct range and bearings to objects.
	# This function also emulates the collector. The function returns the global pose/position of the robot and the objects too. 
	# However, you should not use these return values in your nagivation code, they are there to help you debug if you wish.
	# returns: 
	#		robotPose - a 6 element array representing the robot's pose (x,y,z,roll,pitch,yaw), or None if was not successfully updated from VREP
	#		samplePosition - a 3 element array representing the sample's position (x,y,z), or None if was not successfully updated from VREP
	#		obstaclePositions - a 3 element list, with each index in the list containing a 3 element array representing the sample's position (x,y,z), or None if was not successfully updated from VREP
	def UpdateObjectPositions(self):
		# attempt to get object positions from VREP
		self.GetObjectPositions()

		# update sample
		self.UpdateSample()

		# return object positions		
		return self.robotPose, self.samplePositions, self.obstaclePositions, self.rockPositions


	#########################################
	####### VREP API SERVER FUNCTIONS #######
	#########################################
	# These functions are called within the init function

	# Open connection to VREP API Server
	def OpenConnectionToVREP(self, vrep_server_ip):
		# Close any open connections to vrep in case any are still running in the background
		print('Closing any existing VREP connections.')
		vrep.simxFinish(-1)

		# Attempt to connect to vrep API server
		print('Attempting connection to VREP API Server.')
		self.clientID = vrep.simxStart(vrep_server_ip, 19997, True, True, 5000, 5)
		if self.clientID != -1:
			print('Connected to VREP API Server.')
		else:
			print('Failed to connect to VREP API Server. Terminating Program')
			sys.exit(-1)


	# Get VREP Object Handles
	def GetVREPObjectHandles(self):
		# attempt to get vrep object handles
		errorCode = self.GetRobotHandle()
		if errorCode != 0:
			print('Failed to get Robot object handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode = self.GetCameraHandle()
		if errorCode != 0:
			print('Failed to get Vision Sensor object handle. Terminating Program. Error Code %d'%(errorCode))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetMotorHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get Motor object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetsampleHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get sample object handles. Terminating Program. Error Code %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetRockHandle()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get rock object handles. Terminating Program. Error Code %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)

		landerErrorCode = self.GetLanderHandle()
		if landerErrorCode != 0:
			print('Failed to get lander object handles. Terminating Program. Error Codes %d'%(landerErrorCode))
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetObstacleHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get Obstacle object handles. Terminating Program. Error Codes %d, %d, %d'%(errorCode1, errorCode2, errorCode3))
			sys.exit(-1)


	############################################
	####### VREP OBJECT HANDLE FUNCTIONS #######
	############################################
	# These functions are called by the GetVREPObjectHandles function

	# Get VREP Robot Handle
	def GetRobotHandle(self):
		errorCode, self.robotHandle = vrep.simxGetObjectHandle(self.clientID, 'Robot', vrep.simx_opmode_oneshot_wait)
		return errorCode


	# Get VREP Camera Handle
	def GetCameraHandle(self):
		errorCode, self.cameraHandle = vrep.simxGetObjectHandle(self.clientID, 'VisionSensor', vrep.simx_opmode_oneshot_wait)
		return errorCode

			
	# Get VREP Motor Handles
	def GetMotorHandles(self):
		errorCode1 = 0
		errorCode2 = 0
		errorCode3 = 0

		if self.robotParameters.driveType == 'differential':
			errorCode1, self.leftMotorHandle = vrep.simxGetObjectHandle(self.clientID, 'LeftMotor', vrep.simx_opmode_oneshot_wait)
			errorCode2, self.rightMotorHandle = vrep.simxGetObjectHandle(self.clientID, 'RightMotor', vrep.simx_opmode_oneshot_wait)
		
		return errorCode1, errorCode2, errorCode3

	# Get VREP Lander Handles
	def GetLanderHandle(self):
		landerErrorCode, self.landerHandle = vrep.simxGetObjectHandle(self.clientID, 'Lander', vrep.simx_opmode_oneshot_wait)	
		return landerErrorCode

	# Get VREP sample Handle
	def GetsampleHandles(self):
		error0Code, self.sampleHandles[0] = vrep.simxGetObjectHandle(self.clientID, 'Sample_0', vrep.simx_opmode_oneshot_wait)
		error1Code, self.sampleHandles[1] = vrep.simxGetObjectHandle(self.clientID, 'Sample_1', vrep.simx_opmode_oneshot_wait)
		error2Code, self.sampleHandles[2] = vrep.simxGetObjectHandle(self.clientID, 'Sample_2', vrep.simx_opmode_oneshot_wait)
		return error0Code, error1Code, error2Code

	# Get VREP sample Handle
	def GetRockHandle(self):
		error0Code, self.rockHandles[0] = vrep.simxGetObjectHandle(self.clientID, 'Rock_0', vrep.simx_opmode_oneshot_wait)
		error1Code, self.rockHandles[1] = vrep.simxGetObjectHandle(self.clientID, 'Rock_1', vrep.simx_opmode_oneshot_wait)
		error2Code, self.rockHandles[2] = vrep.simxGetObjectHandle(self.clientID, 'Rock_2', vrep.simx_opmode_oneshot_wait)
		return error0Code, error1Code, error2Code

	# Get VREP Obstacle Handles
	def GetObstacleHandles(self):
		obs0ErrorCode, self.obstacleHandles[0] = vrep.simxGetObjectHandle(self.clientID, 'Obstacle_0', vrep.simx_opmode_oneshot_wait)
		obs1ErrorCode, self.obstacleHandles[1] = vrep.simxGetObjectHandle(self.clientID, 'Obstacle_1', vrep.simx_opmode_oneshot_wait)
		obs2ErrorCode, self.obstacleHandles[2] = vrep.simxGetObjectHandle(self.clientID, 'Obstacle_2', vrep.simx_opmode_oneshot_wait)
		return obs0ErrorCode, obs1ErrorCode, obs2ErrorCode


	###############################################
	####### ROBOT AND SCENE SETUP FUNCTIONS #######
	###############################################
	# These functions are called within the init function

	# Updates the robot within VREP based on the robot paramters
	def UpdateVREPRobot(self):
		# Set Camera Pose and Orientation
		self.SetCameraPose(self.robotParameters.cameraDistanceFromRobotCenter, self.robotParameters.cameraHeightFromFloor, self.robotParameters.cameraTilt)
		self.SetCameraOrientation(self.robotParameters.cameraOrientation)

	# Sets the position of the sample, robot and obstacles based on parameters
	def SetScene(self):
		
		# move sample to starting position
		for index, samplePosition in enumerate([self.sceneParameters.sample0_StartingPosition, self.sceneParameters.sample1_StartingPosition, self.sceneParameters.sample2_StartingPosition]):

			if samplePosition != -1:
				if samplePosition != None:
					vrepStartingPosition = [samplePosition[0], samplePosition[1], 0.725]
					vrep.simxSetObjectPosition(self.clientID, self.sampleHandles[index], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
				else:
					vrep.simxSetObjectPosition(self.clientID, self.sampleHandles[index], -1, [2, 0.2 + (-0.15*index), 0.8125], vrep.simx_opmode_oneshot_wait)
		
		for index, obstaclePosition in enumerate([self.sceneParameters.obstacle0_StartingPosition, self.sceneParameters.obstacle1_StartingPosition, self.sceneParameters.obstacle2_StartingPosition]):

			if obstaclePosition != -1:
				if obstaclePosition != None:
					vrepStartingPosition = [obstaclePosition[0], obstaclePosition[1], 0.725]
					vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[index], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
				else:
					vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[index], -1, [2,  -0.3 + (-0.175*index), 0.8125], vrep.simx_opmode_oneshot_wait)
		
		for index, rockPosition in enumerate([self.sceneParameters.rock0_StartingPosition, self.sceneParameters.rock1_StartingPosition, self.sceneParameters.rock2_StartingPosition]):

			if rockPosition != -1:
				if rockPosition != None:
					vrepStartingPosition = [rockPosition[0], rockPosition[1], 0.725]
					vrep.simxSetObjectPosition(self.clientID, self.rockHandles[index], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
				else:
					vrep.simxSetObjectPosition(self.clientID, self.rockHandles[index], -1, [2, 0.7 + (-0.15*index), 0.8125], vrep.simx_opmode_oneshot_wait)
		
		# move obstacle 0 to starting position
		# if self.sceneParameters.obstacle0_StartingPosition != -1:
		# 	if self.sceneParameters.obstacle0_StartingPosition != None:
		# 		vrepStartingPosition = [self.sceneParameters.obstacle0_StartingPosition[0], self.sceneParameters.obstacle0_StartingPosition[1], 0.8125]
		# 		vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[0], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
		# 	else:
		# 		vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[0], -1, [2, 0, 0.8125], vrep.simx_opmode_oneshot_wait)
			
		# # move obstacle 1 to starting position
		# if self.sceneParameters.obstacle1_StartingPosition != -1:
		# 	if self.sceneParameters.obstacle1_StartingPosition != None:
		# 		vrepStartingPosition = [self.sceneParameters.obstacle1_StartingPosition[0], self.sceneParameters.obstacle1_StartingPosition[1], 0.8125]
		# 		vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[1], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
		# 	else:
		# 		vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[1], -1, [2, -0.3, 0.8125], vrep.simx_opmode_oneshot_wait)
			
		# # move obstacle 2 to starting position
		# if self.sceneParameters.obstacle2_StartingPosition != -1:
		# 	if self.sceneParameters.obstacle2_StartingPosition != None:
		# 		vrepStartingPosition = [self.sceneParameters.obstacle2_StartingPosition[0], self.sceneParameters.obstacle2_StartingPosition[1], 0.8125]
		# 		vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[2], -1, vrepStartingPosition, vrep.simx_opmode_oneshot_wait)
		# 	else:
		# 		vrep.simxSetObjectPosition(self.clientID, self.obstacleHandles[2], -1, [2, -0.6, 0.8125], vrep.simx_opmode_oneshot_wait)
			

	### CAMERA FUNCTIONS ###

	# Sets the camera's pose
	# Inputs:
	#		x - distance between the camera and the center of the robot in the direction of the front of the robot
	#		z - height of the camera relative to the floor in metres
	#		pitch - tilt of the camera in radians
	def SetCameraPose(self, x, z, pitch):
		# assume the students want the camera in the center of the robot (so no y)
		# assume the student only wants to rotate the camera to point towards the ground or sky (so no roll or yaw)

		# update robot parameters
		self.robotParameters.cameraDistanceFromRobotCenter = x
		self.robotParameters.cameraHeightFromFloor = z
		self.robotParameters.cameraTilt = pitch

		# Need to change Z as in VREP the robot frame is in the center of the Cylinder
		# z in VREP robot frame = z - (cylinder height)/2 - wheel diameter
		z = z - 0.075 - 2*self.robotParameters.wheelRadius

		# Need to change the pitch by adding pi/2 (90 degrees) as pitch of 0 points up
		pitch = pitch + math.pi/2.0

		# set camera pose
		vrep.simxSetObjectPosition(self.clientID, self.cameraHandle, vrep.sim_handle_parent, [x,0,z], vrep.simx_opmode_oneshot_wait)
		vrep.simxSetObjectOrientation(self.clientID, self.cameraHandle, vrep.sim_handle_parent, [0,pitch,math.pi/2.0], vrep.simx_opmode_oneshot_wait)

	
	# Sets the camera's height relative to the floor in metres
	def SetCameraHeight(self, z):
		self.SetCameraPose(0, z, 0)


	# Sets the distance between the camera and the center of the robot in the direction of front of the robot
	def SetCameraOffsetFromRobotCentre(self, x):
		self.SetCameraPose(x, 0, 0)


	# Sets the tilt of the camera in radians
	def SetCameraTilt(self, pitch):
		self.SetCameraPose(0, 0, pitch)


	# Set Camera Orientation to either portrait or landscape
	def SetCameraOrientation(self, orientation):
		# get resolution based on orientation
		if orientation == 'portrait':
			x_res = 480
			y_res = 640
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle * x_res / y_res
		elif orientation == 'landscape':
			x_res = 640
			y_res = 480
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle * y_res / x_res
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle
		else:
			print('The camera orientation %s is not known. You must specify either portrait or landscape')
			return


		# update robot parameters
		self.robotParameters.cameraOrientation = orientation

		# set resolution of camera (vision sensor object) - resolution parameters are int32 parameters
		vrep.simxSetObjectIntParameter(self.clientID, self.cameraHandle, vrep.sim_visionintparam_resolution_x, x_res, vrep.simx_opmode_oneshot_wait)
		vrep.simxSetObjectIntParameter(self.clientID, self.cameraHandle, vrep.sim_visionintparam_resolution_y, y_res, vrep.simx_opmode_oneshot_wait)
		

	####################################
	####### API HELPER FUNCTIONS #######
	####################################	

	# Prints the pose/position of the objects in the scene
	def PrintObjectPositions(self):
		print("\n\n***** OBJECT POSITIONS *****")
		if self.robotPose != None:
			print("Robot 2D Pose (x,y,theta): %0.4f, %0.4f, %0.4f"%(self.robotPose[0], self.robotPose[1], self.robotPose[2]))

		if self.cameraPose != None:
			print("Camera 3D Pose (x,y,z,roll,pitch,yaw): %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f"%(self.cameraPose[0], self.cameraPose[1], self.cameraPose[2], self.cameraPose[3], self.cameraPose[4], self.cameraPose[5]))
			
		for index, sample in enumerate(self.samplePositions):	
			if sample != None:
				print("sample %d Position (x,y,z): %0.4f, %0.4f, %0.4f"%(index, sample[0], sample[1], sample[2]))
			
		if self.landerPosition != None:
			print("Lander Position (x,y,z): %0.4f, %0.4f, %0.4f"%(self.landerPosition[0], self.landerPosition[1], self.landerPosition[2]))
			
		for index, obstacle in enumerate(self.obstaclePositions):
			if obstacle != None:
				print("Obstacle %d Position (x,y,z): %0.4f, %0.4f, %0.4f"%(index, obstacle[0], obstacle[1], obstacle[2]))

		for index, rock in enumerate(self.rockPositions):
			if rock != None:
				print("Rock %d Position (x,y,z): %0.4f, %0.4f, %0.4f"%(index, rock[0], rock[1], rock[2]))


	# Gets the pose/position in the global coordinate frame of all the objects in the scene.
	# Stores them in class variables. Variables will be set to none if could not be updated
	def GetObjectPositions(self):
		# Set camera pose and object position to None so can check in an error occurred
		self.robotPose = None
		self.cameraPose = None
		self.samplePositions = [None, None, None]
		self.landerPosition = None
		self.obstaclePositions = [None, None, None]
		self.rockPositions = [None, None, None]

		# GET 2D ROBOT POSE
		errorCode, robotPosition = vrep.simxGetObjectPosition(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
		errorCode, robotOrientation = vrep.simxGetObjectOrientation(self.clientID, self.robotHandle, -1, vrep.simx_opmode_buffer)
		if errorCode == 0:
			self.robotPose = [robotPosition[0], robotPosition[1], robotPosition[1], robotOrientation[0], robotOrientation[1], robotOrientation[2]]

		# GET 3D CAMERA POSE
		errorCode, cameraPosition = vrep.simxGetObjectPosition(self.clientID, self.cameraHandle, -1, vrep.simx_opmode_buffer)
		if errorCode == 0:
			self.cameraPose = [cameraPosition[0], cameraPosition[1], cameraPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
		

		# GET POSITION OF EACH OBJECT
		samplePositions = [None, None, None]
		for index, sample in enumerate(self.samplePositions):
			errorCode, samplePositions[index] = vrep.simxGetObjectPosition(self.clientID, self.sampleHandles[index], -1, vrep.simx_opmode_buffer)
			if errorCode == 0:
				self.samplePositions[index] = samplePositions[index]

		# lander position
		errorCode, landerPosition = vrep.simxGetObjectPosition(self.clientID, self.landerHandle, -1, vrep.simx_opmode_buffer)
		if errorCode == 0:
			self.landerPosition = landerPosition


		# obstacle positions
		obstaclePositions = [None, None, None]
		for index, obs in enumerate(self.obstaclePositions):
			errorCode, obstaclePositions[index] = vrep.simxGetObjectPosition(self.clientID, self.obstacleHandles[index], -1, vrep.simx_opmode_buffer)
			if errorCode == 0:
				self.obstaclePositions[index] = obstaclePositions[index]

		# rock positions
		rockPositions = [None, None, None]
		for index, rock in enumerate(self.rockPositions):
			errorCode, rockPositions[index] = vrep.simxGetObjectPosition(self.clientID, self.rockHandles[index], -1, vrep.simx_opmode_buffer)
			if errorCode == 0:
				self.rockPositions[index] = rockPositions[index]


	# Checks to see if an Object is within the field of view of the camera
	def GetRBInCameraFOV(self, objectPosition):
		# calculate range and bearing on 2D plane - relative to the camera
		cameraPose2d = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose2d, objectPosition)

		# angle from camera's axis to the object's position
		# verticalAngle = math.atan2(objectPosition[2]-self.cameraPose[2], _range)

		#OLD code needs removing

		# # check to see if in field of view
		# if abs(_bearing) > (self.horizontalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# if abs(verticalAngle) > (self.verticalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# return True to indicate is in FOV and range and bearing
		return _range, _bearing

	
	# Determines if a 2D point is inside the arena, returns true if that is the case
	def PointInsideArena(self, position):
		if position[0] > -1 and position[0] < 1 and position[1] > -1 and position[1] < 1:
			return True

		return False


	# Update the sample
	def UpdateSample(self):
		for index, samplePosition in enumerate(self.samplePositions):
		
			if samplePosition != None:

				sampleDist = self.CollectorToSampleDistance(samplePosition)

				# See if need to connect/disconnect sample from robot
				if self.robotParameters.autoCollectSample == True and sampleDist != None and sampleDist < self.robotParameters.maxCollectDistance and self.sampleConnectedToRobot == False:
					# make physical connection between sample and robot to simulate collector
					vrep.simxCallScriptFunction(self.clientID, 'Robot', vrep.sim_scripttype_childscript, 'JoinRobotAndSample',[1],[self.robotParameters.maxCollectDistance],[],bytearray(),vrep.simx_opmode_blocking)
					self.sampleConnectedToRobot = True

				elif self.sampleConnectedToRobot == True:
					# random chance to disconnect
					if np.random.rand() > self.robotParameters.collectorQuality:
						# terminate connection between sample and robot to simulate collector
						vrep.simxCallScriptFunction(self.clientID, 'Robot', vrep.sim_scripttype_childscript, 'JoinRobotAndSample',[0],[self.robotParameters.maxCollectDistance],[],bytearray(),vrep.simx_opmode_blocking)
						self.sampleConnectedToRobot = False

				elif sampleDist != None and sampleDist > 0.03:
					self.sampleConnectedToRobot = False

	
	# Gets the range and bearing to a corner that is within the camera's field of view.
	# Will only return a single corner, as only one corner can be in the field of view with the current setup.
	# returns:
	#	a list containing a [range, bearing] or an empty list if no corner is within the field of view
	def FieldCornerRangeBearing(self, cameraPose):
		rangeAndBearing = []

		# Get range and bearing from camera's pose to each corner
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		return rangeAndBearing


	# Gets the range and bearing to where the edge of camera's field of view intersects with the arena walls.
	# returns:
	#	None - if there are no valid wall points (i.e. the robot is right up against a wall and facing it)
	#	A list of [range, bearing] arrays. There will either be 1 or 2 [range, bearing] arrays depending on the situation
	#		will return 1 if the robot is close to a wall but not directly facing it and one edge of the camera's view limit is up against the wall, while the other can see part of the field
	#		will return 2 if the robot can see the wall but is not facing a corner
	def CameraViewLimitsRangeAndBearing(self, cameraPose):
		viewLimitIntersectionPoints = []
		rangeAndBearings = []

		# Get valid camera view limit points along the east wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'east')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the north wall (wall in positive y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'north')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the west wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'west')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the south wall (wall in negative y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'south')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Calculate range and bearing to the valid view limit wall intersection points and store in a list
		for point in viewLimitIntersectionPoints:
			_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, point)
			rangeAndBearings.append([_range, _bearing])

		# return None if rangeAndBearings list is empty
		if rangeAndBearings == []:
			return None
		else:
			return rangeAndBearings

	
	# Gets the points where the edges of the camera's field of view intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	p1 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	#	p2 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	def CameraViewLimitWallIntersectionPoints(self, cameraPose, wall):
		
		# calculate range to wall along camera's axis using the point where the camera's axis intersects with the specified wall
		x, y = self.CameraViewAxisWallIntersectionPoint(cameraPose, wall)
		centreRange = math.sqrt(math.pow(cameraPose[0]-x, 2) + math.pow(cameraPose[1]-y, 2))


		# determine camera view limit intersection points on wall
		if wall == 'east' or wall == 'west':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 + cameraPose[2])
		elif wall == 'north' or wall == 'south':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(cameraPose[2] - self.horizontalViewAngle/2.0)


		# add d1 and d2 (or subtract) to the camera's axis wall intersection point (add/subtract and x/y depends on wall)
		if wall == 'east' or wall == 'west':
			p1 = [x, y+d1]
			p2 = [x, y-d2]
		elif wall == 'north' or wall == 'south':
			p1 = [x-d1, y]
			p2 = [x+d2, y]

		# determine camera view limit intersection point range and bearings relative to camera
		range1, bearing1 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p1)
		range2, bearing2 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p2)

		# Check that the two view limit intersection points are valid (i.e. occur on the arena boundary and not outside, that the bearing is within view and the range is greater than a minimum distance)
		# Need to add small percentage to the angle due to the numerical evaluation of VREP this is to ensure that after checking against all walls that 2 points are returned this is where the *1.05 comes from
		# make sure p1 is within bounds and that bearing is valid
		if (p1[0] < -1 or p1[0] > 1 or p1[1] < -1 or p1[1] > 1):
			p1 = None
		elif abs(bearing1) > (self.horizontalViewAngle/2.0)*1.05:
			p1 = None
		elif range1 < self.robotParameters.minWallDetectionDistance:
			p1 = None
		
		# make sure p2 is within bounds
		if (p2[0] < -1 or p2[0] > 1 or p2[1] < -1 or p2[1] > 1):
			p2 = None
		elif abs(bearing2) > (self.horizontalViewAngle/2.0)*1.05:
			p2 = None
		elif range2 < self.robotParameters.minWallDetectionDistance:
			p2 = None

		return p1, p2


	# Gets the point where the camera's view axis (centre of image) intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	x - the x coordinate where the camera's axis intersects with the specified wall
	#	y - the y coordinate where the camera's axis intersects with the specified wall
	def CameraViewAxisWallIntersectionPoint(self, cameraPose, wall):
		if wall == 'east':
			x = 1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]
		
		elif wall == 'north':
			y = 1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		elif wall == 'west':
			x = -1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]

		elif wall == 'south':
			y = -1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		return x, y
	

	# Wraps input value to be between -pi and pi
	def WrapToPi(self, radians):
		return ((radians + math.pi) % (2* math.pi) - math.pi)

	# Gets the range and bearing given a 2D pose (x,y,theta) and a point(x,y). 
	# The bearing will be relative to the pose's angle
	def GetRangeAndBearingFromPoseAndPoint(self, pose, point):
		_range = math.sqrt(math.pow(pose[0] - point[0], 2) + math.pow(pose[1] - point[1], 2))
		_bearing = self.WrapToPi(math.atan2((point[1]-pose[1]), (point[0]-pose[0])) - pose[2])

		return _range, _bearing


	# Gets the orthogonal distance (in metres) from the collector to the sample. 
	# Assuming the the sample's centroid is within 70 degrees of the collector's centroid
	def CollectorToSampleDistance(self, samplePosition):
		# get the position of the sample relative to the collector motor
		if self.robotPose != None and samplePosition != None:
			# get the pose of the collector in the x-y plane using the robot's pose with some offsets
			collectorPose = [self.robotPose[0]+0.1*math.cos(self.robotPose[5]), self.robotPose[1]+0.1*math.sin(self.robotPose[5]), self.robotPose[5]]

			# get range and bearing from collector to sample position
			_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(collectorPose, samplePosition)

			# check to see if the bearing to the sample is larger than 70 degrees. If so return None
			if abs(_bearing) > math.radians(70):
				return None

			# return distance to sample from collector orthogonal to collector's rotational axis
			return abs(_range * math.cos(_bearing))

		return None


####################################
###### SCENE PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group VREP scene parameters together

class SceneParameters(object):
	"""docstring for SceneParameters"""
	def __init__(self):
		# sample Starting Position

		# Obstacles Starting Positions - set to none if you do not want a specific obstacle in the scene
		self.sample0_StartingPosition = [0.5, 0]  # starting position of sample 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
		self.sample1_StartingPosition = [0, 0.675]   # starting position of sample 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
		self.sample2_StartingPosition = None   # starting position of sample 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


		# Obstacles Starting Positions - set to none if you do not want a specific obstacle in the scene
		self.obstacle0_StartingPosition = [-0.45, 0.5]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
		self.obstacle1_StartingPosition = [-0.25,-0.675]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
		self.obstacle2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


		# Obstacles Starting Positions - set to none if you do not want a specific obstacle in the scene
		self.rock0_StartingPosition = [0.675, -0.25]  # starting position of rock 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
		self.rock1_StartingPosition = None   # starting position of rock 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
		self.rock2_StartingPosition = None   # starting position of rock 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


####################################
###### ROBOT PARAMETERS CLASS ######
####################################

# This class is a helper class to simply 
# group robot parameters together

class RobotParameters(object):
	"""docstring for RobotParameters"""
	def __init__(self):

		# Body Paramaters
		self.robotSize = 0.15 # This parameter cannot be changed
		
		# Drive/Wheel Parameters
		self.driveType = 'differential'	# specifies the drive type ('differential' is the only type currently)
		self.wheelBase = 0.150 # This parameter should not be changed
		self.wheelRadius = 0.04 # This parameter should not  be changed
		self.minimumLinearSpeed = 0.0 	# minimum speed at which your robot can move forward in m/s
		self.maximumLinearSpeed = 0.25 	# maximum speed at which your robot can move forward in m/s
		self.driveSystemQuality = 1.0 # specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when a told to do so)

		# Camera Parameters
		self.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
		self.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the front of the robot
		self.cameraHeightFromFloor = 0.1 # height of the camera relative to the floor in metres
		self.cameraTilt = 0.0 # tilt of the camera in radians
		self.cameraPerspectiveAngle = math.radians(60) # do not change this parameter

		# Vision Processing Parameters
		self.maxsampleDetectionDistance = 1 # the maximum distance away that you can detect the sample in metres
		self.maxLanderDetectionDistance = 2.5 # the maximum distance away that you can detect the lander in metres
		self.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
		self.maxRockDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
		self.minWallDetectionDistance = 0.1 # the minimum distance away from a wall that you have to be to be able to detect it

		# collector Parameters
		self.collectorQuality = 1.0 # specifies how good your sample collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
		self.autoCollectSample = True #specifies whether the simulator automatically collects samples if near the collector 
		self.maxCollectDistance = 0.03 #specificies the operating distance of the automatic collector function. Sample needs to be less than this distance to the collector


# This class wraps VREP api functions to allow 
# users to start testing Navigation/AI systems
class lunar_object(IntEnum):
	sample0 = 0
	sample1 = 1
	sample2 = 2
	
	rock0 = 3
	rock1 = 4
	rock2 = 5

	obstacle0 = 6
	obstacle1 = 7
	obstacle2 = 8

	lander = 9