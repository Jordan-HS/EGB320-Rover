from roverbot_lib import *


def init_sim():
    # SET SCENE PARAMETERS
    sceneParameters = SceneParameters()

    #sceneParameters.obstacle0_StartingPosition = [0.3, -0.2]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
    sceneParameters.obstacle0_StartingPosition = [-0.5, -0.2]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
    sceneParameters.obstacle1_StartingPosition = [0.6, 0.5]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
    sceneParameters.obstacle2_StartingPosition = None   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene

    sceneParameters.sample0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
    #sceneParameters.sample0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
    sceneParameters.sample1_StartingPosition = [0.6, 0.8]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
    sceneParameters.sample2_StartingPosition = [-0.6, 0.3]   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene


    sceneParameters.rock0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
    #sceneParameters.rock0_StartingPosition = [0.5, 0.8] # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current vrep position, or none if not wanted in the scene
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
    robotParameters.cameraHeightFromFloor = 0.12 # height of the camera relative to the floor in metres
    robotParameters.cameraTilt = 0.45 # tilt of the camera in radians

    # Vision Processing Parameters
    robotParameters.maxBallDetectionDistance = 2 # the maximum distance away that you can detect the ball in metres
    robotParameters.maxLanderDetectionDistance = 2.5 # the maximum distance away that you can detect the goals in metres
    robotParameters.maxObstacleDetectionDistance = 2 # the maximum distance away that you can detect the obstacles in metres

    # Dribbler Parameters
    robotParameters.collectorQuality = 1 # specifies how good your sample collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
    robotParameters.autoCollectSample = False #specifies whether the simulator automatically collects samples if near the collector 
    robotParameters.maxCollectDistance = 0.6 #specificies the operating distance of the automatic collector function. Sample needs to be less than this distance to the collector
    robotParameters.collectorQuality = 1

    return robotParameters, sceneParameters