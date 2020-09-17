import motorControl
import cv_vision
import potentialField
import servoControl
import time

## Set up functions
board = motorControl.initialiseBoard()
start_time = time.time()
arm, claw = servoControl.setupServos()
done = False

while not done:
    delta_x = 0
    delta_y = 0
    targetVel = [0,0]

    samplesRB = []
    landerRB = []
    obstaclesRB = []
    rocksRB = []
    
    # Get current observation
    observation = cv_vision.current_observation()

    # Parse array into each object
    for obj in observation:
        if obj[1] == "SAMP":
            samplesRB.append([obj[3], obj[2]])
        elif obj[1] == "ROCK":
            rocksRB.append([obj[3], obj[2]])
        elif obj[1] == "SAT":
            obstaclesRB.append([obj[3], obj[2]])
        elif obj[1] == "LAND":
            landerRB.append([obj[3], obj[2]])

    if samplesRB is not None:
        # loop through each sample detected using Pythonian way
        for sample in samplesRB:
            sampleRange = sample[0]
            sampleBearing = sample[1]
        
        delta_x, delta_y = potentialField.getForce("sample", sampleRange, sampleBearing)
    
        radial_vel, forward_vel = potentialField.calculateMovement(delta_x, delta_y)

    motorControl.SetTargetVelocities(board, forward_vel, radial_vel)


    if time.time() - start_time > 5:
	    board.motor_stop(board.ALL)        
	    break


## Closing functions
board.motor_stop(board.ALL)   
servoControl.stop(arm, claw)
