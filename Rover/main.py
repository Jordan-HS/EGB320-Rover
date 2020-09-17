from modules import motorControl
from modules import cv_vision
from modules import potentialField
from modules import servoControl
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
    
    # Get current observation
    observation = cv_vision.current_observation()

    # Parse array into each object
    for obj in observation:
        # if type
        if obj[1]

    if time.time() - start_time > 5:
	    board.motor_stop(board.ALL)        
	    break


## Closing functions
board.motor_stop(board.ALL)   
servoControl.stop(arm, claw)