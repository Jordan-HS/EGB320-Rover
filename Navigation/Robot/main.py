import motorControl
import time

board = motorControl.initialiseBoard()
start_time = time.time()
while True:
    motorControl.SetTargetVelocities(board, 0.3, -5)
    if time.time() - start_time > 5:
	board.motor_stop(board.ALL)        
	break
