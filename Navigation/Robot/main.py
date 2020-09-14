import motorControl
import time

board = motorControl.initialiseBoard()
start_time = time.time()
while True:
    motorControl.SetTargetVelocities(board, 1, 0)
    if start_time - time.time() == 10:
        break