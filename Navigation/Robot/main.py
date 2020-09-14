import motorControl

board = motorControl.initialiseBoard()

while True:
    motorControl.SetTargetVelocities(board, 0.5, 0)