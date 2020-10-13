import motorControl

HUD = True

if HUD:
    import os
    import sys
    clear = lambda: os.system('clear')

class Rover:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.movement = None

    def move(self, movement, magnitude=None):
        ## Convert magnitude to duty

        motorControl.move(movement, magnitude)

rover = Rover()
while True:
    rover.move("forward", 300)
    print(motorControl.updatePosition(rover))
    clear()