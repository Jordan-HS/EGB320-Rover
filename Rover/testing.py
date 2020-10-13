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
        self.bearing = 0
        self.movement = None

    def move(self, movement, magnitude=None):
        ## Convert magnitude to duty

        motorControl.move(movement, magnitude)
        self.movement = movement

rover = Rover()
motorControl.sendCommand("clear")
while True:
    rover.move("right", 250)
    motorControl.sendCommand(rover.movement)
    rover.x, rover.y = motorControl.updatePosition(rover)
    print((rover.x, rover.y))
    # clear()