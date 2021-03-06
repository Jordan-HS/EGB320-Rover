import motorControl
import math
import time
from pololu_drv8835 import motors, MAX_SPEED

HUD = True

if HUD:
    import os
    import sys
    clear = lambda: os.system('clear')

class Rover:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.ref_x = 0
        self.ref_y = 0
        self.bearing = 0
        self.ref_bearing = 0
        self.current_movement = None
        self.last_movement = None

    def move(self, movement, magnitude=None):
        ## Convert magnitude to duty

        motorControl.move(movement, magnitude)
        self.current_movement = movement
        

rover = Rover()
motorControl.sendCommand("clear")
# start = time.time()
try:
    while True:
        rover.move("right", "normal")
        # if run_time < 2:
        #     rover.move("forward", 250)
        # elif 2 < run_time < 5:
        #     rover.move("left", 250)
        # elif 5 < run_time < 7:
        #     rover.move("forward", 300)

        
        motorControl.sendCommand(rover.current_movement)
        rover.x, rover.y, rover.bearing = motorControl.updatePosition(rover)
        # time.sleep(.5)

        print("x: {:.2f}    y: {:.2f}    bearing:{:.2f}".format(rover.x, rover.y, math.degrees(rover.bearing)))
        # print("loop time: {}".format(time.time()-start))
        # start = time.time()
except KeyboardInterrupt:
    motorControl.close()
    print("x: {:.2f}    y: {:.2f}    bearing:{:.2f}".format(rover.x, rover.y, math.degrees(rover.bearing)))
    # clear()