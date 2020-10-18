import time
from pololu_drv8835 import motors, MAX_SPEED

def setup():
    motors.setSpeeds(0, 0)

def close():
    motors.setSpeeds(0, 0)

def move(direction, magnitude):
    if direction == "forward":
        motors.motor1.setSpeed(magnitude)
        motors.motor2.setSpeed(magnitude)

try:
    setup()

    while True:
        move("forward", 100)

except KeyboardInterrupt:
    close()