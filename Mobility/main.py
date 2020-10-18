import time
from pololu_drv8835 import motors, MAX_SPEED

def setup():
    motors.setSpeeds(0, 0)

def close():
    motors.setSpeeds(0, 0)


def move(direction, magnitude):
    turn_scale = 1.5

    if direction == "forward":
        motors.motor1.setSpeed(magnitude)
        motors.motor2.setSpeed(magnitude)
    elif direction == "left":
        motors.motor1.setSpeed(int(magnitude*turn_scale))
        motors.motor2.setSpeed(-magnitude)
    elif direction == "right":
        motors.motor1.setSpeed(-magnitude)
        motors.motor2.setSpeed(int(magnitude*turn_scale))

try:
    setup()

    while True:
        move("left", 240)

except KeyboardInterrupt:
    close()