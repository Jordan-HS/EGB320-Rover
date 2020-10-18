import time
from pololu_drv8835 import motors, MAX_SPEED

def setup():
    motors.setSpeeds(0, 0)

def close():
    motors.setSpeeds(0, 0)


def move(direction, speed):
    if direction == "forward":

        if speed == "normal":
            motors.motor1.setSpeed(-250)
            motors.motor2.setSpeed(-250)
        elif speed == "slow":
            motors.motor1.setSpeed(-250)
            motors.motor2.setSpeed(-250)

    elif direction == "left":

        if speed == "normal":
            motors.motor1.setSpeed(350)
            motors.motor2.setSpeed(-275)
        elif speed == "slow":
            motors.motor1.setSpeed(350)
            motors.motor2.setSpeed(-275)

    elif direction == "right":

        if speed == "normal":
            motors.motor1.setSpeed(-275)
            motors.motor2.setSpeed(350)
        elif speed == "slow":
            motors.motor1.setSpeed(-300)
            motors.motor2.setSpeed(350)

try:
    setup()

    while True:
        move("forward", "slow")

except KeyboardInterrupt:
    close()