import time
from pololu_drv8835 import motors, MAX_SPEED

def setup():
    motors.setSpeeds(0, 0)

def close():
    motors.setSpeeds(0, 0)


def move(direction, speed):
    if direction == "forward":

        if speed == "normal":
            motors.motor1.setSpeed(250)
            motors.motor2.setSpeed(250)
        elif speed == "slow":
            motors.motor1.setSpeed(150)
            motors.motor2.setSpeed(150)

    elif direction == "left":

        if speed == "normal":
            motors.motor1.setSpeed(300)
            motors.motor2.setSpeed(-300)
        elif speed == "slow":
            motors.motor1.setSpeed(100)
            motors.motor2.setSpeed(-400)

    elif direction == "right":

        if speed == "normal":
            motors.motor1.setSpeed(-200)
            motors.motor2.setSpeed(300)
        elif speed == "slow":
            motors.motor1.setSpeed(-400)
            motors.motor2.setSpeed(100)

try:
    setup()

    while True:
        move("left", "normal")

except KeyboardInterrupt:
    close()