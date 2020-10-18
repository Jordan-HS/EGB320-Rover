import time
from pololu_drv8835 import motors, MAX_SPEED

def setup():
    motors.setSpeeds(0, 0)

def close():
    motors.setSpeeds(0, 0)


def move(direction, magnitude):
    turn_scale = 1

    if direction == "forward":
        motors.motor1.setSpeed(magnitude)
        motors.motor2.setSpeed(magnitude)
    elif direction == "left":
        motors.motor1.setSpeed(magnitude*1.5)
        motors.motor2.setSpeed(-magnitude)
    elif direction == "right":
        motors.motor1.setSpeed(-magnitude)
        motors.motor2.setSpeed(magnitude)

try:
    setup()

    while True:
        move("left", 200)
        time.sleep(0.005)

except KeyboardInterrupt:
    close()