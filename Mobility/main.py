import time
from pololu_drv8835 import motors, MAX_SPEED

def setup():
    motors.setSpeeds(0, 0)

def close():
    motors.setSpeeds(0, 0)

test_forward_speeds = list(range(0, MAX_SPEED, 1)) + \
  [MAX_SPEED] * 200 + list(range(MAX_SPEED, 0, -1)) + [0]  

def move(direction, magnitude):
    if direction == "forward":
        motors.motor1.setSpeed(-200)
        motors.motor2.setSpeed(300)

try:
    setup()

    while True:
        move("forward", 200)
        time.sleep(0.005)

except KeyboardInterrupt:
    close()