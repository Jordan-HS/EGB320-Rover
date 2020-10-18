import time
from pololu_drv8835 import motors, MAX_SPEED
import serial
import re
import math

ser = serial.Serial('/dev/ttyS0', 9600, 8, 'N', 1, timeout=5)
ser.flush()

radius = 0.020836
time.sleep(2) # Wait for serial to be initialised

def setup():
    motors.setSpeeds(0, 0)

def close():
    motors.setSpeeds(0, 0)


def move(direction, speed):
    if speed == "stop":
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)

    elif direction == "forward":

        if speed == "normal":
            motors.motor1.setSpeed(-300)
            motors.motor2.setSpeed(-300)
        elif speed == "slow":
            motors.motor1.setSpeed(-375)
            motors.motor2.setSpeed(-375)

    elif direction == "left":

        if speed == "normal":
            motors.motor1.setSpeed(350)
            motors.motor2.setSpeed(-275)
        elif speed == "slow":
            motors.motor1.setSpeed(325)
            motors.motor2.setSpeed(-350)

    elif direction == "right":

        if speed == "normal":
            motors.motor1.setSpeed(-275)
            motors.motor2.setSpeed(350)
        elif speed == "slow":
            motors.motor1.setSpeed(-350)
            motors.motor2.setSpeed(325)


def WrapToPi(radians):
		return ((radians + math.pi) % (2* math.pi) - math.pi)


def updatePosition(rover):
    line = ser.readline().decode('utf-8')
    if rover.current_movement == "stop":
        return rover.x, rover.y, rover.bearing
    E1_counter = int(re.search(r'E1: \[(.*?)\]', line).group(1))
    E2_counter = int(re.search(r'E2: \[(.*?)\]', line).group(1))
    dist = 0
    x = rover.x
    y = rover.y
    bearing = rover.bearing

    if rover.last_movement is None:
        rover.last_movement = rover.current_movement

    elif rover.last_movement != rover.current_movement:
        rover.last_movement = rover.current_movement
        rover.ref_x = rover.x
        rover.ref_y = rover.y
        rover.ref_bearing = rover.bearing
        sendCommand("clear")

    elif rover.current_movement == "forward":
        wheel_avg = (E1_counter+E2_counter)/2
        dist = wheel_avg/600 * 2 * math.pi * radius

        x = rover.ref_x + dist*math.cos(rover.bearing)
        y = rover.ref_y + dist*math.sin(rover.bearing)

    elif rover.current_movement == "right":
        # wheel_avg = (abs(E1_counter)+abs(E2_counter))/2
        wheel_avg = max(abs(E1_counter), abs(E2_counter))
        bearing = rover.ref_bearing + -(wheel_avg/2525 * (2*math.pi))

    elif rover.current_movement == "left":
        # wheel_avg = (abs(E1_counter)+abs(E2_counter))/2
        wheel_avg = max(abs(E1_counter), abs(E2_counter))
        bearing = rover.ref_bearing + (wheel_avg/2525 * (2*math.pi))

    return x, y, WrapToPi(bearing)

def sendCommand(command):
    if command == "clear":
        ser.write(str(0).encode('utf-8'))
    elif command == "forward":
        ser.write(str(1).encode('utf-8'))
    elif command == "left":
        ser.write(str(2).encode('utf-8'))
    elif command == "right":
        ser.write(str(3).encode('utf-8'))
    elif command == "stop":
        ser.write(str(4).encode('utf-8'))

# try:
#     setup()

#     while True:
#         move("forward", "normal")

# except KeyboardInterrupt:
#     close()