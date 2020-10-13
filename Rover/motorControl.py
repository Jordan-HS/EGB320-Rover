import gpiozero
import time
import serial
import re
import math

ser = serial.Serial('/dev/ttyS0', 9600, 8, 'N', 1, timeout=5)
ser.flush()
#Setup pins
M1_back = gpiozero.OutputDevice(18) # On/Off output
M1_fwd = gpiozero.OutputDevice(23) #On/Off output
M1_PWM = gpiozero.PWMOutputDevice(24) # set up PWM pin

STBY = gpiozero.OutputDevice(1)  # Standby pin
STBY.on()

M2_back = gpiozero.OutputDevice(25) # On/Off output
M2_fwd = gpiozero.OutputDevice(8) #On/Off output
M2_PWM = gpiozero.PWMOutputDevice(7) # set up PWM pin

## Parameters to adjust
inner_turn_ratio = 1
m1_motor_bias = 1
m2_motor_bias = 1
radius = 0.020836
time.sleep(2) # Wait for serial to be initialised

def move(movement, magnitude=None):
    if movement == "forward":
        # Motor 1
        M1_back.on() 
        M1_fwd.off()
        M1_PWM.value = magnitude*m1_motor_bias/1000

        # Motor 2
        M2_back.off()
        M2_fwd.on()
        M2_PWM.value = magnitude*m2_motor_bias/1000
    elif movement == "left":
        # Motor 1
        M1_back.on() 
        M1_fwd.off()
        M1_PWM.value = magnitude*m1_motor_bias/1000

        # Motor 2
        M2_back.on()
        M2_fwd.off()
        M2_PWM.value = magnitude*inner_turn_ratio*m2_motor_bias/1000
    elif movement == "right":
        # Motor 1
        M1_back.off() 
        M1_fwd.on()
        M1_PWM.value = magnitude*inner_turn_ratio*m1_motor_bias/1000

        # Motor 2
        M2_back.off()
        M2_fwd.on()
        M2_PWM.value = magnitude*inner_turn_ratio/1000
    elif movement == "stop":
        # Motor 1
        M1_back.off() 
        M1_fwd.off()

        # Motor 2
        M2_back.off()
        M2_fwd.Off()


def updatePosition(rover):
    line = ser.readline().decode('utf-8')
    E1_counter = int(re.search(r'E1: \[(.*?)\]', line).group(1))
    E2_counter = int(re.search(r'E2: \[(.*?)\]', line).group(1))
    dist = 0
    x = rover.x
    y = rover.y
    bearing = rover.bearing

    if rover.last_move is None:
        rover.last_move = rover.movement

    elif rover.last_move != rover.movement:
        rover.last_move = rover.movement
        rover.ref_x = rover.x
        rover.ref_y = rover.y
        rover.ref_bearing = rover.bearing
        sendCommand("clear")

    elif rover.movement == "forward":
        wheel_avg = (E1_counter+E2_counter)/2
        dist = wheel_avg/600 * 2 * math.pi * radius

        x = rover.ref_x + dist*math.cos(rover.bearing)
        y = rover.ref_y + dist*math.sin(rover.bearing)

    elif rover.movement == "right":
        wheel_avg = (abs(E1_counter)+abs(E2_counter))/2
        bearing = rover.ref_bearing + -(wheel_avg/2192 * (2*math.pi))

    elif rover.movement == "left":
        wheel_avg = (abs(E1_counter)+abs(E2_counter))/2
        bearing = rover.ref_bearing + (wheel_avg/2192 * (2*math.pi))

    return x, y, bearing

def sendCommand(command):
    if command == "clear":
        ser.write(str(0).encode('utf-8'))
    elif command == "forward":
        ser.write(str(1).encode('utf-8'))
    elif command == "left":
        ser.write(str(2).encode('utf-8'))
    elif command == "right":
        ser.write(str(3).encode('utf-8'))
# send_state = False
# start = time.time()
# ser.write(str(0).encode('utf-8'))
# while True:
#     # if ser.in_waiting > 0:
#     line = ser.readline()
#     print(line)
#     # if send_state == False:
#     #     ser.write(b"forward\n")
#     ser.write(str(1).encode('utf-8'))
#     move("forward", 300)