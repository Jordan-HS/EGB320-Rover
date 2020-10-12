import gpiozero
import time
import serial
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

time.sleep(2) # Wait for serial to be initialised

def move(movement, magnitude=None):
    if movement == "forward":
        # Motor 1
        M1_back.on() 
        M1_fwd.off()
        M1_PWM.value = magnitude/1000

        # Motor 2
        M2_back.off()
        M2_fwd.on()
        M2_PWM.value = magnitude/1000
    elif movement == "left":
        # Motor 1
        M1_back.on() 
        M1_fwd.off()
        M1_PWM.value = magnitude/1000

        # Motor 2
        M2_back.on()
        M2_fwd.off()
        M2_PWM.value = magnitude*2/1000
    elif movement == "right":
        # Motor 1
        M1_back.off() 
        M1_fwd.on()
        M1_PWM.value = magnitude/1000

        # Motor 2
        M2_back.off()
        M2_fwd.on()
        M2_PWM.value = magnitude/1000
    elif movement == "stop":
        # Motor 1
        M1_back.off() 
        M1_fwd.off()

        # Motor 2
        M2_back.off()
        M2_fwd.Off()

while True:
    # if ser.in_waiting > 0:
    line = ser.readline()
    print(line)
    
    move("left", 300)