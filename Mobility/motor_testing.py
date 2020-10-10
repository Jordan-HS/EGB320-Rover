import gpiozero
import time
import serial

#Setup pins
M1_back = gpiozero.OutputDevice(18) # On/Off output
M1_fwd = gpiozero.OutputDevice(23) #On/Off output
M1_PWM = gpiozero.PWMOutputDevice(24) # set up PWM pin

M2_back = gpiozero.OutputDevice(25) # On/Off output
M2_fwd = gpiozero.OutputDevice(8) #On/Off output
M2_PWM = gpiozero.PWMOutputDevice(7) # set up PWM pin

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

time.sleep(2)

while True:
    # if ser.in_waiting > 0:
    line = ser.readline()
    print(line)
    # directionFlag = input("set motor direction: ")
    # if directionFlag == "back": # if user types "back" change direction of motor
    #     Backward.on() # Sets Backward Direction pin on
    #     Forward.off() # Sets Backward Direction pin on
    # else:
    M1_back.off() # Sets Backward Direction off
    M1_fwd.on()   # Sets Backward Direction pin on
    M1_PWM.value = 500/1000 # Sets the duty cycle of the PWM between 0-1
    
    M2_back.off() # Sets Backward Direction off
    M2_fwd.on()   # Sets Backward Direction pin on
    M2_PWM.value = 500/1000 # Sets the duty cycle of the PWM between 0-1