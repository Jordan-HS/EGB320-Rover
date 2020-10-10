import gpiozero
import time
import serial

#Setup pins
Backward = gpiozero.OutputDevice(18) # On/Off output
Forward = gpiozero.OutputDevice(23) #On/Off output

SpeedPWM = gpiozero.PWMOutputDevice(24) # set up PWM pin

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
    Backward.off() # Sets Backward Direction off
    Forward.on()   # Sets Backward Direction pin on
    # speedFlag = float(input("set speed (between 0-1000): ")) # Gets a number from the from the user
    SpeedPWM.value = 500/1000 # Sets the duty cycle of the PWM between 0-1