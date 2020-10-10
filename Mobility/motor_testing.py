import gpiozero
import time

#Setup pins
Backward = gpiozero.OutputDevice(18) # On/Off output
Forward = gpiozero.OutputDevice(23) #On/Off output

SpeedPWM = gpiozero.PWMOutputDevice(24) # set up PWM pin

while True:
    directionFlag = input("set motor direction: ")
    if directionFlag == "back": # if user types "back" change direction of motor
        Backward.on() # Sets Backward Direction pin on
        Forward.off() # Sets Backward Direction pin on
    else:
        Backward.off() # Sets Backward Direction off
        Forward.on()   # Sets Backward Direction pin on
    speedFlag = float(input("set speed (between 0-1000): ")) # Gets a number from the from the user
    SpeedPWM.value = speedFlag/1000 # Sets the duty cycle of the PWM between 0-1