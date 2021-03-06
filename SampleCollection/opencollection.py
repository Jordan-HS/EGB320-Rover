import collectionControl
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

MAXus = 2400
MINus = 600
value = 1600
INCREus = 4

pwm.set_pwm_freq(60)


# Configure min and max servo pulse lengths
servo_min = round(MINus/INCREus)  # Min pulse length out of 4096
servo_max = round(MAXus/INCREus)  # Max pulse length out of 4096
servo_centre = round(value/INCREus)

while True:
    pwm.set_pwm(2, 0, int(servo_centre)) # was pin 10 now 0
    time.sleep(1)
    