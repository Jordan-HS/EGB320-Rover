import collectionControl
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

MAXus = 2400
MINus = 600
CENTREus = 800
INCREus = 4
pwm.set_pwm_freq(60)


# Configure min and max servo pulse lengths
servo_min = round(MINus/INCREus)  # Min pulse length out of 4096
servo_max = round(MAXus/INCREus)  # Max pulse length out of 4096
servo_centre = round(CENTREus/INCREus)

while True:
    pwm.set_pwm(11, 0, int(servo_centre))
    time.sleep(1)