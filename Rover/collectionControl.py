from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

#1500us is 90degrees
#2400us is 180degrees
#600us is 0degrees

MAXus = 2400
MINus = 600
CENTREus = 1300
INCREus = 4

RANGEincre = ((MAXus-MINus)/INCREus)


# Configure min and max servo pulse lengths
servo_min = round(MINus/INCREus)  # Min pulse length out of 4096
servo_max = round(MAXus/INCREus)  # Max pulse length out of 4096
servo_centre = round(CENTREus/INCREus)

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)