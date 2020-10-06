import collectionControl
import time

MAXus = 2400
MINus = 600
CENTREus = 1300
INCREus = 4
pwm.set_pwm_freq(60)


# Configure min and max servo pulse lengths
servo_min = round(MINus/INCREus)  # Min pulse length out of 4096
servo_max = round(MAXus/INCREus)  # Max pulse length out of 4096
servo_centre = round(CENTREus/INCREus)

while True:
    pwm.set_pwm(0, 0, servo_centre)
    time.sleep(1)