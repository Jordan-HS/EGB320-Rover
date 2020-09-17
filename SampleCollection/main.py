import servoControl
import time

arm, claw = servoControl.setupServos()

try:
  while(True):
    servoControl.closeClaw(claw)
    servoControl.setArmAngle(arm, 160)
except KeyboardInterrupt:
  servoControl.stop(arm, claw)
