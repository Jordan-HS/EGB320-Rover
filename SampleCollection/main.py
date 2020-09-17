import servoControl
import time

arm, claw = servoControl.setupServos()

try:
  while(True):
    servoControl.openClaw(claw)
    servoControl.setArmAngle(arm, 150)
except KeyboardInterrupt:
  servoControl.stop(arm, claw)
