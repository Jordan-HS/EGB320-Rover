import servoControl
import time

arm, claw = servoControl.setupServos()

try:
  while(True):
    servoControl.openClaw(claw)
except KeyboardInterrupt:
  servoControl.stop(arm, claw)