import servoControl
import time

arm, claw = servoControl.setupServos()

try:
  servoControl.openClaw(claw)
except KeyboardInterrupt:
  servoControl.stop(arm, claw)