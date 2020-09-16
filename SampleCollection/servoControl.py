import RPi.GPIO as GPIO
import time

def setupServos():
    armPIN = 17 # One with the tape
    clawPIN = 24

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(armPIN, GPIO.OUT)
    GPIO.setup(clawPIN, GPIO.OUT)

    arm = GPIO.PWM(armPIN, 50)
    arm.start(2.5)
    claw = GPIO.PWM(clawPIN, 50)
    claw.start(2.5)

    arm.ChangeDutyCycle(0)
    claw.ChangeDutyCycle(0)

    return arm, claw

def setArmAngle(arm, angle):
    arm.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    arm.ChangeDutyCycle(0)

def openClaw(claw):
    openAngle = 20
    claw.ChangeDutyCycle(2+(openAngle/18))
    time.sleep(0.5)
    claw.ChangeDutyCycle(0)

def closeClaw(claw):
    angle = 0
    claw.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    claw.ChangeDutyCycle(0)

def stop(arm, claw):
    arm.stop()
    claw.stop()
    GPIO.cleanup()