##
## DC motor control with TB6612FNG dual h-bridge motor controller
##

from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False);

# PWM Frequency
pwmFreq = 100

# Setup pins
GPIO.setup(22, GPIO.OUT)    # STBY

motorA_in1_pin = 16
motorA_in2_pin = 18
GPIO.setup(12, GPIO.OUT)    # PWMA
GPIO.setup(motorA_in2_pin, GPIO.OUT)    # AIN2
GPIO.setup(motorA_in1_pin, GPIO.OUT)    # AIN1
motorA = GPIO.PWM(12, pwmFreq)    # pin 18 to PWM
motorA.start(100)
motorA.ChangeDutyCycle(0)

# motorB_in1_pin = 15
# motorB_in2_pin = 19
# GPIO.setup(motorB_in1_pin, GPIO.OUT)    # BIN1
# GPIO.setup(motorB_in2_pin, GPIO.OUT)    # BIN2
# GPIO.setup(11, GPIO.OUT)    # PWMB
# motorB = GPIO.PWM(11, pwmFreq)    # pin 13 to PWM
# motorB.start(100)
# motorB.ChangeDutyCycle(0)

## Functions
#################################################################
def motorA_forward():
    GPIO.output(motorA_in1_pin, True)
    GPIO.output(motorA_in2_pin, False)


def motorA_reverse():
    GPIO.output(motorA_in1_pin, False)
    GPIO.output(motorA_in2_pin, True)

# def motorB_forward():
#     GPIO.output(motorB_in1_pin, True)
#     GPIO.output(motorB_in2_pin, False)

# def motorB_reverse():
#     GPIO.output(motorB_in1_pin, False)
#     GPIO.output(motorB_in2_pin, True)



def motorStop():
    GPIO.output(22, GPIO.LOW)

## Main
#################################################################

GPIO.output(22, GPIO.HIGH)
motorA.ChangeDutyCycle(50)
motorA_forward()
sleep(1)
# motorA_reverse()
# sleep(1)

# motorB.ChangeDutyCycle(50)
# motorB_forward()
# sleep(1)
# motorB_reverse()
# sleep(1)

