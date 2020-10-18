import time
import pigpio
MOTOR_A1 = 25
MOTOR_A2 = 8

#connect to pigpiod daemon
pi = pigpio.pi()

# pi set frequency
pi.set_PWM_frequency(MOTOR_A2, 1000)

pi.set_servo_pulsewidth(MOTOR_A1, 0)
pi.set_PWM_dutycycle(MOTOR_A2,255)
time.sleep(1)

#disconnect
pi.stop()