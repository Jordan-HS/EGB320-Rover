import RPi.GPIO as GPIO    # Import Raspberry Pi GPIO library
from time import sleep     # Import the sleep function from the time module
GPIO.setwarnings(False)    # Ignore warning for now
GPIO.setmode(GPIO.BOARD)   # Use physical pin numbering
GPIO.setup(9, GPIO.OUT, initial=GPIO.LOW) 

while True: # Run forever
    GPIO.output(9, GPIO.HIGH) # Turn on
    sleep(1)                  # Sleep for 1 second
    GPIO.output(9, GPIO.LOW)  # Turn off
    sleep(1)   