import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

GPIO.setup(5, GPIO.OUT) # output rf
GPIO.setup(6, GPIO.OUT) # output rf
GPIO.setup(13, GPIO.OUT) # output rf

# Initial state for LEDs:
print("Testing RF out, Press CTRL+C to exit")

def (collection_stage)
    state = collection_stage
     print("set GIOP high")
     GPIO.output(13, GPIO.HIGH)
     time.sleep(5)               
except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
   print("Keyboard interrupt")

except:
   print("some error") 

finally:
   print("clean up") 
   GPIO.cleanup() # cleanup all GPIO 
