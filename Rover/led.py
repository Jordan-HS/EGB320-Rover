import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

GPIO.setup(16, GPIO.OUT) # Red
GPIO.setup(20, GPIO.OUT) # Yellow
GPIO.setup(21, GPIO.OUT) # Green

# Initial state for LEDs:
print("Testing RF out, Press CTRL+C to exit")

def led_state(collection_stage):
   if collection_stage == "searching":
      # print("STATE: Searching")
      GPIO.output(16, GPIO.HIGH)
      GPIO.output(20, GPIO.LOW)
      GPIO.output(21, GPIO.LOW)
   elif collection_stage == "collecting":
      # print("STATE: Collecting")
      GPIO.output(16, GPIO.LOW)
      GPIO.output(20, GPIO.HIGH)
      GPIO.output(21, GPIO.LOW)
   elif collection_stage == "SampleReturn":
      # print("STATE: Searching")
      GPIO.output(16, GPIO.LOW)
      GPIO.output(20, GPIO.LOW)
      GPIO.output(21, GPIO.HIGH)

def close():
   GPIO.cleanup()
               
# except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
#    print("Keyboard interrupt")

# except:
#    print("some error") 

# finally:
#    print("clean up") 
#    GPIO.cleanup() # cleanup all GPIO 
