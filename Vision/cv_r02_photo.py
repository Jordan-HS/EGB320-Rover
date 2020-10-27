from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import multiprocessing
import concurrent.futures

# Initialse variables
# HSV colour thresholds
obstacle_avoidance = True

HSV_blue = [[76, 88, 18], [113, 255, 255]]
HSV_green = [[33, 77, 18], [74, 255, 255]]
HSV_yellow = [[15, 77, 40], [36, 255, 255]]
HSV_wall = [[0, 0, 0], [179, 255, 80]]
HSV_orange = [[101, 41, 51], [124, 255, 255]]
HSV_thresh = np.array([HSV_blue, HSV_green, HSV_yellow, HSV_wall, HSV_orange])

# Set morphology kernel size for image filtering
kernel = np.ones((5, 5))

# Initiate counter to only show every 10th computation
image_cnt = 0

# Define obstacle size, label, and colour
OBS_size = [0.075, 0.151, 0.56, 0, 0.044]   # size of obstacles in m
OBS_type = ["ROC", "SAT", "LAND", "WALL", "SAMP"] # labels
OBS_col = [[255, 127, 0], [0, 255, 0], [0, 255, 255], [255, 0, 255], [0, 127, 255]] # box colours
# Set camera image frame
#IMG_X = 640
#IMG_Y = 480
IMG_X = 320
IMG_Y = 240
# Calculate pixel focal width
KNOWN_PIXEL_WIDTH = 92  # Pixels
KNOWN_DIST = 0.20         # m
KNOWN_WIDTH = 0.069       # m
FOCAL_PIX = (KNOWN_PIXEL_WIDTH * KNOWN_DIST)/KNOWN_WIDTH

# Initialise camera setup
camera = PiCamera()
camera.resolution = (IMG_X, IMG_Y)
camera.framerate = 16
# Allow time for the camera to warmup
time.sleep(2.0)
camera.video_stabilization = False
camera.exposure_mode = 'off'
camera.awb_mode = 'off'
camera.awb_gains = 2.0

#camera.awb_gains = 3
rawCapture = PiRGBArray(camera, size=(IMG_X, IMG_Y))

# Image crop to decrease image processing time
def crop_image(image):
    now = time.time()
    crop_img = image[int(image.shape[0]*0.25):image.shape[0]]
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    print([rate, "Crop"])
    return crop_img

def current_observation():
    # Grab frame
    camera.capture(rawCapture, format="bgr", use_video_port=True)
    image = rawCapture.array
    rawCapture.truncate(0)

    # Crop image
    image = crop_image(image)
    return image

try:
    img = current_observation()
    print("Booted")
    time.sleep(2)
    input("Press Enter to continue...")
    time.sleep(2)
    while True:
        img = current_observation()
        cv2.imshow("View", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        # Save image output by pressing 's'
        elif key == ord("s"):
            timestr = time.strftime("%Y%m%d-%H%M%S")
            #cv2.imwrite('image_crop.png',img)
            cv2.imwrite(timestr,img)
except KeyboardInterrupt:
    print("Stopped")