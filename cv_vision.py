import numpy as np
import cv2
import imutils
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

# HSV range values for targets
# To call: HSV_BLUE[0] = [0, 140, 30]
HSV_BLUE = np.array([[90, 66, 0], [113, 255, 255]])
HSV_GREEN = np.array([[26, 53, 0], [61, 255, 255]])
HSV_YELLOW = np.array([[15, 59, 90], [39, 255, 255]])
# Orange uses the RGB spectrum
HSV_ORANGE = np.array([[104, 77, 48], [122, 255, 255]])

# Size dimensions for obstacles (mm)
CRASH = 0;
ROCK = 0;
SAMPLE = 0;
LANDER = 0;

# COMMENT
class pi_cam_setup:
    def __init__(self):
        # initialize the camera
        self.camera = PiCamera()
        self.camera.resolution = (320, 240)
        self.camera.framerate = 30
        #self.camera.rotation = 90
        self.camera.video_denoise = False
        self.camera.image_effect = 'blur'
        self.camera.image_effect_params = (2,)
        self.rawCapture = PiRGBArray(self.camera, size=(320, 240))
        time.sleep(0.1)

    def capture_frame(self):
        self.stream = PiRGBArray(self.camera) 
        self.camera.capture(self.stream, format='bgr',use_video_port=True)
        frame = self.stream.array
        
        # Clear the stream between frame captures 
        self.stream.truncate()
        return frame

def crop_bottom_half(image):
    crop_img = image[image.shape[0]/2:image.shape[0]]
    return crop_img

def shape_detection(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_orange = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    
    blue_mask = cv2.inRange(hsv, HSV_BLUE[0], HSV_BLUE[1])
    green_mask = cv2.inRange(hsv, HSV_GREEN[0], HSV_GREEN[1])
    yellow_mask = cv2.inRange(hsv, HSV_YELLOW[0], HSV_YELLOW[1])
    orange_mask = cv2.inRange(hsv_orange, HSV_ORANGE[0], HSV_ORANGE[1])

    blue_mask = shape_filter(blue_mask)
    green_mask = shape_filter(green_mask)
    yellow_mask = shape_filter(yellow_mask)
    orange_mask = shape_filter(orange_mask)
    
    return blue_mask, green_mask, yellow_mask, orange_mask

def shape_filter(image):
    mask = cv2.GaussianBlur(image,(3,3),0)
    mask = cv2.medianBlur(mask,9)
    mask = cv2.erode(mask, None, iterations=5)
    mask = cv2.dilate(mask, None, iterations=3)
    return mask

# COMMENT
if __name__ == '__main__':
    cam = pi_cam_setup()
    
    while True:
    
        frame = cam.capture_frame()
        frame = crop_bottom_half(frame)
        
        blue_mask, green_mask, yellow_mask, orange_mask = shape_detection(frame)	
        
        cv2.imshow('frame', frame)
        
        #cv2.imshow('blue_mask', blue_mask)
        #cv2.imshow('green_mask', green_mask)
        #cv2.imshow('yellow_mask', yellow_mask)
        #cv2.imshow('orange_mask', orange_mask)

        mask = blue_mask|green_mask|yellow_mask|orange_mask
        
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('result', result)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    stream.release()
    cv2.destroyAllWindows()
