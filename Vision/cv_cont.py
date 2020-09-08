import numpy as np
import cv2
import imutils
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import random as rng

# HSV range values for targets
# To call: HSV_BLUE[0] = [90, 66, 0]

#HSV_BLUE = np.array([[90, 66, 0], [113, 255, 255]])
#HSV_GREEN = np.array([[26, 53, 0], [56, 255, 255]])
#HSV_YELLOW = np.array([[15, 59, 90], [39, 255, 255]])
# Orange uses the RGB spectrum
#HSV_ORANGE = np.array([[104, 77, 48], [122, 255, 255]])

HSV_thresh = np.array([[[90, 66, 0], [113, 255, 255]],[[26, 53, 0], [56, 255, 255]],[[15, 59, 90], [39, 255, 255]],[[104, 77, 48], [122, 255, 255]]]);

# Size dimensions for obstacles (mm)
Satelite = 150;    # 150mm x 150mm
Rock = 75;      # 75mm x 75mm
Sample = 55;    # 55mm ball
Lander = 0;

# COMMENT
class pi_cam_setup:
    def __init__(self):
        # initialize the camera
        self.camera = PiCamera()
        self.camera.resolution = (320, 240)
        self.camera.framerate = 30
        self.camera.rotation = 90
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
    crop_img = image[image.shape[0]/4:image.shape[0]]
    return crop_img

def mask_obs(image):
    # Convert to HSV
    HSV_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    HSV_image.append(cv2.cvtColor(image, cv2.COLOR_RGB2HSV))
    
    # HSV_orange = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    masks_HSV = []
    # Apply mask for HSV range
    for thresh, val in HSV_thresh:
        HSV_tempmask = cv2.inRange(HSV_image, thresh[0], thresh[1])
        masks_HSV.append(HSV_filter(HSV_tempmask))
    return masks_HSV

def HSV_filter(image):
    mask = cv2.GaussianBlur(image,(3,3),0)
    mask = cv2.medianBlur(mask,9)
    mask = cv2.erode(mask, None, iterations=5)
    mask = cv2.dilate(mask, None, iterations=3)
    return mask

def identify_obs(masks_HSV):
    for mask in masks_HSV:
        contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        

        contours_poly = [None]*len(contours)
        boundRect = [None]*len(contours)
        centers = [None]*len(contours)
        radius = [None]*len(contours)

        for i, c in enumerate(contours):
            print([i])
            obsType[i] = [i]
            contours_poly[i] = cv2.approxPolyDP(c, 3, True)
            boundRect[i] = cv2.boundingRect(contours_poly[i])
            centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
            obsWidth[i] = 0
            obsAngle[i] = 0


    
    return shape_array

def tracking_obs(new_obstacles, new_obstacles):
    if old_obstacles is None:
        return None

    # ID, object, distance, angle

    return obstacle_position

def result_image(image, new_obstacles)
    for i in range(len(contours)):
            cv2.rectangle(image, (int(boundRect[i][0]), int(boundRect[i][1])), \
                (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), (255, 0, 0), 2)
            cv2.putText(image, 'Obstacle', (int(boundRect[i][0]), int(boundRect[i][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 0), 2)
    return image

# COMMENT
if __name__ == '__main__':

    frame = cv2.imread('img_12.jpg')
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = crop_bottom_half(frame)

    old_obstacles = None

    while True:

        masks_HSV = mask_obs(frame)

        #mask = blue_mask|green_mask|yellow_mask|orange_mask
        #result = cv2.bitwise_and(frame, frame, mask=mask)

        new_obstacles = identify_obs(masks_HSV)
        obstacle_position = tracking(new_obstacles, old_obstacles)
        # Store obstacles for comparison
        old_obstalces = new_obstacles

        # Counter to output every 10th frame
        image_cnt += 1
        if image_cnt == 10:
            obs_image = result_image(frame, new_obstacles)
            cv2.imshow('Rover Obstacles', obs_image)
            image_cnt = 0

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.imwrite('results_img_03.jpg',result)
            break

    cv2.destroyAllWindows()
