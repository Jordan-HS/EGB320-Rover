import numpy as np
import cv2
import imutils
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

X_img = 320
Y_img = 240

# COMMENT
class pi_cam_setup:
    def __init__(self):
        # initialize the camera
        self.camera = PiCamera()
        self.camera.resolution = (X_img, Y_img)
        # Allow time for the camera to warmup
        time.sleep(2.0)
        self.camera.video_stabilization = False
        self.camera.exposure_mode = 'off'
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = 2.0

        self.camera.framerate = 8
        self.rawCapture = PiRGBArray(self.camera, size=(X_img, Y_img))


    def capture_frame(self):
        self.stream = PiRGBArray(self.camera) 
        self.camera.capture(self.stream, format='bgr',use_video_port=True)
        frame = self.stream.array
        # Clear the stream between frame captures 
        self.stream.truncate()
        return frame

def nothing(x):
    pass

def hsv_sliders():
    cv2.namedWindow("Trackbars")

    cv2.createTrackbar("L – H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("U – H", "Trackbars", 179, 179, nothing)

    cv2.createTrackbar("L – S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U – S", "Trackbars", 255, 255, nothing)

    cv2.createTrackbar("L – V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U – V", "Trackbars", 255, 255, nothing)

    save_lower_hsv = np.array([0, 0, 0])
    save_upper_hsv = np.array([179, 255, 255])
    
    return save_lower_hsv, save_upper_hsv


# COMMENT
if __name__ == '__main__':

    save_lower_hsv, save_upper_hsv = hsv_sliders()
    cam = pi_cam_setup()
    
    while True:
    
        frame = cam.capture_frame()
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        l_h = cv2.getTrackbarPos("L – H", "Trackbars")
        u_h = cv2.getTrackbarPos("U – H", "Trackbars")
            
        l_s = cv2.getTrackbarPos("L – S", "Trackbars")
        u_s = cv2.getTrackbarPos("U – S", "Trackbars")

        l_v = cv2.getTrackbarPos("L – V", "Trackbars")
        u_v = cv2.getTrackbarPos("U – V", "Trackbars")

        lower_hsv = np.array([l_h, l_s, l_v])
        upper_hsv = np.array([u_h, u_s, u_v])

        if np.array_equal(lower_hsv, save_lower_hsv) and np.array_equal(upper_hsv, save_upper_hsv):
            pass
        else:
            print('HSV lower: ', lower_hsv)
            print('HSV upper: ', upper_hsv)
            save_lower_hsv = lower_hsv
            save_upper_hsv = upper_hsv
        
        
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        result = cv2.bitwise_and(frame, frame, mask=mask)


        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.imshow('result', result)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):     
            time.sleep(0.1)
            break            

    stream.release()       
    cv2.destroyAllWindows()
