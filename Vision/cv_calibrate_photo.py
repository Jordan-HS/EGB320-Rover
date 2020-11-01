import numpy as np
import cv2
import time


X_img = 320
Y_img = 240

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

kernel = np.ones((3,3))


# Filters HSV image to remove noise
def HSV_filter(image):
    #HSV_sum = np.sum(image)
    #if HSV_sum == 0:
    #    return image
    #else:
    #Gaus_im = cv2.GaussianBlur(image,(3,3),0)
    #n1_im = cv2.fastNlMeansDenoising(image, None, 3, 3, 5)
    #blur_im = cv2.medianBlur(image, 9)
    # Opening - Erosion followed by dilation
    morph_open_im = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    #morph_close_im = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
    erode_im = cv2.erode(morph_open_im, None, iterations=2)
    # Applying dilation a second time removes noise
    dil_im = cv2.dilate(erode_im, None, iterations=1)
    return dil_im

# COMMENT
if __name__ == '__main__':

    save_lower_hsv, save_upper_hsv = hsv_sliders()
    #images = ['Vision/20201030-115813.png']
    #images = ['Vision/20201030-115848.png']
    #images = ['Vision/20201030-115731.png']
    #images = ['Vision/20201030-120016.png']
    images = ['Vision/20201030-115943.png']

    imr = cv2.imread(images[0])
    #im = HSV_filter(imr)
    while True:
        frame = imr

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

    cv2.destroyAllWindows()
