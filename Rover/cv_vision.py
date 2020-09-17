from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2

# Initialse variables
# HSV colour thresholds
HSV_thresh = np.array([[[88, 36, 0], [117, 255, 255]], [[35, 35, 25],
    [75, 255, 255]], [[15, 20, 90], [39, 255, 255]], [[103, 61, 24], [124, 255, 255]]])
# Set morphology kernel size for image filtering
kernel = np.ones((5, 5))
# Initiate counter to only show every 10th computation
image_cnt = 0
# Define obstacle size, label, and colour
OBS_size = [0.075, 0.151, 0.56, 0.044]   # size of obstacles in m
OBS_type = ["ROC", "SAT", "LAND", "SAMP"]
OBS_col = [[255, 127, 0], [0, 255, 0], [0, 255, 255], [0, 127, 255]]
# Set camera image frame
IMG_X = 320
IMG_Y = 240
# Calculate pixel focal width
KNOWN_PIXEL_WIDTH = 92  # Pixels
KNOWN_DIST = 0.20         # m
KNOWN_WIDTH = 0.069       # m
FOCAL_PIX = (KNOWN_PIXEL_WIDTH * KNOWN_DIST)/KNOWN_WIDTH

# Initialise camera setup
camera = PiCamera()
camera.awb_mode = 'off'
camera.awb_mode = 'fluorescent'
camera.awb_gains = 4
camera.exposure_mode = 'off'
camera.resolution = (IMG_X, IMG_Y)
camera.framerate = 8
rawCapture = PiRGBArray(camera, size=(IMG_X, IMG_Y))
# Allow time for the camera to warmup
time.sleep(0.1)

# Image crop to decrease image processing time
def crop_image(image):
    crop_img = image[int(image.shape[0]*0.25):image.shape[0]]
    return crop_img

# HSV colour threshold filter


def mask_obs(image):
    # Convert BGR to HSV image
    HSV_bgy = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Convert RGB to HSV image
    HSV_o = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    masks_HSV = []
    # Apply mask for HSV range
    for indx, thresh in enumerate(HSV_thresh):
        # Blue Green and Yellow threshold and filter
        if indx < 3:
            HSV_tempmask = cv2.inRange(HSV_bgy, thresh[0], thresh[1])
            HSV_sum = np.sum(HSV_tempmask)
            if HSV_sum == 0:
                masks_HSV.append(HSV_tempmask)
            else:
                masks_HSV.append(HSV_filter(HSV_tempmask))
        # Orange threshold and filter
        else:
            HSV_tempmask = cv2.inRange(HSV_o, thresh[0], thresh[1])
            HSV_sum = np.sum(HSV_tempmask)
            if HSV_sum == 0:
                masks_HSV.append(HSV_tempmask)
            else:
                masks_HSV.append(HSV_filter(HSV_tempmask))
    return masks_HSV

# Filters HSV image to remove noise


def HSV_filter(image):
    # Opening - Erosion followed by dilation
    mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    # mask = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
    mask = cv2.erode(mask, None, iterations=1)
    # Applying dilation a second time removes noise
    mask = cv2.dilate(mask, None, iterations=1)
    return mask

# Define obstacles


def detect_obs(hsv_masks):
    obs_array = []
    colour_count = 0
    for indx, mask in enumerate(hsv_masks):
        HSV_sum = np.sum(mask)
        if HSV_sum == 0:
            continue
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print("Number of contours: ", len(contours))
        if contours is not None:
            print([OBS_type[indx], len(contours)])
            if indx < 2:
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    # print(area)
                    if area > 10:
                        # obstacle type index
                        obs_indx = indx
                        # Obstacle label
                        id_type = OBS_type[indx]
                        # Boundary (x,y,w,h) box of contour
                        boundary = cv2.boundingRect(cnt)
                        # Find centre of enclosing circle
                        print(cnt)
                        centre, radius = cv2.minEnclosingCircle(cnt)
                        # Width of contour in pixels
                        pix_width = boundary[2]
                        # Angle from centre of screen in radians
                        obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
                        # Distance from camera in cm
                        obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
                        # Create list of values
                        print([id_type, pix_width, obs_dist])
                        obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary])
            elif indx == 2:
                area = cv2.findNonZero(mask)
                # obstacle type index
                obs_indx = indx
                # Obstacle label
                id_type = OBS_type[indx]
                # Boundary (x,y,w,h) box of contour
                boundary = cv2.boundingRect(area)
                # Find centre of enclosing circle
                centre, radius = cv2.minEnclosingCircle(area)
                # Width of contour in pixels
                pix_width = boundary[2]
                # Angle from centre of screen in radians
                obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
                # Distance from camera in cm
                obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
                # Create list of values
                obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary])
            else:
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    # print(area)
                    if area > 6:
                        # obstacle type index
                        obs_indx = indx
                        # Obstacle label
                        id_type = OBS_type[indx]
                        # Boundary (x,y,w,h) box of contour
                        boundary = cv2.boundingRect(cnt)
                        # Find centre of enclosing circle
                        centre, radius = cv2.minEnclosingCircle(cnt)
                        # Width of contour in pixels
                        pix_width = boundary[2]
                        # Angle from centre of screen in radians
                        obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
                        # Distance from camera in cm
                        obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
                        # Create list of values
                        obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary])
    # print(obs_array)
    return obs_array

def current_observation():
     # Grab frame
    camera.capture(rawCapture, format="bgr", use_video_port=True)
    image = rawCapture.array
    rawCapture.truncate(0)
    # Crop image
    image = crop_image(image)

    # Apply HSV threshold to frame
    hsv_masks = mask_obs(image)

    # Determine distance, angle ID and type
    return detect_obs(hsv_masks)

# Process frame from PiCamera
# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#     # Initiate timer for vision
#     now = time.time()

#     # Grab frame
#     image = frame.array

#     # Crop image
#     image = crop_image(image)
#     cv2.imshow("Original Frame", image)

#     # Apply HSV threshold to frame
#     hsv_masks = mask_obs(image)
#     # Output mask for checking
#     for indx,mask in enumerate(hsv_masks):
#         cv2.imshow(OBS_type[indx], mask)

#     # Determine distance, angle ID and type
#     new_obs = detect_obs(hsv_masks)

#     # Calculate time elapsed
#     elapsed = time.time() - now
#     rate = 1.0 / elapsed
#     print("Processing Rate:{}.".format(rate))
#     # Show the frame every 10th iteration
#     image_cnt += 1
#     if image_cnt == 10:
#         # Combine masks and apply to frame
#         mask = hsv_masks[0]|hsv_masks[1]|hsv_masks[2]|hsv_masks[3]
#         obs_image = cv2.bitwise_and(image, image, mask=mask)
#         for i, obs in enumerate(new_obs):
#             # Draw rectangle
#             cv2.rectangle(obs_image, (int(new_obs[i][5][0]), int(new_obs[i][5][1])),\
#             (int(new_obs[i][5][0] + new_obs[i][5][2]), int(new_obs[i][5][1] +\
#             new_obs[i][5][3])), OBS_col[new_obs[i][0]], 1)
#             # Draw shape type
#             cv2.putText(obs_image, new_obs[i][1], (int(new_obs[i][5][0]),\
#             int(new_obs[i][5][1] + new_obs[i][5][3]) + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.5, OBS_col[new_obs[i][0]],1)
#             # Draw distance in cm
#             cv2.putText(obs_image, "{:.1f}".format(new_obs[i][3]), (int(new_obs[i][5][0]),\
#             int(new_obs[i][5][1] + new_obs[i][5][3]) + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, OBS_col[new_obs[i][0]],1)
#             # Draw angle in radians to obstacle from camera
#             cv2.putText(obs_image, "{:.1f}".format(np.degrees(new_obs[i][2])), (int(new_obs[i][5][0]),\
#             int(new_obs[i][5][1] + new_obs[i][5][3]) + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, OBS_col[new_obs[i][0]],1)
#         cv2.imshow("Frame", obs_image)
#         image_cnt = 0

#     key = cv2.waitKey(1) & 0xFF
#     rawCapture.truncate(0)
#     # Termination by pressing 'q'
#     if key == ord("q"):
#         break
#     # Save image output by pressing 's'    
#     elif key == ord("s"):
#         cv2.imwrite('mask.png',mask)
#         cv2.imwrite('image_frame.png',image)
#         cv2.imwrite('result.png',obs_image)

