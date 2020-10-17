from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2

# Initialse variables
# HSV colour thresholds
HSV_blue = [[88, 36, 0], [117, 255, 255]]
HSV_green = [[33, 20, 25], [75, 255, 255]]
HSV_yellow = [[15, 20, 90], [39, 255, 255]]
HSV_orange = [[103, 30, 55], [125, 255, 255]]
HSV_thresh = np.array([HSV_blue, HSV_green, HSV_yellow, HSV_orange])

# Set morphology kernel size for image filtering
kernel = np.ones((5, 5))

# Initiate counter to only show every 10th computation
image_cnt = 0

# Define obstacle size, label, and colour
OBS_size = [0.075, 0.151, 0.56, 0.044]   # size of obstacles in m
OBS_type = ["ROC", "SAT", "LAND", "SAMP"] # labels
OBS_col = [[255, 127, 0], [0, 255, 0], [0, 255, 255], [0, 127, 255]] # box colours
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
camera.framerate = 8
# Allow time for the camera to warmup
time.sleep(0.5)
camera.video_stabilization = False
camera.exposure_compensation = 0
camera.exposure_mode = 'off'
camera.awb_mode = 'off'
camera.awb_mode = 'horizon'
camera.exposure_mode = 'sports'

#camera.awb_gains = 3
rawCapture = PiRGBArray(camera, size=(IMG_X, IMG_Y))

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
                #masks_HSV.append((HSV_tempmask))
                masks_HSV.append(HSV_orange_filter(HSV_tempmask))
    return masks_HSV

# Filters HSV image to remove noise
def HSV_filter(image):
    # Opening - Erosion followed by dilation
    mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.erode(mask, None, iterations=5)
    # Applying dilation a second time removes noise
    mask = cv2.dilate(mask, None, iterations=5)
    return mask

def HSV_orange_filter(image):
    # Opening - Erosion followed by dilation
    mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.erode(mask, None, iterations=1)
    # Applying dilation a second time removes noise
    mask = cv2.dilate(mask, None, iterations=3)
    return mask

# Define obstacles
def detect_obs(hsv_masks):
    obs_array = []
    colour_count = 0
    for indx, mask in enumerate(hsv_masks):
        HSV_sum = np.sum(mask)
        if HSV_sum == 0:
            continue
        #contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Check for rocks and satellite crash obstacles
        if indx < 2:
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                # print(area)
                if area > 150:
                    # obstacle type index
                    obs_indx = indx
                    # Obstacle label
                    id_type = OBS_type[indx]
                    # Boundary (x,y,w,h) box of contour
                    boundary = cv2.boundingRect(cnt)
                    # Check for error if boundaries outside of expected
                    if ((boundary[3]/boundary[2])<0.6):
                        error = 1   # Obstacle overlapping
                        # Creates boundary for two obstacles with error noted
                        obs_array_overlap = overlap_obs(cnt, obs_indx, id_type, boundary, error)
                        # Appends two obstacles to array
                        for obs in obs_array_overlap:
                            obs_array.append(obs)
                        # Exit loop
                        continue
                    elif (boundary[0] <= 3) or ((boundary[0] + boundary[2]) >= (IMG_X-3)):
                        error = 1   # Obstacle on boundary
                        obs_array_boundary = boundary_obs(cnt, obs_indx, id_type, boundary, error)
                        obs_array.append(obs_array_boundary)
                        # Exit loop
                        continue
                    elif ((boundary[3]/boundary[2]) > 1.4):
                        error = 1   # Obstacle on boundary
                        obs_array_hidden = hidden_obs(cnt, obs_indx, id_type, boundary, error)
                        obs_array.append(obs_array_hidden)
                        # Exit loop
                        continue
                    else:
                        error = 0   # No obstacle overlap
                    # Find centre of enclosing circle
                    centre, radius = cv2.minEnclosingCircle(cnt)
                    # Width of contour in pixels
                    pix_width = boundary[2]
                    # Angle from centre of screen in radians
                    obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
                    # Distance from camera in cm
                    obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
                    # Create list of values
                    #print([id_type, pix_width, obs_dist])
                    obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
        # Check for lander
        elif indx == 2:
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnt = np.concatenate(contours)
            #boundary_area = cv2.findNonZero(mask)
            # obstacle type index
            obs_indx = indx
            # Obstacle label
            id_type = OBS_type[indx]
            # Boundary (x,y,w,h) box of contour
            boundary = cv2.boundingRect(cnt)
            # Error if boundaries outside of norm
            if ((boundary[3]/boundary[2])>0.26):
                error = 1 # Lander partially obscured
            else:
                error = 0 # Lander completely visable
            # Find centre of enclosing circle
            centre, radius = cv2.minEnclosingCircle(cnt)
            # Width of contour in pixels
            pix_width = boundary[2]
            # Angle from centre of screen in radians
            obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
            # Distance from camera in cm
            obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
            # Create list of values
            obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
        # Check for samples
        else:
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 40:
                    # obstacle type index
                    obs_indx = indx
                    # Obstacle label
                    id_type = OBS_type[indx]
                    #print([area, id_type])
                    # Boundary (x,y,w,h) box of contour
                    boundary = cv2.boundingRect(cnt)
                    # Error if boundaries outside of norm
                    error = 0
                    # Find centre of enclosing circle
                    centre, radius = cv2.minEnclosingCircle(cnt)
                    # Width of contour in pixels
                    pix_width = boundary[2]
                    # Angle from centre of screen in radians
                    obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
                    # Distance from camera in cm
                    obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
                    # Create list of values
                    obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
    #print(obs_array)
    return obs_array

def overlap_obs(cnt, obs_indx, id_type, boundary, error):
    # Define arrays
    obs_boundary = []
    obs_array_overlap = []
    # Obstacle type index
    obs_indx = obs_indx
    # Obstacle label
    id_type = id_type
    # Error = 1 -  Values not to be trusted
    error = error
    # Create two boundary obstacles based on largest obstacle size
    obs_01 = (boundary[0],boundary[1],boundary[3],boundary[3])
    obs_02 = ((boundary[0]+boundary[2]-boundary[3]),boundary[1],boundary[3],boundary[3])
    # Create array for obstacles
    obs_boundary.append(obs_01)
    obs_boundary.append(obs_02)
    # Create two obstacle list
    for obs in obs_boundary:
        # Centre point for obstacles
        centre = (int(obs[0]+(obs[2]/2)), int(obs[1]+(obs[3]/2)))
        # Width of contour in pixels
        pix_width = obs[2]
        # Angle from centre of screen in radians
        obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
        # Distance from camera in cm
        obs_dist = ((OBS_size[obs_indx] * FOCAL_PIX) / pix_width)
        # Create list of values
        obs_array_overlap.append([obs_indx, id_type, obs_ang, obs_dist, centre, obs, error])
    return obs_array_overlap

def boundary_obs(cnt, obs_indx, id_type, boundary, error):
    # Obstacle type index
    obs_indx = obs_indx
    # Obstacle label
    id_type = id_type
    # Error = 1 -  Values not to be trusted
    error = error
    # Boundary points
    boundary = boundary
    if (boundary[0] <= 3):
        # Centre point for obstacle based on height
        centre = (int((boundary[0] + boundary[2]) - boundary[3]), int(boundary[1]+(boundary[3]/2)))
        # Width of contour in pixels
        pix_width = boundary[3]
        # Angle from centre of screen in radians
        obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
        # Distance from camera in cm
        obs_dist = ((OBS_size[obs_indx] * FOCAL_PIX) / pix_width)
        # Create list of values
        obs_array_boundary = ([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
        return obs_array_boundary
    else:
        # Centre point for obstacle based on height
        centre = (int(boundary[0] + boundary[3]), int(boundary[1] + (boundary[3]/2)))
        # Width of contour in pixels
        pix_width = boundary[3]
        # Angle from centre of screen in radians
        obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
        # Distance from camera in cm
        obs_dist = ((OBS_size[obs_indx] * FOCAL_PIX) / pix_width)
        # Create list of values
        obs_array_boundary = ([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
        return obs_array_boundary

def hidden_obs(cnt, obs_indx, id_type, boundary, error):
    # Obstacle type index
    obs_indx = obs_indx
    # Obstacle label
    id_type = id_type
    # Error = 1 -  Values not to be trusted
    error = error
    # Boundary points
    boundary = boundary
    # Centre point for obstacle based on height
    centre, radius = cv2.minEnclosingCircle(cnt)
    # Width of contour in pixels
    pix_width = boundary[3]
    # Angle from centre of screen in radians
    obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
    # Distance from camera in cm
    obs_dist = ((OBS_size[obs_indx] * FOCAL_PIX) / pix_width)
    # Create list of values
    obs_array_boundary = ([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
    return obs_array_boundary       

def disp_image(image, obstacle_array):
    obs_image = image
    new_obs = obstacle_array
    for i, obs in enumerate(new_obs):
        # Draw rectangle
        cv2.rectangle(obs_image, (int(new_obs[i][5][0]), int(new_obs[i][5][1])),\
        (int(new_obs[i][5][0] + new_obs[i][5][2]), int(new_obs[i][5][1] +\
        new_obs[i][5][3])), OBS_col[new_obs[i][0]], 1)
        # Draw shape type
        #cv2.putText(obs_image, new_obs[i][1], (int(new_obs[i][5][0]),\
        #int(new_obs[i][5][1] + new_obs[i][5][3]) + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.3, OBS_col[new_obs[i][0]],1)
        # Draw distance in m
        cv2.putText(obs_image, "Dist:{:.1f}".format(new_obs[i][3]), (int(new_obs[i][5][0]),\
        int(new_obs[i][5][1] + new_obs[i][5][3]) + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.3, OBS_col[new_obs[i][0]],1)
        # Draw angle in degrees to obstacle from camera
        cv2.putText(obs_image, "Deg:{:.1f}".format(np.degrees(new_obs[i][2])), (int(new_obs[i][5][0]),\
        int(new_obs[i][5][1] + new_obs[i][5][3]) + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.3, OBS_col[new_obs[i][0]],1)
        # Draw area of obstacle from camera
        #cv2.putText(obs_image, "Area:{:.1f}".format(new_obs[i][6]), (int(new_obs[i][5][0]),\
        #int(new_obs[i][5][1] + new_obs[i][5][3]) + 39), cv2.FONT_HERSHEY_SIMPLEX, 0.3, OBS_col[new_obs[i][0]],1)
    return obs_image

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
    obstacle_array = detect_obs(hsv_masks)

    # Draw from and boundary
    return_im = disp_image(image, obstacle_array)

    return obstacle_array, return_im

