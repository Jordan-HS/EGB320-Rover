import concurrent.futures
import multiprocessing
import sys
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

# Initialse variablescd
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
camera.framerate = 8
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
    crop_img = image[int(image.shape[0]*0.25):image.shape[0]]
    return crop_img

# # Image crop to decrease image processing time
# def crop_image(image):
#     now = time.time()
#     crop_img = image[int(image.shape[0]*0.25):image.shape[0]]
#     elapsed = time.time() - now
#     rate = 1.0 / elapsed
#     print([rate, "Crop"])
#     return crop_img

# HSV colour threshold filter
def mask_obs(image):
    # Convert BGR to HSV image
    now = time.time()
    HSV_bgy = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Convert RGB to HSV image
    HSV_o = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    masks_HSV = []
    # Apply mask for HSV range
    for indx, thresh in enumerate(HSV_thresh):
        # Blue Green and Yellow threshold and filter
        if indx < 4:
            now = time.time()
            masks_HSV.append(cv2.inRange(HSV_bgy, thresh[0], thresh[1]))
            elapsed = time.time() - now
            rate = 1.0 / elapsed
            #print([rate, "HSV_thresh+sum"])
        # Orange threshold and filter
        else:
            masks_HSV.append(cv2.inRange(HSV_o, thresh[0], thresh[1]))
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, "HSV_mask"])
    return masks_HSV

# Filters HSV image to remove noise
def HSV_filter(image):
    now = time.time()
    # HSV_sum = np.sum(image)
    # if HSV_sum > 0:
    # Opening - Erosion followed by dilation
    mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.erode(mask, None, iterations=2)
    # Applying dilation a second time removes noise
    mask = cv2.dilate(mask, None, iterations=2)
    # else:
    #     mask = image
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, "HSV_filter"])
    return mask

def HSV_orange_filter(image):
    now = time.time()
    # Opening - Erosion followed by dilation
    mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.erode(mask, None, iterations=1)
    # Applying dilation a second time removes noise
    mask = cv2.dilate(mask, None, iterations=3)
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, "Orange_filter"])
    return mask

# Define obstacles
def detect_obs(hsv_masks):
    obs_array = []
    for indx, mask in enumerate(hsv_masks):
        HSV_sum = np.sum(mask)
        if HSV_sum == 0:
            continue
        #contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Check for rocks and satellite crash obstacles
        if indx < 2:
            now = time.time()
            _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
                    error = 0
                    if indx == 1:
                        if boundary[1] >= 3:
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
                    centre, _ = cv2.minEnclosingCircle(cnt)
                    # Width of contour in pixels
                    pix_width = boundary[2]
                    # Angle from centre of screen in radians
                    obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
                    # Distance from camera in cm
                    obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
                    # Create list of values
                    #print([id_type, pix_width, obs_dist])
                    obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
            elapsed = time.time() - now
            rate = 1.0 / elapsed
            #print([rate, OBS_type[indx]])
        # Check for lander
        elif indx == 2:
            now = time.time()
            _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            elapsed = time.time() - now
            rate = 1.0 / elapsed
            #print([rate, OBS_type[indx]])
        # Check for Wall
        elif indx == 3:
            now = time.time()
            _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnt = np.concatenate(contours)
            #boundary_area = cv2.findNonZero(mask)
            # obstacle type index
            obs_indx = indx
            # Obstacle label
            id_type = OBS_type[indx]
            # Boundary (x,y,w,h) box of contour
            boundary = cv2.boundingRect(cnt)
            # Error if boundaries outside of norm
            error = 0 # No error for wall
            centre, radius = cv2.minEnclosingCircle(cnt)
            # Width of contour in pixels
            pix_width = boundary[2]
            # Angle from centre of screen in radians
            obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
            # Distance from camera in cm
            obs_dist = boundary[1]-boundary[3]
            if obs_dist > 15:
                error = 2
            # Create list of values
            obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
            elapsed = time.time() - now
            rate = 1.0 / elapsed
            #print([rate, OBS_type[indx]])
        # Check for samples
        else:
            now = time.time()
            _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            elapsed = time.time() - now
            rate = 1.0 / elapsed
            #print([rate, OBS_type[indx]])
    #print(obs_array)
    return obs_array

# Define Rocks
def detect_rock(hsv_masks):
    now = time.time()
    mask = hsv_masks
    HSV_sum = np.sum(mask)
    if HSV_sum == 0:
        return
    obs_array = []
    indx = 0
    _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            error = 0
            if boundary[1] >= 3:
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
            centre, _ = cv2.minEnclosingCircle(cnt)
            # Width of contour in pixels
            pix_width = boundary[2]
            # Angle from centre of screen in radians
            obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
            # Distance from camera in cm
            obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
            # Create list of values
            #print([id_type, pix_width, obs_dist])
            obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, OBS_type[indx]])
    return obs_array

# Define Satellite
def detect_sat(hsv_masks):
    now = time.time()
    mask = hsv_masks
    HSV_sum = np.sum(mask)
    if HSV_sum == 0:
        return
    obs_array = []
    indx = 1
    _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            error = 0
            if boundary[1] >= 3:
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
            centre, _ = cv2.minEnclosingCircle(cnt)
            # Width of contour in pixels
            pix_width = boundary[2]
            # Angle from centre of screen in radians
            obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
            # Distance from camera in cm
            obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
            # Create list of values
            #print([id_type, pix_width, obs_dist])
            obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, OBS_type[indx]])
    return obs_array

# Define Lander
def detect_land(hsv_masks):
    now = time.time()
    mask = hsv_masks
    HSV_sum = np.sum(mask)
    if HSV_sum == 0:
        return
    obs_array = []
    indx = 2
    #_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    _,contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    centre, _ = cv2.minEnclosingCircle(cnt)
    # Width of contour in pixels
    pix_width = boundary[2]
    # Angle from centre of screen in radians
    obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
    # Distance from camera in cm
    obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
    # Create list of values
    obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, OBS_type[indx]])
    return obs_array

# Define Wall
def detect_wall(hsv_masks):
    now = time.time()
    mask = hsv_masks
    HSV_sum = np.sum(mask)
    if HSV_sum == 0:
        return
    obs_array = []
    indx = 3
    #_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    _,contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Sort contours by size
    areas = [cv2.contourArea(cnt) for cnt in contours]
    # index largest object
    max_index = np.argmax(areas)
    cnt = contours[max_index]
    # Draw simplified boundary
    boundary = cv2.convexHull(cnt)
    # obstacle type index
    obs_indx = indx
    # Obstacle label
    id_type = OBS_type[indx]
    # Error if boundaries outside of norm
    error = 0 # No error for wall
    centre, _ = cv2.minEnclosingCircle(cnt)
    # Angle from centre of screen in radians
    obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
    # Distance from camera in cm
    obs_dist = 1
    if obs_dist > 15:
        error = 2
    # If both edge points don't touch the same edge, then the edge is hot and should be recorded as a boundary
    # for i in cnt:
    #     if cnt[] == cnt[i+1][0] or cnt[0][i] == cnt[i+1][0]:

    # Create list of values
    obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, OBS_type[indx]])
    return obs_array

# Define Samples
def detect_samp(hsv_masks):
    mask = hsv_masks
    now = time.time()
    HSV_sum = np.sum(mask)
    if HSV_sum == 0:
        return
    obs_array = []
    indx = 4
    _,contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            centre, _ = cv2.minEnclosingCircle(cnt)
            # Width of contour in pixels
            pix_width = boundary[2]
            # Angle from centre of screen in radians
            obs_ang = np.arctan(((IMG_X/2) - int(centre[0]))/FOCAL_PIX)
            # Distance from camera in cm
            obs_dist = ((OBS_size[indx] * FOCAL_PIX) / pix_width)
            # Create list of values
            obs_array.append([obs_indx, id_type, obs_ang, obs_dist, centre, boundary, error])
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, OBS_type[indx]])
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
    centre, _ = cv2.minEnclosingCircle(cnt)
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
    for i, _ in enumerate(new_obs):
        if new_obs[i][0] != 3:
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
        else:
            # Draw contour around wall
            #cv2.drawContours(canvas, [new_obs[i][5]], -1, (0, 0, 255), 3)
            cv2.drawContours(obs_image, [new_obs[i][5]], -1, OBS_col[new_obs[i][0]], 1)

    return obs_image

def current_observation():
    # Grab frame
    camera.capture(rawCapture, format="bgr", use_video_port=True)
    image = rawCapture.array
    #image = cv2.imread('/home/logic/EGB320-Rover/Vision/20201028-072905.png') # reads image 'opencv-logo.png' as grayscale
    rawCapture.truncate(0)

    # Crop image
    image = crop_image(image)

    # Apply HSV threshold to frame
    hsv_masks = mask_obs(image)

    # Apply filter to mask images to remove noise
    # Loop through filter
    mask_filter_loop = []
    now = time.time()
    for mask in hsv_masks:
        mask_filter_loop.append(HSV_filter(mask))
    elapsed = time.time() - now
    rate = 1.0 / elapsed
    #print([rate, "Filter_Loop"])

    # Multiprocess filter
    # now = time.time()
    # with concurrent.futures.ProcessPoolExecutor() as executor:
    #     mask_filter = executor.map(HSV_filter, hsv_masks)
    # elapsed = time.time() - now
    # rate = 1.0 / elapsed
    # print([rate, "Filter_CC"])

    obstacle_array = []
    # Determine distance, angle ID and type
    #obstacle_array_loop = detect_obs(mask_filter)

    # Rocks distance, angle ID and type
    rock_array = detect_rock(mask_filter_loop[0])
    if rock_array is not None:
        #print(rock_array)
        for obs in rock_array:
            obstacle_array.append(obs)

    # Satellites distance, angle ID and type
    sat_array = detect_sat(mask_filter_loop[1])
    if sat_array is not None:
        #print(sat_array)
        for obs in sat_array:
            obstacle_array.append(obs)

    # Lander distance, angle ID and type
    land_array = (detect_land(mask_filter_loop[2]))
    if land_array is not None:
        #print(land_array)
        for obs in land_array:
            obstacle_array.append(obs)

    # Wall distance, angle ID and type
    wall_array = (detect_wall(mask_filter_loop[3]))
    if wall_array is not None:
        #print(wall_array)
        for obs in wall_array:
            obstacle_array.append(obs)

    # Samples distance, angle ID and type
    samp_array = (detect_samp(mask_filter_loop[4]))
    if samp_array is not None:
        #print(samp_array)
        for obs in samp_array:
            obstacle_array.append(obs)
    #print(obstacle_array)
    # Draw from and boundary
    return_im = disp_image(image, obstacle_array)
    return obstacle_array, return_im

try:
    print("Booted")
    time.sleep(1)
    input("Press Enter to continue...")
    time.sleep(1)
    observation, img = current_observation()
    av_process = 0
    av_count = 0
    rate = 0
    while True:
        total_now = time.time()
        observation, img = current_observation()
        total_elapsed = time.time() - total_now
        total_rate = 1.0 / total_elapsed
        #print(total_rate)
        av_process += total_rate
        av_count += 1
        if av_count == 15:
            av_rate = av_process/15
            print("Average processing rate:{}.".format(av_rate))
            av_count = 0
            av_process = 0
        cv2.imshow("View", img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        # Save image output by pressing 's'
        elif key == ord("s"):
            cv2.imwrite('mask.png',mask)
            cv2.imwrite('image_frame.png',image)
            cv2.imwrite('result.png',obs_image)
except KeyboardInterrupt:
    print("Stopped")
