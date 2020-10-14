import math
import matplotlib.pyplot as plt
from numpy import arange

# Forces of attraction
alpha = 4
sample_radius = 0.07

# Forces of repulsion
beta = 40
obs_spread = 0.2
obs_radius = 0.1

def dist(vec1, vec2):
    x_dist = abs(vec1[0] - vec2[0])
    y_dist = abs(vec1[1] - vec2[1])
    return math.sqrt(x_dist**2 + y_dist**2)

def angle(vec1, vec2):
    x_dist = vec1[0] - vec2[0]
    y_dist = vec1[1] - vec2[1]
    return math.atan2(y_dist, x_dist)

def getForce(rover):
    goal = rover.target
    rocks = rover.rocks
    obstacles = rover.obstacles
    lander = rover.lander
    delta_x = 0
    delta_y = 0

    if rover.target_type == "lander approach" or rover.target_type == "drop off":
        distance = dist([rover.x, rover.y], goal)
        bearing = angle(goal, [rover.x, rover.y])

        delta_x += (alpha * (distance - sample_radius)*math.cos(bearing))
        delta_y += (alpha * (distance - sample_radius)*math.sin(bearing))
    else:
        distance = dist([rover.x, rover.y], lander)
        bearing = angle(lander, [rover.x, rover.y])
        lander_size = 0.4
        lander_strength = 10
        lander_spread = 0.1

        if distance < lander_size:
            delta_x = 0
            delta_y = 0
        elif lander_size <= distance <= lander_spread+lander_size:
            delta_x += (-lander_strength * (lander_spread + lander_size - distance)*math.cos(bearing))
            delta_y += (-lander_strength * (lander_spread + lander_size - distance)*math.sin(bearing))
        elif distance > lander_spread + lander_size:
            delta_x += (alpha * lander_spread * math.cos(bearing))
            delta_y += (alpha * lander_spread * math.cos(bearing))

        # goal 
        distance = dist([rover.x, rover.y], goal)
        bearing = angle(goal, [rover.x, rover.y])

        delta_x += (alpha * (distance - sample_radius)*math.cos(bearing))
        delta_y += (alpha * (distance - sample_radius)*math.sin(bearing))

    if rocks is not None:
        for rock in rocks:
            distance = dist([rover.x, rover.y], rock)
    if obstacles is not None:
        for obstacle in obstacles:
            distance = dist([rover.x, rover.y], obstacle)
            bearing = angle(obstacle, [rover.x, rover.y])
            
            if distance <= (obs_spread + obs_radius)/2:
                delta_x += (-beta * (obs_spread + obs_radius - distance)*math.cos(bearing))
                delta_y += (-beta * (obs_spread + obs_radius - distance)*math.sin(bearing))
            elif (obs_spread + obs_radius)/2 < distance <= obs_spread + obs_radius:
                delta_x += (-beta/1.5 * (obs_spread + obs_radius - distance)*math.cos(bearing))
                delta_y += (-beta/1.5 * (obs_spread + obs_radius - distance)*math.sin(bearing))
            elif distance > obs_spread + obs_radius:
                delta_x += 0
                delta_y += 0

    


    target_angle = math.atan2(delta_y, delta_x)
    target_mag = math.sqrt(delta_x**2 + delta_y**2)/2

    return target_angle, target_mag

def show(rover):
    goal = rover.target
    rocks = rover.rocks
    obstacles = rover.obstacles
    lander = rover.lander
    
    points = 50

    ax = plt.axes()

    for x in arange(-1,1,2/points):
        for y in arange(-1,1,2/points):
            delta_x = 0
            delta_y = 0

            distance = dist([x, y], lander)
            bearing = angle(lander, [x, y])
            lander_size = 0.4
            lander_strength = 10
            lander_spread = 0.2
            if distance < lander_size:
                delta_x = 0
                delta_y = 0
            elif lander_size <= distance <= lander_spread+lander_size:
                delta_x += (-lander_strength * (lander_spread + lander_size - distance)*math.cos(bearing))
                delta_y += (-lander_strength * (lander_spread + lander_size - distance)*math.sin(bearing))
            elif distance > lander_spread + lander_size:
                delta_x += (alpha * lander_spread * math.cos(bearing))
                delta_y += (alpha * lander_spread * math.cos(bearing))

            # goal 
            distance = dist([x, y], goal)
            bearing = angle(goal, [x, y])

            delta_x += (alpha * (distance - sample_radius)*math.cos(bearing))
            delta_y += (alpha * (distance - sample_radius)*math.sin(bearing))

            if rocks is not None:
                for rock in rocks:
                    distance = dist([x, y], rock)
            if obstacles is not None:
                for obstacle in obstacles:
                    distance = dist([x, y], obstacle)
                    bearing = angle(obstacle, [x, y])
                    
                    if distance <= (obs_spread + obs_radius)/2:
                        delta_x += (-beta * (obs_spread + obs_radius - distance)*math.cos(bearing))
                        delta_y += (-beta * (obs_spread + obs_radius - distance)*math.sin(bearing))
                    elif (obs_spread + obs_radius)/2 < distance <= obs_spread + obs_radius:
                        delta_x += (-beta/1.5 * (obs_spread + obs_radius - distance)*math.cos(bearing))
                        delta_y += (-beta/1.5 * (obs_spread + obs_radius - distance)*math.sin(bearing))
                    elif distance > obs_spread + obs_radius:
                        delta_x += 0
                        delta_y += 0


            plt.arrow(x*100, y*100, delta_x, delta_y, head_width=1)
    
    plt.show()
    # if object_type == "sample":
    #     if distance < sample_radius:
    #         delta_x = 0
    #         delta_y = 0
    #     elif sample_radius <= distance <= sample_spread+sample_radius:
    #         delta_x = (alpha * (distance - sample_radius)*math.cos(bearing))
    #         delta_y = (alpha * (distance - sample_radius)*math.sin(bearing))
    #     elif distance > sample_spread + sample_radius:
    #         delta_x = (alpha * sample_spread * math.cos(bearing))
    #         delta_y = (alpha * sample_spread * math.cos(bearing))
    # elif object_type == "lander":
    #     if distance < sample_radius:
    #         delta_x = 0
    #         delta_y = 0
    #     elif sample_radius <= distance <= sample_spread+sample_radius:
    #         delta_x = (alpha * (distance - sample_radius)*math.cos(bearing))
    #         delta_y = (alpha * (distance - sample_radius)*math.sin(bearing))
    #     elif distance > sample_spread + sample_radius:
    #         delta_x = (alpha * sample_spread * math.cos(bearing))
    #         delta_y = (alpha * sample_spread * math.cos(bearing))
    # elif object_type == "obstacle":
    #     if distance <= obs_spread + obs_radius:
    #         delta_x = (-beta * (obs_spread + obs_radius - distance)*math.cos(bearing))
    #         delta_y = (-beta * (obs_spread + obs_radius - distance)*math.sin(bearing))
    #     elif distance > obs_spread + obs_radius:
    #         delta_x = 0
    #         delta_y = 0
    # elif object_type == "rock":
    #     if distance < sample_radius:
    #         delta_x = 0
    #         delta_y = 0
    #     elif sample_radius <= distance <= sample_spread+sample_radius:
    #         delta_x = (alpha * (distance - sample_radius)*math.cos(bearing))
    #         delta_y = (alpha * (distance - sample_radius)*math.sin(bearing))
    #     elif distance > sample_spread + sample_radius:
    #         delta_x = (alpha * sample_spread * math.cos(bearing))
    #         delta_y = (alpha * sample_spread * math.cos(bearing))

    # if force is None:
    #     return delta_x, delta_y
    # else:
    #     return (delta_x + force[0]), (delta_y + force[1])

def calculateMovement(delta_x, delta_y, bearing):
    radial_dir = math.atan2(delta_y, delta_x)
    forward_mag = math.sqrt(delta_x**2 + delta_y**2)/2

    if math.isclose(radial_dir, bearing, abs_tol=math.radians(5)):
        return "forward", 1
    elif radial_dir < bearing:
        return "right", 1
    elif radial_dir > bearing:
        return "left", 1

