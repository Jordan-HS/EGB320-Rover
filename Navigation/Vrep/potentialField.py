import math

# Forces of attraction
alpha = 1
sample_radius = 7E-3

# Forces of repulsion
beta = 3
obs_spread = 0.5
obs_radius = 15E-3

def range(vec1, vec2):
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
    delta_x = 0
    delta_y = 0

    if rocks is not None:
        for rock in rocks:
            distance = distance([rover.x, rover.y], rock)
    if obstacles is not None:
        for obstacle in obstacles:
            distance = distance([rover.x, rover.y], obstacle)

    distance = range([rover.x, rover.y], goal)
    bearing = angle(goal, [rover.x, rover.y])

    delta_x += (alpha * (distance - sample_radius)*math.cos(bearing))
    delta_y += (alpha * (distance - sample_radius)*math.sin(bearing))

    target_angle = math.atan2(delta_y, delta_x)
    target_mag = math.sqrt(delta_x**2 + delta_y**2)/2

    return target_angle, target_mag


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

