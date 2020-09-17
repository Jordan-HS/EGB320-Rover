import math

# Forces of attraction
alpha = 1
sample_spread = 2
sample_radius = 7E-3

# Forces of repulsion
beta = 4
obs_spread = 0.5
obs_radius = 15E-3

def getForce(object_type, distance, bearing, force=None):
    if object_type == "sample":
        if distance < sample_radius:
            delta_x = 0
            delta_y = 0
        elif sample_radius <= distance <= sample_spread+sample_radius:
            delta_x = (alpha * (distance - sample_radius)*math.cos(bearing))
            delta_y = (alpha * (distance - sample_radius)*math.sin(bearing))
        elif distance > sample_spread + sample_radius:
            delta_x = (alpha * sample_spread * math.cos(bearing))
            delta_y = (alpha * sample_spread * math.cos(bearing))
    elif object_type == "lander":
        if distance < sample_radius:
            delta_x = 0
            delta_y = 0
        elif sample_radius <= distance <= sample_spread+sample_radius:
            delta_x = (alpha * (distance - sample_radius)*math.cos(bearing))
            delta_y = (alpha * (distance - sample_radius)*math.sin(bearing))
        elif distance > sample_spread + sample_radius:
            delta_x = (alpha * sample_spread * math.cos(bearing))
            delta_y = (alpha * sample_spread * math.cos(bearing))
    elif object_type == "obstacle":
        if distance <= obs_spread + obs_radius:
            delta_x = (-beta * (obs_spread + obs_radius - distance)*math.cos(bearing))
            delta_y = (-beta * (obs_spread + obs_radius - distance)*math.sin(bearing))
        elif distance > obs_spread + obs_radius:
            delta_x = 0
            delta_y = 0

    if force is None:
        return delta_x, delta_y
    else:
        return (delta_x + force[0]), (delta_y + force[1])

def calculateMovement(delta_x, delta_y):
    radial_vel = math.atan2(delta_y, delta_x)
    forward_vel = math.sqrt(delta_x**2 + delta_y**2)/2

    return radial_vel, forward_vel