import math

# Forces of attraction
alpha = 1
spread = 2
radius = 7E-3

def getForce(object_type, distance, bearing, force=None):
    if object_type == "sample":
        if distance < radius:
            delta_x = 0
            delta_y = 0
        elif radius <= distance <= spread+radius:
            delta_x = (alpha * (distance - radius)*math.cos(bearing))
            delta_y = (alpha * (distance - radius)*math.sin(bearing))
        elif distance > spread + radius:
            delta_x = (alpha * spread * math.cos(bearing))
            delta_y = (alpha * spread * math.cos(bearing))

    if force is None:
        return delta_x, delta_y
    else:
        return (delta_x + force[0]), (delta_y + force[1])

def calculateMovement(delta_x, delta_y):
    radial_vel = math.atan2(delta_y, delta_x)
    forward_vel = math.sqrt(delta_x**2 + delta_y**2)/2

    return radial_vel, forward_vel