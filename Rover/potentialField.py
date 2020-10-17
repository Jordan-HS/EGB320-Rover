import math
# import matplotlib.pyplot as plt
from numpy import arange

def dist(vec1, vec2):
    x_dist = abs(vec1[0] - vec2[0])
    y_dist = abs(vec1[1] - vec2[1])
    return math.sqrt(x_dist**2 + y_dist**2)

def getForce(currentPos, goal, obstacles, avoidLander=True):
    
    ## Calculate attractive force ##
    dstar = 0.1
    att_control = 1
    goal_dist = dist(currentPos, goal)

    if goal_dist <= dstar:
        # calculate Uatt
        Uatt = (att_control * goal_dist**2)/2

        # Calculate delta Uatt
        dx_Uatt = att_control * (currentPos[0] - goal[0])
        dy_Uatt = att_control * (currentPos[1] - goal[1])
    else:
        # calculate Uatt
        Uatt = ( dstar * att_control * goal_dist ) - ( 1/2 * att_control * goal_dist**2 )

        # Calculate delta Uatt
        dx_Uatt = ( dstar * att_control * (currentPos[0] - goal[0]) ) / ( goal_dist )
        dy_Uatt = ( dstar * att_control * (currentPos[1] - goal[1]) ) / ( goal_dist )

        

    ## Calculate repulsive forces ##
    qstar = 0.2
    rep_control = 0.01
    repulsivesForces = []
    if obstacles is not None:
        for obstacle in obstacles:
            rep_dist = dist(currentPos, obstacle)

            if rep_dist <= qstar:
                # Calculate Urep
                Urep = 1/2 * rep_control * ( 1/rep_dist - 1/qstar )**2

                # Calculate delta Urep
                dx_Urep = rep_control * ( 1/qstar - 1/rep_dist ) * 1/(rep_dist**2) * (currentPos[0] - obstacle[0])
                dy_Urep = rep_control * ( 1/qstar - 1/rep_dist ) * 1/(rep_dist**2) * (currentPos[1] - obstacle[1])

                repulsivesForces.append([Urep, dx_Urep, dy_Urep])
 

    ## Add wall avoidance ##
    wall_padding = 0.1
    if currentPos[0] > 1-wall_padding:
        wall_dist = 1 - currentPos[0]
        repulsivesForces.append([0, wall_padding-wall_dist, 0])
    elif currentPos[0] < -1+wall_padding:
        wall_dist = 1 + currentPos[0]
        repulsivesForces.append([0, -(wall_padding-wall_dist), 0])
    if currentPos[1] > 1-wall_padding:
        wall_dist = 1 - currentPos[1]
        repulsivesForces.append([0, 0, wall_padding-wall_dist])
    elif currentPos[1] < -1+wall_padding:
        wall_dist = 1 + currentPos[1]
        repulsivesForces.append([0, 0, -(wall_padding-wall_dist)])

    ## Avoid lander if needed ##


    ## sum the forces ##
    U = [0, 0]

    # Add attractive force
    U[0] = -dx_Uatt
    U[1] = -dy_Uatt

    # Add repulsive forces
    if len(repulsivesForces) > 0:
        for rep in repulsivesForces:
            U[0] += -rep[1]
            U[1] += -rep[2]

    return U



# def show(goal, obstacles, avoidLander=True):
#     points = 100
#     ax = plt.axes()

#     for x in arange(-1,1,2/points):
#         for y in arange(-1,1,2/points):
#             U = getForce([x, y], goal, obstacles)

#             if -1 < U[0]+x < 1 and -1 < U[1]+y < 1:
#                 plt.arrow(x*100, y*100, U[0]*10, U[1]*10, head_width=1)

#     plt.show()

# goal = [-0.75, -0.75]
# obstacles = [[-0.6, -0.6]]
# show(goal, obstacles)