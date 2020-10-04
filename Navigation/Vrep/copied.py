# Import the QUT rover bot library
from roverbot_lib import *
import setup
from math import radians, degrees
from potentialField import getForce, calculateMovement
import time

# GPIO LED output
LED_out = False

# HUD output
HUD = True

# Potential fields view
POT = False

# Initialise the simulation
robotParameters, sceneParameters = setup.init_sim()

if HUD:
    import os
    import sys
    clear = lambda: os.system('cls')

if LED_out:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(26, GPIO.OUT) # RED
    GPIO.setup(16, GPIO.OUT) # GREEN
    GPIO.setup(13, GPIO.OUT) # YELLOW


class Brain:
    def __init__(self):
        self.lander = []
        self.rocks = []
        self.samples = []
        self.obstacles = []
        self.last_time = time.time()
        self.time_scale = 0.2
        self.speed = 0
        self.bearing = 0
        self.x = 0
        self.y = 0

    def update(self, samplesRB, landerRB, obstaclesRB, rocksRB, movement, magnitude):

        if landerRB is not None:
            print('')
        if landerRB is None and len(self.lander) > 0:
             # make an estimation
             self.lander = self.updateObjectsPosition(self.lander, movement, magnitude)
        elif landerRB is not None:
            # update with current posland
            self.lander = self.checkInMemory([landerRB], self.lander)

        if samplesRB is None and len(self.samples) > 0:
            # make an estimation
            self.samples = self.updateObjectsPosition(self.samples, movement, magnitude)
        elif samplesRB is not None:
            # update with current pos
            self.samples = self.checkInMemory(samplesRB, self.samples)
        
        if obstaclesRB is None and len(self.obstacles) > 0:
             # make an estimation
             self.obstacles = self.updateObjectsPosition(self.obstacles, movement, magnitude)
        elif obstaclesRB is not None:
            # update with current pos
            self.obstacles = self.checkInMemory(obstaclesRB, self.obstacles)

        if rocksRB is None and len(self.rocks) > 0:
             # make an estimation
             self.rocks = self.updateObjectsPosition(self.rocks, movement, magnitude)
        elif rocksRB is not None:
            # update with current pos
            self.rocks = self.checkInMemory(rocksRB, self.rocks)

        # Update estimation of current pos and bearing
        if movement == "left":
            delta_time = (time.time() - self.last_time) * self.time_scale
            delta_rad = delta_time * magnitude
            self.bearing -= delta_rad
        elif movement == "right":
            delta_time = (time.time() - self.last_time) * self.time_scale
            self.last_time = delta_time
            delta_rad = delta_time * magnitude
            self.bearing += delta_rad

        # Update the last time a update was performed
        self.last_time = time.time()

    def checkInMemory(self, objects_seen, objects_memory):
        if len(objects_memory) == 0:
            # First samples seen
            for object_seen in objects_seen:
                objects_memory.append(object_seen)
        else:
            # Check that the sample being seen isnt already in memory
            for object_seen in objects_seen:
                for index, object_memory in enumerate(objects_memory):
                    if (math.isclose(object_seen[0], object_memory[0], abs_tol=0.01) 
                        and math.isclose(object_seen[1], object_memory[1], abs_tol=math.pi/12)):
                        objects_memory[index] = object_seen

        return objects_memory

    def updateObjectsPosition(self, objects, movement, magnitude):
        if movement == "forward":
            for obj in objects:
                # Break current position to xy based on polar
                obj_x = obj[0] * math.sin(obj[1])
                obj_y = obj[0] * math.cos(obj[1])

                # Calculate distance travelled by rover
                delta_time = (time.time() - self.last_time) * self.time_scale
                self.last_time = delta_time
                dist = delta_time * magnitude               # distance = time * speed

                # Subtract distace to y
                obj_y -= dist

                # convert back to polar and upadte object pos
                obj[0] = math.sqrt(obj_x**2 + obj_y**2)
                obj[1] = math.atan2(obj_x, obj_y)


        elif movement == "left":
            for obj in objects:
                delta_time = (time.time() - self.last_time) * self.time_scale
                delta_rad = delta_time * magnitude
                obj[1] += delta_rad

                # Bearing checks
                if obj[1] > 0:
                    if (obj[1] % math.pi) != obj[1]:
                        obj[1] = -(math.pi - obj[1]% math.pi)
                elif obj[1] < 0:
                    if (-obj[1] %  math.pi) != -obj[1]:
                        obj[1] = obj[1]% math.pi

        elif movement == "right":
            for obj in objects:
                delta_time = (time.time() - self.last_time) * self.time_scale
                delta_rad = delta_time * magnitude
                obj[1] -= delta_rad
                obj[1] = obj[1] % (2 * math.pi)
        
        return objects


def moveBot(movement, magnitude):
    if movement == "forward":
        lunarBotSim.SetTargetVelocities(magnitude, 0)
    elif movement == "left":
        lunarBotSim.SetTargetVelocities(0, magnitude)
    elif movement == "right":
        lunarBotSim.SetTargetVelocities(0, -magnitude)


try:
    # Create VREP RoverBot object - this will attempt to open a connection to VREP. Make sure the VREP simulator is running.
    lunarBotSim = VREP_RoverRobot('192.168.1.111', robotParameters, sceneParameters)
    lunarBotSim.StartSimulator()
    

    # Get the objects exact position and use that to cheat and test logic

    # memory stuff
    rover = Brain()

    force_memory = [None]
    rover.last_time = time.time()

    while (True):
        delta_x = 0
        delta_y = 0
        movement = 'forward'
        magnitude = 0.1
        
        # Get Detected Objects
        samplesRB, landerRB, obstaclesRB, rocksRB = lunarBotSim.GetDetectedObjects()
        # lunarBotSim.GetObjectPositions()
        # print(lunarBotSim.samplePositions)


        if force_memory[0] is not None:
            if time.time() - force_memory[0][2] > 10:
                force_memory = [None]
            else:
                force_memory[0][0] = force_memory[0][0] * (((10 - (time.time() - force_memory[0][2]))/13))
                force_memory[0][1] = force_memory[0][1] * (((10 - (time.time() - force_memory[0][2]))/13))

        # Check to see if any obstacles are within the camera's FOV
        if obstaclesRB is not None:
            # loop through each obstacle detected using Pythonian way
            for obstacle in obstaclesRB:
                obstacleRange = obstacle[0]
                obstacleBearing = obstacle[1]

                # Obstacle avoidance
                if force_memory[0] is not None:
                            delta_x += force_memory[0][0]
                            delta_y += force_memory[0][1]
                delta_x, delta_y = getForce("obstacle", obstacleRange, obstacleBearing, [delta_x, delta_y])
                force_memory[0] = ([delta_x, delta_y, time.time()])


        ### INITIAL STATE - MOVING OFF LANDER ###
        if not lunarBotSim.SampleCollected() and ((landerRB is not None and landerRB[0] < 1) or (len(rover.lander) != 0 and rover.lander[0][0] < 1)):
            current_state = "INITIAL STATE - MOVING OFF LANDER"

            movement = "forward"
            magnitude = 1
        ### NO OBJECTS SEEN - SEARCH FOR OBJECT ###
        elif samplesRB is None and obstaclesRB is None and rocksRB is None and not lunarBotSim.SampleCollected():
            current_state = "NO OBJECTS SEEN - SEARCH FOR OBJECT"
            if force_memory[0] is not None:
                movement, magnitude = calculateMovement(force_memory[0][0], force_memory[0][1], rover.bearing)
            else:
                radial_vel = 0
                forward_vel = 0

            movement = "left"
            magnitude = 0.2

            if LED_out:
                # set red LED
                GPIO.output(26, GPIO.HIGH)
                GPIO.output(16, GPIO.LOW)
                GPIO.output(13, GPIO.LOW)

        ### SAMPLE SEEN - NAVIGATE TOWARDS SAMPLE ###
        elif not lunarBotSim.SampleCollected():
            current_state = "SAMPLE SEEN - NAVIGATE TOWARDS SAMPLE"
            # Check to see if the sample is within the camera's FOV
            if samplesRB is not None:
                # loop through each sample detected using Pythonian way
                for sample in samplesRB:
                    sampleRange = sample[0]
                    sampleBearing = sample[1]

                    # check if sample is in range and there isnt one currently held
                    if sampleRange < 0.03 and not lunarBotSim.SampleCollected():
                        lunarBotSim.CollectSample()
                    elif not lunarBotSim.SampleCollected():
                        if force_memory[0] is not None:
                            delta_x += force_memory[0][0]
                            delta_y += force_memory[0][1]
                        delta_x, delta_y = getForce("sample", sampleRange, sampleBearing, [delta_x, delta_y])

                    if LED_out:
                        # set Yellow LED
                        GPIO.output(26, GPIO.LOW)
                        GPIO.output(16, GPIO.LOW)
                        GPIO.output(13, GPIO.HIGH)

            if rocksRB is not None:
                for rock in rocksRB:
                    rockRange = rock[0]
                    rockBearing = rock[1]
                
                    # Obstacle avoidance
                    if force_memory[0] is not None:
                                delta_x += force_memory[0][0]
                                delta_y += force_memory[0][1]

                    if rockRange < (0.03+0.072):
                        radial_vel = 0
                        forward_vel = 0
                        break
                    else:
                        delta_x, delta_y = getForce("rock", rockRange, rockBearing, [delta_x, delta_y])

            # Check to see if any obstacles are within the camera's FOV
            # if rocksRB is not None:
            #     # loop through each obstacle detected using Pythonian way
            #     for obstacle in obstaclesRB:
            #         obstacleRange = obstacle[0]
            #         obstacleBearing = obstacle[1]

            movement, magnitude = calculateMovement(delta_x, delta_y, rover.bearing)


        ### SAMPLE COLLECTED - NAVIGATE TOWARDS DROP OFF ###
        elif lunarBotSim.SampleCollected():
            current_state = "SAMPLE COLLECTED - NAVIGATE TOWARDS DROP OFF"
            if LED_out:
                # set green LED
                GPIO.output(26, GPIO.LOW)
                GPIO.output(16, GPIO.HIGH)
                GPIO.output(13, GPIO.LOW)
            if landerRB is None:
                if force_memory[0] is not None:
                    movement, magnitude = calculateMovement(force_memory[0][0], force_memory[0][1], rover.bearing)
                else:
                    radial_vel = 0
                    forward_vel = 0

                radial_vel += 1.2    # rotate on the spot to search
                forward_vel += 0.1
            else:
                landerRange = landerRB[0]
                landerBearing = landerRB[1]

                if landerRange < 0.035:
                    lunarBotSim.DropSample()
                else:
                    delta_x, delta_y = getForce("lander", landerRange, landerBearing, [delta_x, delta_y])

                    # Calculate force to drop off
                    if force_memory[0] is not None:
                            delta_x += force_memory[0][0]
                            delta_y += force_memory[0][1]
                    movement, magnitude = calculateMovement(delta_x, delta_y, rover.bearing)

        # Get Detected Wall Points
        wallPoints = lunarBotSim.GetDetectedWallPoints()

        # Move the robot
        moveBot(movement, magnitude)
        
        # Update memory
        rover.update(samplesRB, landerRB, obstaclesRB, rocksRB, movement, magnitude)
        

        ## Display HUD
        if HUD:
            clear()
            HUD_string = ("#### {} ####\n"
                          "Movement: {}          Memory\n"
                          "Magnitude: {:.2f}       lander: {}\n"
                          "                      Samples: {}\n"
                          "                      Rocks: {}\n"
                          "                      Obstacles: {}"
                          "                      Bearing:{} "
                          "".format(current_state, movement, magnitude, rover.lander, rover.samples, rover.rocks, rover.obstacles, rover.bearing*(180/math.pi)))
            print(HUD_string)

        # Update Ball Position
        lunarBotSim.UpdateObjectPositions()

except KeyboardInterrupt as e:
    # attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
    lunarBotSim.StopSimulator()


