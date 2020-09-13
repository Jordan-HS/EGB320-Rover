import time

from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

board = Board(1, 0x10)    # Select bus 1, set address to 0x10

def board_detect():
  l = board.detecte()
  print("Board list conform:")
  print(l)

''' print last operate status, users can use this variable to determine the result of a function call. '''
def print_board_status():
  if board.last_operate_status == board.STA_OK:
    print("board status: everything ok")
  elif board.last_operate_status == board.STA_ERR:
    print("board status: unexpected error")
  elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
    print("board status: device not detected")
  elif board.last_operate_status == board.STA_ERR_PARAMETER:
    print("board status: parameter error, last operate no effective")
  elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
    print("board status: unsupport board framware version")

# Set Target Velocities
	# inputs:
	#	x - the velocity of the robot in the forward direction (in m/s)
	#	theta_dt - the rotational velocity of the robot (in rad/s)
	def SetTargetVelocities(self, x_dot, theta_dot):
        board_detect()  # If you forget address you had set, use this to detected them, must have class instance

        # Set board controler address, use it carefully, reboot module to make it effective
        '''
        board.set_addr(0x10)
        if board.last_operate_status != board.STA_OK:
            print("set board address faild")
        else:
            print("set board address success")
        '''

        while board.begin() != board.STA_OK:    # Board begin and check board status
            print_board_status()
            print("board begin faild")
            time.sleep(2)
        print("board begin success")

        board.set_encoder_disable(board.ALL)    # Set selected DC motor encoder disable

        board.set_moter_pwm_frequency(1000) # Set DC motor pwm frequency to 1000HZ

        # ensure wheel base and wheel radius are set as these are not allowed to be changed
        wheelBase = 95e-3
        wheelRadius = 43e-3

        # Might not need
        leftWheelBias = 0
        rightWheelBias = 0

        minimumLinearSpeed = ??
        maximumLinearSpeed = ??
    
        # determine minimum wheel speed based on minimumLinear and maximumLinear speed
        minWheelSpeed = minimumLinearSpeed / wheelRadius
        maxWheelSpeed = maximumLinearSpeed / wheelRadius

        # calculate left and right wheel speeds in rad/s
        leftWheelSpeed = (x_dot - 0.5*theta_dot*wheelBase) / wheelRadius + leftWheelBias
        rightWheelSpeed = (x_dot + 0.5*theta_dot*wheelBase) / wheelRadius + rightWheelBias

        # ensure wheel speeds are not greater than maximum wheel speed
        leftWheelSpeed = min(leftWheelSpeed, maxWheelSpeed)
        rightWheelSpeed = min(rightWheelSpeed, maxWheelSpeed)

        # set wheel speeds to 0 if less than the minimum wheel speed
        if abs(leftWheelSpeed) < minWheelSpeed:
            leftWheelSpeed = 0
        if abs(rightWheelSpeed) < minWheelSpeed:
            rightWheelSpeed = 0

        # Scale to duty
        leftDuty = leftWheelSpeed
        rightDuty = rightWheelSpeed

        # set motor speeds
        board.motor_movement([board.M1], board.CW, leftDuty)    # DC motor 1 movement, orientation clockwise
        board.motor_movement([board.M2], board.CW, rightDuty)   # DC motor 2 movement, orientation count-clockwise
        print("Left Wheel Speed: %d     Right Wheel Speed: %d" %(leftWheelSpeed, rightWheelSpeed))