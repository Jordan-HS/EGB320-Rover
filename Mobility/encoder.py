#!/usr/bin/python

import Encoder
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
enc = Encoder.Encoder(23, 24)


def board_detect(board):
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


def motorSetup():
    board = Board(1, 0x10) 

    board_detect(board)

    while board.begin() != board.STA_OK:    # Board begin and check board status
        print_board_status()
        print("board begin faild")
        time.sleep(2)
    print("board begin success")

    # Set selected DC motor encoder enable
    board.set_encoder_enable(board.ALL)
    # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8
    board.set_encoder_reduction_ratio(board.ALL, 40)

    # Set DC motor pwm frequency to 1000HZ
    board.set_moter_pwm_frequency(1000)

    return board

board = motorSetup()

while True:
    print(enc.read())