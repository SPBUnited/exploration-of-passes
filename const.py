"""
Определение необходимых констант
"""

##################################################
# GAME SETTING CONSTS
MAX_SPEED = 1500
MAX_SPEED_R = 30
SOFT_MAX_SPEED = 1000
SOFT_MAX_SPEED_R = 8
ACCELERATION = 3

##################################################
# GEOMETRY CONSTS

BALL_R = 0.0225
ROBOT_R = 100

GOAL_SIZE = 1000
GOAL_PEN_DX = 1000
GOAL_PEN_DY = 1000

FIELD_WIDTH = 9000
FIELD_HEIGH = 6000

SCREEN_WIDTH = 1200 // 2
SCREEN_HEIGH = int(SCREEN_WIDTH * 2 / FIELD_WIDTH * FIELD_HEIGH)
