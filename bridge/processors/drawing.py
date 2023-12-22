# type: ignore
import math

import numpy
import pygame
import time

import bridge.processors.auxiliary as aux
import bridge.processors.const as const


class Image:
    def __init__(self) -> None:
        # info about window
        global screen
        goal_dx, goal_dy = abs(const.GOAL_DX), abs(const.GOAL_DY)
        self.scale = max(400 / goal_dx, 300 / goal_dy)
        self.middle_x, self.middle_y = screen.get_size()
        self.middle_x /= 2
        self.middle_y /= 2
        self.upper_border = self.middle_y - goal_dy * self.scale
        self.lower_border = self.middle_y + goal_dy * self.scale
        self.left_border = self.middle_x - goal_dx * self.scale
        self.right_border = self.middle_x + goal_dx * self.scale
        self.size_x = goal_dx * self.scale * 2
        self.size_y = goal_dy * self.scale * 2


def initialize():
    pygame.init()
    return pygame.display.set_mode((800, 600), pygame.RESIZABLE), pygame.display.set_caption("Football Field")


def draw_field():
    global screen
    img = Image()
    back_color = (128, 128, 128)
    field_color = (0, 192, 0)
    line_color = (255, 255, 255)
    screen.fill(back_color)
    pygame.draw.rect(screen, field_color, (img.left_border, img.upper_border, img.size_x, img.size_y))  # Поле
    pygame.draw.rect(screen, line_color, (img.left_border, img.upper_border, img.size_x, img.size_y), 2)  # Обводка поля
    pygame.draw.line(
        screen, line_color, (img.left_border, img.middle_y), (img.right_border, img.middle_y), 2
    )  # Горизонтальная линия
    pygame.draw.line(
        screen, line_color, (img.middle_x, img.upper_border), (img.middle_x, img.lower_border), 2
    )  # Вертикальная линия
    pygame.draw.circle(screen, line_color, (img.middle_x, img.middle_y), 50, 2)  # Круг в центре


def draw_robot(x, y, angle=0):
    global screen
    robot_color = (0, 0, 255)
    robot_radius = 20
    robot_length = 40
    end_point = (
        int(x + robot_length * math.cos(math.radians(angle))),
        int(y - robot_length * math.sin(math.radians(angle))),
    )
    pygame.draw.circle(screen, robot_color, (int(x), int(y)), robot_radius)
    pygame.draw.line(screen, robot_color, (x, y), end_point, 2)


def draw_dot(pos: aux.Point, size: float = 3, color=(255, 0, 0)):
    global screen
    img = Image()
    pygame.draw.circle(screen, color, (pos.x * img.scale + img.middle_x, -pos.y * img.scale + img.middle_y), size)


def draw_bang_bang_traj(pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: aux.Point = None):
    if v2 is None:
        proj_without_end_speed(pos1, v1, pos2)
    else:
        proj_with_end_speed(pos1, v1, pos2, v2)


def proj_with_end_speed(pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: aux.Point):
    v_max = 100
    a_max = 10
    v = None
    angle = 0
    step = math.pi
    last_dist = None
    while abs(step) > 10e-5:
        v = aux.Point(math.cos(angle + step), math.sin(angle + step)) * v_max
        t1 = (v - v1).mag() / a_max
        t2 = (v2 - v).mag() / a_max
        t = (pos1 + (v1 + v) * t1 / 2 - pos2 + (v2 + v) * t2 / 2).mag() / v_max
        dist = aux.dist(pos1 + (v1 + v) * t1 / 2 + v * t, pos2 - (v2 + v) * t2 / 2)
        angle += step
        step /= math.sqrt(2)
        if last_dist is not None and last_dist < dist:
            step *= -1
        last_dist = dist
    # for alpha in range(1000):
    #     vv = aux.Point(math.cos(math.pi*alpha/500), math.sin(math.pi*alpha/500)) * v_max
    #     a1 = (vv - v1).unity() * a_max
    #     t1 = (vv - v1).mag() / a_max
    #     a2 = (v2 - vv).unity() * a_max
    #     t2 = (v2 - vv).mag() / a_max
    #     t = (pos1 + v1*t1 + a1*t1*t1/2 - pos2 + v2*t2 - a2*t2*t2/2).mag() / v_max
    #     dist = aux.dist(pos1 + v1*t1 + a1*t1*t1/2 + vv*t, pos2 - v2*t2 + a2*t2*t2/2)
    #     if dist_min is None or dist_min > dist:
    #         dist_min = dist
    #         v = vv

    # v = aux.Point(math.cos(time.time()), math.sin(time.time())) * v_max
    print(last_dist)
    a1 = (v - v1).unity() * a_max
    t1 = (v - v1).mag() / a_max
    a2 = (v2 - v).unity() * a_max
    t2 = (v2 - v).mag() / a_max
    t = (pos2 - (v2 + v) * t2 / 2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max
    print("path time:", t1 + t2 + t)
    dt = 0.04
    for tt in numpy.arange(0, t1, dt):
        dot_pos = pos1 + v1 * tt + a1 * tt**2 / 2
        draw_dot(dot_pos)
    extra_point = pos1 + (v1 + v) * t1 / 2
    for tt in numpy.arange(0, t, dt):
        dot_pos = extra_point + v * tt
        draw_dot(dot_pos, 3, (0, 0, 255))
    for tt in numpy.arange(-t2, 0, dt):
        dot_pos = pos2 + v2 * tt + a2 * tt**2 / 2
        draw_dot(dot_pos)


def proj_without_end_speed(pos1: aux.Point, v1: aux.Point, pos2: aux.Point):
    v_max = 100
    a_max = 10
    v = None
    angle = 0
    step = math.pi
    last_dist = None
    while abs(step) > 10e-5:
        v = aux.Point(math.cos(angle + step), math.sin(angle + step)) * v_max
        t1 = (v - v1).mag() / a_max
        t = (pos1 + (v1 + v) * t1 / 2 - pos2).mag() / v_max
        dist = aux.dist(pos1 + (v1 + v) * t1 / 2 + v * t, pos2)
        angle += step
        step /= math.sqrt(2)
        if last_dist is not None and last_dist < dist:
            step *= -1
        last_dist = dist

    a1 = (v - v1).unity() * a_max
    t1 = (v - v1).mag() / a_max
    t = (pos2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max
    print("path time:", t1 + t)
    dt = 0.04
    for tt in numpy.arange(0, t1, dt):
        dot_pos = pos1 + v1 * tt + a1 * tt**2 / 2
        draw_dot(dot_pos)
    extra_point = pos1 + (v1 + v) * t1 / 2
    for tt in numpy.arange(0, t, dt):
        dot_pos = extra_point + v * tt
        draw_dot(dot_pos, 3, (0, 0, 255))


def update_window():
    global screen
    if "screen" not in globals():
        screen, _ = initialize()
        draw_field()

    pygame.display.flip()
    draw_field()
