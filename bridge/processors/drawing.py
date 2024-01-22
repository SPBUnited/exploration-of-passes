"""
draw field with robots and trajectory
"""

import math
from typing import Optional

import numpy
import pygame

import bridge.processors.auxiliary as aux
import bridge.processors.const as const

v_max = 100
a_max = 10


class Image:
    """
    class with image's specs
    """

    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600), pygame.RESIZABLE)
        pygame.display.set_caption("Football Field")

        goal_dx, goal_dy = abs(const.GOAL_DX), abs(const.GOAL_DY)
        self.scale = max(400 / goal_dx, 300 / goal_dy)
        self.middle_x, self.middle_y = self.screen.get_size()
        self.middle_x = round(self.middle_x / 2)
        self.middle_y = round(self.middle_y / 2)
        self.upper_border = self.middle_y - goal_dy * self.scale
        self.lower_border = self.middle_y + goal_dy * self.scale
        self.left_border = self.middle_x - goal_dx * self.scale
        self.right_border = self.middle_x + goal_dx * self.scale
        self.size_x = goal_dx * self.scale * 2
        self.size_y = goal_dy * self.scale * 2

    def draw_field(self) -> None:
        """
        draw green field and white lines
        """
        back_color = (128, 128, 128)
        field_color = (0, 192, 0)
        line_color = (255, 255, 255)
        self.screen.fill(back_color)
        pygame.draw.rect(self.screen, field_color, (self.left_border, self.upper_border, self.size_x, self.size_y))  # Поле
        pygame.draw.rect(
            self.screen, line_color, (self.left_border, self.upper_border, self.size_x, self.size_y), 2
        )  # Обводка поля
        pygame.draw.line(
            self.screen, line_color, (self.left_border, self.middle_y), (self.right_border, self.middle_y), 2
        )  # Горизонтальная линия
        pygame.draw.line(
            self.screen, line_color, (self.middle_x, self.upper_border), (self.middle_x, self.lower_border), 2
        )  # Вертикальная линия
        pygame.draw.circle(self.screen, line_color, (self.middle_x, self.middle_y), 50, 2)  # Круг в центре

    def draw_robot(self, r: aux.Point, angle: float = 0.0) -> None:
        """
        draw robot
        """
        robot_color = (0, 0, 255)
        robot_radius = 20
        robot_length = 40
        end_point = (
            int(r.x + robot_length * math.cos(math.radians(angle))),
            int(r.y - robot_length * math.sin(math.radians(angle))),
        )
        pygame.draw.circle(self.screen, robot_color, r.xy(), robot_radius)
        pygame.draw.line(self.screen, robot_color, r.xy(), end_point, 2)

    def draw_dot(self, pos: aux.Point, size: float = 3, color: tuple[int, int, int] = (255, 0, 0)) -> None:
        """
        draw single point
        """
        pygame.draw.circle(
            self.screen, color, (pos.x * self.scale + self.middle_x, -pos.y * self.scale + self.middle_y), size
        )

    def update_window(self) -> None:

        """
        update image
        """
        pygame.display.flip()
        self.draw_field()

    def draw_bang_bang_traj(self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: Optional[aux.Point] = None) -> None:
        v = trapeze(pos2 - pos1, v1, v2)
        # _, v = triang_dist(pos2 - pos1, 300, v1, v2)
        dt = 0.1
        a1 = (v - v1).unity() * a_max
        t1 = (v - v1).mag() / a_max

        for tt in numpy.arange(0, t1, dt):
            dot_pos = pos1 + v1 * tt + a1 * tt**2 / 2
            self.draw_dot(dot_pos)
        extra_point = pos1 + (v1 + v) * t1 / 2

        if v_max - v.mag() < 10e-3:
            if v2 is None:
                t = (pos2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max

                for tt in numpy.arange(0, t, dt):
                    dot_pos = extra_point + v * tt
                    self.draw_dot(dot_pos, 3, (0, 0, 255))

                self.draw_dot(pos2, 3, (255, 0, 0))
                print("Trapeze without end speed; path time: ", t1 + t)
            else:
                a2 = (v2 - v).unity() * a_max
                t2 = (v2 - v).mag() / a_max
                t = (pos2 - (v2 + v) * t2 / 2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max

                for tt in numpy.arange(0, t, dt):
                    dot_pos = extra_point + v * tt
                    self.draw_dot(dot_pos, 3, (0, 0, 255))
                for tt in numpy.arange(-t2, 0, dt):
                    dot_pos = pos2 + v2 * tt + a2 * tt**2 / 2
                    self.draw_dot(dot_pos)

                # self.draw_dot(pos2, 3, (255, 255, 0))
                print("Trapeze with end speed; path time: ", t1 + t + t2)
        else:
            if v2 is None:
                self.draw_dot(pos2, 3, (0, 0, 255))
                print("Triangle without end speed; path time: ", t1)
            else:
                a2 = (v2 - v).unity() * a_max
                t2 = (v2 - v).mag() / a_max

                for tt in numpy.arange(-t2, 0, dt):
                    dot_pos = pos2 + v2 * tt + a2 * tt**2 / 2
                    self.draw_dot(dot_pos)

                print("Triangle with end speed; path time: ", t1 + t2)


def trapeze(delta_pos: aux.Point, v1: aux.Point, v2: Optional[aux.Point]) -> aux.Point:
    """
    draw full trajectory with speed v2 in end point if max speed isn't achieved
    """
    angle_near: float
    last_dist = None
    n: int = 10
    for i in range(n):
        angle = math.pi * 2 * i / n
        v = aux.Point(math.cos(angle), math.sin(angle)) * v_max
        dist = dist_for_v(delta_pos, v, v1, v2)
        if last_dist is None or last_dist > dist:
            last_dist = dist
            angle_near = angle

    angle_min = angle_near - math.pi * 2 / n
    angle_max = angle_near + math.pi * 2 / n
    while angle_max - angle_min > 10e-5:
        angle1 = (angle_max - angle_min) * 1 / 3 + angle_min
        v = aux.Point(math.cos(angle1), math.sin(angle1)) * v_max
        dist1 = dist_for_v(delta_pos, v, v1, v2)

        angle2 = (angle_max - angle_min) * 2 / 3 + angle_min
        v = aux.Point(math.cos(angle2), math.sin(angle2)) * v_max
        dist2 = dist_for_v(delta_pos, v, v1, v2)

        if dist1 < dist2:
            angle_max = angle2
            last_dist = dist1
        else:
            angle_min = angle1
            last_dist = dist2

    if last_dist > 1:
        print(f"dist for trapez: {last_dist}")
        return triang(delta_pos, v1, v2)

    # v = aux.Point(math.cos(time.time()), math.sin(time.time())) * v_max
    print(last_dist)
    return v


def triang(delta_pos: aux.Point, v1: aux.Point, v2: Optional[aux.Point]) -> aux.Point:
    """
    draw full trajectory with speed v2 in end point if max speed is reached
    """
    if v2 is None:  # TODO
        v2 = aux.Point(0, 0)

    dist_min = None
    n: int = 10
    v_near: float
    for i in range(1, n):
        v_mag = v_max * i / n
        dist, _ = triang_dist(delta_pos, v_mag, v1, v2)
        if dist_min is None or dist < dist_min:
            dist_min = dist
            v_near = v_mag

    mag_min = v_near - v_max / n
    mag_max = v_near + v_max / n
    v: aux.Point
    while mag_max - mag_min > 10e-6:
        v_mag1 = (mag_max - mag_min) * 1 / 3 + mag_min
        v_mag2 = (mag_max - mag_min) * 2 / 3 + mag_min
        dist1, _ = triang_dist(delta_pos, v_mag1, v1, v2)
        dist2, v = triang_dist(delta_pos, v_mag2, v1, v2)
        if dist1 < dist2:
            mag_max = v_mag2
            dist_min = dist1
        else:
            mag_min = v_mag1
            dist_min = dist2

    print(f"dist for triangle: {dist_min}")

    # mass: list[float] = []
    # for i in range(1, 1000):
    #     a = triang_dist(delta_pos, i / 1000, v1, v2)[0]
    #     mass.append(a)
    # print("=======================================")
    # plt.plot(mass)
    # plt.show()

    if dist_min > 10 or v.mag() > v_max:
        print("crash: ", delta_pos, v1, v2)
        _ = 1 / 0

    # t1 = (v - v1).mag() / a_max
    # t2 = (v2 - v).mag() / a_max

    # dist = aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_pos)

    return v


def triang_dist(delta_pos: aux.Point, v_mag: float, v1: aux.Point, v2: aux.Point) -> tuple[float, aux.Point]:
    angle_near: float
    last_dist = None
    n: int = 15  # >10
    for i in range(n):
        angle = math.pi * 2 * i / n
        v = aux.Point(math.cos(angle), math.sin(angle)) * v_mag
        dist = dist_for_v(delta_pos, v, v1, v2)
        if last_dist is None or last_dist > dist:
            last_dist = dist
            angle_near = angle

    angle_min = angle_near - math.pi * 2 / n
    angle_max = angle_near + math.pi * 2 / n
    while angle_max - angle_min > 10e-5:
        angle1 = (angle_max - angle_min) * 1 / 3 + angle_min
        v = aux.Point(math.cos(angle1), math.sin(angle1)) * v_mag
        dist1 = dist_for_v(delta_pos, v, v1, v2)

        angle2 = (angle_max - angle_min) * 2 / 3 + angle_min
        v = aux.Point(math.cos(angle2), math.sin(angle2)) * v_mag
        dist2 = dist_for_v(delta_pos, v, v1, v2)

        if dist1 < dist2:
            angle_max = angle2
            last_dist = dist1
        else:
            angle_min = angle1
            last_dist = dist2

    # mass: list[float] = []
    # for i in range(1, 1000):
    #     angle1 = math.pi * 2 * i / 1000
    #     v = aux.Point(math.cos(angle1), math.sin(angle1)) * v_mag
    #     dist1 = dist_for_v(delta_pos, v, v1, v2)
    #     mass.append(dist1)
    # print("=======================================")
    # plt.plot(mass)
    # plt.show()

    return last_dist, v


def dist_for_v(
    delta_pos: aux.Point, v: aux.Point, v1: aux.Point, v2: Optional[aux.Point]
) -> float:  # TODO: add case for trapeze
    if v_max - v.mag() < 10e-3:
        if v2 is None:
            t1 = (v - v1).mag() / a_max
            t = (delta_pos - (v1 + v) * t1 / 2).mag() / v_max
            return aux.dist(delta_pos, (v1 + v) * t1 / 2 + v * t)
        else:
            t1 = (v - v1).mag() / a_max
            t2 = (v2 - v).mag() / a_max
            t = (delta_pos - (v1 + v) * t1 / 2 - (v2 + v) * t2 / 2).mag() / v_max
            return aux.dist(delta_pos, (v1 + v) * t1 / 2 + v * t + (v2 + v) * t2 / 2)
    else:
        t1 = (v - v1).mag() / a_max
        t2 = (v2 - v).mag() / a_max
        return aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_pos)
