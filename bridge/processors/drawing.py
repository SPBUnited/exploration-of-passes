"""
draw field with robots and trajectory
"""

import math
import typing
import matplotlib.pyplot as plt

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

    def draw_bang_bang_traj(
        self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: typing.Optional[aux.Point] = None
    ) -> None:
        """
        choose func for drawing bang bang trajectory
        """
        self.trapez(pos1, v1, pos2, v2)
        # self.triang(pos1, v1, pos2, v2)

    def triang(self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: typing.Optional[aux.Point]) -> None:
        """
        draw full traj with speed v2 in end point if max speed is reached
        """
        if v1 == v2:
            self.triang1(pos1, v1, pos2)
        else:
            if v2 is None:
                v2 = aux.Point(0, 0)  #TODO: add "v2=None" case
            # self.triang_dist(1.01, pos2 - pos1, v1, v2)
            self.triang2(pos1, v1, pos2, v2)

    def triang1(self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point) -> None:
        """
        case with same start and end speeds

        solving system:
          a*t=v-v1
          (v+v1)*t=r
        v - unknown intermediate speed
        t - time to reach v from v1
        TODO: solve it strong (using projection v and v1 on (pos2-pos1))
        """
        step = v_max / 4
        v = (pos2 - pos1).unity() * (v_max / 2) - v1
        while abs(step) > 10e-5:
            t = (v - v1).mag() / a_max
            dist = (pos2 - pos1).mag() - ((v + v1) * t).mag()
            if dist > 0:
                v += (pos2 - pos1).unity() * step
            else:
                v -= (pos2 - pos1).unity() * step
            step /= 2

        if v.mag() > v_max:
            self.trapez(pos1, v1, pos2, v1)
            return

        a1 = (v - v1).unity() * a_max
        t = (v - v1).mag() / a_max
        a2 = -a1
        print("path time:", 2 * t)

        dt = 0.04
        for tt in numpy.arange(0, t, dt):
            dot_pos = pos1 + v1 * tt + a1 * tt**2 / 2
            self.draw_dot(dot_pos)
        for tt in numpy.arange(-t, 0, dt):
            dot_pos = pos2 + v1 * tt + a2 * tt**2 / 2
            self.draw_dot(dot_pos)

    def triang2(self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: aux.Point) -> None:
        """
        case with different start and end speeds

        TODO: написать доказательство 
        """
        dist_min: typing.Optional[float] = None
        vv_near:aux.Point
        n: int = 40 #n%2==0
        for i in range(n):
            vv = (v1 - v2) * (i + 0.5) / n
            k = vv.mag() / (v1 - v2 - vv).mag()
            dist, _ = self.triang_dist(k, pos2 - pos1, v1, v2)
            if dist_min is None or dist < dist_min:
                dist_min = dist
                vv_near = vv

        vv_min = vv_near - (v1-v2) /n
        vv_max = vv_near + (v1-v2) /n
        v: aux.Point
        while (vv_max - vv_min).mag() > 10e-6:
            vv1 = (vv_max - vv_min) * 1 / 3 + vv_min
            k1 = vv1.mag() / (v1 - v2 - vv1).mag()
            vv2 = (vv_max - vv_min) * 2 / 3 + vv_min
            k2 = vv2.mag() / (v1 - v2 - vv1).mag()
            dist1, _ = self.triang_dist(k1, pos2 - pos1, v1, v2)
            dist2, v = self.triang_dist(k2, pos2 - pos1, v1, v2)
            if dist1 < dist2:
                vv_max = vv2
                dist_min = dist1
            else:
                vv_min = vv1
                dist_min = dist2

        print(f"dist_min: {dist_min}")

        mass: list[float] = []
        for i in range(6000, 9999):
            a= self.triang_dist(10000 / i, pos2 - pos1, v1, v2)[0]
            mass.append(a)
        print("=======================================")
        plt.plot(mass)
        plt.show()

        if dist_min > 10 or v.mag() > v_max:
            print(pos1, v1, pos2, v2)
            # _ = 1/0

        # v = aux.Point(50, -50)

        a1 = (v - v1).unity() * a_max
        t1 = (v - v1).mag() / a_max
        a2 = (v2 - v).unity() * a_max
        t2 = (v2 - v).mag() / a_max

        dist = aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, pos2-pos1)
        print(dist, dist)

        dt = 0.1
        for tt in numpy.arange(0, t1, dt):
            dot_pos = pos1 + v1 * tt + a1 * tt**2 / 2
            self.draw_dot(dot_pos)
        for tt in numpy.arange(-t2, 0, dt):
            dot_pos = pos2 + v2 * tt + a2 * tt**2 / 2
            self.draw_dot(dot_pos)

    def triang_dist(self, k: float, delta_r: aux.Point, v1: aux.Point, v2: aux.Point) -> tuple[float, aux.Point]:
        """
        Find dist for triang case
        k = t1/t2 (k != 1)
        """
        angle_near:float
        last_dist = None
        n: int = 20
        for i in range(n):
            angle = math.pi * 2 * i / n
            R = aux.Point(math.cos(angle), math.sin(angle)) * k / (k**2 - 1) * (v1 - v2).mag()
            vv = v2 + (v1 - v2) * k / (k + 1)
            v = vv + (v1 - v2).unity() * R.mag() * (k - 1) / abs(k - 1) + R
            t1 = (v - v1).mag() / a_max
            t2 = (v2 - v).mag() / a_max
            dist = aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_r)
            if last_dist is None or last_dist > dist:
                last_dist = dist
                angle_near = angle

        angle_min = angle_near - math.pi * 2 / n
        angle_max = angle_near + math.pi * 2 / n
        v: aux.Point
        while angle_max - angle_min > 10e-5:
            vv = v2 + (v1 - v2) * k / (k + 1)

            angle1 = (angle_max-angle_min)* 1 / 3 + angle_min
            R = aux.Point(math.cos(angle1), math.sin(angle1)) * k / (k**2 - 1) * (v1 - v2).mag()
            v = vv + (v1 - v2).unity() * R.mag() * (k - 1) / abs(k - 1) + R
            t1 = (v - v1).mag() / a_max
            t2 = (v2 - v).mag() / a_max
            dist1 = aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_r)

            angle2 = (angle_max-angle_min)* 2 / 3 + angle_min
            R = aux.Point(math.cos(angle2), math.sin(angle2)) * k / (k**2 - 1) * (v1 - v2).mag()
            v = vv + (v1 - v2).unity() * R.mag() * (k - 1) / abs(k - 1) + R
            t1 = (v - v1).mag() / a_max
            t2 = (v2 - v).mag() / a_max
            dist2 = aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_r)

            if dist1 < dist2:
                angle_max = angle2
                last_dist = dist1
            else:
                angle_min = angle1
                last_dist = dist2

        # print(last_dist, angle_min)

        # mass: list[float] = []
        # for i in range(1000, 1020):
        #     angle1 = math.pi * 2 / 1000 * i
        #     R = aux.Point(math.cos(angle1), math.sin(angle1)) * k / (k**2 - 1) * (v1 - v2).mag()
        #     v = vv + (v1 - v2).unity() * R.mag() + R
        #     t1 = (v - v1).mag() / a_max
        #     t2 = (v2 - v).mag() / a_max
        #     dist1 = aux.dist((v1 + v) * t1 / 2 + (v2 + v) * t2 / 2, delta_r)
        #     mass.append(dist1)
        # print("=======================================")
        # mass.append(last_dist)
        # plt.plot(mass)
        # plt.show()


        return last_dist, v

    def trapez(self, pos1: aux.Point, v1: aux.Point, pos2: aux.Point, v2: typing.Optional[aux.Point] = None) -> None:
        """
        draw full trajectory with speed v2 in end point if max speed isn't achieved
        """
        angle = 0.0
        step = math.pi
        last_dist = None
        while abs(step) > 10e-7:
            v = aux.Point(math.cos(angle + step), math.sin(angle + step)) * v_max
            t1 = (v - v1).mag() / a_max
            if v2 is not None:
                t2 = (v2 - v).mag() / a_max
                t = (pos1 + (v1 + v) * t1 / 2 - pos2 + (v2 + v) * t2 / 2).mag() / v_max
                dist = aux.dist(pos1 + (v1 + v) * t1 / 2 + v * t, pos2 - (v2 + v) * t2 / 2)
            else:
                t = (pos1 + (v1 + v) * t1 / 2 - pos2).mag() / v_max
                dist = aux.dist(pos1 + (v1 + v) * t1 / 2 + v * t, pos2)
            if last_dist is not None and last_dist < dist:
                step *= -1
            angle += step
            step /= math.sqrt(2)
            last_dist = dist

        if last_dist > 1:
            print(f"dist for trapez: {last_dist}")
            self.triang(pos1, v1, pos2, v2)
            return

        # v = aux.Point(math.cos(time.time()), math.sin(time.time())) * v_max
        print(last_dist)
        a1 = (v - v1).unity() * a_max
        t1 = (v - v1).mag() / a_max
        if v2 is not None:
            a2 = (v2 - v).unity() * a_max
            t2 = (v2 - v).mag() / a_max
            t = (pos2 - (v2 + v) * t2 / 2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max
            print("path time:", t1 + t2 + t)
        else:
            t = (pos2 - pos1 - (v1 + v) * t1 / 2).mag() / v_max
            print("path time:", t1 + t)
        dt = 0.04
        for tt in numpy.arange(0, t1, dt):
            dot_pos = pos1 + v1 * tt + a1 * tt**2 / 2
            self.draw_dot(dot_pos)
        extra_point = pos1 + (v1 + v) * t1 / 2
        for tt in numpy.arange(0, t, dt):
            dot_pos = extra_point + v * tt
            self.draw_dot(dot_pos, 3, (0, 0, 255))
        if v2 is not None:
            for tt in numpy.arange(-t2, 0, dt):
                dot_pos = pos2 + v2 * tt + a2 * tt**2 / 2
                self.draw_dot(dot_pos)

    def update_window(self) -> None:
        """
        update image
        """
        pygame.display.flip()
        self.draw_field()
