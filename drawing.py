"""
draw field with robots and trajectory
"""

import math
import time
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import pygame

import auxiliary as aux
import const


class Image:
    """
    class with image's specs
    """

    def __init__(self) -> None:
        pygame.init()
        self.width = 1200
        self.heigh = 900
        self.screen = pygame.display.set_mode(
            (self.width / 2, self.heigh), pygame.RESIZABLE
        )
        pygame.display.set_caption("Football Field")

        goal_dx, goal_dy = abs(const.GOAL_DX), abs(const.GOAL_DX * 0.75)
        self.scale = min(self.width / 2 / goal_dx, self.heigh / 2 / goal_dy)
        self.middle_x = 0
        self.middle_y = round(self.heigh / 2)
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
        # field_color = (20, 178, 10)
        # pygame.draw.rect(
        #     self.screen,
        #     field_color,
        #     (self.left_border, self.upper_border, self.size_x, self.size_y),
        # )  # Поле
        line_color = (255, 255, 255)
        pygame.draw.rect(
            self.screen,
            line_color,
            (self.left_border, self.upper_border, self.size_x, self.size_y),
            2,
        )  # Обводка поля
        goal_depth = 10
        pygame.draw.rect(
            self.screen,
            line_color,
            (
                self.width / 2 - goal_depth,
                self.middle_y - self.scale * const.GOAL_DY / 2,
                goal_depth,
                self.scale * const.GOAL_DY,
            ),
        )  # Goal
        pygame.draw.line(
            self.screen,
            line_color,
            (self.middle_x, self.upper_border),
            (self.middle_x, self.lower_border),
            2,
        )  # Вертикальная линия

        goal_hull = [
            aux.Point(const.GOAL_DX, const.GOAL_PEN),
            aux.Point(const.GOAL_DX - const.GOAL_PEN, const.GOAL_PEN),
            aux.Point(const.GOAL_DX - const.GOAL_PEN, -const.GOAL_PEN),
            aux.Point(const.GOAL_DX, -const.GOAL_PEN),
        ]

        for i, dot in enumerate(goal_hull):
            goal_hull[i] = -dot * self.scale + aux.Point(self.middle_x, self.middle_y)

        for i, _ in enumerate(goal_hull):
            pygame.draw.line(
                self.screen,
                (255, 255, 255),
                (goal_hull[i - 1].x, goal_hull[i - 1].y),
                (goal_hull[i].x, goal_hull[i].y),
                2,
            )

    def draw_robot(self, r: aux.Point, angle: float = 0.0) -> None:
        """
        draw robot
        """
        robot_color = (0, 0, 255)
        robot_radius = 20
        robot_length = 40
        r.x = r.x * self.scale + self.middle_x
        r.y = -r.y * self.scale + self.middle_y
        end_point = (
            int(r.x + robot_length * math.cos(math.radians(angle))),
            int(r.y - robot_length * math.sin(math.radians(angle))),
        )
        pygame.draw.circle(self.screen, robot_color, r.xy(), robot_radius)
        pygame.draw.line(self.screen, robot_color, r.xy(), end_point, 2)

    def draw_dot(
        self, pos: aux.Point, size: float = 3, color: tuple[int, int, int] = (255, 0, 0)
    ) -> None:
        """
        draw single point
        """
        pygame.draw.circle(
            self.screen,
            color,
            (pos.x * self.scale + self.middle_x, -pos.y * self.scale + self.middle_y),
            size,
        )

    def draw_pixel(
        self, pos: tuple[int, int], color: tuple[int, int, int] = (255, 0, 0)
    ) -> None:
        """
        draw single point
        """
        pygame.draw.circle(
            self.screen,
            color,
            pos,
            1,
        )

    def estimate_pass_point(
        self,
        enemies: list[aux.Point],
        frm: Optional[aux.Point],
        to: Optional[aux.Point],
    ) -> float:
        """
        Оценивает пас из точки "frm" в точку "to, возвращая положительное значение до 0.8
        """
        if len(enemies) == 0:
            return 1

        if frm is None or to is None:
            return 0
        positions: list[tuple[int, aux.Point]] = []

        poses = enemies
        for idx, rbt in enumerate(poses):
            positions.append([idx, rbt])
        if len(positions) == 0:
            return 1
        positions = sorted(positions, key=lambda x: x[1].y)

        tangents: list[tuple[int, list[aux.Point]]] = []
        for p in positions:
            if aux.dist(frm, p[1]) < aux.dist(frm, to):
                tgs = aux.get_tangent_points(p[1], frm, const.ROBOT_R)
                if tgs is None or len(tgs) < 2:
                    continue
                tangents.append([p[0], tgs])

        min_ = 10e3

        shadows_bots = []
        for tangent in tangents:
            ang1 = aux.get_angle_between_points(to, frm, tangent[1][0])
            ang2 = aux.get_angle_between_points(to, frm, tangent[1][1])

            if ang1 * ang2 < 0:
                shadows_bots.append(tangent[0])
            ang1 = abs(ang1)
            ang2 = abs(ang2)
            # if ang1 > 180:
            #     ang1 = 360 - ang1
            # if ang2 > 180:
            #     ang2 = 360 - ang2

            if ang1 < min_:
                min_ = ang1
            if ang2 < min_:
                min_ = ang2
        # if minId == -1:
        #     return 0

        for pos in positions:
            dist = aux.dist(frm, pos[1])
            not_ang = aux.dist(pos[1], to) - const.ROBOT_R
            if not_ang < dist:
                psevdo_ang = abs(math.asin(not_ang / dist))

                if psevdo_ang < min_:
                    min_ = psevdo_ang

        if min_ == 10e3 or len(shadows_bots) != 0:
            return 0
        dist = (frm - to).mag() / 1000
        # max_ang = abs(aux.wind_down_angle(2 * math.atan2(const.ROBOT_SPEED, -0.25 * dist + 4.5)))
        # max_ang = 10
        return min(abs(min_ / (math.pi / 10)), 1)

    def draw_heat_map(self, enemies: list[aux.Point] = []) -> None:
        for cord_x in range(self.width // 2):
            for cord_y in range(self.heigh):
                point = aux.Point(
                    cord_x / self.scale,
                    -(cord_y - self.middle_y) / self.scale,
                )
                angle_up = aux.angle_to_point(
                    point, aux.Point(-const.GOAL_DX, const.GOAL_DY / 2)
                )
                angle_down = aux.angle_to_point(
                    point, aux.Point(-const.GOAL_DX, -const.GOAL_DY / 2)
                )
                angle = aux.wind_down_angle(angle_up - angle_down)
                for enemy in enemies:
                    tangents = aux.get_tangent_points(enemy, point, const.ROBOT_R)
                    if tangents is None or len(tangents) == 1:
                        continue
                    enemy_angle_up = aux.angle_to_point(point, tangents[0])
                    enemy_angle_down = aux.angle_to_point(point, tangents[1])
                    if enemy_angle_up > angle_up and enemy_angle_down < angle_up:
                        angle -= angle_up - enemy_angle_down
                    elif enemy_angle_up > angle_down and enemy_angle_down < angle_down:
                        angle -= enemy_angle_up - angle_down
                    elif (
                        enemy_angle_up < angle_up
                        and enemy_angle_up > angle_down
                        and enemy_angle_down < angle_up
                        and enemy_angle_down > angle_down
                    ):
                        angle -= enemy_angle_up - enemy_angle_down
                lerp1 = 1 - self.estimate_pass_point(enemies, aux.Point(0, 0), point)
                lerp2 = aux.minmax(angle / math.pi * 4, 0, 1)
                lerp = aux.minmax(lerp2 - lerp1, 0, 1)
                red = round(min(1, 2 - lerp * 2) * 255)
                green = round(min(1, lerp * 2) * 255)
                color = (red, green, 0)
                self.draw_pixel((cord_x, cord_y), color)
            print(f"{cord_x / self.width * 200:.1f} %")
            pygame.event.get()

        for enemy in enemies:
            self.draw_robot(enemy)

    def update_window(self) -> None:
        """
        update image
        """
        self.draw_field()
        pygame.display.flip()
        pygame.event.get()
        back_color = (128, 128, 128)
        self.screen.fill(back_color)
