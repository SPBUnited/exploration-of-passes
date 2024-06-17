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

v_max = 400
a_max = 50


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

                lerp = aux.minmax(angle / math.pi * 4, 0, 1)
                red = round(min(1, 2 - lerp * 2) * 255)
                green = round(min(1, lerp * 2) * 255)
                color = (red, green, 0)
                self.draw_pixel((cord_x, cord_y), color)
            print(f"{cord_x / self.width * 200:.1f} %")

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
