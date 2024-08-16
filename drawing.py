"""
draw field with robots and trajectory
"""

import math
import pygame

import auxiliary as aux
import const


goal_hull_ = [
    aux.Point(const.FIELD_WIDTH // 2, const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2 - const.GOAL_PEN_DX, const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2 - const.GOAL_PEN_DX, -const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2, -const.GOAL_PEN_DY),
]


class Image:
    """
    class with image's specs
    """

    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode(
            (const.SCREEN_WIDTH, const.SCREEN_HEIGH), pygame.RESIZABLE
        )
        pygame.display.set_caption("Football Field")
        pygame.font.init()
        self.font = pygame.font.SysFont("Comic Sans MS", 30)

        goal_dx, goal_dy = abs(const.FIELD_WIDTH // 2), abs(const.FIELD_HEIGH // 2)
        self.scale = min(const.SCREEN_WIDTH / goal_dx, const.SCREEN_HEIGH / 2 / goal_dy)
        self.middle_x = 0
        self.middle_y = round(const.SCREEN_HEIGH / 2)
        self.upper_border = self.middle_y - goal_dy * self.scale
        self.lower_border = self.middle_y + goal_dy * self.scale
        self.left_border = self.middle_x - goal_dx * self.scale
        self.right_border = self.middle_x + goal_dx * self.scale
        self.size_x = goal_dx * self.scale * 2
        self.size_y = goal_dy * self.scale * 2

        self.min_angle = math.pi / 18
        self.num_angles = int(
            math.sqrt(0.25 + 2 * (math.pi / self.min_angle)) - 0.5
        )  # self.num_angles < pi / self.min_angle; self.num_angles -> max

    def draw_field(self) -> None:
        """
        draw green field and white lines
        """
        field_color = (20, 178, 10)
        pygame.draw.rect(
            self.screen,
            field_color,
            (self.left_border, self.upper_border, self.size_x, self.size_y),
        )  # Поле
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
                const.SCREEN_WIDTH - goal_depth,
                self.middle_y - self.scale * const.GOAL_SIZE / 2,
                goal_depth,
                self.scale * const.GOAL_SIZE,
            ),
        )  # Goal
        pygame.draw.line(
            self.screen,
            line_color,
            (self.middle_x, self.upper_border),
            (self.middle_x, self.lower_border),
            2,
        )  # Вертикальная линия

        for i, _ in enumerate(goal_hull_):
            self.draw_line(goal_hull_[i - 1], goal_hull_[i], 2, (255, 255, 255))

    def print_text(
        self, text: str, pos: aux.Point, color: tuple[int, int, int] = (255, 255, 255)
    ) -> None:
        """
        print text on screen
        """
        text_surface = self.font.render(text, False, color)
        size = text_surface.get_size()
        cord = (
            pos.x * self.scale + self.middle_x - size[0] / 2,
            -pos.y * self.scale + self.middle_y - size[1] / 2,
        )
        self.screen.blit(text_surface, cord)

    def draw_robot(self, r: aux.Point, angle: float = 0.0) -> None:
        """
        draw robot
        """
        robot_color = (0, 0, 255)
        robot_radius = const.ROBOT_R * self.scale
        eye_length = const.ROBOT_R * self.scale * 1.1
        rx = r.x * self.scale + self.middle_x
        ry = -r.y * self.scale + self.middle_y
        end_point = aux.Point(rx, ry) + aux.rotate(aux.RIGHT, angle) * eye_length
        pygame.draw.circle(self.screen, robot_color, (rx, ry), robot_radius)
        pygame.draw.line(
            self.screen, robot_color, (rx, ry), (end_point.x, end_point.y), 1
        )

    def draw_dot(
        self, pos: aux.Point, size: float = 1, color: tuple[int, int, int] = (255, 0, 0)
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

    def draw_line(
        self,
        point1: aux.Point,
        point2: aux.Point,
        size: int = 3,
        color: tuple[int, int, int] = (255, 0, 0),
    ) -> None:
        """
        draw single point
        """
        p1 = point1 * self.scale + aux.Point(self.middle_x, -self.middle_y)
        p2 = point2 * self.scale + aux.Point(self.middle_x, -self.middle_y)
        pygame.draw.line(self.screen, color, (p1.x, -p1.y), (p2.x, -p2.y), size)

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

    def update_window(self) -> None:
        """
        update image
        """
        pygame.display.flip()
        pygame.event.get()
