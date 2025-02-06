import math
from typing import Optional
import numpy as np

import auxiliary as aux
import const

from drawing import Image
from cells_tools import Cell, Peak

goal_hull_ = [
    aux.Point(const.FIELD_WIDTH, const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2 - const.GOAL_PEN_DX, const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2 - const.GOAL_PEN_DX, -const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH, -const.GOAL_PEN_DY),
]

goal_up = aux.Point(const.FIELD_WIDTH // 2, const.GOAL_SIZE / 2)
goal_center = aux.Point(const.FIELD_WIDTH // 2, 0)
goal_down = aux.Point(const.FIELD_WIDTH // 2, -const.GOAL_SIZE / 2)

GOAL_HULL_DIST = 400

OBSTACLE_ANGLE = math.pi / 5
GOAL_VIEW_ANGLE = math.pi / 4
DANGER_ZONE_DIST = GOAL_HULL_DIST
SHOOT_ANGLE = math.pi / 8


def estimate_pass_point(
        enemies: list[aux.Point],
        frm: aux.Point,
        to: aux.Point,
) -> float:
    """
    Оценивает пас из точки "frm" в точку "to", возвращая положительное значение до 1
    """
    lerp: float = 0.0

    for enemy in enemies:
        frm_enemy = aux.dist(frm, enemy)
        if frm_enemy > const.ROBOT_R:
            if frm_enemy <= aux.dist(frm, to):
                tgs = aux.get_tangent_points(enemy, frm, const.ROBOT_R)
                if len(tgs) < 2:
                    continue

                ang1 = aux.get_angle_between_points(to, frm, tgs[0])
                ang2 = aux.get_angle_between_points(to, frm, tgs[1])

                ang = min(abs(ang1), abs(ang2))
                if (
                        ang1 * ang2 < 0
                        and abs(ang1) < math.pi / 2
                        and abs(ang2) < math.pi / 2
                ):
                    ang *= -1  # enemy between to and frm
            else:  # circle around enemy
                enemy_to = aux.dist(enemy, to)

                enemy_angle = math.asin(const.ROBOT_R / frm_enemy)
                to_enemy_angle = 2 * math.asin((enemy_to / 2) / frm_enemy)
                ang = to_enemy_angle - enemy_angle

            if ang < OBSTACLE_ANGLE:
                delta_lerp = abs((OBSTACLE_ANGLE - ang) / OBSTACLE_ANGLE) ** 1.5

                lerp += delta_lerp

    return lerp  # 0 - perfect; bigger => worse


def estimate_goal_view(point: aux.Point) -> float:
    goal_angle = abs(aux.get_angle_between_points(goal_up, point, goal_down))

    return min(goal_angle / GOAL_VIEW_ANGLE, 1)  # 1 - perfect; smaller => worse


def estimate_dist_to_boarder(point: aux.Point) -> float:
    """Gives a score for a point based on its distance to the non-playing area"""
    dist_to_goal_zone = aux.dist(point, aux.nearest_point_on_poly(point, goal_hull_))
    if aux.is_point_inside_poly(point, goal_hull_):
        return 10

    return max(3 - dist_to_goal_zone / DANGER_ZONE_DIST, 0)


def estimate_shoot(point: aux.Point, enemies: list[aux.Point]) -> float:
    lerp: float = 0.0

    for enemy in enemies:
        frm_enemy = aux.dist(point, enemy)
        if frm_enemy > const.ROBOT_R:

            ang1 = aux.get_angle_between_points(goal_down, point, enemy)
            ang2 = aux.get_angle_between_points(goal_up, point, enemy)

            ang = min(abs(ang1), abs(ang2))
            if ang1 * ang2 < 0 and abs(ang1) < math.pi / 2 and abs(ang2) < math.pi / 2:
                ang *= -1  # enemy between to and frm

            if ang < SHOOT_ANGLE:
                delta_lerp = abs((SHOOT_ANGLE - ang) / SHOOT_ANGLE) ** 1.5

                lerp += delta_lerp

    return lerp  # 0 - perfect; bigger => worse


def estimate_dist_to_enemy(point: aux.Point, enemies: list[aux.Point]) -> float:
    """Gives a score for a point based on its distance to the non-playing area"""
    min_ = 10e9
    for enemy in enemies:
        if aux.dist(point, enemy) < min_:
            min_ = aux.dist(point, enemy)

    return max(1 - min_ / 1500, 0)


def estimate_point(
        point: aux.Point, kick_point: aux.Point, enemies: list[aux.Point]
) -> float:
    lerp1 = estimate_pass_point(enemies, kick_point, point)
    lerp2 = estimate_goal_view(point)
    lerp3 = estimate_dist_to_boarder(point)
    lerp4 = estimate_shoot(point, enemies)
    lerp5 = estimate_dist_to_enemy(point, enemies)

    lerp = lerp2 - lerp1 - lerp3 - lerp4 - lerp5  # lerp2 - lerp1 - lerp3 - lerp4
    return lerp  # 1 - perfect; smaller => worse


def draw_heat_map(
        screen: Image, kick_point: aux.Point, enemies: list[aux.Point] = []
) -> list[aux.Point]:
    scale_x = const.FIELD_WIDTH // 2 / const.SCREEN_WIDTH
    scale_y = const.FIELD_HEIGH / const.SCREEN_HEIGH

    dots_value = np.zeros((const.SCREEN_WIDTH, const.SCREEN_HEIGH))

    for pixel_x in range(const.SCREEN_WIDTH):
        for pixel_y in range(const.SCREEN_HEIGH):
            point = aux.Point(
                pixel_x * scale_x,
                -(pixel_y - const.SCREEN_HEIGH / 2) * scale_y,
            )  # to cord on the field
            lerp = aux.minmax(estimate_point(point, kick_point, enemies), -2, 1)
            t = (lerp + 2) / 3
            red = int(255 * (1 - t) ** 2)
            green = int(255 * t ** 2)
            blue = 0
            yellow_factor = 4 * t * (1 - t)
            red = min(255, red + int(255 * yellow_factor))
            green = min(255, green + int(255 * yellow_factor))
            color = (red, green, blue)
            # if lerp > 0.5:
            #     color = (int(255 * 2 * (1 - lerp)), 255, 0)
            # elif lerp > 0:
            #     color = (255, int(255 * 2 * lerp), 0)
            # else:
            #     color = (int(255 * (1 + lerp / 2)), 0, 0)
            screen.draw_pixel(
                (pixel_x, pixel_y),
                color,
            )
            dots_value[pixel_x][pixel_y] = lerp
        print(f"{pixel_x / const.SCREEN_WIDTH * 100:.1f} %")
        screen.update_window()

    # max_ = -10e9
    # p_x = 0
    # p_y = 0
    # for pixel_x in range(const.SCREEN_WIDTH):
    #     for pixel_y in range(const.SCREEN_HEIGH):
    #         if dots_value[pixel_x][pixel_y] > max_:
    #             max_ = dots_value[pixel_x][pixel_y]
    #             p_x = pixel_x
    #             p_y = pixel_y
    #
    # point = aux.Point(
    #     p_x * scale_x,
    #     -(p_y - const.SCREEN_HEIGH / 2) * scale_y,
    # )
    return find_k_maximums(dots_value, 1, scale_x, scale_y)

    # # find local maxima
    # maxs = find_local_maxima(dots_value)
    #
    # for max_pos in maxs:
    #     screen.draw_pixel(max_pos, (255, 0, 255))


def find_k_maximums(dots_value, k, scale_x, scale_y, min_distance=500):
    points = []

    # Создаем список всех точек с их значениями
    candidates = []
    for pixel_x in range(const.SCREEN_WIDTH):
        for pixel_y in range(const.SCREEN_HEIGH):
            candidates.append((dots_value[pixel_x][pixel_y], pixel_x, pixel_y))

    # Сортируем по убыванию значений
    candidates.sort(reverse=True, key=lambda x: x[0])

    for value, p_x, p_y in candidates:
        point = aux.Point(
            p_x * scale_x,
            -(p_y - const.SCREEN_HEIGH / 2) * scale_y,
        )

        # Проверяем расстояние до уже добавленных точек
        if all(aux.dist(point, p) > min_distance for p in points):
            points.append(point)
            if len(points) == k:
                break

    return points


def get_cells(kick_point: aux.Point, enemies: list[aux.Point] = []) -> list[Cell]:
    left_top_cell = Peak(aux.Point(0, const.FIELD_HEIGH / 2))
    right_top_cell = Peak(aux.Point(const.FIELD_WIDTH / 2, const.FIELD_HEIGH / 2))
    right_down_cell = Peak(aux.Point(const.FIELD_WIDTH / 2, -const.FIELD_HEIGH / 2))
    left_down_cell = Peak(aux.Point(0, -const.FIELD_HEIGH / 2))

    cells = [Cell([left_top_cell, right_top_cell, right_down_cell, left_down_cell])]

    for enemy in enemies:
        new_cells = []
        for cell in cells:
            new_cell = cell.intersect_cell(enemy, goal_center)
            if new_cell:
                new_cells.append(new_cell)
        cells += new_cells

        new_cells = []
        cells_to_delete: list[int] = []

        for idx, cell in enumerate(cells):
            vec = (enemy - kick_point).unity()

            tangents = aux.get_tangent_points(enemy, kick_point, const.ROBOT_R)
            if len(tangents) < 2:
                continue

            side = aux.sign(
                aux.vec_mult((tangents[0] - kick_point), (enemy - kick_point))
            )

            new_cell = cell.intersect_cell(enemy - vec, enemy, "R")
            if new_cell:
                is_cropped = cell.crop_cell(kick_point, tangents[0], side, "R")
                if not is_cropped:
                    is_cropped = cell.crop_cell(kick_point, tangents[1], -side, "R")
                    if not is_cropped:
                        cells_to_delete = [idx] + cells_to_delete
                        # порядок важен, т к при удалении меняются индексы

                is_cropped = new_cell.crop_cell(kick_point, tangents[0], side, "R")
                if not is_cropped:
                    is_cropped = new_cell.crop_cell(kick_point, tangents[1], -side, "R")

                if is_cropped:
                    new_cells.append(new_cell)
            else:
                vec0 = tangents[0] - kick_point
                cell.crop_cell(tangents[0] - vec0, tangents[0], side, "R")
                vec1 = tangents[1] - kick_point
                cell.crop_cell(tangents[1] - vec1, tangents[1], -side, "R")

        for idx in cells_to_delete:  # NOTE
            cells.pop(idx)

        cells += new_cells

    return cells


def find_local_maxima(mass: list[list[float]] | np.ndarray) -> list[tuple[int, int]]:
    maxima: list[tuple[int, int]] = []
    for x, column in enumerate(mass):
        for y, value in enumerate(column):
            neighbors: list[float] = []
            if x != 0:
                neighbors += [mass[x - 1][y]]
                if y != 0:
                    neighbors += [mass[x - 1][y - 1]]
                if y != len(column) - 1:
                    neighbors += [mass[x - 1][y + 1]]
            if x != len(mass) - 1:
                neighbors += [mass[x + 1][y]]
                if y != 0:
                    neighbors += [mass[x + 1][y - 1]]
                if y != len(column) - 1:
                    neighbors += [mass[x + 1][y + 1]]
            if y != 0:
                neighbors += [column[y - 1]]
            if y != len(column) - 1:
                neighbors += [column[y + 1]]

            is_maximum = True
            for neighbor in neighbors:
                if value < neighbor or value == 0:
                    is_maximum = False

            if is_maximum:
                maxima.append((x, y))

    return maxima
