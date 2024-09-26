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

OBSTACLE_ANGLE = math.pi / 10
GOAL_VIEW_ANGLE = math.pi / 4
GOAL_HULL_DIST = 200.0


def estimate_pass_point(
    enemies: list[aux.Point],
    frm: aux.Point,
    to: aux.Point,
) -> float:
    """
    Оценивает пас из точки "frm" в точку "to, возвращая положительное значение до 1
    """
    if len(enemies) == 0:
        return 1

    tangents: list[tuple[aux.Point, aux.Point]] = []
    for enemy in enemies:
        if aux.dist(frm, enemy) <= aux.dist(frm, to):
            tgs = aux.get_tangent_points(enemy, frm, const.ROBOT_R)
            if len(tgs) < 2:
                continue
            tangents.append(tgs)

    lerp: float = 0.0

    for tangent in tangents:
        ang1 = aux.get_angle_between_points(to, frm, tangent[0])
        ang2 = aux.get_angle_between_points(to, frm, tangent[1])

        ang = min(abs(ang1), abs(ang2))
        delta_lerp = (ang / OBSTACLE_ANGLE) ** 4 / 2

        if ang1 * ang2 < 0 and abs(ang1) < math.pi / 2 and abs(ang2) < math.pi / 2:
            delta_lerp *= -1  # enemy between to and frm

        lerp += max(0, 1 - delta_lerp)

    for enemy in enemies:
        dist_frm = aux.dist(frm, enemy)
        dist_to_enemy = aux.dist(enemy, to)
        if dist_frm > aux.dist(frm, to) and dist_frm > dist_to_enemy:
            if dist_to_enemy > const.ROBOT_R:
                psevdo_ang = math.asin((dist_to_enemy - const.ROBOT_R) / dist_frm)
            else:
                psevdo_ang = -math.asin((const.ROBOT_R - dist_to_enemy) / dist_frm)
            lerp += max(0, 1 - psevdo_ang / OBSTACLE_ANGLE)

    return lerp  # 0 - perfect; bigger => worse


def estimate_goal_view(point: aux.Point, enemies: list[aux.Point]) -> float:
    angle_center = aux.angle_to_point(point, goal_center)
    angle_up = aux.wind_down_angle(aux.angle_to_point(point, goal_up) - angle_center)
    angle_down = aux.wind_down_angle(
        aux.angle_to_point(point, goal_down) - angle_center
    )
    angle_err = 0.0
    for enemy in enemies:
        tangents = aux.get_tangent_points(enemy, point, const.ROBOT_R)
        if len(tangents) < 2:
            continue
        enemy_angle_up = aux.wind_down_angle(
            aux.angle_to_point(point, tangents[0]) - angle_center
        )
        enemy_angle_down = aux.wind_down_angle(
            aux.angle_to_point(point, tangents[1]) - angle_center
        )
        if (
            enemy_angle_down < angle_down < enemy_angle_up
            and enemy_angle_down < angle_up < enemy_angle_up
        ):
            angle_err += angle_up - angle_down
        elif enemy_angle_up > angle_up > enemy_angle_down:
            angle_err += angle_up - enemy_angle_down
        elif enemy_angle_up > angle_down > enemy_angle_down:
            angle_err += enemy_angle_up - angle_down
        elif (
            angle_down < enemy_angle_up < angle_up
            and angle_down < enemy_angle_down < angle_up
        ):
            angle_err += enemy_angle_up - enemy_angle_down

    return min(
        (angle_up - angle_down - angle_err) / GOAL_VIEW_ANGLE, 1
    )  # 1 - perfect; smaller => worse


def estimate_dist_to_goal(point: aux.Point) -> float:
    dist_to_goal_zone = aux.dist(point, aux.nearest_point_on_poly(point, goal_hull_))
    if aux.is_point_inside_poly(point, goal_hull_):
        dist_to_goal_zone *= -1

    return max(
        1 - dist_to_goal_zone / GOAL_HULL_DIST, 0
    )  # 0 - perfect; bigger => worse


def estimate_point(
    point: aux.Point, kick_point: aux.Point, enemies: list[aux.Point]
) -> float:
    lerp1 = estimate_pass_point(enemies, kick_point, point)
    lerp2 = estimate_goal_view(point, enemies)
    lerp3 = estimate_dist_to_goal(point)

    lerp = 1 - lerp1  # lerp2 - lerp1 - lerp3
    return lerp  # 1 - perfect; smaller => worse


def draw_heat_map(
    screen: Image, kick_point: aux.Point, enemies: list[aux.Point] = []
) -> None:
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
            if lerp > 0.5:
                color = (255 * 2 * (1 - lerp), 255, 0)
            elif lerp > 0:
                color = (255, 255 * 2 * lerp, 0)
            else:
                color = (255 * (1 + lerp / 2), 0, 0)
            screen.draw_pixel(
                (pixel_x, pixel_y),
                color,
            )
            dots_value[pixel_x][pixel_y] = lerp
        print(f"{pixel_x / const.SCREEN_WIDTH * 100:.1f} %")
        screen.update_window()

    # # find local maxima
    # maxs = find_local_maxima(dots_value)

    # for max_pos in maxs:
    #     screen.draw_pixel(max_pos, (255, 0, 255))


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
