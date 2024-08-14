import math
from typing import Optional

import auxiliary as aux
import const

from drawing import Image
from cells_tools import Cell

goal_hull_ = [
    aux.Point(const.FIELD_WIDTH // 2, const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2 - const.GOAL_PEN_DX, const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2 - const.GOAL_PEN_DX, -const.GOAL_PEN_DY),
    aux.Point(const.FIELD_WIDTH // 2, -const.GOAL_PEN_DY),
]

goal_up = aux.Point(const.FIELD_WIDTH // 2, const.GOAL_SIZE / 2)
goal_center = aux.Point(const.FIELD_WIDTH // 2, 0)
goal_down = aux.Point(const.FIELD_WIDTH // 2, -const.GOAL_SIZE / 2)


def estimate_pass_point(
    enemies: list[aux.Point],
    frm: Optional[aux.Point],
    to: Optional[aux.Point],
) -> float:
    """
    Оценивает пас из точки "frm" в точку "to, возвращая положительное значение до 1
    """
    if len(enemies) == 0:
        return 1

    if frm is None or to is None:
        return 0
    positions: list[tuple[int, aux.Point]] = []

    poses = enemies
    for idx, rbt in enumerate(poses):
        positions.append((idx, rbt))
    positions = sorted(positions, key=lambda x: x[1].y)

    tangents: list[tuple[int, list[aux.Point]]] = []
    for p in positions:
        if aux.dist(frm, p[1]) < aux.dist(frm, to) + const.ROBOT_R:
            tgs = aux.get_tangent_points(p[1], frm, const.ROBOT_R)
            if tgs is None or len(tgs) < 2:
                continue
            tangents.append((p[0], tgs))

    min_ = 10e3

    shadows_bots = []
    for tangent in tangents:
        ang1 = aux.get_angle_between_points(to, frm, tangent[1][0])
        ang2 = aux.get_angle_between_points(to, frm, tangent[1][1])

        if ang1 * ang2 < 0 and abs(ang1) < math.pi / 2 and abs(ang2) < math.pi / 2:
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
        not_ang = abs(aux.dist(pos[1], to) - const.ROBOT_R)
        if not_ang < dist:
            psevdo_ang = abs(math.asin(not_ang / dist))

            if psevdo_ang < min_:
                min_ = psevdo_ang

    if len(shadows_bots) > 0:
        return 0
    if min_ == 10e3:
        return 1

    dist = (frm - to).mag() / 1000
    # max_ang = abs(aux.wind_down_angle(2 * math.atan2(const.ROBOT_SPEED, -0.25 * dist + 4.5)))
    # max_ang = 10
    return min(abs(min_ / (math.pi / 10)), 1)


def estimate_point(
    point: aux.Point, kick_point: aux.Point, enemies: list[aux.Point]
) -> float:
    if aux.is_point_inside_poly(point, goal_hull_):
        return 0
    angle_up = aux.angle_to_point(point, goal_up)
    angle_down = aux.angle_to_point(point, goal_down)
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
    dist_to_goal_zone = aux.dist(point, aux.nearest_point_on_poly(point, goal_hull_))

    lerp1 = 1 - estimate_pass_point(enemies, kick_point, point)
    lerp2 = aux.minmax(angle / math.pi * 4, 1, 0)
    lerp3 = 1 - min(dist_to_goal_zone / 200, 1)

    lerp = aux.minmax(lerp2 - lerp1 - lerp3, 0, 1)
    return lerp


def draw_heat_map(
    screen: Image, kick_point: aux.Point, enemies: list[aux.Point] = []
) -> None:
    scale_x = const.FIELD_WIDTH // 2 / const.HEAT_MAP_WIDTH
    scale_y = const.FIELD_HEIGH / const.HEAT_MAP_HEIGH

    for pixel_x in range(const.SCREEN_WIDTH):
        cord_x = int(pixel_x / const.SCREEN_WIDTH * const.HEAT_MAP_WIDTH)
        for pixel_y in range(const.SCREEN_HEIGH):
            cord_y = int(pixel_y / const.SCREEN_HEIGH * const.HEAT_MAP_HEIGH)
            point = aux.Point(
                cord_x * scale_x,
                -(cord_y - const.HEAT_MAP_HEIGH / 2) * scale_y,
            )  # to cord on the field
            lerp = estimate_point(point, kick_point, enemies)
            red = round(min(1, 2 - lerp * 2) * 255)
            green = round(min(1, lerp * 2) * 255)
            color = (red, green, 0)
            screen.draw_pixel(
                (pixel_x, pixel_y),
                color,
            )
        print(f"{pixel_x / const.SCREEN_WIDTH * 100:.1f} %")
        screen.update_window()


# def get_cells(kick_point: aux.Point, enemies: list[aux.Point] = []) -> list[Cell]:
#     cells = [
#         Cell(
#             [
#                 aux.Point(0, const.FIELD_HEIGH / 2),
#                 aux.Point(const.FIELD_WIDTH / 2, const.FIELD_HEIGH / 2),
#                 aux.Point(const.FIELD_WIDTH / 2, -const.FIELD_HEIGH / 2),
#                 aux.Point(0, -const.FIELD_HEIGH / 2),
#             ]
#         )
#     ]

#     for enemy in enemies:
#         new_cells = []
#         for cell in cells:
#             new_cell = cell.intersect_cell(enemy, goal_center)
#             if new_cell:
#                 new_cells.append(new_cell)

#         for cell in cells:
#             new_cell = cell.intersect_cell(enemy, kick_point)
#             if new_cell:
#                 new_cells.append(new_cell)


#     return cells
