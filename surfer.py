import math
from typing import Optional, Callable


import auxiliary as aux
import const
from drawing import Image
from field import Field

scale = min(
    const.SCREEN_WIDTH / const.FIELD_WIDTH * 2, const.SCREEN_HEIGH / const.FIELD_HEIGH
)

READ_DELTA = 20.0  # in mm
NUM_DOTS_TO_ESTIMATION = 8

MAX_ITERATIONS = int(1000)

GAIN = 20

# def cord_to_pxl(point: aux.Point) -> Optional[tuple[int, int]]:
#     if abs(point.x) > const.FIELD_WIDTH // 2 or abs(point.y) > const.FIELD_HEIGH // 2:
#         return None
#     pxl_x = int(point.x * scale)
#     pxl_y = int(-point.y * scale + const.SCREEN_HEIGH / 2)
#     return (pxl_x, pxl_y)

# def pxl_to_cord(pxl: tuple[int, int]) ->Optional[aux.Point]:
#     if not (0 <= pxl[0] < const.SCREEN_WIDTH and 0 <= pxl[1] < const.SCREEN_HEIGH):
#         return None
#     point_x = pxl[0] / scale
#     point_y = (pxl[1] - const.SCREEN_HEIGH / 2) / scale
#     return aux.Point(point_x, point_y)


def find_local_minimum(
    screen: Image,
    field: Field,
    estimate_function: Callable[..., float],
    start: aux.Point,
) -> aux.Point:
    last_point: aux.Point = start
    last_point_est = estimate_function(field, last_point)

    for _ in range(MAX_ITERATIONS):
        # estimation_neighbors: list[tuple[aux.Point, float]] = []
        angle = 0.0
        result_vec = aux.Point(0, 0)

        for _ in range(NUM_DOTS_TO_ESTIMATION):
            new_neighbor = last_point + aux.rotate(aux.Point(READ_DELTA, 0), angle)
            neighbor_est = estimate_function(field, new_neighbor)
            angle += math.pi * 2 / NUM_DOTS_TO_ESTIMATION
            # estimation_neighbors.append((new_neighbor, neighbor_est))

            result_vec += (new_neighbor - last_point) * (neighbor_est - last_point_est)

        last_point = last_point + result_vec * GAIN

        screen.draw_dot(last_point, 1)
        print(last_point)

    return last_point
