import time
from random import random

import auxiliary as aux
import drawing
from estimation import draw_heat_map, get_cells
from cells_tools import draw_cells


def sort_enemies(
    enemies_to_sort: list[aux.Point], point_to_sort: aux.Point
) -> list[aux.Point]:
    enemies_dist: list[tuple[aux.Point, float]] = []
    for enemy in enemies_to_sort:
        enemies_dist.append((enemy, -aux.dist(enemy, point_to_sort)))

    enemies_dist = sorted(enemies_dist, key=lambda x: x[1])

    sorted_enemies: list[aux.Point] = []
    for enemy_dist in enemies_dist:
        sorted_enemies.append(enemy_dist[0])

    return sorted_enemies


if __name__ == "__main__":
    screen = drawing.Image()

    kick_point_zero = aux.Point(500, 500)
    enemies_zero = [
        aux.Point(2000, 1500),
        aux.Point(800, 1000),
        aux.Point(2000, -500),
        aux.Point(3500, 1000),
        aux.Point(500, -2000),
    ]

    while True:
        timer = time.time()
        kick_point = kick_point_zero + aux.rotate(aux.Point(100, 150), time.time() / 3)
        enemies = []
        for enemy_zero in enemies_zero:
            enemies.append(enemy_zero + aux.rotate(aux.Point(100, 150), time.time()))
        enemies = sort_enemies(enemies, kick_point)

        cells = get_cells(kick_point, enemies)

        # draw_cells(screen, cells)
        # screen.update_window()

        # draw_heat_map(screen, kick_point, enemies)

        screen.draw_field()

        draw_cells(screen, cells)

        screen.draw_dot(kick_point, 4, (255, 255, 255))
        screen.draw_dot(kick_point, 3)

        for enemy in enemies:
            screen.draw_robot(enemy)

        # print("ball:", kick_point)
        # print("enemies", enemies[0], enemies[1])
        print("delay:", time.time() - timer)
        screen.update_window()
