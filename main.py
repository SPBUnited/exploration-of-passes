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
    screen.update_window()

    kick_point = aux.Point(250 + 500 * random(), -1000 + 1000 * random())

    enemies = [
        # aux.Point(942, -1200),
        # aux.Point(1100, -1600),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
    ]
    enemies = sort_enemies(enemies, kick_point)

    cells = get_cells(kick_point, enemies)

    draw_cells(screen, cells)
    screen.update_window()

    draw_heat_map(screen, kick_point, enemies)

    draw_cells(screen, cells)

    screen.draw_dot(kick_point, 4, (255, 255, 255))
    screen.draw_dot(kick_point, 3)

    for enemy in enemies:
        screen.draw_robot(enemy)

    screen.draw_field()

    # print("ball:", kick_point)
    # print("enemies", enemies[0], enemies[1])
    while True:
        screen.update_window()
