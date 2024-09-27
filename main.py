import time
from random import random, seed

import auxiliary as aux
import drawing
from estimation import draw_heat_map, get_cells, estimate_point
from cells_tools import draw_cells

from scipy.optimize import minimize
import numpy as np
from concurrent.futures import ThreadPoolExecutor


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
        # aux.Point(3000, 200),
        # aux.Point(1100, -1600),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
    ]
    enemies = sort_enemies(enemies, kick_point)

    cells = get_cells(kick_point, enemies)

    def wrp_fnc(x):
        point = aux.Point(x[0], x[1])
        return -estimate_point(point, kick_point, enemies)

    t = time.time()

    _max = -100
    pnts = []

    def process_cell(cell):
        tmp = aux.Point(4500 * random(), 6000 * random() - 3000)
        tmp = aux.average_point(cell.peaks)
        res = minimize(
            wrp_fnc,
            np.array([tmp.x, tmp.y]),
            bounds=[(0, 4500), (-3000, 3000)],
            method="Nelder-Mead",
        )
        return res

    with ThreadPoolExecutor() as executor:
        futures = executor.map(process_cell, cells)

        for res in futures:
            if -res.get("fun") > _max:
                _max = -res.get("fun")
            pnts.append(
                (
                    aux.Point(res.get("x")[0], res.get("x")[1]),
                    aux.minmax(-res.get("fun"), -1, 1),
                )
            )
    print(time.time() - t)

    draw_cells(screen, cells)
    screen.update_window()

    draw_heat_map(screen, kick_point, enemies)

    draw_cells(screen, cells)

    screen.draw_dot(kick_point, 4, (255, 255, 255))
    screen.draw_dot(kick_point, 3)

    for p in pnts:
        print(p[0], p[1])
        if p[1] < 0:
            color = (255 * -p[1], 0, 0)
        else:
            color = (0, 255 * p[1], 0)
        screen.draw_dot(p[0], 10, color)

    for enemy in enemies:
        screen.draw_robot(enemy)

    screen.draw_field()

    # print("ball:", kick_point)
    # print("enemies", enemies[0], enemies[1])
    while True:
        screen.update_window()
