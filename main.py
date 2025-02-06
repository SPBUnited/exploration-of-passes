import time
from random import random, seed

import auxiliary as aux
import drawing
from estimation import draw_heat_map, get_cells, estimate_point
from cells_tools import draw_cells

from scipy.optimize import minimize
import pygame
import numpy as np


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

    seed_ = 421
    # seed_ = 239

    seed(seed_)
    kick_point = aux.Point(250 + 500 * random(), -1000 + 1000 * random())
    enemies = [
        # aux.Point(3000, 200),
        # aux.Point(1100, -1600),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
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

    for cell in cells:
        tmp = aux.average_point(cell.peaks)

        res = minimize(
            wrp_fnc,
            np.array(
                [tmp.x, tmp.y],
            ),
            bounds=[(0, 4500), (-3000, 3000)],
            method="Nelder-Mead",
        )

        if -res.get("fun") > _max:
            _max = -res.get("fun")
        cur = aux.Point(res.get("x")[0], res.get("x")[1])
        f = False
        for pnt in pnts:
            if aux.dist(pnt[0], cur) < 150:
                f = True
                break
        if not f:
            pnts.append(
                (
                    aux.Point(res.get("x")[0], res.get("x")[1]),
                    aux.minmax(-res.get("fun"), -1, 1),
                )
            )
    print(time.time() - t)

    pnts.sort(key=lambda x: -x[1])
    # draw_cells(screen, cells)
    screen.update_window()

    real_maxs = draw_heat_map(screen, kick_point, enemies)

    # draw_cells(screen, cells)

    screen.draw_dot(kick_point, 4, (255, 255, 255))
    screen.draw_dot(kick_point, 3)
    screen.draw_field()


    for enemy in enemies:
        screen.draw_robot(enemy)

    screen.update_window()
    pygame.image.save(screen.screen, f"./{seed_}_1.png")

    for i, p in enumerate(pnts[:2]):
        print(p[0], p[1])
        if p[1] < 0:
            color = (255 * -p[1], 0, 0)
        else:
            color = (0, 255 * p[1], 0)
        # screen.draw_dot(p[0], 8, color)
        if i == 0:
            screen.draw_dot(p[0], 8, (0, 100, 0))
        else:
            screen.draw_dot(p[0], 8, (255, 186, 0))
            print("Point: ", p[0])

    for real_max in real_maxs:
        screen.draw_dot(real_max, 3, (255, 0, 255))

    # print("ball:", kick_point)
    # print("enemies", enemies[0], enemies[1])
    screen.update_window()
    pygame.image.save(screen.screen, f"./{seed_}_2.png")
    while True:
        screen.update_window()
