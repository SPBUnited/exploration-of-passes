import time
from random import random, seed

import auxiliary as aux
import drawing
from estimation import draw_heat_map, get_cells, estimate_point
from cells_tools import draw_cells

from scipy.optimize import minimize, dual_annealing
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
    # 0.11668801307678223
    # 0.053627729415893555

    seed(1242)

    kick_point_zero = aux.Point(250 + 500 * random(), -1000 + 1000 * random())

    enemies_zero = [
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
    ]
    enemies = sort_enemies(enemies_zero, kick_point_zero)


    def wrp_fnc(x):
        point = aux.Point(x[0], x[1])
        return -estimate_point(point, kick_point, enemies)


    # print("ball:", kick_point)
    # print("enemies", enemies[0], enemies[1])
    def process_cell(cell):
        # tmp = aux.Point(4500 * random(), 6000 * random() - 3000)
        tmp = aux.average_point(cell.peaks)
        res = minimize(
            wrp_fnc,
            np.array([tmp.x, tmp.y]),
            bounds=[(0, 4500), (-3000, 3000)],
            method="Nelder-Mead",
        )

        # res = dual_annealing(
        #     wrp_fnc,
        #     [(0, 4500), (-3000, 3000)],
        # )
        return res


    while True:
        timer = time.time()
        kick_point = kick_point_zero + aux.rotate(aux.Point(100, 150), time.time() / 3)
        enemies = []
        for enemy_zero in enemies_zero:
            enemies.append(enemy_zero + aux.rotate(aux.Point(200, 350), time.time()))
        enemies = sort_enemies(enemies, kick_point)

        cells = get_cells(kick_point, enemies)

        _max = -100
        pnts = []

        with ThreadPoolExecutor(max_workers=1) as executor:
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
        print(time.time() - timer)

        screen.update_window()

        # draw_heat_map(screen, kick_point, enemies)

        screen.draw_dot(kick_point, 4, (255, 255, 255))
        screen.draw_dot(kick_point, 3)

        cells = get_cells(kick_point, enemies)

        # draw_cells(screen, cells)
        # screen.update_window()

        # draw_heat_map(screen, kick_point, enemies)

        screen.draw_field()

        # draw_cells(screen, cells)

        screen.draw_dot(kick_point, 4, (255, 255, 255))
        screen.draw_dot(kick_point, 3)

        for enemy in enemies:
            screen.draw_robot(enemy)

        min_distance = 1000
        best = []

        for point in pnts:
            if all((point[0] - existing_point[0]).mag() >= min_distance for existing_point in best):
                best.append(point)

        best = sorted(best, key = lambda x: -x[1])

        # print(best)
        for p in best:
            print(p[0], p[1])
            if p[1] < 0:
                color = (255 * -p[1], 0, 150)
            else:
                color = (0, 255 * p[1], 150)
            screen.draw_dot(p[0], 10, color)
        # screen.draw_dot(best[0][0], 11, (255, 255, 255))
        # screen.draw_dot(best[1][0], 11, (255, 255, 255))
        # screen.draw_dot(best[2][0], 11, (255, 255, 255))

        # print("ball:", kick_point)
        # print("enemies", enemies[0], enemies[1])
        print("delay:", time.time() - timer)
        screen.update_window()
