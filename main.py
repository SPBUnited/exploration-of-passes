import time
import math
from random import random

import auxiliary as aux
import drawing
import const

if __name__ == "__main__":
    screen = drawing.Image()
    screen.update_window()

    # while True:
    kick_point = aux.Point(1000 + 1000 * random(), -1000 + 1000 * random())
    angle = 1 + 1.07 * random()

    enemies = [
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
    ]
    # drawing.estimate_angle(kick_point, angle, enemies)
    screen.update_window()
    screen.draw_heat_map(kick_point, enemies)
    screen.draw_line(
        kick_point, kick_point + aux.rotate(aux.RIGHT, angle) * 4000, 2, (128, 128, 128)
    )
    # screen.find_best_angle(kick_point, 1)
    screen.update_window()
    for enemy in enemies:
        inter = aux.get_line_intersection(
            kick_point,
            kick_point + aux.rotate(aux.RIGHT, angle),
            aux.Point(const.GOAL_DX, 0),
            enemy,
            "RR",
        )
        if inter is not None:
            print(aux.dist(kick_point, inter))
    drawing.estimate_angle(kick_point, angle, enemies)
    time.sleep(1000)
