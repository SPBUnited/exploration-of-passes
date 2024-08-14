import time
from random import random

import auxiliary as aux
import drawing
from estimation import draw_heat_map, get_cells

if __name__ == "__main__":
    screen = drawing.Image()
    screen.update_window()

    # while True:
    kick_point = aux.Point(250 + 500 * random(), -1000 + 1000 * random())

    enemies = [
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
        # aux.Point(random() * 1500 + 500, random() * 3000 - 1500),
    ]

    cells = get_cells(kick_point, enemies)

    for cell in cells:
        grey = 128 + int(random() * 127)
        cell.draw(screen, (grey, grey, grey))
        screen.update_window()

    draw_heat_map(screen, kick_point, enemies)

    for cell in cells:
        grey = 128 + int(random() * 127)
        cell.draw(screen, (grey, grey, grey))
        screen.update_window()

    screen.draw_dot(kick_point, 4, (255, 255, 255))
    screen.draw_dot(kick_point, 3)

    for enemy in enemies:
        screen.draw_robot(enemy)

    screen.draw_field()
    while True:
        screen.update_window()
