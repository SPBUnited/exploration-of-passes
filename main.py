import time

import auxiliary as aux
from drawing import Image

if __name__ == "__main__":
    screen = Image()
    screen.update_window()

    # screen.draw_graph(aux.Point(0, 0), aux.Point(10, 5), aux.Point(345, 545), aux.Point(20, 25))
    while True:
        screen.draw_heat_map(
            [aux.Point(1500, 500), aux.Point(1750, -750), aux.Point(2000, 0)]
        )
        screen.update_window()
        time.sleep(0.01)
