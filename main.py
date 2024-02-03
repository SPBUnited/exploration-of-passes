import time

import auxiliary as aux
from drawing import Image

if __name__ == "__main__":
    screen = Image()
    screen.update_window()
    trajectory = [aux.Point(0, 0), aux.Point(80, 15), aux.Point(1450, -2450), aux.Point(20, 45)]

    # screen.draw_graph(aux.Point(0, 0), aux.Point(10, 5), aux.Point(345, 545), aux.Point(20, 25))
    while True:
        screen.draw_bang_bang_trajectory(aux.Point(0, 0), aux.Point(-140, 60), aux.Point(1800, 2500), aux.Point(-160, 140))
        screen.update_window()
        time.sleep(0.01)
