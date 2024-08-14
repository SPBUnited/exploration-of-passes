import math
from typing import Optional

import auxiliary as aux
from drawing import Image


class Cell:
    def __init__(self, peaks: list[aux.Point]) -> None:
        self.peaks: list[aux.Point] = peaks

    def print(self) -> None:
        print("cell peaks:")
        for peak in self.peaks:
            print("\t", peak)

    def draw(self, screen: Image) -> None:
        """
        draw cell on screen
        """
        for index, _ in enumerate(self.peaks):
            screen.draw_line(
                self.peaks[index - 1], self.peaks[index], 3, (255, 255, 255)
            )

    def get_line_intersections(
        self, line_start: aux.Point, line_end: aux.Point, is_inf: str = "L"
    ) -> list[tuple[aux.Point, int]]:
        """
        returns a list of intersections and numbers of crossed edges
        line_start, line_end - points on line
        is_inf: "S" - segment, "R" - ray, "L" - line
        """
        intersections: list[tuple[aux.Point, int]] = []
        for index, _ in enumerate(self.peaks):
            inter = aux.get_line_intersection(
                self.peaks[index - 1],
                self.peaks[index],
                line_start,
                line_end,
                "S" + is_inf,
            )
            if inter is not None and (
                len(intersections) == 0
                or inter != intersections[-1][0]  # if intersection is peak
            ):
                intersections.append((inter, index))

        if intersections[0][0] == intersections[-1][0]:  # if intersection is peak
            intersections.pop()
        return intersections

    def intersect_cell(
        self, line_start: aux.Point, line_end: aux.Point
    ) -> Optional["Cell"]:
        """
        if the line intersects a cell, changes it and returns a new one (only convex cells)
        """
        inter_idx = self.get_line_intersections(line_start, line_end)
        if not inter_idx:
            return None
        inter1, idx1 = inter_idx[0]
        inter2, idx2 = inter_idx[1]
        if inter1 == inter2:
            return None

        new_cell_peaks = self.peaks[idx1:idx2]

        if new_cell_peaks[-1] != inter2:
            new_cell_peaks += [inter2]
        if new_cell_peaks[0] != inter1:
            new_cell_peaks += [inter1]

        new_cell = Cell(new_cell_peaks)

        updated_peaks = self.peaks[idx2:] + self.peaks[0:idx1]  # + [inter1, inter2]

        if updated_peaks[-1] != inter1:
            updated_peaks += [inter1]
        if updated_peaks[0] != inter2:
            updated_peaks += [inter2]

        self.peaks = updated_peaks

        return new_cell


if __name__ == "__main__":
    cell1 = Cell(
        [
            aux.Point(2, 0),
            aux.Point(3, 1),
            aux.Point(3, 2),
            aux.Point(0, 2),
            aux.Point(-2, 0),
        ]
    )
    line = [aux.Point(3, 0), aux.Point(-2, 2)]

    cell1.print()

    cell2 = cell1.intersect_cell(line[0], line[1])

    cell1.print()
    cell2.print()
