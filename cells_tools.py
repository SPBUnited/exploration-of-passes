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

    def draw(
        self,
        screen: Image,
        color: tuple[int, int, int] = (255, 255, 255),
        name: Optional[str] = None,
    ) -> None:
        """
        draw cell on screen
        """
        center = aux.average_point(self.peaks)

        for index, _ in enumerate(self.peaks):
            point1 = (
                self.peaks[index - 1] + (center - self.peaks[index - 1]).unity() * 15
            )
            point2 = self.peaks[index] + (center - self.peaks[index]).unity() * 15
            screen.draw_line(point1, point2, 2, color)

        if name:
            screen.print_text(name, center)

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

        if (
            intersections and intersections[0][0] == intersections[-1][0]
        ):  # if intersection is peak
            intersections.pop()
        return intersections

    def intersect_cell(
        self, line_start: aux.Point, line_end: aux.Point, is_inf: str = "L"
    ) -> Optional["Cell"]:
        """
        if the line intersects a cell, changes it and returns a new one (only convex cells)
        is_inf: "S" - segment, "R" - ray, "L" - line
        """
        inter_idx = self.get_line_intersections(line_start, line_end, is_inf)
        if len(inter_idx) < 2:
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

    def crop_cell(
        self,
        line_start: aux.Point,
        line_end: aux.Point,
        side_to_delete: int,
        is_inf: str = "L",
    ) -> bool:
        """
        crop the cell with line
        side_to_delete: -1 - delete left part, 1 - delete right part (to look relative to the direction of the vector)
        return True if cell been cropped, False if can't crop cell (line don't intersect cell)
        is_inf: "S" - segment, "R" - ray, "L" - line
        """
        cropped_part = self.intersect_cell(line_start, line_end, is_inf)
        if cropped_part:
            center = aux.average_point(self.peaks)
            sign = aux.sign(
                aux.vec_mult((center - line_start), (line_end - line_start))
            )
            if sign != side_to_delete:
                self.peaks = cropped_part.peaks
            return True
        else:
            return False


def draw_cells(screen: Image, cells: list[Cell]) -> None:
    """
    draw cells with different grey color
    """
    grey_min = 64
    grey_max = 255
    step = (grey_max - grey_min) / (len(cells) - 1)

    for num, cell in enumerate(cells):
        grey = int(grey_min + num * step)
        cell.draw(screen, (grey, grey, grey), str(num))


# if __name__ == "__main__":
#     cell1 = Cell(
#         [
#             aux.Point(2, 0),
#             aux.Point(3, 1),
#             aux.Point(3, 2),
#             aux.Point(0, 2),
#             aux.Point(-2, 0),
#         ]
#     )
#     line = [aux.Point(3, 0), aux.Point(-2, 2)]

#     cell1.print()

#     cell2 = cell1.intersect_cell(line[0], line[1])

#     cell1.print()
#     cell2.print()
