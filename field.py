import auxiliary as aux


class Field:
    def __init__(self, kick_point: aux.Point, enemies: list[aux.Point]) -> None:
        self.kick_point = kick_point
        self.enemies = enemies
