"""
Модуль маршрутизации роботов
Получает от стратега требуемые координаты для каждого робота и
считает оптимальный маршрут для достижения этой точки
"""

class Router:
    def __init__(self) -> None:
        pass

    """
    Установить единственную путевую точку для робота с индексом idx
    """
    def setWaypoint(self, idx, target):
        pass

    """
    Рассчитать маршруты по актуальным путевым точкам
    """
    def calcRoutes(self, field):
        pass