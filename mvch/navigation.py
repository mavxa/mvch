"""Геометрия и кратчайший путь. Работает без ROS, проверяется на хосте."""

import heapq
import json
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def compose(a, b):
    """a = положение системы B в A; b = положение C в B. Результат: C в A."""
    x, y, yaw = a
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c*b[0] - s*b[1], y + s*b[0] + c*b[1], wrap(yaw+b[2])


def inverse(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return -c*x-s*y, s*x-c*y, -yaw


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def segment_distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx+dy*dy
    t = clamp(((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2, 0, 1) if length2 else 0
    return distance(point, (a[0]+t*dx, a[1]+t*dy))


class Field:
    def __init__(self, data):
        # Можно передать выданный экспертами граф вместо регулярной сетки.
        if "nodes" in data:
            nodes = data["nodes"]
        else:
            rows, cols, step = data["rows"], data["cols"], data["spacing"]
            if rows < 1 or cols < 1 or not math.isfinite(step) or step <= 0:
                raise ValueError("Неверные размеры сетки")
            nodes = []
            for row in range(rows):
                for col in range(cols):
                    neighbors = [r*cols+c for r, c in
                                 [(row+1, col), (row, col+1), (row-1, col), (row, col-1)]
                                 if 0 <= r < rows and 0 <= c < cols]
                    nodes.append(dict(id=row*cols+col, x=-row*step, y=col*step,
                                      yaw=0, neighbors=neighbors))
        self.poses = {n["id"]: (n["x"], n["y"], n.get("yaw", 0)) for n in nodes}
        self.graph = {n["id"]: n["neighbors"] for n in nodes}
        if not nodes or len(self.poses) != len(nodes):
            raise ValueError("Граф пуст или содержит повторяющиеся ID")
        for node, neighbors in self.graph.items():
            if not isinstance(node, int) or not all(math.isfinite(v) for v in self.poses[node]):
                raise ValueError("Неверные ID или координаты")
            if any(n not in self.poses or n == node for n in neighbors):
                raise ValueError("Неверный сосед в графе")
        self.blocked = set(data.get("blocked", []))
        if not self.blocked <= self.poses.keys():
            raise ValueError("Заблокирован неизвестный ID")

    def nearest(self, x, y):
        return min(self.poses, key=lambda n: distance((x, y), self.poses[n]))

    def route(self, start, goal, points=(), clearance=0.5, forbidden_edges=()):
        """Дейкстра: кратчайшая длина пути по известным свободным рёбрам графа."""
        if start not in self.poses or goal not in self.poses:
            raise ValueError("ID старта/цели отсутствует в графе")
        if start in self.blocked or goal in self.blocked:
            raise ValueError("Старт/цель заблокированы")
        forbidden = {frozenset(e) for e in forbidden_edges}
        costs, parents, queue = {start: 0.0}, {start: None}, [(0.0, start)]
        while queue:
            cost, current = heapq.heappop(queue)
            if cost > costs[current]:
                continue
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = parents[current]
                return path[::-1]
            for neighbor in self.graph[current]:
                if neighbor in self.blocked or frozenset((current, neighbor)) in forbidden:
                    continue
                a, b = self.poses[current], self.poses[neighbor]
                if any(segment_distance(p, a, b) < clearance for p in points):
                    continue
                new_cost = cost + distance(a, b)
                if new_cost < costs.get(neighbor, math.inf):
                    costs[neighbor], parents[neighbor] = new_cost, current
                    heapq.heappush(queue, (new_cost, neighbor))
        raise ValueError(f"Нет безопасного маршрута {start} -> {goal}")


def load_config(path):
    with open(path, encoding="utf-8") as file:
        return json.load(file)

