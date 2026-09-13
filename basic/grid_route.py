#!/usr/bin/env python3
"""Строит кратчайший маршрут по регулярной сетке ArUco без ROS."""

import argparse
from collections import deque


def neighbors(marker: int, size: int):
    row, column = divmod(marker, size)
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        next_row, next_column = row + dr, column + dc
        if 0 <= next_row < size and 0 <= next_column < size:
            yield next_row * size + next_column


def build_route(start: int, target: int, blocked: set[int], size: int):
    queue = deque([start])
    previous = {start: None}

    while queue:
        current = queue.popleft()
        if current == target:
            break
        for candidate in neighbors(current, size):
            if candidate not in blocked and candidate not in previous:
                previous[candidate] = current
                queue.append(candidate)

    if target not in previous:
        return None

    route = []
    current = target
    while current is not None:
        route.append(current)
        current = previous[current]
    return list(reversed(route))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("start", type=int, help="ID стартового маркера")
    parser.add_argument("target", type=int, help="ID целевого маркера")
    parser.add_argument("--size", type=int, default=5, help="5 для реального поля, 6 для Webots")
    parser.add_argument(
        "--blocked",
        type=int,
        nargs="*",
        default=[],
        help="Маркеры, через которые нельзя ехать",
    )
    args = parser.parse_args()

    if args.size < 2:
        parser.error("--size должен быть не меньше 2")
    if not 0 <= args.start < args.size**2 or not 0 <= args.target < args.size**2:
        parser.error(f"start и target должны быть от 0 до {args.size**2 - 1}")

    blocked = set(args.blocked)
    blocked.discard(args.start)
    blocked.discard(args.target)
    route = build_route(args.start, args.target, blocked, args.size)
    if route is None:
        raise SystemExit("Маршрут не найден")
    print(" -> ".join(map(str, route)))


if __name__ == "__main__":
    main()
