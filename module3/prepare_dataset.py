#!/usr/bin/env python3
"""Преобразует polygon-разметку Roboflow в обычные YOLO bounding boxes."""

import argparse
from pathlib import Path


HERE = Path(__file__).resolve().parent


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="training", help="Каталог Roboflow export")
    return parser.parse_args()


def convert_line(line, filename):
    values = line.split()
    if len(values) == 5:
        return line.strip(), False
    if len(values) < 7 or (len(values) - 1) % 2:
        raise ValueError(f"Некорректная строка в {filename}: {line.strip()}")

    class_id = int(values[0])
    coordinates = [float(value) for value in values[1:]]
    xs = coordinates[0::2]
    ys = coordinates[1::2]
    x1, x2 = min(xs), max(xs)
    y1, y2 = min(ys), max(ys)
    box = (class_id, (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)
    return f"{box[0]} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f} {box[4]:.6f}", True


def main():
    args = arguments()
    dataset = Path(args.dataset).expanduser()
    if not dataset.is_absolute():
        dataset = HERE / dataset

    files = sorted(dataset.glob("*/labels/*.txt"))
    if not files:
        raise SystemExit(f"Не найдены labels в {dataset}")

    converted = 0
    objects = 0
    for filename in files:
        output = []
        for line in filename.read_text().splitlines():
            if not line.strip():
                continue
            result, changed = convert_line(line, filename)
            output.append(result)
            converted += int(changed)
            objects += 1
        filename.write_text("\n".join(output) + "\n")

    print(f"Готово: файлов={len(files)}, объектов={objects}, polygon->bbox={converted}")


if __name__ == "__main__":
    main()
