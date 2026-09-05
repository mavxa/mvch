#!/usr/bin/env python3
"""Рисует bbox, название класса и confidence модели на фотографии."""

import argparse
from pathlib import Path


HERE = Path(__file__).resolve().parent


def local_path(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else HERE / path


def select_device(requested, torch):
    if requested == "auto":
        if torch.cuda.is_available():
            return 0
        if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            return "mps"
        return "cpu"
    if requested in ("cuda", "gpu"):
        if not torch.cuda.is_available():
            raise SystemExit("CUDA/ROCm не найден. Используйте --device cpu.")
        return 0
    return requested


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="Фотография для проверки")
    parser.add_argument("--weights", default="runs/tools_yolo11n/weights/best.pt")
    parser.add_argument("--output", default="predictions", help="Каталог результата")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--device", default="auto", help="auto, cpu, cuda или номер GPU")
    parser.add_argument("--show", action="store_true", help="Открыть окно с результатом")
    return parser.parse_args()


def main():
    args = arguments()
    try:
        import cv2
        import torch
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(f"Нет зависимости {error.name}. Выполните: pip install -r requirements.txt")

    source = local_path(args.source)
    weights = local_path(args.weights)
    output = local_path(args.output)
    if not source.is_file():
        raise SystemExit(f"Не найдена фотография: {source}")
    if not weights.is_file():
        raise SystemExit(f"Не найдены веса: {weights}")
    if not 0 <= args.conf <= 1:
        raise SystemExit("--conf должен быть от 0 до 1")

    model = YOLO(str(weights))
    result = model.predict(
        source=str(source),
        conf=args.conf,
        imgsz=args.imgsz,
        device=select_device(args.device, torch),
        verbose=False,
    )[0]
    output.mkdir(parents=True, exist_ok=True)
    destination = output / f"{source.stem}_detected.jpg"
    annotated = result.plot(labels=True, conf=True, line_width=2)
    if not cv2.imwrite(str(destination), annotated):
        raise SystemExit(f"Не удалось сохранить {destination}")

    if result.boxes is None or len(result.boxes) == 0:
        print("Объекты не найдены")
    else:
        for box in result.boxes:
            class_id = int(box.cls.item())
            confidence = float(box.conf.item())
            x1, y1, x2, y2 = (round(float(v), 1) for v in box.xyxy[0])
            print(f"{result.names[class_id]}: {confidence:.3f}, bbox=({x1}, {y1}, {x2}, {y2})")
    print(f"Изображение с классами: {destination}")

    if args.show:
        cv2.imshow("Module 3 detection - press any key", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
