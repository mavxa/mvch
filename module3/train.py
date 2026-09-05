#!/usr/bin/env python3
"""Обучение YOLO11 на размеченном датасете инструментов."""

import argparse
import shutil
from pathlib import Path


HERE = Path(__file__).resolve().parent
TRAINING = HERE / "training"


def local_path(value):
    path = Path(value).expanduser()
    return path if path.is_absolute() else HERE / path


def select_device(requested, torch):
    if requested == "cpu":
        return "cpu"
    if requested in ("cuda", "gpu"):
        if not torch.cuda.is_available():
            raise SystemExit("CUDA/ROCm не найден. Используйте --device cpu или установите PyTorch для GPU.")
        return 0
    if requested == "auto":
        if torch.cuda.is_available():
            return 0  # NVIDIA CUDA и AMD ROCm используют этот интерфейс PyTorch.
        if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            return "mps"
        return "cpu"
    return requested


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="training/data.yaml", help="data.yaml из Roboflow")
    parser.add_argument("--model", default="yolo11n.pt", help="Стартовые веса Ultralytics")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument(
        "--patience",
        type=int,
        default=7,
        help="Остановиться после N эпох без улучшения validation metrics",
    )
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument(
        "--device",
        default="auto",
        help="auto, cpu, cuda или номер GPU (например 0)",
    )
    parser.add_argument("--name", default="tools_yolo11n")
    return parser.parse_args()


def main():
    args = arguments()
    try:
        import torch
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit(f"Нет зависимости {error.name}. Выполните: pip install -r requirements.txt")

    data = local_path(args.data)
    if not data.is_file():
        raise SystemExit(f"Не найден датасет: {data}\nРаспакуйте YOLO11 export из Roboflow в module3/dataset.")
    device = select_device(args.device, torch)
    if device == 0:
        backend = "ROCm" if getattr(torch.version, "hip", None) else "CUDA"
        print(f"Устройство: {backend} GPU — {torch.cuda.get_device_name(0)}")
    else:
        print(f"Устройство: {device}")

    model = YOLO(args.model)
    result = model.train(
        data=str(data),
        epochs=args.epochs,
        patience=args.patience,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=device,
        project=str(TRAINING / "runs"),
        name=args.name,
        seed=42,
        deterministic=True,
        hsv_h=0.0,
        hsv_s=0.0,
        hsv_v=0.0,
        degrees=12.0,
        translate=0.05,
        scale=0.15,
        shear=2.0,
        fliplr=0.5,
        flipud=0.5,
    )
    save_dir = Path(result.save_dir)
    best_weights = save_dir / "weights" / "best.pt"
    model_copy = HERE / "models" / "latest.pt"
    model_copy.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_weights, model_copy)
    print(f"Лучшие веса: {best_weights}")
    print(f"Модель для test_model.py: {model_copy}")


if __name__ == "__main__":
    main()
