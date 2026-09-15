#!/usr/bin/env python3
import random
import shutil
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "dataset/raw"
LABELS = SOURCE / "labels"
OUTPUT = HERE / "training_new"
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def main():
    images = sorted(path for path in SOURCE.iterdir() if path.suffix.lower() in IMAGE_EXTENSIONS)
    if not images:
        raise SystemExit(f"Нет кадров в {SOURCE}")
    if not LABELS.is_dir():
        raise SystemExit(f"Сначала экспортируйте YOLO-разметку в {LABELS}")

    random.Random(42).shuffle(images)
    test_count = max(1, round(len(images) * 0.1)) if len(images) >= 3 else 0
    valid_count = max(1, round(len(images) * 0.2)) if len(images) >= 3 else 0
    parts = {
        "test": images[:test_count],
        "valid": images[test_count : test_count + valid_count],
        "train": images[test_count + valid_count :],
    }

    if OUTPUT.exists():
        shutil.rmtree(OUTPUT)

    for part, part_images in parts.items():
        image_dir = OUTPUT / part / "images"
        label_dir = OUTPUT / part / "labels"
        image_dir.mkdir(parents=True)
        label_dir.mkdir(parents=True)
        for image in part_images:
            shutil.copy2(image, image_dir / image.name)
            label = LABELS / f"{image.stem}.txt"
            destination = label_dir / f"{image.stem}.txt"
            if label.exists():
                shutil.copy2(label, destination)
            else:
                destination.touch()

    classes = (HERE / "dataset/classes.txt").read_text(encoding="utf-8").splitlines()
    names = "\n".join(f"  {number}: {name}" for number, name in enumerate(classes))
    (OUTPUT / "data.yaml").write_text(
        f"path: {OUTPUT}\ntrain: train/images\nval: valid/images\ntest: test/images\n\nnames:\n{names}\n",
        encoding="utf-8",
    )

    print(f"Готово: train={len(parts['train'])}, valid={len(parts['valid'])}, test={len(parts['test'])}")
    print(OUTPUT / "data.yaml")


if __name__ == "__main__":
    main()
