#!/usr/bin/env python3
import argparse
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", help="Фото, видео, папка с фото или 0 для веб-камеры")
    parser.add_argument("--weights", default=str(HERE / "models/latest.pt"))
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    source = int(args.source) if args.source.isdigit() else args.source
    model = YOLO(args.weights)
    log_path = HERE / "logs/simple_module_b.log"
    log_path.parent.mkdir(exist_ok=True)

    still_images = False
    if isinstance(source, str):
        path = Path(source)
        still_images = path.is_dir() or path.suffix.lower() in {
            ".jpg",
            ".jpeg",
            ".png",
            ".bmp",
            ".webp",
        }

    with log_path.open("a", encoding="utf-8") as log:
        results = model.predict(source=source, conf=args.conf, stream=True, verbose=False)
        for result in results:
            now = datetime.now().isoformat(timespec="seconds")
            detections = []
            if result.boxes is not None:
                for box in result.boxes:
                    name = result.names[int(box.cls.item())]
                    confidence = float(box.conf.item())
                    detections.append(f"{name} {confidence:.2f}")

            line = f"{now} | {', '.join(detections) if detections else 'ничего не найдено'}"
            print(line)
            log.write(line + "\n")
            log.flush()

            cv2.imshow("YOLO: q или Esc для выхода", result.plot())
            key = cv2.waitKey(0 if still_images else 1) & 0xFF
            if key in (ord("q"), 27):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
