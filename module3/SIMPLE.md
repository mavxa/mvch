# Простой модуль В

## Кадры

```bash
mkdir -p module3/dataset/raw
ffmpeg -i video.mp4 -vf fps=2 module3/dataset/raw/frame_%05d.jpg
```

## Разметка

```bash
python3 -m venv ~/.venvs/xanylabeling
~/.venvs/xanylabeling/bin/pip install "x-anylabeling-cvhub[cpu]"
~/.venvs/xanylabeling/bin/xanylabeling module3/dataset/raw --labels module3/dataset/classes.txt --autosave
```

Рамки рисуются клавишей `R`, следующий кадр — `D`. Экспорт: `File -> Export
Annotations -> Export YOLO Annotations -> Detection`. Затем:

```bash
python3 module3/simple_prepare_dataset.py
```

## Обучение

```bash
cd module3
source .venv/bin/activate
yolo detect train model=yolo11n.pt data=training_new/data.yaml epochs=30 patience=7 imgsz=512 batch=8 device=0 project=training_new/runs name=tools exist_ok=True
cp training_new/runs/tools/weights/best.pt models/latest.pt
```

## Запуск

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/scripts/mvch
module3/.venv/bin/python module3/simple_module_b.py
```

Ввод предмета нужен только для показа. Скрипт выводит YOLO в окне OpenCV, через
5 секунд открывает захват, а через 2 секунды закрывает его. Выход — `Q` или
`Esc`.
