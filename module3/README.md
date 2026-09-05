# Модуль В: датасет и YOLO

Камера: `/RMC1/arm95/camera_gripper/image_color` (`1600x1200`).

## Что размечать

Классы строго в таком написании:

```text
hammer
pliers
wrench
```

Рамкой выделяйте изображение инструмента на карточке, а класс задавайте по
картинке. Используйте одинаковый стиль разметки в симуляторе и на реальном поле.
Размечайте каждый видимый экземпляр: в штатной сцене две карточки `pliers`, одна
`hammer` и одна `wrench`. Классы `box` и `background` не создавайте.

В Roboflow создайте `Object Detection` project, загрузите `dataset/raw/*.jpg`,
разметьте кадры, сделайте split 70/20/10 и экспортируйте в формате YOLO11.
Распакуйте экспорт в `module3/training`: там должны появиться `data.yaml` и
каталоги `train`, `valid`, `test`. Не переставляйте номера классов в `data.yaml`:
они назначаются Roboflow и должны совпадать с ID внутри label-файлов.

В `dataset/raw` уже лежат 40 кадров из штатной сцены. Расположение объектов
менять не требуется: кадры сняты под разными углами камеры, а дополнительные
повороты и сдвиги выполняются аугментациями во время обучения.

## Сбор кадров из симулятора

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 module3/capture_dataset.py --count 30
```

Во время сбора плавно меняйте положение RMC1/манипулятора. Скрипт пропускает почти
одинаковые кадры. Для одиночного снимка используйте `--count 1 --min-change 0`.

## Установка для обучения

```bash
cd module3
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
pip install -r requirements.txt
```

Ultralytics ставит PyTorch как зависимость. Для конкретной NVIDIA CUDA или AMD
ROCm лучше сначала установить подходящую сборку PyTorch, затем `requirements.txt`.

## Обучение

Если часть объектов была размечена полигонами, один раз приведите экспорт к
обычным bounding boxes:

```bash
python prepare_dataset.py
```

Для теста используются максимум 30 эпох и early stopping после 7 эпох без
улучшения validation metrics.

Автовыбор: NVIDIA CUDA / AMD ROCm, а если GPU недоступен — CPU:

```bash
python train.py --device auto
```

Только CPU (подойдёт на AMD без настроенного ROCm):

```bash
python train.py --device cpu --batch 4 --workers 2
```

Принудительно NVIDIA CUDA или настроенный AMD ROCm:

```bash
python train.py --device cuda
```

Лучшие веса сохраняются в `training/runs/tools_yolo11n/weights/best.pt` и
автоматически копируются в `models/latest.pt` для простого запуска проверки.

## Проверка на фотографии

```bash
python test_model.py training/test/images/example.jpg --device cpu
```

С `--show` откроется окно. Независимо от окна размеченная фотография сохраняется
в `predictions/`: на ней видны bbox, названия классов и confidence.
