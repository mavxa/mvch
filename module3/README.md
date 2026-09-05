# Модуль В: датасет и YOLO

Камера: `/RMC1/arm95/camera_gripper/image_color` (`1600x1200`).

## Что размечать

Классы строго в таком написании:

```text
hammer
wrench
pliers
```

Рамкой выделяйте **всю белую карточку/коробку с изображением инструмента**, а класс
задавайте по картинке. Это даст модели центр коробки, нужный затем для захвата.
Размечайте каждый видимый экземпляр: в штатной сцене две карточки `pliers`, одна
`hammer` и одна `wrench`. Классы `box` и `background` не создавайте.

В Roboflow создайте `Object Detection` project, загрузите `dataset/raw/*.jpg`,
разметьте кадры, сделайте split 70/20/10 и экспортируйте в формате YOLO11.
Распакуйте экспорт так, чтобы появился `module3/dataset/data.yaml` и каталоги
`train`, `valid`, `test`. Проверьте порядок классов в экспортированном `data.yaml`:
он должен совпадать с `dataset/classes.txt`.

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

Результат: `runs/tools_yolo11n/weights/best.pt`.

## Проверка на фотографии

```bash
python test_model.py dataset/test/images/example.jpg --device cpu
```

С `--show` откроется окно. Независимо от окна размеченная фотография сохраняется
в `predictions/`: на ней видны bbox, названия классов и confidence.
