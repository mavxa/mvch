# Модуль В: простой вариант

## 1. Нарезать видео на кадры

Из корня репозитория:

```bash
mkdir -p module3/dataset/raw
ffmpeg -i video.mp4 -vf fps=2 module3/dataset/raw/frame_%05d.jpg
```

Двух кадров в секунду обычно достаточно. Удали смазанные и почти одинаковые
кадры. Оставь хотя бы 30–60 нормальных изображений.

## 2. Разметить локально

Один раз установить X-AnyLabeling в отдельное окружение:

```bash
python3 -m venv ~/.venvs/xanylabeling
~/.venvs/xanylabeling/bin/pip install -U uv
~/.venvs/xanylabeling/bin/uv pip install "x-anylabeling-cvhub[cpu]"
```

Открыть кадры:

```bash
~/.venvs/xanylabeling/bin/xanylabeling module3/dataset/raw \
  --labels module3/dataset/classes.txt --autosave
```

Классы уже лежат в `dataset/classes.txt`:

```text
hammer
pliers
wrench
```

Нажми `R`, обведи каждый видимый инструмент прямоугольником, выбери класс,
затем `D` для следующего кадра. После разметки выбери `File -> Export
Annotations -> Export YOLO Annotations -> Detection`, укажи
`dataset/classes.txt`. В `dataset/raw/labels` должны появиться TXT-файлы.

Подготовить train/valid/test:

```bash
python3 module3/simple_prepare_dataset.py
```

Скрипт создаст `module3/training_new` и при повторном запуске пересоздаст только
эту папку.

## 3. Обучить через Ultralytics CLI

```bash
cd module3
source .venv/bin/activate
yolo detect train model=yolo11n.pt data=training_new/data.yaml epochs=30 patience=7 imgsz=512 batch=8 device=0 workers=4 project=training_new/runs name=tools
cp training_new/runs/tools/weights/best.pt models/latest.pt
```

Если не хватает видеопамяти, поменяй только `batch=8` на `batch=4`.

## 4. Показать работу модели в OpenCV

На тестовых фотографиях:

```bash
python simple_module_b.py training_new/test/images
```

На исходном видео:

```bash
python simple_module_b.py /путь/к/video.mp4
```

На обычной USB-камере:

```bash
python simple_module_b.py 0
```

На изображении будут bounding box, класс и confidence. Результаты также пишутся
в `logs/simple_module_b.log`. `Q` или `Esc` закрывает окно.

## 5. Открыть и закрыть захват

После запуска симулятора и контроллеров:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 module3/simple_gripper.py
```

Скрипт сначала открывает захват, по `Enter` закрывает и ещё по `Enter` снова
открывает. Команды сохраняются в `module3/logs/simple_gripper.log`.

Если нужно отдельно показать движение всей руки, используй выданный пример:

```bash
ros2 launch ar_webots_fms_ros2 moveit_api_example.launch.py
```

## Что сказать эксперту

«Я нарезал видео на кадры, разметил три класса bounding box-ами, разделил
датасет на train/valid/test и обучил YOLO11n через Ultralytics CLI. В OpenCV
показываю класс, confidence и bounding box. Управление захватом отправляет ROS 2
команду контроллеру RMC1, а события выводятся в терминал и сохраняются в лог».
