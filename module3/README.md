# Модуль В: датасет и YOLO

Камера: `/RMC1/arm95/camera_gripper/image_color` (`1600x1200`).

Основной зачетный скрипт: `module3.py`. Он показывает обработанный видеопоток,
определяет координаты выбранной детали относительно `Base_link`, захватывает ее,
возвращает ARM95 в исходное положение с деталью, кладет деталь обратно на
освободившееся место и снова возвращает руку в исходное положение.
После первого безопасного подхода координаты повторно уточняются по близкому
кадру, поэтому рука не едет к детали только по одному дальнему измерению.
Если в кадре несколько деталей одного класса, скрипт выбирает ближайшую к базе.

## Запуск задания

В VM один раз создайте окружение с доступом к системным пакетам ROS:

```bash
cd ~/scripts/mvch/module3
sudo apt install python3-venv
python3 -m venv --system-site-packages .venv
.venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
.venv/bin/pip install -r requirements.txt tqdm ultralytics-thop
```

Если окружение уже обновило NumPy до 2.x и `cv_bridge` ругается на ABI:

```bash
.venv/bin/pip install --force-reinstall "numpy<2" "opencv-python<4.12" lark
```

`lark` — зависимость ROS 2 Launch/MoveIt, а не часть алгоритма распознавания.
Ограничение `numpy<2` нужно потому, что бинарный `cv_bridge` в этом образе ROS
собран с NumPy 1.x.

Если venv уже был создан без `--system-site-packages`, пересоздавать его не
нужно. Включите системные зависимости ROS и заново активируйте окружение:

```bash
sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' .venv/pyvenv.cfg
deactivate
source .venv/bin/activate
```

Запустите симулятор модуля В, затем в новом терминале:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/scripts/mvch
module3/.venv/bin/python module3/module3.py --target 1
```

Цели: `1/hammer`, `2/wrench`, `3/pliers`. После того как эксперт зафиксировал
видеопоток, нажмите `G` или `Space` в окне либо введите `g` и
нажмите Enter в терминале. `Esc` останавливает скрипт до начала движения.

Сначала обязательно проверьте только координаты:

```bash
module3/.venv/bin/python module3/module3.py --target hammer --dry-run
```

Для проверки через SSH без окна:

```bash
module3/.venv/bin/python module3/module3.py --target hammer --dry-run --no-window
```

Геометрию реального стенда нельзя брать из симулятора вслепую. Перед зачетом
измерьте и подстройте `--plane-z`, `--pick-z` и `--approach-z`. По умолчанию
деталь возвращается туда, откуда была взята; другую свободную точку можно задать
через `--drop-x X --drop-y Y`.

События распознавания, координаты, построение движений и состояния схвата
одновременно выводятся в терминал и сохраняются в `module3/logs/*.log`.
Туда же пишутся контрольные кадры `*_gripped.jpg`, `*_with_detail.jpg` и
`*_finished.jpg`.

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

Локально в `dataset/raw` могут лежать собранные кадры, но Git их намеренно
игнорирует: после клонирования датасет нужно перенести отдельно или собрать
заново. Расположение объектов менять не требуется: дополнительные повороты и
сдвиги выполняются аугментациями во время обучения.

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
