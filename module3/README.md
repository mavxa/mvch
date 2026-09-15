# Модуль В: датасет и YOLO

Максимально простой сценарий находится в [SIMPLE.md](SIMPLE.md). Один
`simple_module_b.py` показывает YOLO в OpenCV и сам открывает и закрывает захват.

Камера Webots: `/RMC1/arm95/camera_gripper/image_color`. На физическом RMC1 новая
документация указывает `/RMC1/arm95/svcam/right/image/compressed` с типом
`sensor_msgs/msg/Image` — несмотря на суффикс `compressed`.

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
module3/.venv/bin/python module3/module3.py --target 1 --sim
```

Для Webots `--sim` обязателен: он выбирает симуляторную камеру, глобальные TF и
`use_sim_time`.

На физическом RMC1 `--sim` не нужен. До запуска проверьте фактический тип камеры:

```bash
ros2 topic info /RMC1/arm95/svcam/right/image/compressed -v
ros2 topic echo /RMC1/arm95/svcam/right/camera_info --once
ros2 run tf2_ros tf2_echo Base_link ИМЯ_FRAME_КАМЕРЫ
```

Если `ros2 topic info` вопреки PDF показывает
`sensor_msgs/msg/CompressedImage`, добавьте `--compressed`. Если `CameraInfo`
публикуется под другим именем, задайте `--camera-info-topic`. При полном отсутствии
`CameraInfo` можно передать калибровку камеры вручную:

```bash
module3/.venv/bin/python module3/module3.py --target hammer \
  --fx FX --fy FY --cx CX --cy CY
```

Без CameraInfo либо четырёх параметров калибровки скрипт распознает класс, но
намеренно не двигает ARM95: по одному bounding box нельзя получить метрические
координаты детали.

MoveIt-конфигурация в скрипте повторяет выданный
`moveit_api_example.launch.py`: `arm95_webots.urdf`, группы `arm95_group` и
`gripper`, namespace `/RMC1/arm95`. Для физического запуска меняется только
`use_sim_time=false`. Если на площадке организаторы выдадут обновлённый launch или
URDF для железа, он имеет приоритет — сначала сравните его с этим примером.

Цели: `1/hammer`, `2/wrench`, `3/pliers`. После того как эксперт зафиксировал
видеопоток, нажмите `G` или `Space` в окне либо введите `g` и
нажмите Enter в терминале. `Esc` останавливает скрипт до начала движения.

Сначала обязательно проверьте только координаты:

```bash
module3/.venv/bin/python module3/module3.py --target hammer --sim --dry-run
```

В режиме `--dry-run` подтверждать запуск клавишей не нужно: скрипт завершится
после первой валидной детекции и вывода рассчитанных координат.

Для проверки через SSH без окна:

```bash
module3/.venv/bin/python module3/module3.py --target hammer --sim --dry-run --no-window
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
python3 module3/capture_dataset.py --sim --count 30
```

На реальном RMC1 `capture_dataset.py` по умолчанию использует новую камеру. Если
её реальный тип окажется `CompressedImage`, добавьте `--compressed`.

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
