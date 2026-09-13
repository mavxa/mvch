sudo apt update
sudo apt install -y git curl wget unzip xz-utils snapd locales software-properties-common ca-certificates gnupg python3 python3-pip python3-venv python3-dev build-essential openssh-client ffmpeg libgl1 libglib2.0-0 mesa-utils v4l-utils iputils-ping pciutils ubuntu-drivers-common

sudo snap install code --classic
export PATH="/snap/bin:$PATH"
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-python.vscode-python-envs
code --install-extension ms-toolsai.jupyter
code --install-extension ms-vscode-remote.remote-ssh
code --install-extension ms-iot.vscode-ros
code --install-extension redhat.vscode-yaml
code --install-extension bradlc.vscode-tailwindcss
code --install-extension dbaeumer.vscode-eslint
code --install-extension esbenp.prettier-vscode

curl -fsSL https://bun.com/install | bash
export BUN_INSTALL="$HOME/.bun"
export PATH="$BUN_INSTALL/bin:$PATH"

curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.7/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
nvm install --lts
nvm use --lts

curl -fsSL https://opencode.ai/install | bash

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-rmw-fastrtps-cpp ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit ros-jazzy-tf2-tools ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-webots-ros2 python3-colcon-common-extensions python3-rosdep ros-dev-tools

sudo install -d /etc/apt/keyrings
sudo wget -q -O /etc/apt/keyrings/Cyberbotics.asc https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update
sudo apt install -y webots

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi
rosdep update

mkdir -p "$HOME/ros2_ws/src"
cd "$HOME/ros2_ws/src"
test -d ar_webots_fms_ros2/.git || git clone https://gitlab.mobird.dev/fms_group/ar_webots_fms_ros2.git
test -d ar_arm95_moveit_config/.git || git clone https://gitlab.mobird.dev/fms_group/ar_arm95_moveit_config.git
test -d ar_aruco_detect_ros2/.git || git clone https://gitlab.mobird.dev/fms_group/ar_aruco_detect_ros2.git
test -d ar_dual_lidar_merge_ros2/.git || git clone https://gitlab.mobird.dev/fms_group/ar_dual_lidar_merge_ros2.git
test -d ar_nav_ros2/.git || git clone https://gitlab.mobird.dev/fms_group/ar_nav_ros2.git

cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src --rosdistro jazzy
colcon build --symlink-install
source "$HOME/ros2_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

MVCH_DIR="$(git rev-parse --show-toplevel)"
cd "$MVCH_DIR/module3"
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/pip install -r requirements.txt tqdm ultralytics-thop

cd "$MVCH_DIR/module4"
bun install --frozen-lockfile
bun test
bun run build

cd "$MVCH_DIR"
git status --short
git --version
code --version
python3 --version
node --version
npm --version
bun --version
opencode --version
ros2 --help >/dev/null
webots --version
nvidia-smi
glxinfo -B | grep -E 'OpenGL vendor|OpenGL renderer'
source "$MVCH_DIR/module3/.venv/bin/activate"
python3 -c "import numpy, cv2, torch, ultralytics, lark, transforms3d, rclpy, cv_bridge; print('Ultralytics', ultralytics.__version__); print('CUDA', torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
ros2 node list
ros2 topic list
