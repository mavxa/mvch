sudo apt update
sudo apt install -y git curl wget unzip xz-utils snapd ca-certificates python3 python3-pip python3-venv python3-dev build-essential openssh-client ffmpeg libgl1 libglib2.0-0 mesa-utils v4l-utils iputils-ping pciutils

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

source /opt/ros/jazzy/setup.bash
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
ros2 --help >/dev/null
webots --version
nvidia-smi
glxinfo -B | grep -E 'OpenGL vendor|OpenGL renderer'
source "$MVCH_DIR/module3/.venv/bin/activate"
python3 -c "import numpy, cv2, torch, ultralytics, lark, transforms3d, rclpy, cv_bridge; print('Ultralytics', ultralytics.__version__); print('CUDA', torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
ros2 node list
ros2 topic list
