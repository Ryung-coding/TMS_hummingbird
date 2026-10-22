## 1. 시작 전 설정

Ubuntu 24.04와 ROS 2 Jazzy 기준이다. ROS 2가 아직 없다면 [공식 Jazzy 설치 절차](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 먼저 완료한다. 이 Workspace를 처음 받은 뒤 필요한 빌드·센서·GUI 의존성을 설치한다.

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  python3-venv python3-pip python3-numpy setserial libxcb-cursor0 libgl1 \
  ros-jazzy-desktop ros-jazzy-dynamixel-sdk \
  ros-jazzy-rosidl-default-generators ros-jazzy-std-srvs ros-jazzy-example-interfaces
sudo usermod -aG dialout "$USER"
```

`dialout` 권한은 로그아웃 후 다시 로그인해야 적용된다. F/T 센서와 Dynamixel은 서로 다른 USB 포트를 사용한다. 포트 이름은 `ls -l /dev/serial/by-id/`로 확인한다.

ROS 2의 Python 패키지를 가상환경에서도 사용할 수 있도록 `--system-site-packages`로 만들고, GUI 패키지만 그 안에 설치한다. 가상환경은 Workspace 밖에 생성된다.

```bash
cd ~/Desktop/tms_hummingbird
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages "$HOME/.venvs/tms_hummingbird"
source "$HOME/.venvs/tms_hummingbird/bin/activate"
python -m pip install --upgrade pip
python -m pip install 'PySide6==6.10.3' 'pyqtgraph==0.14.0'
python -c 'import rclpy, PySide6, pyqtgraph, numpy; print("Python dependencies OK")'
colcon build --symlink-install
```

PX4 DDS를 사용할 때 `MicroXRCEAgent`가 없다면 다음을 한 번 실행한다. 아래 `v2.4.3`은 기본 PX4 XRCE 클라이언트용이며, 펌웨어에서 XRCE v3를 선택했다면 그에 맞는 Agent 버전을 사용한다. [PX4 Agent 설치 안내](https://docs.px4.io/main/en/middleware/uxrce_dds).

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  "$HOME/Desktop/Micro-XRCE-DDS-Agent"
cmake -S "$HOME/Desktop/Micro-XRCE-DDS-Agent" \
  -B "$HOME/Desktop/Micro-XRCE-DDS-Agent/build"
cmake --build "$HOME/Desktop/Micro-XRCE-DDS-Agent/build" --parallel
sudo cmake --install "$HOME/Desktop/Micro-XRCE-DDS-Agent/build"
sudo ldconfig
```

## 2. 런치 방법

PX4 DDS를 사용할 때는 별도 터미널에서 PX4 클라이언트의 전송 방식·포트에 맞춰 Agent를 먼저 실행한다. 로컬 UDP 8888 설정의 명령은 다음과 같다. 센서 또는 Dynamixel만 확인할 때는 Agent가 필요 없다.

```bash
MicroXRCEAgent udp4 -p 8888
```

새 터미널에서 Workspace를 실행한다. `/dev/ttyUSB0`, `/dev/ttyUSB1`은 확인한 실제 포트로 바꾼다. F/T 센서·Dynamixel·PX4 중 연결되지 않은 장치가 있어도 GUI는 실행되며, 연결된 F/T 센서의 그래프는 계속 표시된다.

```bash
cd ~/Desktop/tms_hummingbird
source /opt/ros/jazzy/setup.bash
source "$HOME/.venvs/tms_hummingbird/bin/activate"
source install/local_setup.bash
ros2 launch tms_hummingbird tms_experiment.launch.py \
  ft_port:=/dev/ttyUSB0 dxl_port:=/dev/ttyUSB1
```
