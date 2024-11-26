#FROM ghcr.io/epicgames/unreal-engine:dev-slim-5.4.3
FROM ghcr.io/epicgames/unreal-engine:dev-slim-5.3.2

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  sudo apt-get update -y -qq && \
  sudo apt-get upgrade -y -qq --no-install-recommends && \
  sudo apt-get install -y -qq \
    libyaml-cpp-dev curl software-properties-common \
    vim v4l-utils exfat-* \
    openssh-server \
    terminator dbus-x11 \
    python3-pip && \
  : "remove cache" && \
  sudo apt-get autoremove -y -qq && \
  sudo rm -rf /var/lib/apt/lists/*

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
    sudo apt update && \
    sudo apt install -y -qq cmake

# for ros2
RUN sudo ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN sudo apt -y -qq update && sudo apt -y -qq install software-properties-common mesa-utils libglu1-mesa-dev\
  freeglut3-dev mesa-common-dev libglew-dev libglfw3-dev libglm-dev \
  libao-dev libmpg123-dev libasio-dev && \
  sudo add-apt-repository universe && \
  sudo apt -y -qq update && sudo apt install curl -y && \
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  sudo apt -y -qq update && \
  sudo apt -y -qq upgrade && \
  sudo apt -y -qq install ros-humble-desktop \
    ros-humble-ament-cmake ros-humble-angles ros-humble-controller-manager \
    ros-humble-pluginlib ros-humble-urdf ros-humble-yaml-cpp-vendor ros-humble-joint-state-pub* \
    ros-humble-cv-bridge ros-humble-diagnostic-updater \
    ros-humble-moveit && \
  sudo apt-get install -y -qq ros-humble-xacro ros-humble-realtime-tools ros-humble-control-toolbox ros-humble-ros2-control* ros-humble-ros-testing \
    ros-humble-teleop-twist-keyboard ros-humble-vision-msgs ros-humble-tf-transformations && \
  : "remove cache" && \
  sudo apt-get autoremove -y -qq && \
  sudo rm -rf /var/lib/apt/lists/*

RUN pip3 install transforms3d
  
RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    telnet \
    && sudo apt-get clean

# 必要なパッケージをインストール
RUN sudo apt-get update && \
    sudo apt-get install -y wget apt-transport-https software-properties-common

# VS CodeのGPGキーをインポート
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/ && \
    sudo rm -f packages.microsoft.gpg

# VS Codeリポジトリを追加
RUN sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'

# VS Codeをインストール
RUN sudo apt-get update && \
    sudo apt-get install -y code

#RUN sudo echo '#!/bin/bash\n /usr/bin/code --user-data-dir="/root/.config/Code" --no-sandbox "$@"' | sudo tee /usr/local/bin/code && \
#    sudo chmod +x /usr/local/bin/code

RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    iproute2 \
    && sudo apt-get clean

RUN sudo apt-get update
RUN sudo apt-get install -y xz-utils

# blenderのバイナリはいい感じに持ってくる
RUN mkdir /tmp/blender \
  && cd /tmp/blender \
  && wget https://mirror.freedif.org/blender/release/Blender4.2/blender-4.2.1-linux-x64.tar.xz \
  && tar -xvf "./blender-4.2.1-linux-x64.tar.xz"

# blender依存関係のインストール
RUN sudo apt-get install glibc-source -y
RUN sudo apt-get install -y libsm6 libxext6
RUN sudo apt-get install -y libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev libxrandr-dev libxinerama-dev libegl-dev
RUN sudo apt-get install -y libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev linux-libc-dev

# blender本体のインストール
ENV BLENDER_DIR="/usr/local/blender"
ENV BLENDER_PATH="${BLENDER_DIR}/blender"
ENV BLENDER_PYTHON_PATH="${BLENDER_DIR}/4.2/python/bin/python3.11"
RUN sudo cp -r /tmp/blender/blender-4.2.1-linux-x64 ${BLENDER_DIR}
ENV PATH="${BLENDER_PATH}:${PATH}"

# pythonのセットアップ
RUN ${BLENDER_PYTHON_PATH} -m ensurepip \
  && ${BLENDER_PYTHON_PATH} -m pip install --upgrade pip \
  && ${BLENDER_PYTHON_PATH} -m pip install pyyaml \
  && ${BLENDER_PYTHON_PATH} -m pip install rospkg \
  && ${BLENDER_PYTHON_PATH} -m pip install urdf_parser_py

RUN cd /tmp \
  && git clone https://github.com/HoangGiang93/urdf_importer.git \
  && cd urdf_importer \
  && zip -r urdf_importer_addon.zip .

#RUN echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/root/colcon_ws/src/isaac_ros2_utils/isaac_ros2_scripts/config/fastdds.xml' >> ~/.bashrc
#RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc

RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
RUN echo 'ip link set lo multicast on' >> ~/.bashrc

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source ~/colcon_ws/install/setup.sh' >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
