ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV VIRTUAL_ENV=/opt/gemini-venv
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake git libdw-dev libgflags-dev libgl1 libgoogle-glog-dev \
    libssl-dev libusb-1.0-0-dev mesa-utils nlohmann-json3-dev \
    python3-colcon-common-extensions python3-pip python3-rosdep python3-venv \
    ros-${ROS_DISTRO}-backward-ros \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-statistics-msgs \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
RUN git clone --depth 1 --branch v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git
COPY . /ws/src/pi5-gemini336l-yolo-seg

RUN python3 -m venv --system-site-packages "${VIRTUAL_ENV}" \
    && pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu \
        'torch==2.10.0+cpu' 'torchvision==0.25.0+cpu' \
    && pip install --no-cache-dir -r /ws/src/pi5-gemini336l-yolo-seg/requirements.txt \
    && python -c 'import torch; assert torch.version.cuda is None'
RUN rosdep init 2>/dev/null || true
RUN rosdep update \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep install --from-paths /ws/src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /ws
CMD ["bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ws/install/setup.bash && ros2 launch gemini336l_bringup all.launch.py model:=/models/yolo26n-seg_ncnn_model"]
