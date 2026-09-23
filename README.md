# Raspberry Pi 5 + Gemini 336L + YOLO Segmentation

面向 Raspberry Pi 5 8GB 的原生 ROS 2 感知栈：官方 Orbbec Gemini 336L 驱动、低频 YOLO26n-seg/NCNN 实例分割，以及基于对齐深度的 3D 目标坐标。

本仓库**不会把整套 ROS 2 或底盘放进 Docker**。推荐把它作为独立 overlay workspace 使用，通过 `ROS_DOMAIN_ID` 与已有底盘节点通信；已有底盘 workspace 不会被修改。`docker/` 仅作为隔离排障备用。

## 数据流与实时性

```text
Gemini 336L ── RGB 30 Hz ───────► 最新帧槽 ─► YOLO NCNN 约 5 Hz ─┬─► mask
       │                                                        ├─► detections_2d
       └──── depth 30 Hz ────────────────┐                      ├─► overlay
                                        └─ 最近时间戳深度 + K ──┴─► objects_3d

Gemini 336L ── depth ─► Orbbec point cloud（独立，完全不经过 YOLO）
```

ROS 图像回调只替换“最新帧”，模型运行在单独线程，队列容量为 1。推理落后时会丢弃旧 RGB 帧，而不会阻塞 depth 或 `/camera/depth/points`。

## 支持环境

- Raspberry Pi 5 8GB，64-bit Ubuntu
- ROS 2 Humble（Ubuntu 22.04）或 Jazzy（Ubuntu 24.04）
- OrbbecSDK ROS 2 Wrapper `v2-main`
- Gemini 336L，官方推荐固件 `1.8.10`
- 默认模型：YOLO26n-seg，NCNN，`320x320`，目标 5 Hz

> 5 FPS 是起始目标，不是对所有散热、系统负载和模型类别配置的硬保证。先用主动散热和 320 输入验证，再尝试 384。

## 最快安装

不要进入或覆盖已有底盘 workspace；新建一个独立目录：

```bash
mkdir -p ~/gemini336l_ws/src
cd ~/gemini336l_ws/src
git clone https://github.com/Rainy-Day04/pi5-gemini336l-yolo-seg.git
cd pi5-gemini336l-yolo-seg
./scripts/install.sh ~/gemini336l_ws
```

安装脚本会：

1. 检查已安装的 Humble/Jazzy；
2. 安装 Orbbec 编译依赖；
3. 在当前独立 workspace 内拉取官方 `OrbbecSDK_ROS2:v2-main`（若系统尚未安装）；
4. 安装 336L udev 规则；
5. 创建带 ROS 系统包可见性的 `.venv`；
6. 构建 workspace；
7. 下载 `yolo26n-seg.pt` 并导出 `320x320` NCNN 模型。

安装器会先从 PyTorch 官方 CPU wheel 索引安装 `torch`/`torchvision`。不要在 Pi 5 上安装 `cuda-toolkit`、`nvidia-cudnn-cu13` 等 NVIDIA 依赖；Pi 5 的 NCNN 路径不使用 CUDA。

模型导出可能需要几分钟。若只想先验证相机，可用 `SKIP_MODEL_EXPORT=1 ./scripts/install.sh ~/gemini336l_ws`。

安装 udev 后拔插一次相机，然后启动：

```bash
./scripts/run.sh all ~/gemini336l_ws
```

只启动相机或只启动感知：

```bash
./scripts/run.sh camera ~/gemini336l_ws
./scripts/run.sh perception ~/gemini336l_ws
```

## 接入已有机械臂（标定参数可稍后填写）

新增 `times_arm_perception` 原生 ROS 2 包，连接已经部署的 timesmanipulator HTTP
服务，默认 `preview` 只读取状态。后续填参数即可切换 `plan` / `execute`；启动本身不会运动。

```bash
bash scripts/build_arm_adapter.sh ~/gemini336l_ws
./scripts/run.sh arm ~/gemini336l_ws mode:=preview
```

独立配置文件、每端命令、标定参数和动作调用见 [机械臂接入说明](docs/arm-integration.md)。

## 与已有底盘 ROS 2 共存

两个 workspace 不需要合并，也不要互相 `colcon build`。只要使用相同的 RMW 网络和 `ROS_DOMAIN_ID` 即可：

```bash
# 底盘终端
export ROS_DOMAIN_ID=23
source ~/base_ws/install/setup.bash
ros2 launch your_base_package base.launch.py

# 感知终端
export ROS_DOMAIN_ID=23
./scripts/run.sh all ~/gemini336l_ws
```

如果同一网络中还有其他机器人，为每台机器人设置不同的 `ROS_DOMAIN_ID`。本仓库不写入 `.bashrc`，也不改底盘 workspace。

## ROS 2 topics

默认输入：

| Topic | Type | 说明 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 640x480 RGB，30 Hz |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 跨机器查看用 JPEG，不影响本机原图推理 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 对齐到 color 的深度 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 用于 3D 反投影的 color 内参 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Orbbec 独立生成，不经过 YOLO |

默认输出：

| Topic | Type | 说明 |
|---|---|---|
| `/perception/gemini336l_yolo_seg/mask` | `sensor_msgs/Image` (`mono16`) | 0 为背景，1..N 为 `instance_id` |
| `/perception/gemini336l_yolo_seg/detections_2d` | `gemini336l_msgs/Object2DArray` | 类别、置信度、bbox、mask 面积 |
| `/perception/gemini336l_yolo_seg/overlay` | `sensor_msgs/Image` (`bgr8`) | 彩色 mask、bbox 与对齐深度距离 |
| `/perception/gemini336l_yolo_seg/overlay/compressed` | `sensor_msgs/CompressedImage` | 跨机器查看用 JPEG overlay；有订阅者时才编码 |
| `/perception/gemini336l_yolo_seg/objects_3d` | `gemini336l_msgs/Object3DArray` | mask 内有效深度中值反投影的 XYZ |

`objects_3d.header.frame_id` 使用 color `CameraInfo` 的 optical frame；ROS 相机坐标约定为 X 向右、Y 向下、Z 向前。每个 3D 对象都有 `position_valid`，无匹配深度时仍保留分类结果但置为 false。

距离取每个实例 mask 内有效对齐深度的中位数。节点通过 TF 把相机坐标转换到 `base_link` 后，每个目标的 overlay 显示 `base x 1.20 y -0.30 z 0.42 m`：x 向前、y 向左、z 向上。`objects_3d.position` 同样位于其 header 指明的坐标系中；TF 暂时不可用时标签使用 `cam` 前缀，XYZ 位于相机光学坐标系。

相机原始图传和 YOLO 分割使用独立频率。下面让 RGB/depth 按 30 FPS 发布，YOLO/overlay 按 5 FPS 运行：

```bash
./scripts/run.sh vision ~/gemini336l_ws \
  camera_fps:=30 inference_hz:=5.0 \
  enable_point_cloud:=false align_mode:=HW
```

`/camera/color/image_raw` 是高帧率原图，`/perception/gemini336l_yolo_seg/overlay` 是带 mask、类别和 XYZ 的低频推理结果。不要把旧 mask 重画到后续原图来伪造高帧率检测结果。
`vision` 与原来的 `all` 都只启动相机和 YOLO，不启动导航、任务协调器或机械臂。
每个检测框都会有第二行坐标；深度、内参或对齐不满足时显示红色 `xyz unavailable`，并在 `objects_3d` 中保持 `position_valid=false`。

### Pi 5 低负载与 50 Mbps 网络推荐配置

Pi 内部继续用原始 RGB/depth 做推理和 XYZ，MiniPC 只订阅一条合成 JPEG。树莓派已经明显卡顿时，推荐先用 RGB/depth 10 FPS、segmentation 3 FPS：

```bash
./scripts/run.sh vision ~/gemini336l_ws \
  camera_fps:=10 inference_hz:=3.0 \
  depth_registration:=true align_mode:=HW \
  enable_frame_sync:=false enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  overlay_jpeg_quality:=50 overlay_max_width:=640 \
  preview_hz:=0.0 retina_masks:=true \
  confidence_threshold:=0.20 max_detections:=15 \
  run_inference_when_unsubscribed:=false
```

MiniPC 只运行一个合成查看器；它同时显示画面、mask、目标框与 XYZ，并支持点击任意像素查询 XYZ：

```bash
ros2 run gemini336l_yolo_seg pixel_picker
```

合成流保留相机原生 640x480，再以 JPEG 质量 50 编码。mask 在原图坐标生成，避免 320x320 letterbox 直接拉伸导致人物与 mask 错位。mask、未压缩 overlay、JPEG 和 2D/3D 消息都按订阅者惰性生成；没有任何检测输出订阅者时 YOLO 自动暂停，但相机与 `query_pixel_3d` 服务仍可用。不要同时打开原图和 overlay 查看器。

### 点击任意像素查询 XYZ

感知节点提供 `/perception/gemini336l_yolo_seg/query_pixel_3d`。MiniPC 上运行可点击查看器：

```bash
ros2 run gemini336l_yolo_seg pixel_picker
```

点选器默认订阅压缩 overlay。若要临时在压缩原图上点选，使用：

```bash
ros2 run gemini336l_yolo_seg pixel_picker --ros-args \
  -p image_topic:=/camera/color/image_raw/compressed \
  -p compressed:=true
```

MiniPC 第一次使用需要克隆同一仓库并只编译接口和查看器（不安装模型）：

```bash
mkdir -p ~/pixel_view_ws/src
cd ~/pixel_view_ws/src
git clone https://github.com/Rainy-Day04/pi5-gemini336l-yolo-seg.git
source /opt/ros/jazzy/setup.bash
cd ~/pixel_view_ws
rosdep install --from-paths \
  src/pi5-gemini336l-yolo-seg/gemini336l_msgs \
  src/pi5-gemini336l-yolo-seg/gemini336l_yolo_seg \
  --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install \
  --base-paths \
    src/pi5-gemini336l-yolo-seg/gemini336l_msgs \
    src/pi5-gemini336l-yolo-seg/gemini336l_yolo_seg \
  --packages-up-to gemini336l_yolo_seg
source ~/pixel_view_ws/install/setup.bash
```

鼠标左键点击任意像素后，窗口冻结点击的那一帧并显示查询结果；按空格恢复实时画面，按 `Q` 或 `Esc` 退出。默认用点击点附近 `5x5` 像素的有效深度中值，但 XYZ 仍沿点击像素对应的相机射线计算。结果优先转换到 `base_link`；窗口和终端都会显示实际 `frame_id`。

没有图形界面时，也可以直接查询最新 RGB 帧的像素 `(320, 240)`：

```bash
ros2 service call \
  /perception/gemini336l_yolo_seg/query_pixel_3d \
  gemini336l_msgs/srv/QueryPixel3D \
  '{image_stamp: {sec: 0, nanosec: 0}, pixel_u: 320, pixel_v: 240, window_radius: 2}'
```

透明、反光、过近/过远或没有有效深度的区域会返回 `valid: false`，不会伪造 XYZ。

单独启动感知节点时必须明确声明相机已经开启 D2C，否则节点会安全地禁用距离，避免把未对齐深度误当成目标距离：

```bash
./scripts/run.sh perception ~/gemini336l_ws depth_aligned_to_color:=true
```

底盘必须发布真实的 `base_link -> camera_link` 安装变换。以下仅展示命令格式，`TX TY TZ ROLL PITCH YAW` 必须换成实测的前、左、上偏移（米）以及三个安装角（弧度）：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x TX --y TY --z TZ --roll ROLL --pitch PITCH --yaw YAW \
  --frame-id base_link --child-frame-id camera_link
```

Orbbec 驱动继续负责 `camera_link -> camera_color_optical_frame`。没有完整 TF 链时，节点保留光学深度但不会冒充导航 x/y。

运行后检查：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gemini336l_ws/install/setup.bash
./scripts/health_check.sh
```

## 336L 配置与 udev

官方 `v2-main` 对 Gemini 336L 使用：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

本仓库的 `camera.launch.py` 额外启用 D2C 和 depth point cloud。为兼容部分 336L 配置并降低 Pi 5 CPU 负载，默认关闭软件/硬件 noise removal filter；同时默认使用 `SW` 对齐，并关闭 frame sync 与 IR auto exposure。升级至官方推荐固件并确认 USB 稳定后可按需重新启用同步和硬件对齐：

```bash
./scripts/run.sh all ~/gemini336l_ws \
  align_mode:=HW \
  enable_frame_sync:=true \
  enable_ir_auto_exposure:=true
```

如果相机反复出现 `Device is deactivated/disconnected status:108`，先运行最小相机模式：

```bash
./scripts/run.sh camera ~/gemini336l_ws \
  depth_registration:=false \
  align_mode:=SW \
  enable_frame_sync:=false \
  enable_ir_auto_exposure:=false \
  enable_point_cloud:=false
```

最小模式稳定而完整模式不稳定，通常是旧固件或某个高级功能的兼容问题；最小模式仍重连则优先排查 USB 线、供电、Hub/USB 控制器，并查看内核 USB 日志。

手动安装 udev 规则：

```bash
cd ~/gemini336l_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

不要用 `sudo ros2 launch` 绕过 udev；这会引入错误的环境和文件权限。

## 升级 Gemini 336L 固件

Gemini 336L 属于 Gemini 330 系列，官方当前推荐固件为 `1.8.10`。仓库提供的升级脚本会下载并校验官方固件，以及官方 SDK2 v2.9.3 ARM64 命令行升级器；不会安装 SDK 到系统，也不会修改 ROS 或底盘 workspace。

先停止正在运行的 Orbbec 相机 launch，并确保相机在没有相机节点访问时不会自行反复断连。底盘节点可以继续运行。然后传入相机机身序列号：

```bash
cd ~/gemini336l_ws/src/pi5-gemini336l-yolo-seg
./scripts/update_firmware_336l.sh CPC8763000VT
```

脚本会先列出识别到的设备，并要求再次输入同一序列号才开始写入。升级过程中可能发生一次或两次正常的 USB 重连；在程序明确完成以前不要拔线、重启或断电。升级完成后重新插拔相机，再用 SDK2 驱动确认日志中的固件版本为 `1.8.10`。

如果不运行脚本，也可以从奥比中光的 [Gemini 330 系列固件页](https://www.orbbec.com/docs/g330-firmware-release/) 下载 `Gemini330_Release_1.8.10.zip`，并使用官方 SDK2 `ob_device_firmware_update` 工具升级。不要使用 Gemini 2、Gemini 340 或其他系列的 `.bin` 文件。

## 模型下载、导出和调参

重新导出 320 模型：

```bash
./scripts/prepare_model.sh ~/gemini336l_ws 320
```

要测试 384，先删除或移动旧的 `models/yolo26n-seg_ncnn_model`，再运行：

```bash
./scripts/prepare_model.sh ~/gemini336l_ws 384
sed -i 's/imgsz: 320/imgsz: 384/' gemini336l_bringup/config/pi5.yaml
cd ~/gemini336l_ws && colcon build --symlink-install --packages-select gemini336l_bringup
```

自训练模型同样可导出：

```bash
source ~/gemini336l_ws/.venv/bin/activate
yolo export model=/path/to/best.pt format=ncnn imgsz=320 batch=1 device=cpu
GEMINI336L_MODEL=/path/to/best_ncnn_model ./scripts/run.sh perception ~/gemini336l_ws
```

YOLO26n-seg 预训练模型识别 COCO 80 类；非 COCO 目标需要自训练。Ultralytics 是外部依赖，并有其自己的许可条款，请在商业发布前核对。

主要参数在 `gemini336l_bringup/config/pi5.yaml`：

- `inference_hz`: 默认 `5.0`
- `imgsz`: 默认 `320`
- `confidence_threshold`: 默认 `0.20`，减少人物检测在相邻帧间闪烁
- `classes`: COCO class id，以逗号分隔，如 `"0,39,56"`
- `max_depth_time_delta_sec`: RGB 与 depth 最大时间差
- `min_depth_m` / `max_depth_m`: 3D 有效深度范围

## Topic remap / 接已有相机 launch

若 Orbbec 已由其他 launch 启动，只运行 perception，并通过 launch 参数指定 topics：

```bash
./scripts/run.sh perception ~/gemini336l_ws \
  color_topic:=/front_camera/color/image_raw \
  depth_topic:=/front_camera/depth/image_raw \
  camera_info_topic:=/front_camera/color/camera_info
```

前提是 depth 已对齐到 color，且分辨率一致。节点不会偷偷缩放或插值深度；不一致时 2D 结果照常发布，3D `position_valid=false`。

## Docker 备用方案

只建议用于依赖隔离或排障；与宿主底盘 DDS 共存需要 host network，并需要把 USB 设备交给容器：

```bash
docker build -f docker/Dockerfile -t gemini336l-perception:local .
./docker/run.sh
```

Docker 方案使用 `--privileged --network host`，安全边界更宽，而且相机 USB 重连和 DDS 排查更复杂，所以不是默认交付形式。

## 多人协作接口：选物 → 导航靠近 → 抓取

给导航/前端同学直接看 **[完整接口约定与两侧启动命令](docs/team-interfaces.md)**。
采用通用 ROS 2，不绑定 Nav2，不改已有底盘 workspace，也不发布 `/cmd_vel`。

| 模块 | 对接入口 | 当前交付 |
|---|---|---|
| 选定目标 | `/robot_task/select_target` | 按图像时间戳 + 实例编号选择，生成任务目标 UUID |
| 执行任务 | `/robot_task/start`、`/robot_task/status`、`/robot_task/cancel` | 导航、停稳、重新观测、抓取的协调器 |
| LiDAR 导航 | `/navigation/approach_target` | 消息/Action + 无运动模板，由导航同学接入自己的导航 |
| 机械臂 | `/arm_perception/pick_target` | 复用现有 HTTP 驱动的适配器，不新增串口占用 |

只构建团队接口和适配节点，不重新下载模型/重编译相机：

```bash
./scripts/build_team_interfaces.sh ~/robot_team_ws
source /opt/ros/jazzy/setup.bash
source ~/robot_team_ws/install/setup.bash
ros2 launch robot_task_coordinator task.launch.py
```

默认 `motion_enabled=false`、`calibration_confirmed=false`，只允许选目标/检查接口。
真实导航后端、外参、臂展和安全检查尚需对应同学接入/实测，不能仅打开开关就当作完成标定。
底盘停稳后必须取得新的观测；目标缺失/歧义、旧时间戳或丢失运动反馈时不进入抓取。
完成动作但没有夹持传感器确认时只返回 `SEQUENCE_COMPLETE`。

## 仓库结构

```text
gemini336l_msgs/          2D/3D、目标、任务状态与导航/抓取 Action
gemini336l_yolo_seg/      非阻塞分割与 RGB-D 投影节点
gemini336l_bringup/       camera/perception/all launch 与 Pi 5 参数
robot_task_coordinator/   选定目标、导航到抓取的任务协调器（默认不运动）
times_arm_perception/    现有机械臂 HTTP 服务适配与 PickTarget Action
examples/                 导航同学接入自己后端的模板
docs/team-interfaces.md   多人对接协议与启动/联调说明
scripts/                  安装、模型导出、启动、健康检查
docker/                   可选备用容器
```

## 故障排查

### SDK2 报 `G330FrameUnpacker` / `depth frame processor status:114`

OrbbecSDK 在 Linux 上只扫描扩展目录中的普通文件。`colcon --symlink-install`
会把私有滤镜库安装为软链接，因此文件虽然能被 `ldd`/`ctypes` 加载，
仍会被 SDK 枚举器忽略。安装器已自动将这两个插件转换为普通文件。
既有 workspace 可直接修复，无需重编译：

```bash
./scripts/fix_orbbec_symlink_plugins.sh ~/gemini336l_ws
```

修复后脚本必须显示两个 `regular file`，然后重启相机进程。

### 固件 1.4.60 与 SDK2 v2.4.3 报 `NoiseRemovalFilter#1 doesn't exist`

部分 Gemini 336L 会报告旧版降噪属性，但相机中没有对应的可选滤波配置。
OrbbecSDK 2.4.3 即使在关闭滤波器时仍会查询该配置，导致 RGB/Depth topic
无法启动。保持滤波器关闭，并应用仓库内的窄范围兼容补丁：

```bash
cd ~/gemini336l_ws/src/pi5-gemini336l-yolo-seg
./scripts/patch_orbbec_v243.sh ~/gemini336l_ws

cd ~/gemini336l_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to orbbec_camera \
  --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

补丁只会在 `enable_noise_removal_filter:=false` 时跳过两个可选降噪参数查询；
原始 RGB 和 Depth 处理仍然启用。

- **看不到相机**：拔插 USB 3 线；运行 `ros2 run orbbec_camera list_devices_node`；重装 udev。
- **`apt update` 提示 `Mirror sync in progress`**：这是所选镜像站正在同步，不是本仓库损坏。安装脚本会自动重试；仍失败时等待镜像同步完成，或只把 ROS 2 软件源切换到另一个可信镜像后重跑。已有 apt 索引确定可用时，也可用 `SKIP_APT_UPDATE=1 ./scripts/install.sh ~/gemini336l_ws` 跳过更新。
- **克隆 Orbbec 驱动时出现 `early EOF` / `Connection reset by peer`**：安装器会用 HTTP/1.1 自动重试，并在 GitHub 失败后切换官方 Gitee 镜像。也可直接指定：`ORBBEC_REPOSITORY_URL=https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS2.git ./scripts/install.sh ~/gemini336l_ws`。不完整的目录会先改名保留，不会直接删除。
- **pip 开始下载 `nvidia-cudnn-cu13` / `cuda-toolkit`**：立即取消并 `git pull`。Pi 5 没有 NVIDIA GPU；新版安装器会固定官方 CPU-only PyTorch、清理专用 venv 中残留的 `nvidia-*-cu*`/`cuda-*` 包与对应下载缓存，并验证 `torch.version.cuda is None`。
- **无 3D 坐标**：确认 D2C 已开启、RGB/depth 尺寸一致，且 `/camera/color/camera_info` 存在。
- **推理低于 5 FPS**：先用 320、限制 `classes`/`max_detections`、开启主动散热；确认加载的是 `_ncnn_model` 而不是 `.pt`。
- **底盘节点互相不可见**：两边检查 `echo $ROS_DOMAIN_ID` 与 `echo $RMW_IMPLEMENTATION`。
- **点云频率下降**：先停 perception 对照测试；本节点不订阅点云，若仍下降重点检查 USB3、供电、温度和 Orbbec profile。

## 上游资料

- [OrbbecSDK ROS 2 Wrapper](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)
- [Orbbec ROS 2 topics](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/topics.html)
- [Ultralytics Raspberry Pi guide](https://docs.ultralytics.com/guides/raspberry-pi/)
- [Ultralytics segmentation models](https://docs.ultralytics.com/tasks/segment/)

License: Apache-2.0（不覆盖第三方依赖和模型的许可证）。
