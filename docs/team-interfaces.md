# 多人对接：视觉选物 → 底盘靠近 → 到位复核 → 机械臂抓取

这是通用 ROS 2 接口，不要求导航团队使用 Nav2。相机、导航、机械臂分别保留各自实现，
只对齐下面的消息、Action、坐标与停止条件。机械臂安装参数暂时待定也可以先编译、接通接口。
本仓库已提供协调器、消息接口、机械臂适配器和可运行的导航接口模板；**真实导航后端仍由导航同学接入**，
模板不是避障/导航算法，没有自动向 `/cmd_vel` 发指令。

**默认不允许运动。** `robot_task_coordinator/config/pending.yaml` 中
`motion_enabled: false`、`calibration_confirmed: false`。占位外参、临时零 TF、示例臂展距离
都不是实机标定结果；接口联调通过也不等于可以直接开始抓取。

```text
相机 / YOLO              任务协调器                  导航同学                 机械臂适配器
objects_3d ──选定一帧──→ 固定目标世界坐标
                         approach_target ─────────→ 规划安全停靠位并导航
                         ←────────── 到位且已停止 ──┘
                         检查 /odom 持续静止
新一帧 objects_3d ─────→ 重新识别同一位置的目标
                         pick_target ────────────────────────────────────→ 检查可达性并抓取
                         ←─────────────────────────────────────────────── 返回序列/抓取结果
```

## 1. 每个人负责什么

| 模块 | 提供 | 不负责 |
|---|---|---|
| 视觉（Pi） | RGB / depth 对齐、目标类别与三维观测、时间戳和坐标系 | 不发布底盘速度，不以相机坐标直接控制底盘 |
| 协调器（运行一份即可） | 选定目标、目标锁定、阶段切换、静止检查、到位后复核、超时/取消 | 不做导航避障，也不直接发布 `/cmd_vel` |
| LiDAR 导航同学 | `/navigation/approach_target` Action、定位 TF、`/odom`、安全停靠位、取消后停车 | 不把物体坐标当底盘终点，不直接触发机械臂 |
| 机械臂适配器（Pi） | `/arm_perception/pick_target` Action、运动学/范围检查、执行与保持 | 不替导航团队停车，不默认声称“已经夹住” |

运行位置可以调整；接口不依赖 IP。底盘必须有唯一运动控制权：自主导航、遥控、其他导航节点
不能同时向控制器抢发速度。机械臂同理，执行期间停止原 GUI 点动、Home、轨迹等操作。
本仓库没有替现有底盘或机械臂实现跨客户端控制租约，团队需要在驱动/控制网关处保证互斥。

## 2. 坐标与时间：先对齐再写适配

统一米、弧度、秒。推荐 TF 链：

```text
map → odom → base_link → camera_link → camera_color_optical_frame
                     └→ arm_base_link
```

- 导航提供 `map → odom → base_link`；若当前只使用 `odom` 作固定世界系，双方把 `world_frame`
  一起设为 `odom`，不能一方用 `map` 另一方用 `odom`。
- 安装外参提供 `base_link → camera_link` 和 `base_link → arm_base_link`；每条 TF 只允许一个发布者。
- `base_link`：X 向前、Y 向左、Z 向上。相机 optical frame 通常是 X 向右、Y 向下、Z 向前，不能混用。
- 相机必须确实将 depth 对齐 color，感知的 `depth_aligned_to_color` 设置必须与驱动匹配。
- 用**图像原始时间戳**查 TF，不用“最新 TF”代替；两台主机保持时钟同步。
  `use_sim_time` 也必须一致；实机一般为 false。

`gemini336l_msgs/msg/Target`：

| 字段 | 约定 |
|---|---|
| `target_id` | 选定时分配的 UUID；同一次任务始终保持不变 |
| `source_instance_id` | 来源那一帧的实例编号，只能和图像时间戳组合使用；不是持续跟踪 ID |
| `class_id/class_name/confidence` | 来源观测的类别与置信度 |
| `world_point` | 物体在固定世界系中的三维位置；header.stamp 是该次观测的图像时间 |
| `base_point` | 同一观测时刻，物体相对底盘的三维位置；车辆运动后会过时 |
| `planar_distance_m` | `hypot(base_point.x, base_point.y)`，不是相机射线距离 |
| `bearing_rad` | `atan2(base_point.y, base_point.x)`，左正右负 |

**导航用 `world_point`，不追着旧 `base_point` 走。物体位置也不是底盘目标位置。**
导航应结合地图、车体轮廓、障碍物、目标朝向、安装偏置和机械臂可达范围，选物体周围的安全停靠位。
`stand_off_m` 只是希望保留的距离，不能单独证明物体落在臂展内；到位后仍有机械臂可达性检查。

当前关联方法是“同类别 + 固定世界位置附近”，不是视觉身份跟踪。目标移动、重定位导致坐标跳变、
同类物体靠得太近时必须失败/重选，不能悄悄替换成另一个物体。

## 3. 对接入口

| 名称 | 类型 | 提供方 / 用途 |
|---|---|---|
| `/perception/gemini336l_yolo_seg/objects_3d` | `gemini336l_msgs/msg/Object3DArray` | 视觉：连续观测 |
| `/robot_task/select_target` | `gemini336l_msgs/srv/SelectTarget` | 协调器：按精确时间戳和实例编号选定 |
| `/robot_task/selected_target` | `gemini336l_msgs/msg/Target` | 协调器：发布已选目标，保留最后一条 |
| `/robot_task/start` | `gemini336l_msgs/srv/StartTask` | 协调器：用目标 UUID 明确启动任务 |
| `/robot_task/cancel` | `std_srvs/srv/Trigger` | 协调器：请求取消，不代表已停车 |
| `/robot_task/status` | `gemini336l_msgs/msg/TaskStatus` | 协调器：状态约 2 Hz，保留最后一条 |
| `/navigation/approach_target` | `gemini336l_msgs/action/NavigateToTarget` | **导航同学实现 ActionServer** |
| `/arm_perception/pick_target` | `gemini336l_msgs/action/PickTarget` | 本仓库机械臂适配器实现 |
| `/odom` | `nav_msgs/msg/Odometry` | 底盘：必须是持续更新的实际运动反馈 |

### 导航 Action

Goal：`target`、`stand_off_m`、`position_tolerance_m`、`yaw_tolerance_rad`。
Feedback：`phase`（例如 `PLANNING/NAVIGATING/STOPPING`）、`remaining_distance_m`。
Result：`success`、`code`、`message`、`reached_base_pose`。

导航适配器必须遵守：

1. 校验坐标系、单位、数值与目标合法性；使用配置的唯一固定世界系。
2. 选安全的**底盘停靠姿态**，不要把 target.world_point 直接交给导航器当终点。
3. 只有导航终止、停靠误差满足要求且底盘实际停止，才能 Action `SUCCEEDED` 并返回 `success=true`。
   `reached_base_pose` 是同一固定世界系中的实际底盘位姿，包含新鲜时间戳与单位四元数。
   协调器还会验证此位姿距物体的平面距离和朝向是否在请求容差内；默认时间戳需在 0.5 秒内。
4. 导航失败必须返回失败；取消时先取消底层待发/在途目标并停车，然后才能返回 `CANCELED`。
   接受 cancel 请求只是开始处理，不是停车成功。
5. 无法确认停车时返回明确失败（如 `STOP_NOT_CONFIRMED`），保持阻塞并要求人工介入。
   网络断开、节点崩溃时需要驱动侧 watchdog / 急停，ROS 取消不是硬件急停。

模板见 [`examples/navigation_adapter_template.py`](../examples/navigation_adapter_template.py)。
它默认只返回 `NAV_ADAPTER_NOT_CONFIGURED`，**不发速度、不伪造导航成功**。
导航同学实现其中 `begin/poll/stop/is_stopped` 后端即可接自己的导航。
`begin/poll/stop` 要快速返回，内部使用异步导航客户端；不要在回调里递归 spin 等待自己的 future。
模板若遇到停车无法确认会锁住后续目标，必须人工检查后重启，不提供“忽略故障继续”开关。

### 机械臂 Action

Goal：导航完成、底盘静止并重新观测后的 `Target`，沿用原 `target_id`。
Feedback：当前 `phase`。Result：`success`、`grasp_verified`、`code`、`message`。

必须精确执行给定目标，不能退化成“抓当前最近的物体”。机械臂适配器仍检查新鲜观测、
目标偏移、相机到机械臂 TF、标定开关、运动学开关、反馈和工作范围。`preview/plan` 模式不允许运动。

`success=true, grasp_verified=false` 仅表示运动序列完成，协调器记为 `SEQUENCE_COMPLETE`；
只有另有传感器支持的夹持确认才可 `grasp_verified=true` 并进入 `COMPLETED`。
当前适配器没有力觉/夹持检测，不会伪报物体已抓住。

## 4. 导航同学只需安装接口包

在**导航同学的 Ubuntu + ROS 2 Jazzy 主机**上执行。不安装相机 SDK、不安装 YOLO / torch，
也不构建 Pi 相机驱动。下面新建独立工作区，不覆盖既有底盘工作区：

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/robot_interfaces_ws/src
cd ~/robot_interfaces_ws/src
git clone https://github.com/Rainy-Day04/pi5-gemini336l-yolo-seg.git
cd ~/robot_interfaces_ws
rosdep install --from-paths src/pi5-gemini336l-yolo-seg/gemini336l_msgs \
  --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install \
  --base-paths src/pi5-gemini336l-yolo-seg/gemini336l_msgs \
  --packages-select gemini336l_msgs
source ~/robot_interfaces_ws/install/local_setup.bash
ros2 interface show gemini336l_msgs/action/NavigateToTarget
```

此后在导航启动终端，先 source 原底盘环境，再 source 上面的 `install/local_setup.bash`。
统一使用同一版本 `gemini336l_msgs`，避免另一个工作区中的旧同名包覆盖新 Action。

运行无运动模板，检查对接端点：

```bash
python3 ~/robot_interfaces_ws/src/pi5-gemini336l-yolo-seg/examples/navigation_adapter_template.py
```

另一个相同 ROS 环境的终端：

```bash
ros2 action list -t
ros2 action info /navigation/approach_target
```

同名真实导航适配器上线前先关掉模板；同一个 Action 只允许一个 Server。

## 5. 树莓派启动与选定目标

相机 + YOLO 沿用已跑通的方式；不要为接口升级重新安装模型。协调器和机械臂节点分别启动，
标定未完成时两者都保持非运动模式。下面只编译接口、协调器与机械臂适配器到单独工作区，
不重编译 Orbbec、不重新安装 YOLO，也不改变现有底盘工作区。

在树莓派准备个人参数并启动协调器：

```bash
cd ~/gemini336l_ws/src/pi5-gemini336l-yolo-seg
git -c http.version=HTTP/1.1 pull --ff-only
./scripts/build_team_interfaces.sh ~/robot_team_ws
source /opt/ros/jazzy/setup.bash
source ~/robot_team_ws/install/setup.bash
mkdir -p ~/robot_config
cp -n robot_task_coordinator/config/pending.yaml ~/robot_config/task.yaml
ros2 launch robot_task_coordinator task.launch.py config:="$HOME/robot_config/task.yaml"
```

`cp -n` 不覆盖自己的参数。启动本身不会导航或抓取；未启用运动时 `start` 会拒绝。
其他用于本接口的终端也 source `~/robot_team_ws/install/setup.bash`，不要随后又 source
旧版本消息包把新接口覆盖掉。机械臂校准参数见 [机械臂文档](arm-integration.md)。
在另一个树莓派终端启动非运动的机械臂 ActionServer：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_team_ws/install/setup.bash
cd ~/gemini336l_ws/src/pi5-gemini336l-yolo-seg
mkdir -p ~/robot_config
cp -n times_arm_perception/config/arm.pending.yaml ~/robot_config/arm.yaml
ros2 launch times_arm_perception integration.launch.py \
  mode:=preview config:="$HOME/robot_config/arm.yaml"
```

`task.yaml` 中主要联调参数（修改文件后重启协调器）：

| 参数 | 默认值 / 约定 |
|---|---|
| `world_frame / base_frame` | `map / base_link`，与定位、外参保持一致 |
| `motion_enabled / calibration_confirmed` | 都为 false；实机确认前不要改成 true |
| `navigation_action / pick_action` | `/navigation/approach_target` / `/arm_perception/pick_target` |
| `odom_topic` | `/odom`；`child_frame_id` 必须为配置的 `base_frame` |
| `stand_off_m` | 0.35 m，仅示例；按实际安装与可达范围调整 |
| `position_tolerance_m / yaw_tolerance_rad` | 0.05 m / 0.15 rad；停靠距离与面向目标的误差 |
| `observation_max_age_sec / odom_max_age_sec` | 1.0 s / 0.5 s；图像与里程计新鲜度 |
| `stopped_linear_mps / stopped_angular_rps / stop_settle_sec` | 0.02 m/s / 0.03 rad/s / 连续 1.0 s |
| `association_radius_m / confirmation_frames` | 0.06 m / 至少 2 个新的到位后观测帧 |
| `navigation_timeout_sec / reacquire_timeout_sec / pick_timeout_sec` | 120 s / 8 s / 60 s |
| `stop_timeout_sec / cancel_timeout_sec` | 10 s / 5 s；未确认安全结束则保持阻塞 |

预览选物也需要实际存在的 `map` 和 `base_link` TF；缺定位/外参时会明确拒绝。
不要用虚构的零变换绕过检查来启动运动。

前端选中目标时应提交**那一帧**的 `header.stamp` 和 `objects[i].instance_id`。
现有简单图像查看器只用于查看；它不会因为点击图像就自动调用选定服务。UI 同学需要接此服务。
手工联调优先用小工具捕获新帧并自动填时间戳，避免人工复制时过期：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_team_ws/install/setup.bash
ros2 run robot_task_coordinator select_target --class-name bottle
```

默认只选定，不运动；有多个满足条件的目标会拒绝，不擅自选最近物体。
有多个同类目标时请让 UI 调用下面的原始服务，精确提交那一帧的时间戳和实例编号。
工具不提供单独的 `--instance-id` 快捷参数，避免把之前看到的编号套到新帧上。
工具的 `--start` 是明确请求启动任务，只应在标定、安全与控制权均确认后使用。

以下原始服务示例供 UI / 其他程序对接。先查看观测和服务定义：

```bash
ros2 topic echo /perception/gemini336l_yolo_seg/objects_3d --once
ros2 interface show gemini336l_msgs/srv/SelectTarget
```

取最近一条 `position_valid: true` 的目标，把实际数值填入以下请求；不要原样使用示例时间戳：

```bash
ros2 service call /robot_task/select_target gemini336l_msgs/srv/SelectTarget \
  '{observation_stamp: {sec: 1789711792, nanosec: 597358000}, instance_id: 1}'
```

时间戳为 0、缓存中没有这一帧、观测过期、目标不合法或 TF 缺失都会拒绝。人工复制过慢时应重新取帧；
正式 UI 从同一份接收缓存直接调用，不能只传实例编号让后端猜当前物体。
响应 `accepted=true` 后保存响应中的 `target.target_id`。确认标定、控制权与安全后，启用运动配置、
重启相应节点，再选定一个新鲜目标并发起：

```bash
ros2 service call /robot_task/start gemini336l_msgs/srv/StartTask \
  "{target_id: '替换为刚才响应的目标UUID'}"
```

`accepted=true` 只表示任务已受理，**不是导航/抓取成功**。查看全过程与取消：

```bash
ros2 topic echo /robot_task/status --qos-durability transient_local
# 另一个终端请求取消：
ros2 service call /robot_task/cancel std_srvs/srv/Trigger '{}'
```

## 6. 状态与联调验收

正常阶段：`SELECTED → NAVIGATING → WAITING_FOR_STOP → REACQUIRING → GRASPING`。
结束是 `SEQUENCE_COMPLETE`（只完成序列）或 `COMPLETED`（有抓取确认）。
`FAILED` 表示失败；`CANCELING` 是等待下游停止确认；`CANCELED` 才表示取消结束。
`HOLD_REQUIRED` 表示下游安全停止无法确认，需要人工检查；不能在未确认停稳时开始新任务。
`task_id` 标识一次启动，`target_id` 标识选定物体；界面与日志必须一起记录，不要串用旧结果。
`selected_target` 保留最后一条用于查看，不代表当前仍可执行；是否可启动以 `status` 为准。
动作超时暂时无法确认时保持锁定，迟到的终止结果与新鲜静止反馈可使取消收尾；
明确的 `HOLD_FAILED` / `STOP_NOT_CONFIRMED` 则持续锁定，必须人工检查硬件后重启协调器及故障控制器。

协调器不只相信导航成功标志，还检查新鲜 `/odom` 的线/角速度连续静止；到位后重新取图、
按世界坐标关联原目标，默认至少连续确认 2 个新帧。同类候选不唯一、目标消失、定位 TF 缺失
或观测过期时不抓取。启动任务前也要求已有新鲜静止里程计。
抓取阶段继续检查底盘运动反馈；底盘恢复运动或反馈失联会取消机械臂并报告失败/保持要求。
速度检查不是底盘位置精度证明，也不是安全认证。

上线前按此顺序验收：

1. **只通接口：** 运动开关保持 false。双方能看到 action/type，选定坐标正确，start 明确拒绝运动。
2. **软件仿真：** 用独立 ROS_DOMAIN_ID 测试正常、拒绝、超时、取消、目标丢失、歧义与旧时间戳；
   模拟导航/机械臂绝不能与真机 Action 同名同域运行。
3. **仅导航：** 机械臂保持非运动，验证安全停靠位、朝向、误差、实际停车和取消。
4. **标定与静态抓取：** 固定底盘，实测安装 TF、运动学、TCP、夹持点和工作范围；人工保留急停。
5. **全链：** 低速、空旷环境、单一易辨识目标；确认到位后的新图像再抓，记录状态与最终夹持结果。

两机 DDS 配置沿用已经跑通的底盘设置：同一个 `ROS_DOMAIN_ID`、兼容的 RMW、允许跨机发现，
不能一侧 LOCALHOST。必要时用 `ROS_STATIC_PEERS` 指向另一台实际 IP。
环境变量要在启动相机/YOLO/协调器/导航/机械臂**之前**设置；仅在新终端 export 不会改变已有进程。
不要为了联调盲目改底盘域为 0，也不要用重启底盘节点解决接口类型未 source 的问题。

本协议只负责模块间协调。真实障碍物避碰、底盘看门狗、机械臂自碰撞/环境碰撞、
急停和抓取确认仍需对应模块实现；这些能力不能由一个 ROS 消息或“成功”日志替代。

## 7. 无硬件回归测试

仓库包含模拟导航 Action、模拟机械臂 Action/HTTP 服务和虚拟里程计/视觉观测的测试。
编译团队工作区后，在**独立测试终端**运行（不启动真实底盘/相机/机械臂进程）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_team_ws/install/setup.bash
cd ~/gemini336l_ws/src/pi5-gemini336l-yolo-seg
export ROS_DOMAIN_ID=94
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
python3 -m pytest -q -p no:cacheprovider times_arm_perception/test robot_task_coordinator/test
```

测试域请确认未被本机真实控制器占用；测试代码只使用内存模拟和 localhost 临时 HTTP 端口。
通过软件回归不代表已完成真机导航或抓取验证。
