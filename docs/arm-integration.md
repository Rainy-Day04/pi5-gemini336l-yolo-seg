# 视觉目标接入已有 timesmanipulator 机械臂

`times_arm_perception` 运行在树莓派宿主机的原生 ROS 2 中，读取已有
`http://127.0.0.1:18080/api/state` 和 `/perception/gemini336l_yolo_seg/objects_3d`。
原来的 MiniPC 控制界面和 Pi `arm-driver` 继续使用；不需要重装机械臂服务。
新节点不打开 USB/串口，也不加载旧视觉模块或上游 `hand_eye.json`。

## 现在：标定待定也能启动

在树莓派 SSH 终端执行；已有相机与 YOLO 可以继续运行：

```bash
cd ~/gemini336l_ws/src/pi5-gemini336l-yolo-seg
git -c http.version=HTTP/1.1 pull --ff-only
bash scripts/build_arm_adapter.sh ~/gemini336l_ws

mkdir -p ~/robot_config
cp -n times_arm_perception/config/arm.pending.yaml ~/robot_config/arm.yaml

./scripts/run.sh arm ~/gemini336l_ws \
  mode:=preview config:="$HOME/robot_config/arm.yaml"
```

编译脚本只构建新包，依赖使用已有 workspace 的 `gemini336l_msgs`。
它不重编译 Orbbec、不更新模型、不重建机械臂容器。`cp -n` 保留已有个人参数文件；
以后 `git pull` 不会覆盖 `~/robot_config/arm.yaml`。

另一个树莓派终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source ~/gemini336l_ws/install/setup.bash
ros2 topic echo /arm_perception/status --once
```

此时 API 未运行、机械臂未连接、无目标、标定待定或缺少 TF，都会显示相应原因。
`preview` 只发送 GET `/api/state`，不会自行连接、使能、归零或移动机械臂。
若要看关节反馈，沿用原 GUI 手动连接，再检查：

```bash
ros2 topic echo /arm_perception/joint_states --once
```

本节点的反馈使用自己的话题名，避免与底盘的 `/joint_states` 冲突。
这些数值是原服务返回的电机反馈（rad），不是已经完成关节零偏映射的 URDF joint states。

## 参数文件与坐标

底盘 `base_link` 与机械臂基座 `arm_base_link` 必须区分。输入消息的 `header.frame_id`
决定输入坐标，节点按图像时间戳查 TF，输出到 `arm_base_link`：

```text
base_link
  ├─ camera_link ─ camera_color_optical_frame
  └─ arm_base_link
```

相机仍须开启 D2C；YOLO 启动时使用 `depth_aligned_to_color:=true`。
相机到 `base_link` 的 TF 沿用感知配置。临时零变换只能用于显示联调，不能算完成标定。

`arm.yaml` 中主要参数：

| 参数 | 含义 |
|---|---|
| `calibration_confirmed` | 相机 D2C、相机安装与机械臂基座安装变换均已实测确认 |
| `publish_mount_tf` | 是否由本节点发布 `base_link -> arm_base_link`；已有 URDF 发布时保持 false |
| `mount_xyz` / `mount_rpy` | 机械臂基座在底盘坐标中的位置（米）和 roll/pitch/yaw（弧度） |
| `kinematics_confirmed` | 已验证杆长、运动轴向、零偏、TCP 和夹爪开合角 |
| `joint_origins_xyz/rpy` | 5 个定位关节的固定变换；平铺数组，各 15 个值 |
| `joint_signs` / `joint_zero_offsets_deg` | 模型关节角 = 电机角 × 方向 + 零偏；不写电机编码器零点 |
| `tool_offset_xyz` | 第 5 关节坐标到工具接触点的偏移（米）；J6 作为夹爪 |
| `workspace_min/max` | 机械臂基座坐标中的 TCP 工作范围 |
| `grasp_offset_xyz` | 目标代表点到期望工具位置的偏移（米）；mask 中位点不一定就是夹持点 |
| `target_class` | YOLO 类别名，如 `bottle`；空字符串选当前最近有效目标 |
| `gripper_open_deg/close_deg` | 夹爪开、合电机角度，默认取参考程序的 30/0 度 |
| `approach_m/lift_m` | 接近、抬升高度，沿机械臂基座 +Z |
| `require_tool_down` | 要求工具 Z 轴朝机械臂基座 -Z；无满足解时拒绝规划 |
| `velocity_rad_s` | 执行速度，允许 0.1–0.5 rad/s，默认 0.1 |

所有参数在启动时读取，运行中只读；改文件后重启本节点。launch 的 `mode:=...`
覆盖文件中的 mode，默认永远是 preview。配置中的杆长来自
[timesmanipulator 运动学参考](https://gitee.com/xufuxiang/timesmanipulator/blob/8d80afe8de0d7d630ff56b3ce2a0f6b7da0fc341/Python/u2can/vision/kinematics.py)，
它们是待验证初值，不是你的实机标定结果。

当有已验证的安装数据时填入 `mount_xyz/rpy`。如果 TF 没有其他发布者，设
`publish_mount_tf: true`；它还要求 `calibration_confirmed: true`，以免自动发布占位零变换。

## 标定后：计算与执行

填完参数并验证后，将 `calibration_confirmed`、`kinematics_confirmed` 设为 true。
先停止 preview 节点，以同一参数文件启动只计算模式：

```bash
./scripts/run.sh arm ~/gemini336l_ws \
  mode:=plan config:="$HOME/robot_config/arm.yaml"
```

在另一个已 source ROS 和 workspace 的树莓派终端触发计算：

```bash
ros2 service call /arm_perception/plan std_srvs/srv/Trigger '{}'
ros2 topic echo /arm_perception/status --once
```

计算完成后，`/arm_perception/plan` 会发布 JSON 动作序列；也可以预先开一个终端
`ros2 topic echo /arm_perception/plan` 等待输出。流程为开爪、接近、抓取位、闭爪、抬升。
该模式没有 HTTP 写权限，规划成功不会移动机械臂。

需要实际执行时，停止 plan 节点，使用同一参数文件启动：

```bash
./scripts/run.sh arm ~/gemini336l_ws \
  mode:=execute config:="$HOME/robot_config/arm.yaml"
```

在原 GUI 手动连接/使能，确认六轴状态正确后停止 GUI 点动、Home 和轨迹操作。
原服务没有跨客户端控制租约，因此本节点执行期间应由它独占发送运动目标。
本节点不调用现有 `times_arm_bridge` 的轨迹 action；直接调用同一个 `arm-driver`
API，所以旧 bridge 的 `allow_trajectory/allow_home` 可以保持 false。

执行模式启动后仍不运动。明确请求一个新计划，状态变成 `plan_ready` 后执行：

```bash
ros2 service call /arm_perception/plan std_srvs/srv/Trigger '{}'
ros2 topic echo /arm_perception/status --once
# 确认 plan_ready 后，在 plan_max_age_sec（默认 10 秒）内执行：
ros2 service call /arm_perception/execute std_srvs/srv/Trigger '{}'
```

计划一次有效，重复请求不会并行执行。执行前重新检查目标新鲜度、位置变化、六轴状态
和起始角度；执行中检查反馈、关节限位、跟踪误差、速度与超时。取消：

```bash
ros2 service call /arm_perception/cancel std_srvs/srv/Trigger '{}'
```

取消/执行异常会尽力调用原 `/api/hold`；HTTP 断开时会报告保持请求失败。
这些请求不是硬件急停。正常完成后保持末端目标，不自动失能，速度保留配置值。

`complete` 仅表示命令序列结束，未验证是否夹到物体。这里只做数值 IK、关节限位和
TCP 工作范围检查，没有完整臂体自碰撞/环境碰撞规划，也没有力觉抓取确认。
原服务的反馈缺少逐电机采样时间戳，HTTP 新鲜度不能证明底层每轴通信正常。
当前已通过软件和模拟 API 验证；真机抓取要在标定后独立验证。

## 验证与已有感知修正

测试使用模拟 HTTP API，不接触电机。Jazzy 容器用于开发测试，不是 Pi 上的部署形式。
本次还修复了 3D 坐标转换时共享 RGB header 的问题：更新 3D frame 不再影响 overlay
的 frame，TF 查询使用观测时间而非最新时间。更新后重启 YOLO 即可加载 symlink-install
中的 Python 修复。
