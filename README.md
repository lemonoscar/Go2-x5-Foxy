# rl_sar_go2-x5

> This fork is **ROS2 Foxy only**.

## Foxy Delta And Usage

This directory (`/home/lemon/Issac/rl_ras_n`) is a Foxy-only clone of `rl_sar`.
The original `/home/lemon/Issac/rl_sar` is untouched.
Legacy upstream sections below may still mention Noetic/Humble; use this Foxy block as the source of truth for this fork.

What was changed:

- `build.sh` is Foxy-only (`ROS_DISTRO` must be `foxy`).
- ROS build path is fixed to `colcon build --merge-install --symlink-install`.
- ROS CMake paths are Foxy-only:
  - `src/robot_msgs/CMakeLists.txt`
  - `src/robot_joint_controller/CMakeLists.txt`
  - `src/rl_sar/CMakeLists.txt`
  - `src/rl_sar_zoo/*_description/CMakeLists.txt`
- Controller spawner path is fixed to Foxy `spawner.py`:
  - `src/rl_sar/launch/gazebo.launch.py`
  - `src/rl_sar/src/rl_sim.cpp`
  - `src/robot_joint_controller/ros2/examples/launch/*.launch.py`
- `robot_joint_controller` source/header logic is reduced to Foxy path only.

Quick usage:

```bash
source /opt/ros/foxy/setup.bash
cd /home/lemon/Issac/rl_ras_n
./build.sh
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=go2_x5
```

In another terminal:

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
ros2 run rl_sar rl_sim
```

面向 **Unitree Go2-X5** 的 RL 仿真与部署仓库（基于 rl_sar）。本仓库聚焦 Gazebo Classic + ROS2 Foxy 的仿真落地，配套 go2_x5 的模型与策略配置，并保留完整的 rl_sar 运行框架。

## 目录结构（核心）

- `src/rl_sar/`：主程序与 FSM（含 `go2_x5` 的状态机）
- `src/rl_sar_zoo/`：机器人描述包（URDF/Xacro/mesh）
  - `go2_x5_description/`：Go2-X5 机器人模型
- `policy/go2_x5/`：策略与参数
  - `robot_lab/policy.pt`：策略模型（IsaacLab 导出）
  - `robot_lab/config.yaml`：观测/控制参数
- `scripts/`：依赖与下载脚本
- `build.sh`：一键构建（ROS1/ROS2 + 依赖准备）

## 仓库拆分（主仓库 + zoo）

本仓库与机器人描述仓库分离维护：
- 主仓库：`git@github.com:lemonoscar/rl_sar_go2-x5.git`
- 描述仓库：`git@github.com:lemonoscar/rl_sar_zoo.git`

初始化示例：
```bash
git clone git@github.com:lemonoscar/rl_sar_go2-x5.git
cd rl_sar_go2-x5
git clone git@github.com:lemonoscar/rl_sar_zoo.git src/rl_sar_zoo
```

## 环境要求

- Ubuntu 22.04（建议）
- ROS2 Humble
- Gazebo Classic（gazebo-11）
- 依赖：`ros-humble-gazebo-ros`、`ros-humble-gazebo-ros-pkgs`、`ros-humble-ros2-control` 等

安装依赖示例：

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs ros-humble-xacro
```

## 快速开始（Gazebo + ROS2）

### 1) 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/lemon/Issac/rl_sar
./build.sh
```

### 2) 启动 Gazebo

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /home/lemon/Issac/rl_sar/install/setup.bash
ros2 launch rl_sar gazebo.launch.py
```
> `rname` 默认是 `go2_x5`，也可手动指定：  
> `ros2 launch rl_sar gazebo.launch.py rname:=go2_x5`
>
> 默认世界：`stairs.world`。  
> 如需切到 `terrain.world`，编辑 `src/rl_sar/launch/gazebo.launch.py` 的 `wname`。

### 3) 启动控制与策略

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /home/lemon/Issac/rl_sar/install/setup.bash
ros2 run rl_sar rl_sim
```

## 快速开始（MuJoCo）

> MuJoCo 版本不依赖 Gazebo/ROS 启动。  
> 详细模型与场景说明见：`src/rl_sar_zoo/go2_x5_description/mjcf/README_MUJOCO.md`

### 1) 编译（MuJoCo）

```bash
cd /home/lemon/Issac/rl_sar
./build.sh -mj
```

### 2) 运行（MuJoCo）

```bash
export LD_LIBRARY_PATH=/home/lemon/Issac/rl_sar/library/mujoco/lib:$LD_LIBRARY_PATH
./cmake_build/bin/rl_sim_mujoco go2_x5 scene_flat
```

可用场景（对应 `src/rl_sar_zoo/go2_x5_description/mjcf/*.xml`）：
- `scene_flat`
- `go2_x5_scene`

## 运行与控制

> RL 默认从 **被动模式**启动，必须触发状态切换才会进入策略控制。

**键盘控制（在 `rl_sim` 终端内按键）**
- `0`：起身（Passive → GetUp）
- `1`：进入 RL 控制（GetUp → RLLocomotion）
- `P`：回被动
- `9`：趴下
- `W/A/S/D/Q/E`：速度指令（x/y/yaw）
- `Space`：清零速度
- `N`：切换到 `/cmd_vel` 导航模式
- `2`：播放机械臂动作序列（一次）
- `3`：停止机械臂动作序列

**Go2-X5 额外键位行为**
- `1`：进入 RL 后设置固定速度（`fixed_cmd_x/y/yaw`），关闭导航模式
- `2`：机械臂保持到指定姿态（`arm_key_pose`）
- `3`：机械臂恢复默认姿态（`default_dof_pos` 最后 6 轴）

**命令话题**
- `/cmd_vel`：基础速度命令（当 `N` 开启导航模式）
- `/arm_joint_pos_cmd`：机械臂目标角度（Float32MultiArray，长度 6，作为 RL 观测输入）

例：发布机械臂稳态指令（减少晃动）：
```bash
ros2 topic pub /arm_joint_pos_cmd std_msgs/msg/Float32MultiArray \
"{data: [0.0, 1.57, 1.57, 0.0, 0.0, 0.0]}" -r 10
```

## 机械臂序列与插值

机械臂动作支持“按键触发既定序列”，并带有插值平滑，避免突变抖动。  
Go2-X5 模式下，`2/3` 按键会优先触发 **单姿态保持/恢复**（见 `arm_key_pose`），序列仅作为备用。

**配置文件**：`policy/go2_x5/robot_lab/config.yaml`  
关键字段：
```yaml
arm_command_size: 6
arm_joint_command_topic: "/arm_joint_pos_cmd"
arm_hold_enabled: true
arm_command_smoothing_time: 3.0
arm_key_pose: [0.60, 3.00, 1.00, 0.20, 0.00, 0.00]
arm_sequence_interval: 2.0
arm_sequence_steps: 3
arm_sequence: [0.00, 1.57, 1.57, 0.00, 0.00, 0.00,
               0.60, 2.00, 1.00, 0.20, 0.00, 0.00,
               0.20, 1.00, 1.40, -0.20, 0.00, 0.00]
```
说明：
- `arm_sequence` 每一帧 6 个值（`arm_joint1..6`），总长度 = `arm_sequence_steps * 6`
- `arm_sequence_interval` 为帧间隔（秒）
- `arm_command_smoothing_time` 为插值平滑时间（秒）

**使用方式**
- `2` 播放一次序列
- `3` 停止序列
- `4` 切换机械臂保持（ON/OFF）

**Go2-X5 使用方式**
- `2`：保持 `arm_key_pose`
- `3`：恢复默认姿态

> 注意：`/arm_joint_pos_cmd` 是 RL 的观测输入，不是直接控制指令。需要进入 RL 控制（`0` → `1`）后才会生效。  
> 机械臂默认保持固定姿态，只有触发序列（`2`）后才会运动。

## 策略与观测对齐（非常重要）

策略来源于 IsaacLab（robot_lab）。**必须保证观测维度与训练一致**，否则会报形状不匹配错误。

### Flat 策略（推荐）
- 观测 **无 height_scan**
- 维度：`69`
- `policy/go2_x5/robot_lab/config.yaml` 已设置为：
  - `observations` 不含 `height_scan`
  - `num_observations: 69`
  - `observations_history: []`

### Rough 策略
如果使用 rough 策略，通常会包含 `height_scan`，维度会显著增加。需要同步：
- `observations` 增加 `height_scan`
- `height_scan_size/width/height` 与训练一致
- `num_observations` 与策略输入匹配

## 常见问题

### 1) Gazebo 看不到机器人
- 确认 `SpawnEntity: Successfully spawned entity` 出现
- Gazebo 里选中 `<rname>_gazebo`（默认 `go2_x5_gazebo`），按 `F` 聚焦
- 确保 `go2_x5_description` 在 `src/rl_sar_zoo`

### 2) `/spawn_entity` 不存在
- 需要加载 Gazebo ROS 插件
```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
echo $GAZEBO_PLUGIN_PATH
```
确保包含 `/opt/ros/humble/lib`。

### 3) `Failed to start joint controller`
- 说明控制器已被加载过
```bash
ros2 control list_controllers
ros2 control set_controller_state robot_joint_controller inactive
ros2 control unload_controller robot_joint_controller
```
再重新启动 `rl_sim`。

### 4) 走路抖动 / 机械臂乱晃
- 降低 `lin_vel_scale` / `ang_vel_scale` / `commands_scale`
- 提高机械臂阻尼（`rl_kd` 的手臂 6 关节）
- 使用 `arm_command_smoothing_time` 做插值平滑
- 必要时降低 `action_scale` 或提高 `fixed_kd`

## 录像（Gazebo GUI）

Gazebo Classic 自带录屏：`View -> Video Recorder`  
- 先点击 **Record**，结束时点 **Stop**  
- 默认输出 `.ogv`，可转为 mp4：
```bash
ffmpeg -i input.ogv -c:v libx264 -pix_fmt yuv420p -an output.mp4
```

## 同步机器人描述包（rl_sar_zoo）

当前仓库默认使用你的 zoo 仓库：
`git@github.com:lemonoscar/rl_sar_zoo.git`

```bash
cd /home/lemon/Issac/rl_sar/src/rl_sar_zoo
git remote -v
git pull

## gazebo使用方法

- Go2-X5：连续按2次 `1` 进入 RL 后设置固定速度依赖 `/cmd_vel`
- Go2-X5：按 `2/3` 机械臂单姿态保持 / 恢复默认，`arm_command_smoothing_time` 控制慢速到位
- ROS2 仿真：增加线程安全保护，避免回调与控制线程数据竞争（IMU/RobotState/cmd_vel）
```

## 免责声明

本仓库仅用于研究与仿真验证，机器人实机运行存在风险，请确保安全防护到位，使用风险由使用者自行承担。
