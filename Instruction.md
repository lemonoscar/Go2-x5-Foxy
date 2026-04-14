# Instruction

`rl_ras_n` 现在只保留 `Go2-X5 sim2real` 主路径。

目标环境：
- Jetson NX
- Ubuntu 20.04
- ROS2 Foxy
- ARM64

## 1. 环境

以下命令默认仓库位于 `~/rl_ras_n`。

```bash
source /opt/ros/foxy/setup.bash
echo $ROS_DISTRO
# 必须输出 foxy
```

设置 SDK 路径：

```bash
export ARX5_SDK_ROOT=~/arx5-sdk
export ARX5_SDK_LIB_PATH=$ARX5_SDK_ROOT/lib/aarch64
export UNITREE_SDK2_ROOT=~/Desktop/unitree_sdk2
export LD_LIBRARY_PATH=$ARX5_SDK_LIB_PATH:$LD_LIBRARY_PATH
```

注意：
- Ubuntu 路径区分大小写，桌面目录应写成 `~/Desktop`，不是 `~/desktop`
- 如果 `UNITREE_SDK2_ROOT` 不在 `~/Desktop/unitree_sdk2`，按实际目录修改

## 2. CAN 检查

机械臂链路先独立确认：

```bash
cd ~/rl_ras_n
./scripts/setup_arx_can.sh /dev/ttyACM0 can0 8
ip -details link show can0
```

## 3. 构建

```bash
cd ~/rl_ras_n
./build.sh
source ~/rl_ras_n/install/setup.bash
```

## 4. 启动

标准真机启动：

```bash
source /opt/ros/foxy/setup.bash
source ~/rl_ras_n/install/setup.bash
export ARX5_SDK_ROOT=~/arx5-sdk
export ARX5_SDK_LIB_PATH=$ARX5_SDK_ROOT/lib/aarch64
export UNITREE_SDK2_ROOT=~/Desktop/unitree_sdk2
export LD_LIBRARY_PATH=$ARX5_SDK_LIB_PATH:$LD_LIBRARY_PATH

ros2 launch rl_sar go2_x5_real_dual.launch.py \
  deploy_manifest_path:=~/rl_ras_n/deploy/go2_x5_real.yaml \
  network_interface:=eth0
```

注意：
- `start_arm_bridge` 参数已移除 - 机械臂现在直接通过 InProcessSdk 控制
- `arm_interface_name` 参数已移除 - 从 manifest 中读取 CAN 接口配置

## 5. 启动后必查

- 日志打印 manifest path 和 manifest hash
- `Supervisor initialized ... manifest_valid=true`
- `UnitreeAdapter active: iface=eth0 ...`
- `ArxAdapter active: can=can0, backend=sdk_inprocess_arxcan`
- 主控进入 `PASSIVE/READY`
- `/arx_x5/joint_state` 有持续更新
- 不应出现 `go2_x5 layered config validation failed`
- 不应出现 `joint_mapping size mismatch`
- 不应出现 `arm backend unhealthy`

快速检查：

```bash
ros2 topic hz /arx_x5/joint_state
```

## 6. 操作顺序

1. 确认急停可用
2. 按 `0` 起身
3. 按 `1` 进入 RL
4. 如需 arm 进入预设位，按 `2`
5. 如需 arm 回 home / hold，按 `3`
6. 如需让狗立即停止速度指令，按 `Space`
7. 全程先做低速、小幅度验证

当前键盘语义说明：

- `0`: 只负责 get-up
- `1`: 只负责进入 RL
- `2`: 进入 `MANUAL_ARM` 并发送 preset arm target
- `3`: 进入 `MANUAL_ARM` 并发送 home / hold target
- `Space`: 只清零 body 速度，不等价于 e-stop
- `Esc`: e-stop
- `R`: reset

## 7. 停止与回退

- 立即按 `P`
- 必要时直接硬急停
- 先查 `joint_state`、manifest、生效 RMW，再查策略和 CAN

## 8. 常见启动异常

- `rl_real_go2_x5` 在启动后立刻退出，并出现
  - `go2_x5 layered config validation failed`
  - `joint_mapping size mismatch`

  这说明运行时配置没有正确展开或加载，先 `git pull` 到最新 `main`，重新 `./build.sh`，再启动。

- `ArxAdapter init failed. ARX SDK must be available.`

  说明 ARX5 SDK 不可用。检查：
  - `ARX5_SDK_ROOT` 环境变量是否正确设置
  - `libhardware.so` 是否存在
  - CAN 接口是否配置正确 (`ip link show can0`)

## 9. 当前约束

- PPO 只控制 `12` 条腿
- arm 控制通过 `ArxAdapter` 使用 InProcessSdk（dlopen 加载 libhardware.so）
- 最终输出统一经过 `coordinator`
- ROS `Float32MultiArray` 不再作为控制主路径
