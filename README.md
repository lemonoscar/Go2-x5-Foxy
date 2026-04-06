# rl_ras_n

`rl_ras_n` 现在只保留 `Go2-X5 sim2real` 主路径。

当前仓库目标：
- Jetson NX
- Ubuntu 20.04
- ROS2 Foxy
- Go2 dog-only PPO
- external arm pipeline
- hybrid coordinator

## 保留内容

- `deploy/`：真机 deploy manifest
- `src/rl_sar/`：真实运行时、protocol、supervisor、coordinator、adapter
- `policy/go2_x5/`：Go2-X5 策略配置
- `scripts/setup_arx_can.sh`：机械臂 CAN 启动
- `Instruction.md`：最小上机清单

## 已删除方向

- Gazebo
- MuJoCo
- sim2sim
- 非 Go2-X5 机器人描述与控制包

## 构建

```bash
source /opt/ros/foxy/setup.bash
cd /home/lemon/Issac/rl_ras_n
./build.sh
source /home/lemon/Issac/rl_ras_n/install/setup.bash
```

如果需要单独做硬件向 CMake 构建：

```bash
cd /home/lemon/Issac/rl_ras_n
./build.sh --cmake
```

## 启动

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk

ros2 launch rl_sar go2_x5_real_dual.launch.py \
  network_interface:=eth0 \
  arm_interface_name:=can0
```

## 入口文档

- [AGENT.md](/home/lemon/Issac/rl_ras_n/AGENT.md)
- [README.md](/home/lemon/Issac/rl_ras_n/README.md)
- [Instruction.md](/home/lemon/Issac/rl_ras_n/Instruction.md)
