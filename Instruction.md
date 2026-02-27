# Instruction

本文件用于 `rl_ras_n`（ROS2 Foxy 专用）下的 Go2-X5 `sim2real` 部署。

## 0. 前置

- 系统已安装 ROS2 Foxy。
- 你有机械臂 SDK 仓库权限（`arx5-sdk`）。
- 机械臂总线接口为 `can0`（按实际调整）。

## 1. 进入 Foxy 环境

```bash
source /opt/ros/foxy/setup.bash
echo $ROS_DISTRO
# 需要输出: foxy
```

## 2. 安装 Unitree SDK2（机器狗底盘）

```bash
cd /home/lemon/Issac/rl_ras_n/src/rl_sar/library/thirdparty/robot_sdk/unitree
rm -rf unitree_sdk2
git clone -b 2.0.0 https://github.com/unitreerobotics/unitree_sdk2.git unitree_sdk2

cd unitree_sdk2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

验证：

```bash
test -f /home/lemon/Issac/rl_ras_n/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2/CMakeLists.txt && echo "unitree_sdk2 OK"
```

## 3. 安装 ARX5 SDK（机械臂）

```bash
cd /home/unitree
git clone https://github.com/real-stanford/arx5-sdk.git
cd arx5-sdk
```

### 3.1 创建环境（无 mamba 用 conda）

```bash
conda env create -f conda_environments/py310_environment.yaml
conda activate arx-py310
```

如果你要复用现有环境，例如 `go2x5_s2r`，可改成：

```bash
conda activate go2x5_s2r
```

### 3.2 编译 C++ 与 Python 绑定

```bash
cd /home/unitree/arx5-sdk
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

python -m pip install pybind11
PYBIND_PATH=$(python -m pip show pybind11 | awk '/Location:/{print $2}')
cmake -S python -B python/build -DPYBIND_PATH=$PYBIND_PATH
cmake --build python/build -j$(nproc)
```

### 3.3 设置运行时路径

```bash
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk
export PYTHONPATH=$ARX5_SDK_ROOT/python:$PYTHONPATH
export LD_LIBRARY_PATH=$ARX5_SDK_ROOT/lib/aarch64:$LD_LIBRARY_PATH
python -c "import arx5_interface; print('arx5_interface OK')"
```

建议写入 `~/.bashrc`：

```bash
echo 'export ARX5_SDK_ROOT=/home/unitree/arx5-sdk' >> ~/.bashrc
echo 'export PYTHONPATH=$ARX5_SDK_ROOT/python:$PYTHONPATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$ARX5_SDK_ROOT/lib/aarch64:$LD_LIBRARY_PATH' >> ~/.bashrc
```

## 4. 构建 `rl_ras_n`

```bash
source /opt/ros/foxy/setup.bash
cd /home/lemon/Issac/rl_ras_n
./build.sh
source /home/lemon/Issac/rl_ras_n/install/setup.bash
```

## 5. 确认双通道配置（12腿 + 6臂）

检查以下文件：

- `policy/go2_x5/base.yaml`
- `policy/go2_x5/robot_lab/config.yaml`

关键字段：

```yaml
arm_control_mode: "split"
arm_joint_start_index: 12
arm_joint_count: 6
arm_bridge_require_state: true
arm_bridge_cmd_topic: "/arx_x5/joint_cmd"
arm_bridge_state_topic: "/arx_x5/joint_state"
```

## 6. 启动双通道真机（推荐）

```bash
source /opt/ros/foxy/setup.bash
source /home/lemon/Issac/rl_ras_n/install/setup.bash
export ARX5_SDK_ROOT=/home/unitree/arx5-sdk

ros2 launch rl_sar go2_x5_real_dual.launch.py \
  network_interface:=<YOUR_NETWORK_INTERFACE> \
  arm_interface_name:=can0
```

## 7. 运行前检查

```bash
ip -br a
ip -details link show can0
ros2 topic hz /arx_x5/joint_cmd
ros2 topic hz /arx_x5/joint_state
```

`/arx_x5/joint_state` 有稳定频率后，再执行动作流程。

## 8. 上机动作顺序

1. 机器人离地/吊装，急停可触发。
2. 按 `0` 起身。
3. 观察稳定后按 `1` 进入 RL。
4. 再落地低速测试（先小速度）。

## 9. 失败回退

1. 立即按 `P` 或硬急停。
2. 保留日志与配置快照。
3. 优先排查 bridge 状态链路，再排查策略参数。
