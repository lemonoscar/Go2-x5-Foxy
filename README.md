# rl_ras_n

Go2-X5 sim2real真机部署框架。

## 当前状态（2026-04-14）

**核心组件已完整实现**，包括：
- ✅ Arx In-Process SDK Backend（dlopen动态加载）
- ✅ ObservationBuilder with validation
- ✅ Policy Freshness完整语义
- ✅ FallbackSmoother（五次多项式）
- ✅ 诊断聚合系统
- ✅ Drift数据记录

详见[开发思路总览](doc/开发思路/00-总览.md)

## 目标平台

- Jetson NX / Orin
- Ubuntu 20.04
- ROS2 Foxy
- ARM64

## 控制架构

```
┌─────────────────────────────────────────────────────────────┐
│  Supervisor (模式机 + Watchdog)                             │
│  BOOT → PROBE → PASSIVE → READY → RL_DOG_ONLY_ACTIVE        │
│                    ↓              ↘                         │
│               MANUAL_ARM    DEGRADED_ARM/BODY               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  Coordinator (混合控制协调器)                                │
│  - 接收 Dog Policy Command (12维)                           │
│  - 接收 Arm Command (6维)                                   │
│  - 输出 Body Command (12维) + Arm Command (6维)              │
│  - 降级平滑处理                                              │
└─────────────────────────────────────────────────────────────┘
              ↓                           ↓
┌──────────────────────┐    ┌──────────────────────┐
│  UnitreeAdapter      │    │  ArxAdapter          │
│  (Go2 body, 12 DOF)  │    │  (X5 arm, 6 DOF)    │
│  - DDS通信           │    │  - In-Process SDK    │
│  - 200Hz控制         │    │  - 500Hz内部伺服     │
└──────────────────────┘    └──────────────────────┘
```

## 快速开始

### 1. 环境设置

```bash
source /opt/ros/foxy/setup.bash
export ARX5_SDK_ROOT=~/arx5-sdk
export UNITREE_SDK2_ROOT=~/Desktop/unitree_sdk2
export LD_LIBRARY_PATH=$ARX5_SDK_ROOT/lib/aarch64:$LD_LIBRARY_PATH
```

### 2. 构建

```bash
cd ~/rl_ras_n
./build.sh
source install/setup.bash
```

### 3. 启动

```bash
ros2 launch rl_sar go2_x5_real_dual.launch.py \
  deploy_manifest_path:=~/rl_ras_n/deploy/go2_x5_real.yaml \
  network_interface:=eth0
```

注意：
- `arm_interface_name` 参数已移除，从 manifest 中的 `arm_adapter.can_interface` 读取
- 默认 `enable_ros2_runtime=false`（从 manifest 读取）

### 4. 操作

| 按键 | 功能 |
|------|------|
| `0` | 起身 |
| `1` | 进入RL模式 |
| `2` | Arm预设位置 |
| `3` | Arm回home/hold |
| `Space` | 清零速度 |
| `Esc` | 急停 |
| `R` | 重置 |

## 目录结构

```
rl_ras_n/
├── deploy/              # 部署manifest
├── doc/                 # 文档
│   ├── 开发思路/        # 实施计划
│   ├── 问题发现与解决思路.md
│   └── 上机验证Checklist.md
├── src/rl_sar/          # 源代码
│   ├── adapters/        # 硬件适配器
│   │   ├── arx/         # ARX机械臂
│   │   └── unitree/     # Unitree Go2
│   ├── runtime/         # 运行时
│   │   ├── coordinator/ # 控制协调器
│   │   └── supervisor/  # 模式机+看门狗
│   ├── observation/     # 观测构建
│   ├── diagnostics/     # 诊断系统
│   └── protocol/        # 协议定义
└── policy/              # 策略文件
```

## 关键文档

- [开发思路总览](doc/开发思路/00-总览.md) - 整体规划
- [第一阶段：配置增强](doc/开发思路/01-第一阶段-配置增强.md)
- [第二阶段：训练一致性](doc/开发思路/02-第二阶段-训练一致性.md)
- [上机验证Checklist](doc/上机验证Checklist.md)
- [第1轮冻结契约](doc/第1轮冻结契约.md)

## 配置文件

- `deploy/go2_x5_real.yaml` - 主部署manifest
- `policy/go2_x5/` - 策略配置

## 已实现功能

### 核心控制

- [x] 12自由度腿部PPO控制
- [x] 6自由度机械臂独立控制
- [x] 混合协调器
- [x] 降级平滑轨迹

### 硬件适配

- [x] Unitree Go2 DDS通信（200Hz）
- [x] ARX X5 In-Process SDK（500Hz内部）
- [x] CAN接口管理

### 安全与诊断

- [x] 10状态模式机
- [x] Watchdog freshness检测
- [x] 诊断聚合系统
- [x] Event日志
- [x] Drift数据记录

### 验证与测试

- [x] 观测一致性验证
- [x] 40+单元测试
- [x] 协议序列化测试

## 待完善（按优先级）

### P0 - 配置增强

- [ ] 观测验证纳入manifest
- [ ] Drift阈值台架校准
- [ ] 状态判断逻辑统一（ArxAdapter健康检查）
- [ ] snapshot生成工具

### P1 - 训练一致性

- [ ] Python ObservationBuilder
- [ ] Snapshot生成工具
- [ ] DR参数对齐验证

### P2 - 验收增强

- [ ] 自动化验收测试
- [ ] 长期稳定性监控

## 贡献

当前为个人项目，欢迎讨论但暂不接受PR。

## 许可

MIT License
