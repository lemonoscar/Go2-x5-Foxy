# Go2-X5 新范式部署文档总览

## 1. 文档目标

本目录用于把基于 `newPPO.md` 的新部署范式闭合成一套可执行文档。

新范式的核心前提：

- PPO 只控制 12 个腿关节
- 机械臂由外部控制链路负责
- gripper 暂不进入 locomotion PPO 主闭环
- 实机系统的重点是 arm-conditioned base stability，而不是 whole-body PPO 统一出动作


## 2. 文档清单

### 2.1 总体设计

- [项目蓝图.md](/home/lemon/Issac/rl_ras_n/doc/项目蓝图.md)
  说明新范式下的模块结构、职责边界、频率合同和系统目标。

- [消息协议设计.md](/home/lemon/Issac/rl_ras_n/doc/消息协议设计.md)
  说明 typed IPC/ROS 镜像协议、状态与命令 frame 设计、时序与 freshness 规则。

### 2.2 可执行落地

- [实施路线图.md](/home/lemon/Issac/rl_ras_n/doc/实施路线图.md)
  说明建议的阶段拆分、代码改造顺序、每阶段完成标准和推荐文件落点。

- [模式机设计.md](/home/lemon/Issac/rl_ras_n/doc/模式机设计.md)
  说明 supervisor 的模式机、事件、转移条件、接管策略和 fault latch 规则。

- [部署配置规范.md](/home/lemon/Issac/rl_ras_n/doc/部署配置规范.md)
  说明统一 deploy manifest 的结构、字段、默认策略与校验要求。

- [上机验证Checklist.md](/home/lemon/Issac/rl_ras_n/doc/上机验证Checklist.md)
  说明从 preflight、台架、低风险上机到正式部署的逐级验证清单。


## 3. 推荐阅读顺序

建议按以下顺序阅读和实施：

1. `项目蓝图.md`
2. `消息协议设计.md`
3. `模式机设计.md`
4. `部署配置规范.md`
5. `实施路线图.md`
6. `上机验证Checklist.md`

理由：

- 先统一系统范式
- 再统一消息与时序合同
- 再统一 supervisor 的模式行为
- 再固化配置真值源
- 最后按路线图落地并按 checklist 验证


## 4. 直接执行建议

如果按工程落地优先级执行，建议直接照以下顺序推进：

### 第一优先级

- 建立统一 deploy manifest
- 建立 typed protocol
- 将当前 arm bridge 的 `Float32MultiArray` 和裸 IPC payload 替换成 typed frame

### 第二优先级

- 建立 `deploy_supervisor`
- 收拢模式机
- 明确 `RL_DOG_ONLY_ACTIVE` / `DEGRADED_ARM` / `DEGRADED_BODY`

### 第三优先级

- 引入 `hybrid_motion_coordinator`
- 将腿部 PPO 输出和 arm 外部命令正式分流
- 将限幅、过期、tracking error 判断统一迁入 coordinator

### 第四优先级

- 统一 diagnostics
- 增加 drift / tracking / stale / jitter 指标
- 开始台架和上机分级验证


## 5. 当前范式的关键结论

为了避免文档执行过程中又回退到旧范式，这里再次明确：

- 不再把 arm 放回 PPO output head
- 不再把系统理解为 whole-body PPO 部署问题
- 真实部署的主问题是：
  - arm 条件下的底盘稳定
  - 零速时的平面漂移
  - 低速行走下的抗扰和 tracking


## 6. 完成标志

只有当以下条件同时成立，才能认为这套文档闭环完成并具备执行价值：

- 模块结构清楚
- 协议清楚
- 模式机清楚
- 配置规范清楚
- 实施顺序清楚
- 上机验证清单清楚

本目录现在的目标就是满足这 6 点。
