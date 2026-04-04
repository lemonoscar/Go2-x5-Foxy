# Go2-X5 上机验证 Checklist

## 1. 使用说明

本清单面向 dog-only PPO 新范式。

验证目标不是“证明 whole-body PPO 很强”，而是逐级确认：

- arm 外部控制链路正常
- PPO 在 arm motion 条件下维持底盘稳定
- 零速时无明显平面漂移
- 低速 locomotion 时能抗 arm 扰动


## 2. 启动前 Preflight

### 2.1 环境检查

- [ ] `unitree_sdk2` 可初始化
- [ ] `arx5-sdk` 可初始化
- [ ] DDS 网络接口正确
- [ ] CAN 接口已拉起
- [ ] 机器人物理急停可用
- [ ] 电池/供电状态正常

### 2.2 配置检查

- [ ] deploy manifest 存在
- [ ] manifest hash 已打印
- [ ] policy mode 为 `dog_only`
- [ ] `policy.action_dim == 12`
- [ ] `arm_adapter.background_send_recv == true`
- [ ] `coordinator.rate_hz == body_adapter.command_rate_hz`
- [ ] `arm_adapter.servo_rate_hz >= coordinator.rate_hz`

### 2.3 初始状态检查

- [ ] body state 非 stale
- [ ] arm state 非 stale
- [ ] arm initial state 非全零
- [ ] joint count 匹配
- [ ] joint limits 已加载
- [ ] topic / IPC / 端口无冲突


## 3. 台架验证

### 3.1 body-only

- [ ] 可持续读取 lowstate
- [ ] lowcmd 写成功率正常
- [ ] body state age 在阈值内
- [ ] 无异常 mode 跳变

### 3.2 arm-only

- [ ] arm backend 可连续读取状态
- [ ] arm tracking 正常
- [ ] arm tracking error 不持续发散
- [ ] arm stale 不出现连续告警

### 3.3 fault injection

- [ ] 注入 arm state stale 后进入 `DEGRADED_ARM`
- [ ] 注入 body stale 后进入 `DEGRADED_BODY`
- [ ] 注入 e-stop 后进入 `FAULT_LATCHED`
- [ ] `SOFT_STOP` 能平滑完成


## 4. 低风险上机验证

### 4.1 被动模式

- [ ] `PASSIVE` 下整机不自主运动
- [ ] arm 可进入 damping / hold
- [ ] 退出时不会突然抽动

### 4.2 READY 模式

- [ ] 状态新鲜度持续稳定
- [ ] diagnostics 指标可持续输出
- [ ] 不进入主动步态时整机稳定


## 5. 静态 arm motion 验证

这是新范式下最关键的第一阶段验证。

### 5.1 小幅动作

- [ ] base command = 0
- [ ] arm 做小幅位姿变化
- [ ] 无明显 `xy drift`
- [ ] 无明显 `yaw drift`
- [ ] roll/pitch 补偿幅度可接受

### 5.2 中幅动作

- [ ] arm 做中等幅度 sweep
- [ ] PPO 未引发步态异常
- [ ] 无明显足端打滑
- [ ] 无安全限幅连续爆发

### 5.3 大幅合法动作

- [ ] arm 做大幅合法位姿变化
- [ ] 整机保持可控
- [ ] 若 tracking error 超阈值，系统进入 `DEGRADED_ARM`


## 6. 零速漂移验证

### 6.1 平面漂移

- [ ] 连续站立窗口内 `xy drift` 在阈值内
- [ ] 连续站立窗口内 `yaw drift` 在阈值内

### 6.2 诊断记录

- [ ] `xy_drift_error` 已记录
- [ ] `yaw_drift_error` 已记录
- [ ] 可回放对应 arm command 和 arm tracking error


## 7. 低速 locomotion under arm motion

### 7.1 低速前进

- [ ] 小速度前进命令可跟踪
- [ ] arm 小幅动作不破坏步态
- [ ] 足端滑动在可接受范围

### 7.2 低速转向

- [ ] 小 yaw command 可跟踪
- [ ] arm motion 下无明显异常漂移

### 7.3 混合扰动

- [ ] arm 中幅动作 + 低速行走仍可保持稳定
- [ ] coordinator jitter 正常
- [ ] policy latency 正常


## 8. 降级与恢复验证

### 8.1 arm 降级

- [ ] 拔掉 arm backend / 制造 stale
- [ ] 系统进入 `DEGRADED_ARM`
- [ ] arm 切 hold / damping
- [ ] body 不继续高风险动作

### 8.2 body 降级

- [ ] 模拟 DDS 失败
- [ ] 系统进入 `DEGRADED_BODY`
- [ ] 进入 `SOFT_STOP`

### 8.3 锁存恢复

- [ ] `FAULT_LATCHED` 后不能自动恢复
- [ ] 人工 reset 后可重新 probe


## 9. 记录指标

每次上机必须记录以下指标：

- [ ] `body_state_age_us`
- [ ] `arm_state_age_us`
- [ ] `policy_latency_us`
- [ ] `coordinator_jitter_us`
- [ ] `arm_tracking_error_norm`
- [ ] `xy_drift_error`
- [ ] `yaw_drift_error`
- [ ] `clip_count`
- [ ] `seq_gap_count`
- [ ] `mode_transition_log`


## 10. 判定标准

只有满足以下条件，才允许从“验证阶段”进入“可持续部署阶段”：

- [ ] 静态 arm motion 不引发明显平面漂移
- [ ] 低速 locomotion 在 arm motion 下保持可控
- [ ] arm tracking error 可监控且不过度发散
- [ ] stale / timeout / e-stop 都能正确驱动模式机
- [ ] 所有关键诊断数据都有记录


## 11. 禁止事项

以下情况禁止继续扩大测试：

- [ ] body stale 仍继续主动步态
- [ ] arm stale 仍继续 whole-body 扰动验证
- [ ] `FAULT_LATCHED` 后绕过 reset 继续运行
- [ ] manifest 未确认一致时直接上机
- [ ] tracking error 明显发散仍继续加动作范围


## 12. 结论

这份 checklist 的目的不是“多写几个打勾框”，而是强制把新的 dog-only PPO 范式按正确顺序验证：

先证明 arm 条件下的静态稳定，
再证明低速 locomotion 抗扰，
最后才进入长期部署判断。
