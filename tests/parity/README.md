# tests/parity — Drake 差分预言机台架

项目的裁判。**M0 就位、贯穿全程**。以 Drake 为 oracle，逐层证明 ORVD 复现当前行为。

## 三层金标

1. **树 pass 级**：序列化 `q/v/参数/外力` → 各 pass（PositionKinematics / VelocityKinematics /
   InverseDynamics / ForceElements / MassMatrix / ArticulatedBodyAccelerations）输出逐位对比。
2. **整车 RHS 级**：`x → ẋ` 逐位对比。
3. **短时积分级**：含事件的 0.1–1s，再到 30s 正式算例。

## 主指标：有序轨迹（不是调用计数）

调用**总数相同**不能证明顺序或数值相同。主指标是**有序轨迹**：
- 阶段序号（试算 / 插值 / 输出 / 接受提交 / 事件）；
- `t / q / v / 外力 / 温启动` 的逐位哈希；
- 缓存索引、变更序号、命中/重算记录；
- 接触内核输入输出哈希；
- CVODE 接受/拒绝步统计与同刻事件顺序。

调用计数降为**辅助**指标。

## 台架自证

M0 验收门：Drake **自重放**跨进程逐位一致（同平台同构建）——台子先证明自己可信，再拿去裁别人。

## parity 口径

同平台同构建身份逐位；跨平台仅工程容差回归。parity 构建强制 `-ffast-math` / `-march=native` 关闭。

后续改造按
[Drake 多体运行时脱耦路书](../../docs/planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md)
中的 G06–G08 执行。
