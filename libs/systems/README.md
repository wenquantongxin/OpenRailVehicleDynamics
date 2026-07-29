# libs/systems

**职责**：mini-LeafSystem 运行时——保留 LeafSystem 等价的整车组装层（约束 C6），
便于车型模板化与 CVODE 单点耦合，但不含 Drake 依赖追踪图的通用机制。

支持的框架特性（GZ18 实际子集，接口按跨车型超集设计）：
- 端口：Vector/Abstract 的 Input/Output
- 状态：Discrete / Abstract / **Continuous**（GZ18 仅 3 处连续状态，展平进 CVODE `N_Vector`）
- 事件：Periodic / Initialization 的 UnrestrictedUpdate、Periodic Publish
- 缓存条目、数值参数

**里程碑**：M2。

**必须复现的陷阱**：同刻事件稳定序、x⁻/x⁺ 可见性、试算/提交/回滚事务性（B4 七项，见 ADR-0003）。
`motor_pid` / `motor_bridge_proxy` / `torque_applier` 属 IRW 线，GZ18 首版按需实现。

`include/orvd/systems/` = public 头；`src/` = 实现。
