# libs/core

**职责**：项目的数值与运行时地基。
- 空间数学垫片（RigidTransform / RotationMatrix / RollPitchYaw / Spatial* — M4 前由 vendored 树提供，M4 可自研替换）
- **单一权威 Context**（根持有 t/x/参数/输入版本；树与子系统为零复制视图）
- 静态求值调度（`Finalize()` 期算定 pass 顺序 + 预分配缓冲，替代 Drake 依赖追踪图）
- 覆盖 vendored 树的 facade（建模/状态/运动学/动力学接口面）

**里程碑**：M2 主体（Context/cache 约 4–8 kLOC 为核心工作量与核心风险）。

**必须复现的陷阱**：15 项缓存中 12 状态项每 RHS 失效、3 参数项按需重算；`tree_system_` 反向指针须消解；
拒步时向量外状态事务性回滚。详见 ADR-0002 与设计基线 §4.3、§8。

`include/orvd/core/` = public 头；`src/` = 实现。
