# libs/equilibrium

**职责**：静平衡求解（t=0 初始条件）。Ceres 外置为第三方，functor 内的 Drake 调用换成自有核。

**里程碑**：M3（1–2 pw）。

**必须复现的陷阱**：Ceres 数值差分 `relative_step_size=1e-6`、近零回退 `sqrt(eps)`、
`DENSE_QR` 信赖域；as-built libceres 链 LAPACK——若重建 Ceres 切到 Eigen dense 后端，t0 会变，须钉死后端。
静平衡是每个正式跑的初始条件锚点，t0 差一点 = 全程无法归因的常量偏移。详见设计基线 §3.7、§10.1（A5）。

`include/orvd/equilibrium/` = public 头；`src/` = 实现。
