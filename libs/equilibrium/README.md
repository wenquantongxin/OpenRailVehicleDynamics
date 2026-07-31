# libs/equilibrium

**职责**：静平衡求解。

**实施归属**：G46 之后另建的 wheel-rail-lab 迁移路书；当前工作流不实施。

Ceres 为外置第三方。求解器的数值配置（差分步长、稠密后端）会决定初始条件，因此必须
显式声明，不取实现默认值。

`include/orvd/equilibrium/` 为公开头，`src/` 为实现。
