# libs/equilibrium

**职责**：静平衡求解。

**对应 Goal**：G49。

Ceres 为外置第三方。求解器的数值配置（差分步长、稠密后端）会决定初始条件，因此必须
显式声明，不取实现默认值。

`include/orvd/equilibrium/` 为公开头，`src/` 为实现。
