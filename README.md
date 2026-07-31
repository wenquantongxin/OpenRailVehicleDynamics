# OpenRailVehicleDynamics (ORVD)

模型中立、仅 `double`、以**零 Drake 运行期依赖**为目标的 C++ 多体运行时。

## 当前状态

vendored Drake common support、刚性 topology、double 位姿数学与完整 double-only 刚性树
均已有内部构建目标。第一方运行时拥有单一多体状态、类型化参数、版本缓存与具名工作区；
G29–G39 已接通模型中立建模、运动学、质量矩阵、逆动力学、具名外力和 O(n) 前向动力学，
G40–G44 已接通静态系统组装、上下文局部阻尼、连续状态原子事务、RHS 桥与真实 CVODE 后端。
当前 Goal 为 G45，当前路书止于 G46。

唯一实施依据是
[Drake 多体运行时脱耦路书](docs/planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md)：
它定义 17 个子目标、46 个原子 Goal 的顺序与完成门。当前进度以路书中的「当前 Goal」为准。

## 已锁定的架构决策

| 决策 | 内容 |
|---|---|
| [ADR-0001](docs/adr/0001-vendor-tree-behind-allowlist.md) | 方案 B：先按允许清单 vendor Drake 刚性多体树与拓扑（仅 `double`），再逐步替换 |
| [ADR-0002](docs/adr/0002-single-authoritative-context.md) | 单一权威 Context：状态只有一个所有者，树与子系统均为零复制视图 |
| [ADR-0003](docs/adr/0003-abstract-advancer-cvode-first.md) | 抽象推进器接口，首版唯一后端为 CVODE |

## 数值验收口径

- 条件良好的 `double` 跨实现连续量默认采用 `1e-8` 相对误差。
- 近零量使用按单位声明的绝对限。
- 旋转使用 `1e-8 rad` 的 SO(3) 测地角。
- 代数恒等式、有限差分与迭代求解分别使用与其误差来源相符的判据，不套用一个万能阈值。
- 不保存数值金标、输出快照或文件哈希；不要求逐位一致。
- 参考端与候选端始终位于不同进程：`libdrake.so` 导出的符号与 vendored 副本同处
  `namespace drake`，同进程链接会构成 ODR 违规，其最可能的症状是一次看起来通过的比较。

## 目录结构

```text
OpenRailVehicleDynamics/
├── CMakeLists.txt        顶层构建（内部刚性树产品目标；启用测试时另建模型中立自检）
├── CMakePresets.json     构建预设：dev / release / drake-reference
├── cmake/                CMake 辅助模块
├── docs/
│   ├── planning/         唯一实施路书
│   ├── engineering/      第一方工程约束
│   ├── adr/              架构决策记录
│   ├── design/           历史调研输入（非现行依据）
│   └── review/           历史审查往来（非现行依据）
├── external/
│   └── drake_mbtree/     vendored topology/tree 源码、第一方替代实现、处置与许可证
├── tools/
│   ├── drake_source_boundary/  源码闭包、编译前沿、来源与禁入边界工具（开发期）
│   └── product_boundary_gate/  链接边界闸门的判别力自检（开发期）
├── libs/
│   ├── multibody_runtime/ 多体状态、缓存与刚性树求值运行时
│   ├── multibody_model/   模型中立的程序化多体建模门面
│   ├── system_assembly/  模型中立系统组装层
│   ├── integrators/      抽象推进器与 CVODE 后端
│   ├── forces/           力元
│   └── equilibrium/      静平衡
└── tests/
    ├── comparison/       必需观测与容差判定
    ├── contract/         模型中立场景与观测语义
    ├── drake_reference/  Drake 参考发射器、跨进程比较与缓存语义探针（默认不构建）
    ├── math/             double 位姿组合的代数与输出重叠契约
    ├── multibody_model/  程序化建模门面的加入与拒绝语义
    ├── multibody_runtime/ 多体状态、类型化缓存、刚性树全对象链接与最小模型契约
    ├── topology/         vendored topology 的索引与顺序结构契约
    ├── toolchain/        工具链自检（Eigen + C++23）
    └── unit/             单元测试
```

已建模块的 `include/orvd/<module>/` 是仓内目标的公共编译接口头，`src/` 是实现；例如 `#include "orvd/multibody_runtime/multibody_state_instance.h"`。这些目标当前不安装、不导出；G29–G33 已建立程序化建模门面、最终化、稳定查询、位置、空间速度与微分运动学求值，安装与交付边界归 G46。尚未开工的模块仍只保留职责骨架。

## 外置第三方

Eigen、fmt 与精确版本 SUNDIALS 7.7.0 是当前产品的必需依赖，缺失时配置立即失败。开发者通过
标准 CMake 搜索前缀提供依赖，仓库不记录机器路径；SUNDIALS 的随仓供应与精简组件构建归 G46。
Ceres 尚无消费者，因此不查找、不设选项、不建目标。

## GZ18

GZ18 与 wheel-rail-lab 的复杂动力学模型迁移不在当前路书实施。G46 完成后将依据届时真实
消费者另建独立路书，逐步迁移车辆组装、力元、静平衡与运行流程；当前不冻结自由度、拓扑或
力元清单，模型中立运行时的底层接口不由它们提前塑形。

## 许可证

项目自有代码的许可证**待项目负责人选择**，在此之前默认版权规则适用：公开仓库本身不授予
使用、修改或再分发第一方代码的许可。后续发布工作流必须把“未选定第一方许可证不得宣称
可再分发”作为发布硬门。

已落位的 Drake 源码受 BSD-3-Clause 及其逐文件注明的 Apache-2.0 条款约束；被修改的
Apache-2.0 文件按第 4(b) 条携带改动声明。第三方与再分发说明见仓库根的
[`NOTICE`](NOTICE)，逐文件处置与来源事实见
[external/drake_mbtree/README.md](external/drake_mbtree/README.md)。
