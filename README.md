# OpenRailVehicleDynamics (ORVD)

模型中立、仅 `double`、以**零 Drake 运行期依赖**为目标的 C++ 多体运行时。

## 当前状态

基础设施阶段，**尚无产品实现**。仓库目前只有工程骨架与实施依据。

唯一实施依据是
[Drake 多体运行时脱耦路书](docs/planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md)：
它定义 18 个子目标、50 个原子 Goal 的顺序与完成门。当前进度以路书中的「当前 Goal」为准。

## 已锁定的架构决策

| 决策 | 内容 |
|---|---|
| [ADR-0001](docs/adr/0001-vendor-tree-behind-allowlist.md) | 方案 B：先按允许清单 vendor Drake 刚性多体树与拓扑（仅 `double`），再逐步替换 |
| [ADR-0002](docs/adr/0002-single-authoritative-context.md) | 单一权威 Context：状态只有一个所有者，树与子系统均为零复制视图 |
| [ADR-0003](docs/adr/0003-abstract-advancer-cvode-first.md) | 抽象推进器接口，首版唯一后端为 CVODE |

## 数值验收口径

- 连续量以工程相对误差 `1e-3` 为主。
- 近零量使用按单位声明的绝对限。
- 旋转使用 SO(3) 角度误差。
- 不保存数值金标、输出快照或文件哈希；不要求逐位一致。
- 参考端与候选端始终位于不同进程：`libdrake.so` 导出的符号与 vendored 副本同处
  `namespace drake`，同进程链接会构成 ODR 违规，其最可能的症状是一次看起来通过的比较。

## 目录结构

```text
OpenRailVehicleDynamics/
├── CMakeLists.txt        顶层构建（骨架，尚不产出目标）
├── CMakePresets.json     构建预设：dev / release
├── cmake/                CMake 辅助模块
├── docs/
│   ├── planning/         唯一实施路书
│   ├── adr/              架构决策记录
│   ├── design/           历史调研输入（非现行依据）
│   └── review/           历史审查往来（非现行依据）
├── external/
│   └── drake_mbtree/     vendored Drake 刚性多体树与拓扑的落位（尚未 vendor）
├── libs/
│   ├── core/             运行时地基：Context、缓存、求值调度
│   ├── systems/          模型中立组装层
│   ├── integrators/      抽象推进器与 CVODE 后端
│   ├── forces/           力元
│   └── equilibrium/      静平衡
└── tests/
    └── unit/             单元测试
```

模块的 `include/orvd/<module>/` 为公开头，`src/` 为实现；下游 include 形如
`#include "orvd/core/context.h"`。各模块 `src/` 目前为空。

## 外置第三方

Eigen、SUNDIALS CVODE、Ceres。构建入口由 G04 建立，Eigen 在 G05 引入；SUNDIALS
CVODE 与 Ceres 留到首个真实消费者出现时再引入。

## GZ18

GZ18 刚性轮对是**晚期适配对象**，在 G48–G50 接入。模型中立运行时的底层接口不由它塑形。

## 许可证

项目自有代码许可证待定。日后 vendored 的 Drake 源码为 BSD-3-Clause（Robot Locomotion
Group），须随分发保留其许可证与 NOTICE，详见
[external/drake_mbtree/README.md](external/drake_mbtree/README.md)。
