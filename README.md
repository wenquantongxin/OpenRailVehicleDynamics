# OpenRailVehicleDynamics (ORVD)

> 对 Drake **零运行期依赖**、自包含、跨平台的 C++ 轨道车辆动力学求解核心。
> 保真基准是**复现 `wheel-rail-lab` 中 GZ18 刚性轮对的当前 Drake 行为**（同平台同编译选项逐位一致）。

---

## 这是什么

ORVD 把 `wheel-rail-lab` 里 GZ18 刚性轮对的 Drake 仿真线，重构为一套不链接 `libdrake.so`
的独立 C++ 软件。多体动力学 RHS（Featherstone ABA、质量阵、运动学、Jacobian）以
**允许清单方式 vendor** 自 Drake 的 `multibody/tree` + `multibody/topology`（BSD-3，仅 double 标量），
外面套自写的单一权威 Context、mini-LeafSystem 运行时与 CVODE 积分后端。

**它不做什么**：不承担与 SIMPACK 的对齐（那是并行的、`wheel-rail-lab` 侧的工作）；
不移植 Drake 的碰撞检测、SceneGraph、优化求解器、可变形体、自动微分。

## 当前状态

**预实施阶段（pre-implementation）。** 本仓库目前只包含项目结构、构建骨架与经过两轮
对抗审查的设计文档；尚无实现代码。工作量预评估：**M0–M3 合计 50–80 人周**（单人约 12–19 个月）。

设计基线已冻结为 **REV2**（见 [docs/design/DESIGN_BASIS.md](docs/design/DESIGN_BASIS.md) §10）。

## 目录结构

```text
OpenRailVehicleDynamics/
├── README.md                  # 本文件
├── LICENSE                    # 项目许可证（待定；vendored Drake 为 BSD-3，见 external/）
├── CMakeLists.txt             # 顶层构建骨架（预实施，尚不产出目标）
├── CMakePresets.json          # 构建预设：parity（fastmath 关）/ dev / release
├── .clang-format              # 代码风格
├── .gitignore
├── cmake/                     # Find*.cmake、工具链（Linux / MSVC）、CI 辅助模块
├── docs/
│   ├── README.md              # 文档索引
│   ├── planning/
│   │   └── PROJECT_PHASES.md  # ★ 规划书：分阶段难点 / 测试点 / 工时
│   ├── design/
│   │   └── DESIGN_BASIS.md    # ★ 设计基线 + 两轮对抗审查记录（REV2）
│   ├── adr/                   # 架构决策记录（已锁定的关键决策）
│   └── review/                # 审查往来存档
├── external/
│   └── drake_mbtree/          # vendored multibody/tree + topology（允许清单、double-only、BSD-3 NOTICE）
├── libs/                      # 第一方模块库（public 头在 include/orvd/<module>/）
│   ├── core/                  # 空间数学垫片 + 单一 Context/cache + 静态求值调度 + facade
│   ├── systems/               # mini-LeafSystem 运行时（端口 / 状态 / 事件）
│   ├── integrators/           # 抽象推进器接口 + CVODE 后端（drake_family 仅预留）
│   ├── forces/                # 力元：重力 / bushing / Maxwell 阻尼 / specialized suspension
│   ├── vehicles/              # 整车组装模板（gz18/ 首车型）
│   └── equilibrium/           # 静平衡（Ceres 外置）
├── apps/
│   └── gz18_sim/              # GZ18 仿真可执行入口
├── tests/
│   ├── unit/                  # 逐模块单元测试
│   ├── parity/                # ★ Drake 差分预言机台架（M0 三层金标）
│   └── golden/                # 冻结金标向量 / 有序轨迹夹具
└── tools/                     # 脚本：oracle 冻结、哈希清单、快照归档、符号扫描
```

`libs/` 下每个模块自带 `include/orvd/<module>/`（public 头）与 `src/`（实现）；
下游 include 形如 `#include "orvd/core/context.h"`。各模块目录内 `README.md` 记录其职责、
所属里程碑、以及必须复现的 Drake 保真陷阱。

## 构建（预实施占位）

要求：C++23、CMake ≥ 3.24。外置第三方：Eigen3、SUNDIALS CVODE、Ceres（跨平台、许可宽松、按需调用）。

```bash
cmake --preset parity      # parity 构建：-ffast-math / -march=native 强制关闭
cmake --build build/parity
```

> **parity 口径**：同平台同构建身份下逐位一致；跨平台仅做工程容差回归——
> 逐位跨系统相等既不追求也不可达（瓶颈是 libm，非编译选项，见设计基线 §6）。

当前各模块 `src/` 为空，构建骨架仅完成 project/标准/选项声明，尚不产出库或可执行文件。

## 路线图（里程碑）

| 里程碑 | 目标 | 工时 |
|---|---|---:|
| **M0** | 基线冻结 + 三层差分预言机台架 | 5–8 pw |
| **M1** | 允许清单抽取 vendored tree + 被动门面（仍链 Drake） | 6–10 pw |
| **M2** | 单 Context + mini-systems + CVODE 切换 → **零 Drake 达成** | 22–35 pw |
| **M3** | 快照 / 静平衡 / 输出契约 / CLI / Linux+MSVC 打包 | 13–20 pw |
| M4（可选） | 逐 pass 自研化，摆脱 vendored 源码 | +25–40 pw |

完整的难点 / 测试点 / 摆动因子见 **[docs/planning/PROJECT_PHASES.md](docs/planning/PROJECT_PHASES.md)**。

## 关键设计决策（已锁定，详见 docs/adr/）

- **方案 B**：先 vendor `multibody/tree`+`topology` 再逐 pass 替换，而非一次性全自研 —— [ADR-0001](docs/adr/0001-vendor-tree-behind-allowlist.md)
- **单一权威 Context**：根上下文唯一持有状态，树与子系统均为零复制视图 —— [ADR-0002](docs/adr/0002-single-authoritative-context.md)
- **抽象推进器 + CVODE 优先**：`Simulator::AdvanceTo` 仅留接口不实现，首版只做 CVODE 后端 —— [ADR-0003](docs/adr/0003-abstract-advancer-cvode-first.md)

## 参考基准（均只读）

- 源仓库：`wheel-rail-lab` @ `9355d17`
- Drake：v1.54.0 @ `231c260201ee`（安装于 `/opt/drake`，源码浅克隆于 `../drake-extraction-study/drake-src`）

## 许可证

项目自有代码许可证待项目负责人确定。`external/drake_mbtree/` 下 vendored 代码为
**BSD-3-Clause**（Robot Locomotion Group），须随分发保留其许可证与 NOTICE，详见
[external/drake_mbtree/README.md](external/drake_mbtree/README.md)。
