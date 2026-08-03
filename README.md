# OpenRailVehicleDynamics (ORVD)

模型中立、仅 `double`、以**零 Drake 运行期依赖**为目标的 C++ 多体运行时。

## 当前状态

vendored Drake common support、刚性 topology、double 位姿数学与完整 double-only 刚性树
均已有内部构建目标。第一方运行时拥有单一多体状态、类型化参数、版本缓存与具名工作区；
G29–G39 已接通模型中立建模、运动学、质量矩阵、逆动力学、具名外力和 O(n) 前向动力学，
G40–G45 已接通静态系统组装、上下文局部阻尼、连续状态原子事务、RHS 桥、真实 CVODE 后端
以及接受/试算提交边界。G46 已落下可安装包、离线依赖源码包和独立消费者资格，并在
Ubuntu 24.04 的 GCC 13、Clang 18 以及 Windows 10 的 MSVC 19.29 上实际构建并运行同一
CVODE 消费者。底座的 46 个 Goal 已全部完成；车辆动力学迁移路书已经启用，当前 G47 的
轨道几何实现正在对抗复核收口，G48 尚未进入。

底座的实施记录是
[Drake 多体运行时脱耦路书](docs/planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md)；
当前车辆工作的唯一进度权威是
[轨道车辆动力学迁移与重构路书](docs/planning/rail_vehicle_dynamics_migration/MIGRATION_ROADMAP.md)。

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
├── distribution/         离线依赖源码包的锁表与超级构建模板
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
│   ├── product_boundary_gate/  链接边界闸门的判别力自检（开发期）
│   └── package_distribution/   开发者侧离线源码包组装工具
├── libs/
│   ├── track_geometry/    线路惯性系、轨道几何、轨型系与站位投影
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

已建模块的 `include/orvd/<module>/` 是公共编译接口头，`src/` 是实现；例如
`#include "orvd/multibody_runtime/multibody_state_instance.h"`。安装包导出
`ORVD::track_geometry`、`ORVD::multibody_runtime`、`ORVD::multibody_model`、
`ORVD::system_assembly` 与 `ORVD::integrators`；vendored Drake 类型和接入头不安装。
尚未开工的模块仍只保留职责骨架。

## 外置第三方

Eigen 3.4.0、fmt 9.1.0 与精确版本 SUNDIALS 7.7.0 是当前产品的必需依赖，缺失时配置立即
失败。开发构建可通过标准 CMake 搜索前缀提供依赖；离线源码包则携带三者的锁定官方归档与
许可证，由同一工具链安装到私有前缀后仍经唯一的 `find_package()` 路径消费。SUNDIALS 只启用
CVODE 软件包、串行向量和稠密求解所需目标，关闭 OpenMP、MPI、BLAS、LAPACK 等当前无消费者
后端；上游无条件构建的基础模块不作私有补丁裁剪。Ceres 尚无消费者，因此不查找、不设选项、
不进源码包。

## 构建与安装

已有依赖前缀时可直接安装并由外部工程消费：

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=<dependency-prefix> \
  -DCMAKE_INSTALL_PREFIX=<orvd-prefix> \
  -DBUILD_TESTING=OFF
cmake --build build
cmake --install build
```

外部 CMake 工程使用 `find_package(OpenRailVehicleDynamics CONFIG REQUIRED)`，按所需最高层链接
例如 `ORVD::integrators`。无需包含源码树、vendored Drake 头或接入层头。

不依赖系统开发包的离线方式见
[`distribution/dependencies/README.md`](distribution/dependencies/README.md)：开发者先用显式工具
组装源码包，普通使用者只需 CMake、C/C++ 工具链和构建器即可完成全部依赖与 ORVD 的构建安装。
该机制已经在 Ubuntu 24.04 上以 GCC 13 和 Clang 18、在 Windows 10 上以 Visual Studio 2019
16.11 / MSVC 19.29 运行同一真实 CVODE 消费者。Windows 构建还验证了迁移后的安装前缀、
运行期依赖闭包与 Drake 标记动态库阳性对照。

## GZ18

GZ18 是当前车辆迁移路书的首个真实车型消费者。迁移按轨道几何、配置、车辆拓扑、启动状态、
力元、轮轨接触、数值历史与端到端被动纵切逐 Goal 推进；当前只收口 G47，不以尚不存在的
后续消费者提前塑造产品接口。

## 许可证

项目自有代码的许可证**待项目负责人选择**，在此之前默认版权规则适用：公开仓库本身不授予
使用、修改或再分发第一方代码的许可。后续发布工作流必须把“未选定第一方许可证不得宣称
可再分发”作为发布硬门。

已落位的 Drake 源码受 BSD-3-Clause 及其逐文件注明的 Apache-2.0 条款约束；被修改的
Apache-2.0 文件按第 4(b) 条携带改动声明。第三方与再分发说明见仓库根的
[`NOTICE`](NOTICE)，逐文件处置与来源事实见
[external/drake_mbtree/README.md](external/drake_mbtree/README.md)。
