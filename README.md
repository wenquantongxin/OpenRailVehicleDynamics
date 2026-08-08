# OpenRailVehicleDynamics (ORVD)

模型中立、仅 `double`、以**零 Drake 运行期依赖**为目标的 C++ 多体运行时。

## 当前状态

vendored Drake common support、刚性 topology、double 位姿数学与完整 double-only 刚性树
均已有内部构建目标。第一方运行时拥有单一多体状态、类型化参数、版本缓存与具名工作区；
G29–G39 已接通模型中立建模、运动学、质量矩阵、逆动力学、具名外力和 O(n) 前向动力学，
G40–G45 已接通静态系统组装、上下文局部阻尼、连续状态原子事务、RHS 桥、真实 CVODE 后端
以及接受/试算提交边界。G46 已落下可安装包、离线依赖源码包和独立消费者资格，并在
Ubuntu 24.04 的 GCC 13、Clang 18 以及 Windows 10 的 MSVC 19.29 上实际构建并运行同一
CVODE 消费者。底座的 46 个 Goal 已全部完成；车辆动力学迁移路书已经启用，G47 轨道几何、
G48 严格 JSON 线路配置、G49 GZ18 车型装配、G50 复合连续状态与 GZ18 车辆力元、G51 已解析启动
状态与初始上下文装配均已签收。G52 轮轨型面与串行接触核心按 G52a／G52b／G52c 三个强制审查子步推进；
G52a 与 G52b 已完成阶段审查，G52c 产品落地与两轮自检已经完成，正在等待阶段审查。G52 总 Goal 尚未
签收，当前暂停于 G53 前。
G52a 已落下型面值对象、JSON 型面模式及其严格加载、与 SIMPACK
`.prw/.prr` 的语义读写与双向转换、自然三次与单调保形两种插值原语、弧长求积与站点求根，
以及两套车轮型面预处理及其构造期互斥、型面/轨道侧滚横垂输运的数学与其类型化启用策略。G52b 已接通
共同法向接触几何、三维互穿与解析长度基线、EEC 法向力、蠕滑率、Kalker 系数、FASTSIM 切向力和
同点约化的轮侧／轨侧成对扳手；G52c 已接通随包 GZ18 JSON 型面、类型化人格与安装数据根绑定。

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
| [ADR-0004](docs/adr/0004-focused-dynamics-qualification.md) | 多模型本地性质门，单个高耦合在线漂移门 |
| [ADR-0005](docs/adr/0005-bind-wheel-rail-low-level-strategies-by-vehicle.md) | 按车型绑定两处轮轨低层策略，禁止运行期混搭 |

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
│   ├── wheel_rail_contact/ 型面、接触几何、法向／切向力学、成对扳手与每上下文工作区
│   ├── configuration/     严格 JSON 的一次性类型化加载边界、车辆装配与初始上下文装配
│   ├── multibody_runtime/ 多体状态、缓存与刚性树求值运行时
│   ├── multibody_model/   模型中立的程序化多体建模门面
│   ├── system_assembly/  模型中立系统组装层
│   ├── integrators/      抽象推进器与 CVODE 后端
│   ├── forces/           力元
│   └── equilibrium/      静平衡
├── track_library/
│   ├── geometries/       可安装的线路几何 JSON 记录
│   └── rail_profiles/    可安装的轨型 JSON 记录
├── vehicle_library/
│   └── gz18/             可安装的 GZ18 车型 JSON 记录
│       ├── startup_states/  可安装的已解析启动状态 JSON 记录
│       └── wheel_profiles/  可安装的 GZ18 轮型 JSON 记录
└── tests/
    ├── comparison/       必需观测与容差判定
    ├── configuration/    线路、车型与启动状态 JSON 的严格加载边界及初始上下文装配
    ├── contract/         模型中立场景与观测语义
    ├── drake_reference/  Drake 参考发射器、跨进程比较与缓存语义探针（默认不构建）
    ├── forces/           力元与复合连续状态
    ├── header_diagnostics/ vendored 刚性树接入头的自包含（这些头不安装）
    ├── installation/     迁移后安装前缀的消费者资格
    ├── integrators/      推进器后端与接受事务
    ├── math/             double 位姿组合的代数与输出重叠契约
    ├── multibody_model/  程序化建模门面的加入与拒绝语义
    ├── multibody_runtime/ 多体状态、类型化缓存、刚性树全对象链接与最小模型契约
    ├── orvd_candidate/   第一方候选实现的跨进程发射器
    ├── system_assembly/  系统实例与编译计划
    ├── topology/         vendored topology 的索引与顺序结构契约
    ├── toolchain/        工具链自检（Eigen + C++23）
    ├── track_geometry/   轨道几何与站位投影
    ├── vehicle_library/  GZ18 车型记录与多体装配
    ├── wheel_rail_contact/ 型面、接触核心、GZ18 人格与安装绑定
    └── unit/             单元测试
```

已建模块的 `include/orvd/<module>/` 是公共编译接口头，`src/` 是实现；例如
`#include "orvd/multibody_runtime/multibody_state_instance.h"`。安装包导出
`ORVD::track_geometry`、`ORVD::wheel_rail_contact`、`ORVD::configuration`、`ORVD::multibody_runtime`、
`ORVD::multibody_model`、`ORVD::forces`、`ORVD::system_assembly` 与 `ORVD::integrators`；
vendored Drake 类型和接入头不安装。`tools/profile_conversion/` 是开发期工具，不导出、不安装。
安装包还带上线路几何、轨型、GZ18 车型、轮型与已解析启动状态 JSON 记录。G52c 已提供从逻辑标识到
显式安装数据根下 JSON 资产的解析；它不搜索环境变量、当前目录或源码树。
尚未开工的模块仍只保留职责骨架。

## 外置第三方

Eigen 3.4.0、fmt 9.1.0、nlohmann/json 3.12.0 与精确版本 SUNDIALS 7.7.0 是当前产品的构建
依赖，缺失时配置立即失败。开发构建可通过标准 CMake 搜索前缀提供依赖；离线源码包则携带四者
的具名官方归档与许可证，由同一工具链安装到私有前缀后仍经唯一的 `find_package()` 路径消费。
SUNDIALS 只启用
CVODE 软件包、串行向量和稠密求解所需目标，关闭 OpenMP、MPI、BLAS、LAPACK 等当前无消费者
后端；上游无条件构建的基础模块不作私有补丁裁剪。nlohmann/json 只编译进配置实现，不进入
公共头或安装包依赖。Ceres 尚无消费者，因此不查找、不设选项、
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

GZ18 是当前车辆迁移路书的首个真实车型消费者。迁移按轨道几何、配置、车辆拓扑、力元、启动状态、
轮轨接触、数值历史与端到端被动纵切逐 Goal 推进。已落位的是：随包线路几何与 GZ18 车型记录、
十七体多体装配、复合连续状态与车辆力元、以及 60 km/h 已解析移动启动状态及其初始上下文装配。
轮轨型面基础设施与串行接触核心已经通过 G52a／G52b 阶段审查；G52c 已接入随包 JSON 型面、GZ18
类型化接触人格和安装数据根，并完成两轮自检。当前等待阶段审查，G52 尚未签收，也不进入 G53 的系统接线。

轮轨型面资产的边界：ORVD 随包发布本项目自有、自包含的 JSON 型面记录，它是运行时型面的唯一真源；
第一方设施同时提供与 SIMPACK `.prw/.prr` 的语义读写与双向转换（`tools/profile_conversion/`），
供本地科研核对使用。该兼容通路只实现资格化资产用到的那个子集，对它未实现的平移、镜像、旋转、
缩放与裁剪声明一律响亮拒绝，而不是默默忽略；直接 `.prw/.prr` 读写时，文件声明的有效遍历方向
被记录并同义写回。经过产品 JSON 后不保留 SIMPACK 专有元数据，再导出时按 ORVD 的规范型面角色
生成遍历声明；输出点行统一升序，方向若不另行承载就会在往返中被静默抹平。供应商型面文件不进入
本仓库、不随安装包发布，也不作为运行时权威；随包 JSON 型面资产已经由 G52c 落地。运行时以无扩展名的
逻辑标识经安装数据根解析 JSON；本地 `.prw/.prr` 只参加一次性语义互操作核对，不成为产品身份、数值金标
或第二份运行时输入。项目不依赖 CONTACT。

## 许可证

项目负责人已允许非盈利用户使用、修改与再分发第一方代码；商业生产使用仍须另行授权。
正式许可证主体、法律文本和统一转换日期尚未落地，因此当前只陈述该使用边界，不把仓库称为
已经按标准许可证正式发布。计划中的正式文本为初期 BSL 1.1、三年后统一转 Apache-2.0。

已落位的 Drake 源码受 BSD-3-Clause 及其逐文件注明的 Apache-2.0 条款约束；被修改的
Apache-2.0 文件按第 4(b) 条携带改动声明。第三方与再分发说明见仓库根的
[`NOTICE`](NOTICE)，逐文件处置与来源事实见
[external/drake_mbtree/README.md](external/drake_mbtree/README.md)。
