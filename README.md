# OpenRailVehicleDynamics (ORVD)

模型中立、仅 `double`、**无需外部 Drake 运行时**的 C++ 多体动力学库。

## 当前能力边界

ORVD 是面向轨道车辆的 C++23 双精度刚性多体动力学库。现已具备多体模型装配、运动学、质量矩阵、
逆动力学与前向动力学、外力投影、复合连续状态，以及基于 CVODE 的时域积分；同时提供严格 JSON
配置、可安装的 CMake 包和独立安装消费者验证。产品运行时不链接外部 `libdrake`。

轨道车辆功能包括轨道几何与站位投影、轮轨型面处理、轮轨接触几何、EEC 法向接触、
Kalker/FASTSIM 切向接触、悬挂与车辆力元、轨道不平顺，以及八个轮轨接口的确定性 OpenMP 并行计算。

端到端资格已覆盖 GZ18 直线 AAR6 `20 s`、R300+AAR5 `16 s`，以及 IRW R300 无轨道不平顺／
冻结 AAR5 两层 `30 s` 被动工况。四轴横移与摇头均已和 SIMPACK、近期 WRL 作原生时序及同里程
核对；轮轨力的已知差异继续作为诊断边界。IRW 100 Hz 全状态控制、转矩指令调理与原子事件已经
接入真实车辆，并完成 P179 同人格 `30 s` 主动控制长窗。
实施顺序与完成条件见
[轨道车辆动力学迁移与重构路书](docs/planning/rail_vehicle_dynamics_migration/MIGRATION_ROADMAP.md)，
底层多体运行时的演进记录见
[Drake 多体运行时脱耦路书](docs/planning/drake_multibody_runtime_decoupling/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md)。

## 目录结构

```text
OpenRailVehicleDynamics/
├── CMakeLists.txt        顶层构建与安装入口
├── CMakePresets.json     开发、发布与 Drake 参考构建预设
├── cmake/                CMake 辅助模块
├── distribution/         离线依赖源码包与超级构建模板
├── docs/
│   ├── planning/         实施路书、决策账本与观察记录
│   ├── adr/              架构决策记录
│   └── engineering/      第一方工程约束
├── external/
│   └── drake_mbtree/     vendored 刚性多体树源码、替代实现与许可证
├── libs/
│   ├── track_geometry/    轨道几何、轨道坐标系与站位投影
│   ├── wheel_rail_contact/ 型面、接触几何、法向／切向接触与工作区
│   ├── configuration/     严格 JSON 加载、车辆与初始状态装配
│   ├── multibody_runtime/ 多体状态、缓存与求值运行时
│   ├── multibody_model/   模型中立的多体建模接口
│   ├── control/           状态显式的采样控制算法
│   ├── actuation/         状态显式的转矩指令调理算法
│   ├── forces/            车辆力元与轮轨接触力计划
│   ├── system_assembly/   系统装配与计算计划
│   └── integrators/       连续状态推进接口与 CVODE 后端
├── tools/
│   ├── dynamics_qualification/ 长窗动力学资格运行器（不安装）
│   ├── profile_conversion/     JSON 与 SIMPACK 型面格式转换工具（不安装）
│   └── drake_source_boundary/  Drake 源码边界检查工具
├── track_library/         轨道几何、轨型与轨道不平顺 JSON 资产
├── controller_library/    按控制人格组织的严格 JSON 资产
├── vehicle_library/       按车型组织的机械、启动、轮型、作动资产与 SIMPACK 参考模型
│   ├── gz18/              GZ18 车型资产与参考模型
│   └── irw/               IRW 车型、H3 启动、S1002 轮型、作动资产与参考模型
└── tests/                 单元、系统、安装与跨实现验证
```

公共头位于 `libs/<module>/include/orvd/<module>/`，实现位于相邻的 `src/`。安装包导出
`ORVD::control`、`ORVD::actuation`、`ORVD::track_geometry`、`ORVD::wheel_rail_contact`、`ORVD::configuration`、
`ORVD::multibody_runtime`、`ORVD::multibody_model`、`ORVD::forces`、
`ORVD::system_assembly` 与 `ORVD::integrators`；vendored Drake 类型和开发期工具不安装。

## 迁移主线

| 分册 | Goal | 目标边界 |
|---|---:|---|
| GZ18 首次短窗 | G47–G55 | 无轨道不平顺直线上的 `10 ms` 被动 CVODE 首次端到端闭环 |
| GZ18 不平顺长窗 | G56–G63 | 直线 AAR6 `10 s/20 s`、精度—性能联合基线，以及 R300 站位传播和 AAR5 `16 s` 响应验证 |
| IRW 被动 | G64–G72 | Ball-RPY、完整衬套、IRW 车型／启动／接触，以及无轨道不平顺 R300 与冻结 AAR5 两层 `30 s` |
| IRW 100 Hz 控制与平台验证 | G73–G82 | 构架—独立车轮纯力偶、全状态轮速导向、转矩指令调理、P179 同人格长窗，以及 Linux／Windows 库构建与安装验证 |

完整长窗不进入默认 CTest；跨 WRL／SIMPACK 的原始轨迹和全量统计留在版本控制之外，仓内只保留
少量摘要、对比图和短真实消费者。迁移主线已经收口于 G82：现有库、离线依赖、安装前缀重定位和
独立安装消费者完成 Linux／Windows 验证；不冻结统一场景／输出合同，也不提供安装应用或正式
发行包。Radau3、SH17、编组和后续 MPC／学习控制均不在本路书范围内。

## 已锁定的架构决策

| 决策 | 内容 |
|---|---|
| [ADR-0001](docs/adr/0001-vendor-tree-behind-allowlist.md) | 方案 B：先按允许清单 vendor Drake 刚性多体树与拓扑（仅 `double`），再逐步替换 |
| [ADR-0002](docs/adr/0002-single-authoritative-context.md) | 单一权威 Context：状态只有一个所有者，树与子系统均为零复制视图 |
| [ADR-0003](docs/adr/0003-abstract-advancer-cvode-first.md) | 抽象推进器接口，首版唯一后端为 CVODE |
| [ADR-0004](docs/adr/0004-focused-dynamics-qualification.md) | 多模型本地性质门，单个高耦合在线漂移门 |
| [ADR-0005](docs/adr/0005-bind-wheel-rail-low-level-strategies-by-vehicle.md) | 按车型绑定两处轮轨低层策略，禁止运行期混搭 |
| [ADR-0006](docs/adr/0006-deterministic-wheel-contact-openmp-batch.md) | 八接口轮轨接触采用每接口独占工作区、顺序发布的确定性 OpenMP 批求值 |
| [ADR-0007](docs/adr/0007-finite-typed-vehicle-control-event-session.md) | 周期车辆控制采用有限类型化事件会话和显式接受态同步事务 |

## 数值验收口径

- 条件良好的 `double` 跨实现连续量默认采用 `1e-8` 相对误差。
- 近零量使用按单位声明的绝对限。
- 旋转使用 `1e-8 rad` 的 SO(3) 测地角。
- 代数恒等式、有限差分与迭代求解分别使用与其误差来源相符的判据，不套用一个万能阈值。
- 车辆长窗与 SIMPACK 的比较同时检查时序和里程序列。每个工况使用一张由子图组成的主响应图，
  展示四个轴位参考体（GZ18 刚性轮对、IRW 轴桥）的轨道横向位移与摇头角；每个子图只叠加
  ORVD、SIMPACK 两条曲线，需要历史 WRL 控制列时最多三条。
- 均方根、最大值和分段统计用于量化与定位，不单独决定工况成败；还须检查相位、局部峰值、激活区段、
  接触力和动力学残差，避免低均方根掩盖局部错误，也避免仅凭一个汇总数否决可解释差异。
- `5/10/30 s` 级运行同时报告实际墙钟、动力学推进实时因子、端到端实时因子及分段耗时；不从起步
  无轨道不平顺的快速区段线性外推进入不平顺后的全程性能。实时或快于实时是明确的优化目标，但不以
  牺牲已资格响应为代价。
- 不保存数值金标、输出快照或文件哈希；不要求逐位一致。
- 参考端与候选端始终位于不同进程：`libdrake.so` 导出的符号与 vendored 副本同处
  `namespace drake`，同进程链接会构成 ODR 违规，其最可能的症状是一次看起来通过的比较。

## 外置第三方

Eigen 3.4.0、fmt 9.1.0、nlohmann/json 3.12.0、OpenMP C++ 运行时与精确版本 SUNDIALS 7.7.0
是当前产品的构建
依赖，缺失时配置立即失败。开发构建可通过标准 CMake 搜索前缀提供依赖；离线源码包携带 OpenMP
之外四个非工具链依赖的具名官方归档与许可证，由同一具备 OpenMP C++ 支持的工具链安装到私有
前缀后仍经唯一的 `find_package()` 路径消费。
SUNDIALS 自身只启用 CVODE 软件包、串行向量和稠密求解所需目标，关闭其 OpenMP、MPI、BLAS、
LAPACK 后端；ORVD 的 OpenMP 消费者是 GZ18 八接口轮轨接触批，而不是 SUNDIALS。上游无条件
构建的基础模块不作私有补丁裁剪。nlohmann/json 只编译进配置实现，不进入
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
Ubuntu 24.04 的 GCC 13 已通过完整测试；Ubuntu Clang 18 与 Windows 10 的 MSVC 19.29／clang-cl 22
已验证 Release 库和独立安装消费者。其他工具链要运行完整测试，还须配套支持当前 C++23 检查的
标准库。Windows 源码树、构建树和安装树应使用短的真实目录，不依赖目录联接。

## GZ18

GZ18 是采用刚性轮对的十七体车辆模型。车辆力元、已解析移动启动、型面、八轮接触、轨道不平顺、
CVODE 时域积分和确定性八接口并行已形成完整车辆系统；车型与资格场景由随包严格 JSON 资产装配。

### 已完成的 GZ18 工况

当前已完成以下三项长窗计算：

- **直线 AAR6 10 s 激励传播**：接入冻结的横向与垂向 AAR6 轨道不平顺，核对激励开始后的传播及
  四轮对横移、摇头响应。
- **直线 AAR6 20 s 全激励过程**：从 `t=0` 独立连续运行，覆盖激励前、淡入、全幅、淡出和恢复区段，
  形成当前直线不平顺长窗的精度与性能基线。
- **R300+AAR5 曲线工况（16 s）**：接入半径 `300 m` 的曲线线路和冻结 AAR5 轨道不平顺，完成线路站位、
  四轮对横移与摇头响应资格，并保留八轮轮轨力诊断。

三项工况均以 `60 km/h` 启动并采用 `0.5 ms` 观察周期；比较不作移时、插值、滤波、拟合、缩放或
去均值。

#### 直线 AAR6 20 s 全激励过程

![GZ18 AAR6 20 s 四轮对横移与摇头三方对比](docs/figures/gz18/gz18_aar6_20s_wheelset_response.png)

四轮对横移与摇头的同里程分段统计如下。均方根列取五个物理区段中的最差值，而不是全程平均；
最大值为全部区段的峰值。

| 响应量 | ORVD–SIMPACK 最差分段均方根 | ORVD–SIMPACK 最大差 | ORVD–历史 WRL 最大差 | 资格结论 |
|---|---:|---:|---:|---|
| 轮对质心轨道横向位移 | `38.10 µm` | `93.46 µm` | `0.0767 µm` | 通过 `0.1 mm` 门 |
| 轮对摇头角 | `11.86 µrad` | `36.51 µrad` | `0.0325 µrad` | 通过 `0.1 mrad` 门 |

![GZ18 AAR6 20 s 三向轮侧力三方对比](docs/figures/gz18/gz18_aar6_20s_wheel_force_response.png)

图中每行选择该分量 ORVD–SIMPACK 峰值最大的轮位；法向力 `N` 保留在表中。

| 力分量 | ORVD–SIMPACK 全窗均方根／最大差 | ORVD–历史 WRL 最大差 |
|---|---:|---:|
| `Q` | `1.642 / 67.111 kN` | `12.4 N` |
| `N` | `1.639 / 66.971 kN` | `12.3 N` |
| `Tx` | `0.0865 / 1.564 kN` | `3.79 N` |
| `Ty` | `0.111 / 4.351 kN` | `4.20 N` |

宏观响应已通过；轮轨力仍复现历史 WRL 相对 SIMPACK 的既有尖峰，只作诊断观察。

#### R300+AAR5 16 s 曲线响应

![GZ18 R300+AAR5 16 s 四轮对横移与摇头对比](docs/figures/gz18/gz18_r300_aar5_16s_wheelset_response.png)

主门在四轴各自的 `100–250 m / 0.01 m` 同里程格上比较 ORVD 与 SIMPACK。四轴均方根均通过
`0.1 mm / 0.1 mrad` 目标；后构架前轴在约 `223 m` 的短窄峰如实保留。

| 轮对 | 横移均方根／最大差 | 摇头均方根／最大差 |
|---|---:|---:|
| 前构架前轴 | `9.93 / 41.57 µm` | `6.82 / 36.10 µrad` |
| 前构架后轴 | `6.49 / 17.85 µm` | `5.88 / 24.73 µrad` |
| 后构架前轴 | `18.62 / 113.34 µm` | `15.69 / 103.12 µrad` |
| 后构架后轴 | `6.98 / 19.72 µm` | `5.37 / 20.86 µrad` |

![GZ18 R300+AAR5 16 s 八轮三向轮侧力对比](docs/figures/gz18/gz18_r300_aar5_16s_wheel_force_response.png)

八轮在本工况中全程保持单斑接触；轮轨力仍只作诊断观察。

| 力分量 | ORVD–SIMPACK 全接口均方根／最大差 |
|---|---:|
| 轨道竖向支承力 `Q` | `0.409 / 12.119 kN` |
| 局部法向力 `N` | `0.414 / 12.199 kN` |
| 规范轮侧纵向力 `Tx` | `0.113 / 2.821 kN` |
| 规范轮侧横向力 `Ty` | `0.087 / 2.779 kN` |

## IRW

IRW 采用轴桥与左右独立旋转车轮拓扑，并包含 Ball-RPY 纵向拉杆、半角共同中点六分量衬套、
S1002/UIC60 接触及八个独立轮轨接口。当前被动系统忠实复现冻结 WRL 公式；与 SIMPACK Type-43
的局部语义差异另行登记。

### 已完成的 IRW 工况

- **R300 A 层（30 s）**：无轨道不平顺；同站位横移／摇头最大差为 `3.05 µm / 1.16 µrad`。
- **R300+AAR5 B 层（30 s）**：保持车辆、启动、线路、积分器和时钟不变，只增加冻结 AAR5。
- **P179 100 Hz 主动控制（30 s）**：H3 启动、R300、冻结 AAR5、全状态轮速导向与转矩指令调理；
  SIMPACK Realtime 是当前主参考，冻结 Drake/WRL 是历史对照，SIMAT 按原生 100 Hz 时相单列。

![IRW R300+AAR5 30 s 四轴桥横移与摇头三方对比](docs/figures/irw/irw_r300_aar5_30s_wheelset_response.png)

四轴分别在 `100–450 m / 0.01 m` 同站位格上比较；不平顺增量先逐点扣除各平台 A 层基差，再做统计。

| 比较 | 横移最差均方根／最大差 | 摇头最差均方根／最大差 |
|---|---:|---:|
| B 层 ORVD–SIMPACK | `6.83 / 44.85 µm` | `21.35 / 78.64 µrad` |
| 扣除 A 层基差后的不平顺增量 | `6.47 / 46.71 µm` | `21.36 / 78.48 µrad` |

![IRW R300+AAR5 30 s 三向轮侧力三方对比](docs/figures/irw/irw_r300_aar5_30s_wheel_force_response.png)

图中每行选择该分量原生时序 ORVD–SIMPACK 峰值最大的轮位；`N` 保留在表中。下表取八轮中最差的
逐轮均方根和全轮最大差。

| 力分量 | 同站位均方根／最大差 | 同时间均方根／最大差 |
|---|---:|---:|
| 轨道竖向支承力 `Q` | `0.140 / 7.962 kN` | `0.194 / 18.636 kN` |
| 局部法向力 `N` | `0.139 / 6.079 kN` | `0.174 / 18.453 kN` |
| 规范轮侧纵向力 `Tx` | `0.0270 / 0.287 kN` | `0.0248 / 0.727 kN` |
| 规范轮侧横向力 `Ty` | `0.0552 / 1.726 kN` | `0.0703 / 2.659 kN` |

同站位宏观响应已通过；同时间横移／摇头峰值仍为 `180.0 µm / 125.2 µrad`，轮轨力继续只作
诊断。约 `3.659 s` 的短暂七轮接触仅作拓扑观察，不构成安全资格。

#### P179 100 Hz 主动控制 30 s

![IRW P179 30 s 四轴桥横移与摇头三方对比](docs/figures/irw/irw_p179_controlled_30s_axlebridge_response.png)

主图在 ORVD／Realtime 的原生 `0.5 ms` 时序上比较，并从冻结 WRL 的原生 `0.1 ms` 时序取共同瞬时，
不移时、不滤波。SIMAT 另按自身原生 `100 Hz` 序号核对，同样不作通信拍补偿；两条 SIMPACK 参考下，
ORVD 的宏观响应均略优于冻结 WRL。

| SIMPACK 参考 | 实现 | 横移最差轴均方根／最大差 | 摇头最差轴均方根／最大差 |
|---|---|---:|---:|
| Realtime | ORVD | `0.138 / 1.249 mm` | `0.107 / 0.996 mrad` |
| Realtime | 冻结 WRL | `0.188 / 1.363 mm` | `0.139 / 1.045 mrad` |
| SIMAT | ORVD | `0.139 / 1.238 mm` | `0.107 / 0.956 mrad` |
| SIMAT | 冻结 WRL | `0.189 / 1.353 mm` | `0.139 / 1.009 mrad` |

![IRW P179 30 s 主斑局部接触力对比](docs/figures/irw/irw_p179_controlled_30s_wheel_force_response.png)

力图左列为原生同时间三方对比，右列为 ORVD 与 Realtime 的 `100–450 m / 0.01 m` 同里程对比；
每行选择同时间峰值差最大的轮位。局部 `N/Tx/Ty` 只在各自接触系内比较，不跨斑求和或跨主斑
切换插值。下表分别在 Realtime 原生 `0.5 ms` 时序和 SIMAT 原生 `100 Hz` 序号上合并八轮有效样本。

| SIMPACK 参考 | 实现 | `N` 全轮合并均方根／最大差 | `Tx` 全轮合并均方根／最大差 | `Ty` 全轮合并均方根／最大差 |
|---|---|---:|---:|---:|
| Realtime | ORVD | `2.756 / 205.809 kN` | `0.523 / 16.873 kN` | `0.744 / 53.658 kN` |
| Realtime | 冻结 WRL | `2.968 / 214.455 kN` | `0.563 / 16.625 kN` | `0.810 / 55.257 kN` |
| SIMAT | ORVD | `2.883 / 117.289 kN` | `0.586 / 17.664 kN` | `0.768 / 31.595 kN` |
| SIMAT | 冻结 WRL | `3.004 / 127.164 kN` | `0.607 / 17.239 kN` | `0.817 / 32.963 kN` |

三向力峰值集中在离散主斑切换或采样瞬时失联附近，未见轮位长期零力；因此力响应仍作诊断，
不扩张为安全资格。本机 `30 s` 动力学推进墙钟为 ORVD `2350 s`、SIMPACK Realtime 循环
`2402 s`；求解器与统计边界不同，该数字只作真实性能观察。

## 许可证

项目负责人已允许非盈利用户使用、修改与再分发第一方代码；商业生产使用仍须另行授权。
正式许可证主体、法律文本和统一转换日期尚未落地，因此当前只陈述该使用边界，不把仓库称为
已经按标准许可证正式发布。计划中的正式文本为初期 BSL 1.1、三年后统一转 Apache-2.0。

已落位的 Drake 源码受 BSD-3-Clause 及其逐文件注明的 Apache-2.0 条款约束；被修改的
Apache-2.0 文件按第 4(b) 条携带改动声明。第三方与再分发说明见仓库根的
[`NOTICE`](NOTICE)，逐文件处置与来源事实见
[external/drake_mbtree/README.md](external/drake_mbtree/README.md)。
