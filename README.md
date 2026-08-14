# OpenRailVehicleDynamics (ORVD)

模型中立、仅 `double`、**无需外部 Drake 运行时**的 C++ 多体动力学库。

## 当前能力边界

ORVD 是面向轨道车辆的 C++23 双精度刚性多体动力学库，提供多体模型装配、运动学、质量矩阵、
逆动力学与前向动力学、外力投影、复合连续状态，以及基于 CVODE 的时域积分；同时提供严格 JSON
配置和可安装的 CMake 包。产品运行时不链接外部 `libdrake`。

轨道车辆功能包括轨道几何与站位投影、轮轨型面处理、轮轨接触几何、EEC 法向接触、
Kalker/FASTSIM 切向接触、悬挂与车辆力元、轨道不平顺，以及八个轮轨接口的确定性 OpenMP 并行计算。

下文给出 GZ18 直线 AAR6 `20 s`、R300+AAR5 `16 s`，以及 IRW R300+AAR5 被动和 P179
100 Hz 受控 `30 s` 工况的响应与性能结果。

## 目录结构

```text
OpenRailVehicleDynamics/
├── CMakeLists.txt        顶层构建与安装入口
├── CMakePresets.json     开发、发布与 Drake 参考构建预设
├── cmake/                CMake 辅助模块
├── distribution/         离线依赖源码包与超级构建模板
├── docs/
│   ├── models_and_algorithms/ 物理模型、数学关系、算法语义与参考实现比较
│   ├── planning/         实施路书、决策账本与观察记录
│   ├── performance/      计算性能路书与专项研究档案
│   ├── adr/              架构决策记录
│   ├── design/           软件架构、运行时契约与早期设计输入
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

## 结果口径

- 响应曲线直接使用原始输出，不作移时、滤波、拟合、缩放或去均值。
- GZ18 曲线工况按相同里程比较，IRW P179 工况按相同时间点比较；表格同时给出均方根和峰值差。
- 实时系数为仿真时长除以实际用时；分别报告动力学推进和包含观测、控制同步及写盘的总计算结果。

## 构建与安装

构建依赖为 Eigen 3.4.0、fmt 9.1.0、nlohmann/json 3.12.0、OpenMP C++ 运行时和 SUNDIALS 7.7.0。
已有依赖可通过标准 CMake 搜索前缀提供；缺失时配置失败。

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
构建工具链及其标准库须支持项目使用的 C++23 特性。Windows 源码树、构建树和安装树应使用短的
真实目录，不依赖目录联接。

## GZ18

GZ18 是采用刚性轮对的十七体车辆模型，包括车辆力元、解析移动启动、型面、八轮接触、轨道不平顺、
CVODE 时域积分和确定性八接口并行；车型与工况由随包严格 JSON 资产装配。

### GZ18 工况

- **直线 AAR6 20 s 全激励过程**：从 `t=0` 独立连续运行，覆盖激励前、淡入、全幅、淡出和恢复区段，
  比较完整激励过程的响应。
- **R300+AAR5 曲线工况（16 s）**：接入半径 `300 m` 的曲线线路和 AAR5 轨道不平顺，比较线路站位、
  四轮对横移与摇头响应。

两项工况均以 `60 km/h` 启动并采用 `0.5 ms` 观察周期；比较不作移时、插值、滤波、拟合、缩放或
去均值。

#### 直线 AAR6 20 s 全激励过程

![GZ18 AAR6 20 s 四轮对横移与摇头对比](docs/figures/gz18/gz18_aar6_20s_wheelset_response.png)

四轮对横移与摇头的同里程分段统计如下。均方根列取五个物理区段中的最差值，而不是全程平均；
最大值为全部区段的峰值。

| 响应量 | ORVD–SIMPACK 最差分段均方根 | ORVD–SIMPACK 最大差 | 目标 |
|---|---:|---:|---:|
| 轮对质心轨道横向位移 | `38.10 µm` | `93.46 µm` | `≤ 0.1 mm` |
| 轮对摇头角 | `11.86 µrad` | `36.51 µrad` | `≤ 0.1 mrad` |

![GZ18 AAR6 20 s 三向轮侧力对比](docs/figures/gz18/gz18_aar6_20s_wheel_force_response.png)

图中每行选择该分量 ORVD–SIMPACK 峰值最大的轮位；法向力 `N` 保留在表中。

| 力分量 | ORVD–SIMPACK 全窗均方根／最大差 |
|---|---:|
| `Q` | `1.642 / 67.111 kN` |
| `N` | `1.639 / 66.971 kN` |
| `Tx` | `0.0865 / 1.564 kN` |
| `Ty` | `0.111 / 4.351 kN` |

#### R300+AAR5 16 s 曲线响应

![GZ18 R300+AAR5 16 s 四轮对横移与摇头对比](docs/figures/gz18/gz18_r300_aar5_16s_wheelset_response.png)

四个轮对分别在 `100–250 m`、步长 `0.01 m` 的同里程网格上比较 ORVD 与 SIMPACK。均方根均低于
`0.1 mm / 0.1 mrad`；后构架前轴在约 `223 m` 出现短窄峰。

| 轮对 | 横移均方根／最大差 | 摇头均方根／最大差 |
|---|---:|---:|
| 前构架前轴 | `9.93 / 41.57 µm` | `6.82 / 36.10 µrad` |
| 前构架后轴 | `6.49 / 17.85 µm` | `5.88 / 24.73 µrad` |
| 后构架前轴 | `18.62 / 113.34 µm` | `15.69 / 103.12 µrad` |
| 后构架后轴 | `6.98 / 19.72 µm` | `5.37 / 20.86 µrad` |

![GZ18 R300+AAR5 16 s 八轮三向轮侧力对比](docs/figures/gz18/gz18_r300_aar5_16s_wheel_force_response.png)

八轮在本工况中全程保持单斑接触。

| 力分量 | ORVD–SIMPACK 全接口均方根／最大差 |
|---|---:|
| 轨道竖向支承力 `Q` | `0.409 / 12.119 kN` |
| 局部法向力 `N` | `0.414 / 12.199 kN` |
| 规范轮侧纵向力 `Tx` | `0.113 / 2.821 kN` |
| 规范轮侧横向力 `Ty` | `0.087 / 2.779 kN` |

## IRW

IRW 采用轴桥与左右独立旋转车轮拓扑，并包含 Ball-RPY 纵向拉杆、半角共同中点六分量衬套、
S1002/UIC60 接触及八个独立轮轨接口。

### 30 s 结果

| 工况 | 动力学用时／实时系数 | 总计算用时／实时系数 |
|---|---:|---:|
| B：R300+AAR5 被动 | `27.473 s / 1.092×` | `31.915 s / 0.940×` |
| C：P179 100 Hz 受控 | `33.435 s / 0.897×` | `38.683 s / 0.776×` |

#### P179 100 Hz 受控响应

![IRW P179 30 s 四轴桥横移与摇头对比](docs/figures/irw/irw_p179_controlled_30s_axlebridge_response.png)

ORVD 与 SIMPACK Realtime 按相同时间点直接比较，四个轴桥中最大横移均方根／峰值差为
`0.147 / 1.290 mm`，最大摇头均方根／峰值差为 `0.115 / 0.994 mrad`。

![IRW P179 30 s 主斑局部接触力对比](docs/figures/irw/irw_p179_controlled_30s_wheel_force_response.png)

按相同时间点直接比较并合并八个车轮，主接触斑局部力 `N / Tx / Ty` 的均方根为
`2.697 / 0.527 / 0.739 kN`，峰值差为 `206.168 / 16.635 / 53.932 kN`。

## 许可证

项目负责人已允许非盈利用户使用、修改与再分发第一方代码；商业生产使用仍须另行授权。
正式许可证主体、法律文本和统一转换日期尚未落地，因此当前只陈述该使用边界，不把仓库称为
已经按标准许可证正式发布。计划中的正式文本为初期 BSL 1.1、三年后统一转 Apache-2.0。

已落位的 Drake 源码受 BSD-3-Clause 及其逐文件注明的 Apache-2.0 条款约束；被修改的
Apache-2.0 文件按第 4(b) 条携带改动声明。第三方与再分发说明见仓库根的
[`NOTICE`](NOTICE)，逐文件处置与来源事实见
[external/drake_mbtree/README.md](external/drake_mbtree/README.md)。
