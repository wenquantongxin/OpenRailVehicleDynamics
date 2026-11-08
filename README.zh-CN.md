<div align="center">

# OpenRailVehicleDynamics

[English](README.md) · **简体中文**

**首款面向轨道车辆的开源 C++ 动力学计算平台**

[![License: BUSL-1.1](https://img.shields.io/badge/license-BUSL--1.1-blue)](LICENSE) ![C++23](https://img.shields.io/badge/C%2B%2B-23-00599C) ![CMake 3.24+](https://img.shields.io/badge/CMake-3.24%2B-064F8C) ![Linux | Windows | macOS](https://img.shields.io/badge/platform-Linux%20%7C%20Windows%20%7C%20macOS-lightgrey)

<img src="apps/scene_viewer_web/screenshots/irw_r300_overview.png" width="900" alt="ORVD 场景查看器回放 IRW 车辆通过 R300 曲线的 ORVD 仿真记录">

</div>

**OpenRailVehicleDynamics**（ORVD）将三维线路、轨道不平顺、轮轨接触、车辆多体系统、悬挂与作动、采样控制和时间积分连接为一条透明的计算链。用户以 JSON 定义装配完整车辆，计算车体与轮对运动、逐轮接触斑、三向轮轨力、蠕滑率和控制事件。

## 核心能力

| 领域 | 能力 |
|---|---|
| 多体车辆 | 刚体树装配、位置／速度运动学、质量矩阵、逆／前向动力学与外力投影；平动弹簧阻尼、串联弹簧黏性阻尼、点对点串联弹簧双斜率阻尼、饱和分段阻尼、侧滚元、参考架线性衬套、重力与八路轮轨力和力矩 |
| 线路与不平顺 | 平面曲率、超高、恒坡、抛物线与圆弧竖曲线、五次接缝及其三维耦合，局部站位投影与边界切线延长；冻结空间点列与可复核的 AAR5／AAR6 空间谱，AAR5／AAR6 横向、竖向、轨距与侧滚不平顺，ERRI low 横／垂向不平顺 |
| 轮轨接触 | 等弧长型面预处理、轮轨外形插值、多接触斑几何、EEC 法向力、Kalker 系数、FASTSIM 切向力与逐斑蠕滑率 |
| 控制与作动 | 全状态轮速导向、纵向巡航控制与转矩整形；周期控制事件在接受态上原子提交 |
| 时间积分 | CVODE BDF2／BDF5、Radau5、带有界缩步恢复的 Newmark、Zhai；统一强类型配置、目标时刻推进、步内采样与接受态事务 |
| 高效计算 | 八处轮轨接触的 OpenMP 并行求值，固定槽发布、无跨轮浮点归约，结果不随工作者数量变化；CVODE 稠密差分 Jacobian 按列并行 |
| 集成与展示 | JSON 定义装配、可重定位 CMake 安装、位置无关静态归档、离线依赖源码包；`scene_observation` 采样、`scene_record` 记录与资源、`scene_stream` 编码、`state_stream` UDP 发送、浏览器三维回放 |

```mermaid
flowchart LR
  A[车型、线路与控制器 JSON] --> B[三维轨道与不平顺]
  B --> C[轮轨接触]
  A --> D[多体车辆与力元]
  C --> D
  D --> E[时间积分]
  E --> G[车辆状态、接触斑与轮轨力]
  G -->|采样| F[采样控制与转矩整形]
  F -->|外部输入| D
```

整条计算链以双精度状态运行。线路层负责三维姿态和站位，轮轨层把型面与轨道激励转换为接触斑及作用力，多体层形成连续状态右端，时间积分完成推进与稠密输出；周期控制事件在接受态上原子提交，再进入下一段动力学。线路库覆盖直线、多种半径的圆曲线，以及同时包含平面曲线、超高和竖向剖面的线路；IRW 被动场景覆盖 60–200 km/h 的多个速度等级，可组合 AAR5、AAR6 与 ERRI low 不平顺。

## 整车验证

SIMPACK 承担整车参考。轮轨力按车轮汇总接触斑后，在共同时刻上统计均方根与最大差；GZ18 的共同统计步长为 `0.5 ms`，IRW 为 `10 ms`。组图保留各自原生采样时刻；蓝色实线为 ORVD BDF，橙色虚线为 SIMPACK。IRW 图另以橄榄色点线展示 Radau5 内部参考。

| 车辆 | 机械拓扑 | 对标工况 | 参考求解 |
|---|---|---|---|
| GZ18 | 17 体、四个刚性轮对 | 直线＋AAR6，60 km/h，20 s | SIMPACK 离线积分 |
| GZ18 | 17 体、四个刚性轮对 | R300＋AAR5，60 km/h，50 s | SIMPACK 离线积分 |
| IRW | 四轴桥、八个独立旋转车轮 | R300＋AAR5，零转矩被动，60 km/h，50 s | SIMPACK 离线积分 |
| IRW | 四轴桥、八个独立旋转车轮 | R300＋AAR5，100 Hz 全状态受控，60 km/h，50 s | SIMPACK Realtime FIXBDF，步长 62.5 µs |

### GZ18：刚性轮对车辆

GZ18 的 109 维连续状态覆盖整车、两个构架、四个刚性轮对、悬挂力元和八处轮轨接触。图中展示四个轮对通过直线、入缓和曲线、圆曲线、出缓和曲线及末段直线的横移与摇头响应。

![GZ18 R300 AAR5 50 s 四轮对横移与摇头对比](docs/figures/gz18/gz18_r300_aar5_50s_wheelset_response.png)

<details>
<summary><strong>展开查看 GZ18 直线工况、对标精度总览与全轮三向轮轨力</strong></summary>

**直线＋AAR6**：四个轮对在 AAR6 激励下的横移、摇头幅值和相位由两套求解器共同复现；不平顺退出后，衰减过程也保持同步。

![GZ18 直线 AAR6 20 s 四轮对横移与摇头对比](docs/figures/gz18/gz18_aar6_20s_wheelset_response.png)

**对标精度总览**：表内依次为均方根／最大差；运动学均方根给出四轮对范围。

| 工况 | 横移（µm） | 摇头（µrad） | `N`（kN） | `Tx`（kN） | `Ty`（kN） |
|---|---:|---:|---:|---:|---:|
| 直线＋AAR6 | `14.01–22.00 / 126.43` | `5.44–6.58 / 36.48` | `0.669 / 17.702` | `0.094 / 1.607` | `0.059 / 1.274` |
| R300＋AAR5 | `26.16–33.78 / 187.47` | `13.16–15.54 / 132.02` | `0.798 / 44.230` | `0.184 / 8.353` | `0.166 / 10.623` |

**全轮三向轮轨力**

直线＋AAR6：

![GZ18 直线 AAR6 20 s 全轮三向轮轨力对比](docs/figures/gz18/gz18_aar6_20s_wheel_force_response.png)

R300＋AAR5：

![GZ18 R300 AAR5 50 s 全轮三向轮轨力对比](docs/figures/gz18/gz18_r300_aar5_50s_wheel_force_response.png)

</details>

### IRW：独立旋转车轮车辆

IRW 包含 25 个刚体、169 维连续状态，其中有 14 项串联内力状态。零转矩被动工况为 R300、60 km/h、四通道 AAR5、50 s。BDF5、Radau5 与 SIMPACK 给出的四轮对横移、摇头在幅值和相位上吻合，覆盖曲线进入及退出后的恢复过程。

![IRW R300 AAR5 被动 50 s 四轮对横移与摇头对比](docs/figures/irw/irw_r300_aar5_50s_axlebridge_response.png)

<details>
<summary><strong>展开查看完整对比</strong></summary>

**零转矩被动：轮轨力**

![IRW R300 AAR5 被动 50 s 全轮三向轮轨力对比](docs/figures/irw/irw_r300_aar5_50s_wheel_force_response.png)

**100 Hz 全状态受控**：车辆采用全状态轮速导向与转矩整形。BDF5、Radau5 与 SIMPACK FIXBDF 分别独立闭环，曲线内及末段直线上的横移、摇头响应保持吻合。

![IRW 100 Hz 受控 50 s 四轮对横移与摇头对比](docs/figures/irw/irw_r300_aar5_full_state_controlled_50s_axlebridge_response.png)

![IRW 100 Hz 受控 50 s 全轮三向轮轨力对比](docs/figures/irw/irw_r300_aar5_full_state_controlled_50s_wheel_force_response.png)

**对标精度总览**：表内依次为各工况默认 BDF 与 SIMPACK 在共同 10 ms 时刻上的均方根／最大差。运动学均方根给出四轮对范围，轮轨力均方根合并八轮样本计算；最大差取对应全部通道与时刻。

| 工况 | 横移（mm） | 摇头（mrad） | `N`（kN） | `Tx`（kN） | `Ty`（kN） |
|---|---:|---:|---:|---:|---:|
| 零转矩被动 | `0.036–0.048 / 0.271` | `0.014–0.015 / 0.141` | `0.553 / 46.771` | `0.062 / 3.394` | `0.183 / 9.214` |
| 100 Hz 全状态受控 | `0.010–0.016 / 0.113` | `0.004–0.007 / 0.075` | `0.455 / 48.652` | `0.052 / 3.855` | `0.113 / 12.399` |

</details>

### 验证层级

| 层级 | 验证内容 |
|---|---|
| 解析与物理性质 | 刚体变换、运动学、质量矩阵、逆／前向动力学、轨道几何、接触、控制递推和 CVODE 解析系统 |
| 多体算子参照 | 通过跨进程 Drake 参考程序核对位置、速度、微分运动学、质量矩阵、逆／前向动力学和外力 |
| 整车时序参照 | 通过 SIMPACK 核对刚性轮对、独立车轮、被动运行和闭环控制的车辆响应 |

默认测试集合在 Ubuntu 24.04 x86-64（GCC 13；Clang 20＋LLVM libomp）、Windows 10／11 x86-64（MSYS2 UCRT64 GCC；CLANG64 Clang＋LLVM libomp）与 macOS Apple silicon arm64（AppleClang 21＋Homebrew libomp；Homebrew GCC 15）上运行，包含真实 OpenMP 团队探针、可重定位安装消费者、普通依赖闭包和共享 Drake 排除门。八轮接触的固定槽结果不随工作者数量变化；并行执行仍必须实际形成多线程团队。

## 计算性能

Linux GCC Release，60 km/h，8 个物理核、8 个 OpenMP 线程，输出周期 0.5 ms；表列两次完整运行均值。GZ18 使用 BDF2，IRW 使用 BDF5。

推进／控制计时包含数值推进，受控运行另计保持量同步与控制计算。完整进程另含模型载入、观测和结果写出。实时系数为仿真时长除以相应墙钟时间。

| 工况 | 仿真时长 | 线程 | 推进／控制 | 推进／控制实时系数 | 完整进程 | 完整进程实时系数 |
|---|---:|---:|---:|---:|---:|---:|
| GZ18 直线＋AAR6 | `20 s` | `8` | `3.926 s` | `5.095×` | `8.062 s` | `2.481×` |
| GZ18 R300＋AAR5 | `16 s` | `8` | `3.336 s` | `4.797×` | `6.812 s` | `2.349×` |
| IRW R300＋AAR5，被动 | `50 s` | `8` | `46.101 s` | `1.085×` | `58.458 s` | `0.855×` |
| IRW R300＋AAR5，100 Hz 受控 | `50 s` | `8` | `39.158 s` | `1.277×` | `51.008 s` | `0.980×` |

硬件、容差与计时范围见 [Linux 计算评价](docs/performance/platform_evaluations/LINUX_X86_64.md)，其他平台与计时定义见[性能说明](docs/performance/README.md)。

## 快速开始

ORVD 依赖 Eigen 3.4.0、fmt 9.1.0、nlohmann/json 3.12.0、OpenMP C++ 运行时和 SUNDIALS 7.7.0。已有依赖前缀时可以直接构建和安装：

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=<dependency-prefix> \
  -DCMAKE_INSTALL_PREFIX=<orvd-prefix> \
  -DBUILD_TESTING=OFF
cmake --build build
cmake --install build
```

外部工程通过标准 CMake 包消费：

```cmake
find_package(OpenRailVehicleDynamics CONFIG REQUIRED)
target_link_libraries(my_vehicle_simulation PRIVATE ORVD::integrators)
```

<details>
<summary><strong>安装包导出的目标</strong></summary>

按依赖层级组织的 14 个产品模块目标：`control`、`actuation`、`track_geometry`、`track_irregularity`、`wheel_rail_contact`、`configuration`、`multibody_runtime`、`multibody_model`、`forces`、`system_assembly`、`integrators`，以及面向三维观察的 `scene_observation`（刚体世界状态采样、车轮放置与线路采样）、`scene_record`（场景记录与资源包读写）和 `scene_stream`（可移植 OSTS 编解码）；使用时添加 `ORVD::` 命名空间。它们依赖的 vendored 刚体多体树一并安装为位置无关静态归档。

</details>

Linux 可显式构建并安装 `ORVD::simpack_realtime` 可选组件，以机械观测和四轴差动转矩回调驱动本机 SIMPACK Realtime 模型；默认产品及普通安装消费者不依赖 SIMPACK 或其专有 ABI，完整命令见 [Linux 构建说明](docs/build_and_install/LINUX.md#可选-simpack-realtime-组件)。Linux 另有默认关闭的 `ORVD::state_stream` 组件，把 `scene_stream` 编码的观测以非阻塞 UDP 发送给独立显示器，通过 `-DORVD_BUILD_STATE_STREAM=ON` 构建安装；可移植协议与 Python 接收示例见 [scene_stream](libs/scene_stream/README.md)。

Linux、Windows 与 macOS 的依赖安装、离线构建、测试及消费命令见[分平台构建与安装说明](docs/build_and_install/README.md)；离线源码包的内部结构见[依赖超级构建说明](distribution/dependencies/README.md)。

## 仓库导航

| 类别 | 入口 | 内容 |
|---|---|---|
| 产品库 | [`libs/`](libs/) | 可安装的 C++ 产品模块与公共头 |
| 资产 | [`vehicle_library/`](vehicle_library/)、[`track_library/`](track_library/)、[`controller_library/`](controller_library/) | GZ18、IRW 机械模型、移动启动、轮型、作动与参考模型；线路几何、钢轨型面与轨道不平顺；IRW 全状态导向、巡航控制与转矩整形参数 |
| 应用 | [`apps/`](apps/) | 场景查看器 |
| 工具 | [`tools/`](tools/) | 动力学资格、型面转换、打包、源码边界与 UDP 状态流 |
| 实验 | [`experiments/`](experiments/) | 可复用的实验编排、结果提取与绘图示例 |
| 测试 | [`tests/`](tests/) | 解析、性质、跨实现、安装和平台测试 |
| 第三方与分发 | [`external/`](external/)、[`distribution/`](distribution/) | vendored Drake 多体树子集与 Radau5；离线依赖源码包 |
| 文档 | [`docs/`](docs/) | 构建安装、模型算法、架构决策、工程规则和性能说明 |

## 文档入口

- [分平台构建与安装](docs/build_and_install/README.md)
- [模型与算法](docs/models_and_algorithms/README.md)
- [架构决策](docs/adr/README.md)
- [完整文档索引](docs/README.md)

## 许可

第一方代码以 [Business Source License 1.1](LICENSE) 源码公开。标准条款允许非生产使用；附加使用授权进一步允许非商业的生产使用，包括学术研究、教学与个人评估。商业生产使用需向许可方（wenquantongxin@126.com）另行取得授权。2029-10-19 起，授权作品转为 Apache License 2.0。

vendored Drake 源码遵循其 BSD-3-Clause 与逐文件注明的 Apache-2.0 条款；第三方来源、修改声明和再分发信息见 [`NOTICE`](NOTICE) 与 [vendored 源码说明](external/drake_mbtree/README.md)。
