# 时间积分器迁移路书

本文是 ORVD 增加研究型时间积分后端的实施依据。算法公式、稳定性和车辆适用边界见
[BDF、Radau5、Newmark 与 Zhai 时间积分方法](../../models_and_algorithms/numerical_methods/TIME_INTEGRATION_METHODS.md)；
本文只定义实施顺序、允许修改面和完成门。

本轮唯一展开实施的新增后端是 **三阶段、五阶 Radau IIA（Radau5）**。CVODE BDF 继续作为默认
后端；Newmark 和 Zhai 只登记为待做，不在 Radau5 搬运前预建接口、枚举值、空类或配置键。

## 1. 当前状态

| 对象 | 当前状态 | 本路书处置 |
|---|---|---|
| CVODE BDF | 已实现；公共默认实例最大二阶，部分源码树资格配方最大五阶 | 作为基线保留，不重写 |
| Radau5 | 纯 C++ 核心、私有适配器与系统短窗资格已完成；等误差性能和长窗资格未完成 | 按 INT-01–INT-08 推进，资格前不公开 |
| Newmark | 未实现 | 待做；本轮不启动接口审计或实现 |
| Zhai 简单显式二步法 | 未实现 | 待做；本轮不启动接口审计或实现 |
| 历史 Radau3 | 只存在于旧迁移记录和仓外参考语境 | 不属于本路书，不用 Radau5 静默替代其历史身份 |

- 当前实施状态：`INT-01`–`INT-06` 已完成正确性复核，`INT-07A` 的协议与串行工件入口保留；
  `INT-07B` 暂停。公共默认仍为 CVODE BDF2，Radau5 选择仍只存在于源码树资格边界。
- 实／复近奇异线性系统的相对残差资格、跨构建严格浮点标志门和双工具链完整复核已经闭合；
  `INT-07B` 仍保持暂停。正确性前置闭合不自动授权 configuration-aware comparator、双 reference
  gate 或任何性能排名；后续恢复必须另行裁决。
- 本路书完成边界：Radau5 作为显式选择的研究后端通过完整合同和车辆资格；不改变默认积分器。

## 2. 已冻结的架构前提

### 2.1 后端与系统层职责

[ADR-0003](../../adr/0003-abstract-advancer-cvode-first.md)已经把两类职责分开：

- `ContinuousStateAdvancer` 负责一个数值后端的端点、单成功内步、稠密输出、统计和重初始化；
- `SystemContinuousStateAdvancer` 负责 accepted/candidate Context、轮轨站位历史、公开推进事务和
  外部事件后的同步；
- RHS 试算不得写入 accepted 状态；
- 每个真实成功内部端点必须先返回系统层，系统层更新站位提示后才允许下一内部步；
- 后端失败保持最近公开端点，稠密输出失效，并要求显式重初始化后才能继续。

Radau5 必须适配这些既有语义，不能用搬运上游循环为由改变系统事件或提交顺序。

### 2.2 已关闭的唯一组合接缝

数值后端抽象已经存在；迁移前系统实现曾：

- 在构造函数中只按 BDF 最大阶数创建 `CvodeContinuousStateAdvancer`；
- 以 `std::unique_ptr<CvodeContinuousStateAdvancer>` 持有后端；
- 通过 CVODE 专用内部访问器查询 BDF 身份；
- 以 CVODE 专用注册路径接入并行稠密有限差分 Jacobian。

INT-06 已把这些真实接缝统一收口到源码树私有 `SystemContinuousStateBackend`：外层 owning backend
bundle 统一拥有 RHS bridge 与所需辅助资源，闭集 runtime alternative 拥有具体 advancer，并由实际
alternative 报告 recipe 身份。
`SystemContinuousStateAdvancer` 只保留共同事务循环。源码树只提供一个
`SystemContinuousStateIntegrationAccess`；资格场景的物理 recipe 与可选的闭集
`TimeIntegratorQualificationCase` 分离，不扩张公开配置语言。

### 2.3 Radau5 首版能力边界

首版只准入：

- 正向时间、显式形态的一阶 ODE \(\dot y=f(t,y)\)（非质量矩阵／DAE 形式）；
- `double` 状态；
- 三阶段、五阶 Radau IIA；
- 自适应步长；
- 标量相对容差和逐分量绝对容差；
- 完整稠密数值 Jacobian；
- simplified Newton 与实／复稠密线性系统；
- 最近成功步的配点稠密输出；
- 现有 recoverable/fatal RHS、统计、stop 和重初始化合同。

首版明确不做：

- 质量矩阵、DAE、带状或稀疏 Jacobian；
- 解析 Jacobian、Krylov、预条件器、GPU 或反向积分；
- Hairer 的二阶结构优化、Hessenberg 和用户输出回调；
- 新事件系统、接触 sample-and-hold 或轮轨模型改变；
- 把全部 Newton 和控制器常数暴露成公共旋钮；
- 默认后端切换；
- Newmark、Zhai 或历史 Radau3 的顺带实现。

## 3. 目标结构

INT-06 后的实际所有权关系为：

```text
SystemContinuousStateAdvancer
├── accepted/candidate Context 与站位事务
└── unique_ptr<SystemContinuousStateBackend>  ← owning backend bundle
    └── unique_ptr<Implementation>
        ├── SystemRhsBridge
        ├── RecoverableFailureObservingRhs
        ├── unique_ptr<SystemDenseFiniteDifferenceJacobian>  ← nullable，仅 CVODE
        └── optional<ConcreteRuntime>
            └── variant<CvodeBdf2Runtime, CvodeBdf5Runtime, Radau5Runtime>
                ├── CvodeBdf2Runtime { CvodeContinuousStateAdvancer }
                ├── CvodeBdf5Runtime { CvodeContinuousStateAdvancer }
                └── Radau5Runtime    { Radau5ContinuousStateAdvancer }
```

`SystemContinuousStateIntegrationRecipe` 只进入唯一私有工厂；构造后的 recipe 身份由
`ConcreteRuntime` 的真实 alternative 反向导出，不另存一个可与对象漂移的标签。销毁顺序
保证 concrete advancer 早于它借用的 Jacobian provider、observer 和 RHS bridge 销毁。

派生自上游的 Radau5 数值核心与 ORVD 适配层分开：

| 区域 | 职责 |
|---|---|
| `external/radau5/` | 上游来源、许可、逐文件处置、修改记录和收窄后的数值核心 |
| `libs/integrators/` | `ContinuousStateAdvancer` 适配、系统私有 recipe/factory 和统计映射 |
| `tests/integrators/` | 算法核、后端中立合同和系统事务测试 |
| `tools/dynamics_qualification/` | 显式选择研究后端的短窗／长窗资格入口与报告 |

资格完成前不把 Radau5 选择加入安装头。届时若确需安装给外部研究消费者，应另行裁决一个只列真实
实现的强类型选择接口；现有无选择参数的构造函数始终委托到 CVODE BDF2。

## 4. 来源与许可证策略

### 4.1 权威来源

来源优先级冻结为：

1. [UNIGE 官方软件目录](https://www.unige.ch/~hairer/software.html)及其托管的
   [`IntegratorT.tgz` C++ 版本](https://www.unige.ch/~hairer/prog/IntegratorT.tgz)，作为实际搬运来源；
2. Hairer/Wanner 官方 Fortran RADAU5、DECSOL 支撑例程及 `M=I` driver 的 URL，具体清单由
   [`SOURCE_DISPOSITION.txt`](../../../external/radau5/SOURCE_DISPOSITION.txt)唯一维护，作为算法、
   常数、控制器和参考结果的仓外交叉依据；
3. [UNIGE 许可原文](https://www.unige.ch/~hairer/prog/licence.txt)，作为再分发条件的唯一文本依据；
4. [SciPy Radau 文档](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.Radau.html)，
   只作现代单步 API 和测试的旁证，不混入第二套控制器后再宣称与 Hairer 实现同一。

INT-01 在复制源码前固定实际 C++ donor 归档，记录来源 URL、取得日期、摘要和上游作者；许可原文
同样记录 URL、落位文本和上游摘要。项目负责人裁决 Fortran oracle 只登记官方 URL，不提交源码，
也不在仓库记录其 SHA-256。C++ donor 与许可摘要只用于来源完整性，不作为算法正确性或永久性能门。

### 4.2 许可与源码处置

UNIGE 页面把 RADAU5 纳入其两条款式许可范围；搬运时仍只按许可原文登记，不自行发明 SPDX 结论。
若落入任何派生源码，必须同时落下：

- `external/radau5/LICENSE.UNIGE.txt`；
- `external/radau5/SOURCE_DISPOSITION.txt`；
- `external/radau5/RADAU5_SOURCE_MODIFICATIONS.md`；
- `external/radau5/README.md`；
- 根 `NOTICE` 中的再分发来源、作者和许可位置；
- 安装包中的许可、来源与修改记录。

上游源码不得混成无归属的 ORVD 第一方代码。Radau5 作为随 ORVD 源树编译的再分发源码登记为产品
模块；不通过 `find_package()` 查找，不加入 `dependency_sources.cmake`，也不成为一份独立的离线依赖
归档。产品构建仍只启用 C++；官方 Fortran 可在仓外临时工作目录中作为数值 oracle 编译运行，但不
成为 `orvd_integrators` 的运行时、安装或离线构建依赖。

## 5. 实施阶段

| Goal | 产物 | 直接前置 | 状态 |
|---|---|---|---|
| INT-01 | 来源、许可和完整 Radau5 求解器合同 | 本路书 | 已完成；正确性勘误（2026-08-24） |
| INT-02 | 后端中立的系统组合接缝与共同合同测试 | INT-01 | 已完成（2026-08-23） |
| INT-03 | 收窄且可独立验证的 Radau5 数值核心 | INT-01 | 已完成；正确性修复（2026-08-24） |
| INT-04 | 自适应、误差控制、稠密输出和失败恢复 | INT-03 | 已完成；正确性修复（2026-08-24） |
| INT-05 | `Radau5ContinuousStateAdvancer` | INT-02、INT-04 | 已完成；异常因果复核（2026-08-24） |
| INT-06 | 系统事务层的私有 Radau5 研究入口 | INT-05 | 已完成；投影历史闭合（2026-08-24） |
| INT-07 | Jacobian 复用与等误差性能资格 | INT-06 | **暂停**（保留 INT-07A 协议与串行工件；INT-07B 未启动） |
| INT-08 | 双车型、跨平台、安装与文档收口 | INT-07 | 待做 |

INT-02 与 INT-03 在 INT-01 完成后可以分开开发，但 INT-05 前必须共同收口。

2026-08-24 的独立审查确认三阶段五阶主链，但发现 stop 吸附、持续 recoverable 因果、线性建立分类
和 accepted 投影历史下 Jacobian/LU 复用缺陷。定向修复与官方 oracle 已闭合这些正确性项；现有
INT-07A manifest、串行工件协议和测试保留。实／复近奇异资格现在直接复用生产矩阵形成、分解与求解，
并以逐级缩小谱隙的实／复块检查相对后向残差；严格浮点门在配置期、编译期和运行工件三处共同
闭合。两项关闭不自动授权 comparator、并行 Jacobian 候选或正式性能排名，`INT-07B` 继续暂停。

## 6. 原子 Goal

### INT-01 — 冻结来源、许可与完整求解器合同

前置 Goal：本路书。

唯一产物：可追溯的上游来源处置，以及一份没有数值空白的首版 Radau5 合同。

完成产物：

- [ADR-0009：保持 CVODE 默认，增加纯 C++ Radau5 研究后端](../../adr/0009-cvode-default-radau5-research-backend.md)；
- [Radau5 首版完整求解器合同](INT_01_RADAU5_SOLVER_CONTRACT.md)；
- [`external/radau5/` 来源、许可与修改账本](../../../external/radau5/README.md)。

允许修改：`docs/adr/`、`docs/planning/integrator_migration/`、`external/radau5/` 的来源与许可文件。

明确不做：不修改积分器产品源码，不增加构建目标，不生成参考金标。

完成门：

1. 固定实际 C++ donor 归档和许可文本，记录 URL、摘要、作者、许可与逐文件处置；Fortran oracle
   按项目负责人裁决只记录官方 URL，不提交源码或摘要。
2. 冻结三阶段五阶身份、`M=I`、前向时间、误差估计器、步长控制器、Newton 收敛、Jacobian／分解
   刷新和配点稠密输出；不得只冻结 Butcher 表。
3. 单独冻结容差适配：是否接受 `rtol=0` 的纯绝对容差、上游 \(0.1\,rtol^{2/3}\) 变换是否保留、
   极小值／下溢／非有限权重的拒绝规则，以及逐分量 `atol` 的尺度。任何变换只作用于核心私有副本，
   不得改写调用方 `ContinuousStateErrorTolerances`。
4. 逐字段定义 ORVD 统计映射。`error_test_failure_count` 计入重初始化后所有误差检验失败，包括首次
   成功步之前的拒步；若上游 `nrejct` 排除其中一部分，就不得仅按计数器名称直接照搬。
5. 明确区分 Radau5 与历史 Radau3，并以新 ADR 记录“CVODE 默认不变、Radau5 为研究第二后端”。
6. 已在仓外构建官方参考程序并运行一个 `M=I` 非刚性线性衰减和一个 `M=I` 刚性 Van der Pol；
   临时源码、driver、输出、数值快照和跨平台哈希均未提交。

### INT-02 — 建立后端中立的组合接缝

前置 Goal：INT-01。

唯一产物：`SystemContinuousStateAdvancer` 能持有任一真实 `ContinuousStateAdvancer`，同时当前
所有公共构造仍只创建 CVODE。

允许修改：`libs/integrators/`、`tests/integrators/` 及其构建文件。

明确不做：不添加 Radau5 数值代码，不增加公共后端枚举或字符串选择，不泛化事件系统。

完成门：

1. 以私有 `BackendBundle` 同时保存强类型 recipe、`std::unique_ptr<ContinuousStateAdvancer>` 和只在
   CVODE recipe 下有效的窄诊断句柄；不以 RTTI、诊断字符串或公开任意整数判断后端。
2. 保留现有公共 BDF2 默认、源码树 BDF5 资格身份和 BDF 专属阶数查询；Radau5 recipe 上的 BDF
   查询必须明确拒绝，且不得给后端中立基类增加 BDF 虚函数或统计字段。
3. 从现有 CVODE 测试抽取 same-time、输入原子性、单成功步、稠密区间、失败封锁和重初始化共同
   合同；CVODE 专有 BDF 阶数测试继续独立存在。
4. 现有 RHS bridge、CVODE 后端和系统推进测试全部通过，接受态、站位和稠密样本行为不变。

完成记录（2026-08-23）：

- 私有闭集 recipe 当前只包含真实的 CVODE BDF2／BDF5；`BackendBundle` 同时持有 recipe、
  `std::unique_ptr<ContinuousStateAdvancer>` 和 CVODE 窄句柄，推进、统计、稠密输出与重初始化均经
  后端中立基类派发；
- 公共构造仍映射 CVODE BDF2，源码树 BDF5 资格工厂仍映射 CVODE BDF5。BDF 查询先验证 CVODE
  recipe 与窄句柄，未来真实 Radau5 bundle 会明确抛出 `logic_error`；本阶段不为测试预建 Radau5
  枚举值或 fake backend；
- 共同合同已从 CVODE 专属测试中抽出，覆盖 same-time、非法输入原子性、单成功内步、稠密区间、
  失败封锁和重初始化；BDF 配置／实际阶数／重初始化身份保留为独立测试；
- RHS bridge、共同合同、CVODE BDF 身份、系统事务和迁移安装消费者均通过，资格工具仍可链接私有
  BDF5 入口。

### INT-03 — 搬运收窄的 Radau5 数值核心

前置 Goal：INT-01。

唯一产物：不依赖 ORVD 系统 Context、一次调用最多提交一个成功内部步的 Radau5 核心；调用内部可
包含有界数量的拒绝试步。

允许修改：`external/radau5/`、根 `CMakeLists.txt`、`tests/CMakeLists.txt` 和仅服务该核心的
`tests/integrators/` 测试。

明确不做：不接 `ContinuousStateAdvancer`，不接整车，不实现首版排除能力。

完成门：

1. 只搬显式形态的一阶 ODE、full-dense、三阶段 Radau IIA 和实际需要的实／复线性代数；不搬
   DOPRI5、示例主程序、全局输出框架、DAE、带状或二阶特殊分支。
2. `external/radau5/` 提供不安装的 `STATIC` 或 `OBJECT` 核心 target，并在根产品模块清单登记；测试
   和 `orvd_integrators` 复用同一编译目标，不各自编译一份漂移源码。
3. 全局回调、裸所有权、控制台退出和整段 `Integrate()` 改为借用回调、RAII 工作区和结构化状态；
   每次调用至多提交一个成功内部步。
4. 保留上游算法常数、误差公式和主要运算顺序；若必须改变，逐项记录原因并增加针对性测试。
5. 不朴素分解 `3N×3N` 方阵；以一个实 `N×N` 和一个复 `N×N` 系统或数学等价的实块实现。
6. 常导数、非自治多项式、线性衰减、线性振子和固定步加密显示五阶终点收敛；固定步测试使用测试
   私有的步长夹持或确保单次接受的宽容差，禁止把自适应细分混入阶数测量。两个核心实例还需交错
   推进，证明不存在全局或线程局部数值状态串扰。

完成记录（2026-08-23）：

- `external/radau5` 新增不安装的 `orvd_radau5_core_objects`；它和 `orvd_integrators`、独立核心测试
  共享同一编译对象。产品仍只有 C++，Fortran oracle 仍只记录官方 URL；
- 核心只借用 `noexcept` 状态码 RHS，不依赖 `ContinuousStateAdvancer`、System Context 或全局／
  `thread_local` 数值状态；三阶段耦合由一个实 `N×N` 与一个复 `N×N` Eigen LU 求解；
- 官方变换常数、simplified Newton、误差公式、Gustafsson 控制器和三次配点稠密输出保留，来源、
  许可和工程／算法修改均在 `external/radau5` 账本闭合；
- 独立验证覆盖常导数、非自治多项式、线性衰减五阶加密、振子、刚性
  Prothero–Robinson、超大初始试步拒绝、Jacobian／阶段可恢复 RHS、fresh/stale Jacobian 拒试、
  大绝对时间下的稠密步长身份和 trial-budget 边界。实例隔离使用不同维数、方程、容差与非对称时钟
  交错，并与各自独立运行逐字段比较统计；每个固定步调用均断言恰好一次接受，避免自适应细分污染
  阶数；
- 本 Goal 原定不接后端。负责人随后在同一任务中明确要求“先接入已经建立的后端中立组合接缝”，
  因而私有适配器与真实 `kRadau5` 系统 recipe 同步提前落位。它们不进入安装头、不增加字符串／环境
  选择、不改变公共 CVODE BDF2 默认；这项提前实现不等同于 INT-04–INT-06 的全部车辆资格完成；
- 完成两遍独立复核。第一遍发现并修正负维工作区分配、容差中间值语义、稠密输出原始步长、统计
  溢出／Newton 计数时相和 adapter `noexcept` 维数回调等问题，并加固 fresh/stale Jacobian、扰动
  重试、trial budget 与异构双实例测试；当时还更正了变换方向注释。进入 INT-07 前的
  综合复核后续发现了一项步长控制缺陷；勘误和回补记录见 INT-04。

### INT-04 — 补齐自适应、稠密输出与恢复状态机

前置 Goal：INT-03。

唯一产物：具备完整首版求解器行为的有状态 Radau5 核心。

允许修改：`external/radau5/`、根 `CMakeLists.txt`、`tests/CMakeLists.txt` 和对应
`tests/integrators/` 测试。

明确不做：不接系统 Context，不进行并行 Jacobian 或性能优化。

完成门：

1. 实现初始步长、阶段外推、误差估计、接受／拒绝、下一步长、Newton 失败缩步和 Jacobian／分解
   复用；所有试步状态只有在接受后才成为历史。
2. 最近成功步保存完整配点插值系数；插值只覆盖该步闭区间，并以边界时间特判复制步首或接受端点，
   不依赖多项式浮点回代碰巧逐位相等。
3. 阶段 RHS 的可恢复失败采用有限缩步重试；有限差分 Jacobian 扰动失败则有限减小扰动，重试耗尽
   后明确终止，不能只缩小与扰动无关的时间步。致命失败原样传出。
4. 原始 `std::exception_ptr` 可越过核心状态机并由 ORVD 适配器重抛；任何 C++ 异常不得越过
   `noexcept`／状态码回调边界，也不得留下半提交历史。
5. 通过刚性标量、Prothero–Robinson、Robertson 和刚性 Van der Pol；官方 DAE amplifier 因
   `M=I` 边界明确不运行。
6. 统计能区分普通 RHS、Jacobian 差分 RHS、全部误差检验失败、Newton 迭代／失败、Jacobian 和
   线性分解，并符合 INT-01 的逐字段映射。

完成记录（2026-08-23）：

- 完整自适应状态机通过刚性标量、Prothero–Robinson、Robertson 与官方 `M=I` 刚性 Van der Pol
  问题；自适应容差加密、固定步终点五阶和非精确稠密内点四阶趋势均独立验证；
- 恢复资格覆盖误差拒步、非奇异 Newton 失败、累计第五次奇异分解、阶段／Jacobian／误差修正 RHS
  的 recoverable/fatal 分支、Jacobian 缩扰动耗尽、极小 stop、失败封锁与原子重初始化；
- 普通 RHS、Jacobian RHS、误差失败、Newton 迭代／失败、分解与 Jacobian 统计均由精确路径断言，
  Debug 与 Release 验证通过。进入 INT-07 前的综合复核发现，有成功步历史时的误差拒试
  路径误用了只属于 accepted trial 的 Gustafsson predictive factor，使拒试后步长被额外缩小。
  本轮已把普通控制器与 accepted-only predictive controller 分开，并增加“先建立成功历史、
  后发生一次误差拒试”的定向回归；该勘误取代原“复核未发现缺陷”结论。

正确性修复记录（2026-08-24）：

- stop 吸附改为每次正长度调用首个 trial 的一次性资格；构造、重初始化、后续调用和大绝对时间均
  精确到达 stop，被拒的吸附 trial 不会在同一次调用中覆盖控制器缩步；
- trial retry cause 与 linear setup status 均改为窄类型，持续 recoverable 保留原异常因果，真奇异、
  非有限 shift、非有限矩阵和非有限分解不再混为 `repeatedly singular`；
- Newton／误差 WRMS 使用缩放平方和，有限大状态的 Jacobian 扰动使用有符号可表示 `nextafter`；
  `atol/rtol` 中间上溢继续作为合同能力边界，但构造失败改为 `invalid_argument`；
- 仓外官方 oracle 的 `N=1/2/4` 单步、刚性 Van der Pol 和 Robertson 在端点、稠密值及归一化统计上
  与 C++ 主链一致；Fortran 源码、驱动、输出和摘要均未进入仓库。

### INT-05 — 适配 `ContinuousStateAdvancer`

前置 Goal：INT-02、INT-04。

唯一产物：`Radau5ContinuousStateAdvancer` 完整履行既有后端合同。

允许修改：`libs/integrators/`、`tests/integrators/` 和必要构建文件。

明确不做：不接 `SystemContinuousStateAdvancer`，不公开安装选择接口；资格期适配器声明留在
`libs/integrators/src/` 私有边界，不进入会随现有规则安装的 `libs/integrators/include/`。

完成门：

1. 同刻请求为零 RHS、零统计改变的 no-op；非法 stop、错维输出和非有限输入在进入核心前失败，
   且不改变调用方存储、端点、旧稠密区间或继续推进能力。
2. 每次只返回一个成功内部端点；最后一步夹持到 stop，并以 stop 的精确浮点身份发布终点。
3. 后端内失败保留最近公开端点、清除稠密输出并封锁继续推进；显式重初始化成功后清空历史和统计。
4. 可恢复 RHS 失败缩步后若成功，不把被拒绝异常泄漏为公开失败；致命异常保持原类型。
5. 运行 INT-02 抽取的全部后端中立合同，并增加 Radau5 五阶、刚性和稠密输出专属测试。

完成记录（2026-08-23）：INT-04 关闭后重新运行 Radau5 的全部后端中立合同；单成功内步、stop
夹持、稠密输出、原异常重抛、失败封锁、显式重初始化和逐字段统计映射在 Debug 与 Release 均通过。

2026-08-24 的补充回归以自定义 recoverable 异常验证持续阶段失败的原类型重抛，同时逐项守住调用方
输出 sentinel、端点、稠密区间、统计、失败封锁及成功重初始化；构造期容差能力失败不再冒充正长度
推进失败。

### INT-06 — 接入系统事务层

前置 Goal：INT-05。

唯一产物：源码树资格工具可显式构造 Radau5 系统推进器，公共默认仍构造 CVODE BDF2。

允许修改：`libs/integrators/`、`tests/integrators/`、`tests/configuration/`、`tests/CMakeLists.txt`、
聚焦资格工具及对应局部 README。

明确不做：不增加安装 API，不改变资格场景物理模型、事件顺序或默认配方。

完成门：

1. 私有具名 Radau5 recipe 只在有真实消费者时加入工厂；不建立任意字符串、环境变量或未实现枚举值。
2. 每个 Radau5 接受端点严格经过“安装 candidate 时间与状态 → 更新轮轨站位 → 下一内部步”。
3. 公开推进成功才一次提交时间、`[q;v;z]` 和站位；后端、插值或端点投影失败保持 accepted Context
   不变并要求显式同步。
4. 通过阻尼／Maxwell 解析系统、外部参数同步、真实 GZ18 投影窗可恢复失败、GZ18 稠密短窗和
   IRW 100 Hz 重初始化语义。
5. 初次整车接入允许串行数值 Jacobian；如果成本很高，记录统计而不是在同一 Goal 扩张并行设计。

完成记录（2026-08-23）：

- 一个闭集强类型 recipe、一个 `SystemContinuousStateIntegrationAccess` 和一个 owning backend bundle
  取代 BDF／Radau5 平行 access；具体 runtime alternative 决定实际身份。公共构造、安装 API 和 CVODE
  BDF2 默认不变，Newmark／Zhai 没有占位值；
- 系统事务测试对 CVODE BDF2 与 Radau5 共用阻尼、外部参数、稠密输出、Maxwell／标称力和 fatal RHS
  原子性断言；源码树私有诊断还证明真实 GZ18 Radau5 运行至少分类过一次可恢复的
  `TrackStationProjectionWindowMiss`，并完成 30 ms／1 cm 站位推进。Radau5 另通过 10 ms／101 点
  稠密短窗，以及 IRW 20 ms 的 U0→U1→同步→U2 事件序列；
- 资格 runner 将物理场景默认 recipe 与源码树私有闭集 qualification case 分离。摘要和 metadata 以
  `cvode_bdf2`／`cvode_bdf5`／`radau5` 为主身份，CVODE 的 `maximum_bdf_order` 兼容字段在 Radau5
  上明确为 `null`；Radau5 串行数值 Jacobian worker identity 为 `1`；
- Debug 与 Release 各 86 项完整 CTest、迁移安装消费者、既有 CVODE 多线程／发布回归均通过；不比较
  两种方法的逐位状态或统计。

正确性修复记录（2026-08-24）：

- accepted 轮轨投影提示被确认为 `(t,y)` 外的数值 RHS 历史。系统层在每个成功内步安装 candidate
  状态并更新非空提示后，经后端中立通知让 Radau5 仅失效旧 Jacobian/LU；不重置 accepted 状态、
  控制器、阶段外推或稠密区间；
- 真实 GZ18 的 `30 ms / 1 cm` 局部投影窗运行实际分类可恢复窗口失败、推进提示并到达目标，同时
  断言下一内步不复用更新前线性化；IRW 被动与 100 Hz 受控短窗也通过；
- 该轮 Release 完整 CTest 为 86 项全绿。该正确性闭合不构成性能资格，轮轨场景下每个 accepted
  内步保守重建 Radau5 线性化的成本留到 INT-07 以后裁决。

### INT-07 — 复用 Jacobian 能力并做等误差性能资格

前置 Goal：INT-06。

唯一产物：Radau5 在不改变数值合同的前提下具备可复核 Jacobian 路径和性能证据。

资格定义由[INT-07 等误差资格协议与串行基线](INT_07_EQUAL_ERROR_QUALIFICATION_PROTOCOL.md)冻结；任何
比较实现不得静默改变其中的状态、reference、接触事件或计时语义。

允许修改：`libs/integrators/`、`tests/integrators/`、`tests/dynamics_qualification/`、
`tools/dynamics_qualification/`、性能记录，以及
`external/radau5/include/`、`external/radau5/src/` 中为后端中立 Jacobian 计算层所必需的窄回调／
核心 API 改动。任何核心改动必须同时更新 `tests/integrators/` 的 Radau5 核心测试和
`external/radau5/RADAU5_SOURCE_MODIFICATIONS.md`；必要的 CMake 目标文件也属于允许修改面。

明确不做：不以单机一次最快结果切换默认，不改变轮轨物理，不增加解析 Jacobian。

完成门：

1. 以可完整回退的候选改造验证：将现有稠密有限差分 provider 分成后端中立的
   扰动 RHS 批计算层与 CVODE/Radau5 窄适配；每个 worker 继续独占 Context、RHS bridge 和
   工作区。各数值后端仍自己拥有扰动公式、实际可表示增量、按列缩扰动重试、
   fresh/stale Jacobian 状态和计数语义；批计算层只求值已指定的扰动状态，不发明
   CVODE 或 Radau5 的数值策略。
2. 串行与并行 Jacobian 在同一试算状态下按量纲化容差一致；异常分类、accepted 隔离和统计一致。
3. 任何墙钟比较前，先为 GZ18 和 IRW 分别固定一份可重算的比较 manifest：
   - 共同已解析初态、车辆／线路资产身份、基于整数纳秒的输出时钟和外部事件时钟；
   - 私有闭集 `TimeIntegratorQualificationCase`：backend 为 `scenario_default_cvode` 或
     `radau5`，tier 为 `coarse`、`nominal`、`fine` 或 `reference`，相对场景配方的固定
     缩放分别为 `10`、`1`、`0.1`、`0.01`。前三档构成性能加密梯子；metadata 必须写出
     完整 case、tier、scale、实际后端身份及解析后的 `rtol` 和 q/v/z 绝对容差；
   - 两个 backend 各自的 `reference` 是比 `fine` 再严一档的独立参考候选。参考不取默认
     CVODE 输出作为真值；先证明两个候选在预登记参考预算内一致，才允许用于评估；
   - 平滑短窗的量纲化 q/v/z 位形误差、RMS 和最大范数，以及非光滑车辆窗的接触事件
     时刻、波形包络、峰值和物理残差；非对齐的接触切换点不做逐位绝对差裁决。
   只在该冻结规范下按物理误差匹配档位，不把相同 tolerance 数值当作公平条件。
4. 记录现有后端中立可比类别：`successful_internal_step_count`、
   `error_test_failure_count`、`nonlinear_solver_convergence_failure_count`、两类 RHS、Jacobian、
   Newton iteration、linear setup、墙钟和峰值内存。前两类 failure 不得相加并标成通用
   “rejected steps”；现有合同没有计数 recoverable RHS 等所有 trial abandonment。若 INT-07
   确实需要该指标，必须先对两后端同时冻结“一个已启动 trial 未接受便放弃，每 trial
   只计一次”的共同语义和分原因映射，否则不采集、不比较。GZ18 与 IRW 分开报告。
5. INT-07 允许且只允许两种完成结果：若候选路径正确且有稳定收益，保留后端中立 provider；若无
   稳定收益，保存负面证据并完整回滚候选 provider 改造，以串行 Jacobian 的研究后端完成本 Goal。

INT-07A 完成记录（2026-08-23）：

- 机器 manifest 冻结 GZ18 直线 AAR6 `20 s / 0.5 ms` 与 IRW R300 AAR5 被动 `30 s / 0.5 ms`、
  两车共同 `[0,10 ms]` 正且跨轨迹一致的恒定逐接口 patch counts 平滑窗、八个闭集 case、场景默认
  容差、双 reference、候选 q/v/z 状态门、物理预算及全串行 correctness oracle；严格验证器展开
  16 组现有 runner argv，并拒绝未知字段、非整数时钟、
  非串行处方和性能排名；manifest-bound 执行还对实际 OMP/affinity 与工件 worker identity fail closed；
- manifest-bound postflight 进一步核对 exact COMPLETE、完整 q/v/z 与 observation 时钟、逐样本逐接口
  patch ordinal、九项统计和 primary advance timing；runner 成功但工件残缺时独立记录 wrapper failure；
- 被动资格工件新增直接来自既有 dense state matrix 的 `continuous_states.tsv`，以
  `(sample_index,time_nanoseconds)` 为 join key 无损保存 `[q;v;z]`，不增加积分 stop 或 RHS；协议将
  自由体 q 冻结为 rotation-log、quaternion norm difference 与平移误差，Ball-RPY 也按 rotation-log
  比较，并为 reference／候选分别冻结 quaternion norm-defect 门，避免 raw 坐标假差与共同范数漂移；
- GZ18 与 IRW 被动真实短窗以全串行运行作 correctness oracle，并要求 4/8/16-worker 的完整状态、
  物理 TSV、终态与可比统计一致；Radau5 四维核心基线冻结逐列求值、中间列 recoverable 重试、fatal
  截止和端点原子性；
- 执行 identity 补记 `OMP_MAX_ACTIVE_LEVELS`／`OMP_NESTED`。本阶段未运行正式 `20/30 s` 矩阵、未
  计算 speedup、未改公共默认或根 README；刷新构建后的完整 CTest 为 86 项。

暂停记录（2026-08-24）：INT-07A 的协议、manifest 和串行工件入口继续有效，但此前“下一步进入
INT-07B”的时序被撤销。独立正确性审查后的修复不追溯生成性能排名。实／复近奇异相对残差资格与
跨构建严格浮点标志门现已完成；本轮只闭合正确性证据，仍不实现 comparator、不运行正式矩阵，也
不开始并行 Jacobian 候选。

前置闭合记录（2026-08-24）：三个逐级缩小谱隙的实／复移位系统直接复用生产矩阵形成、分解与求解，
以解析解、相对后向残差和仓外官方 DECSOL 临时 oracle 交叉核对；产品没有新增条件数阈值。顶层 CMake
拒绝已知不严格浮点选项，Radau5 核心以编译宏守门；INT-07A manifest 升为
`int07a_serial_baseline_v2`；唯一编译启动器检查生成表达式、目标选项和可读响应文件展开后的实际命令，
通过后亲自注入资格 runner 要求的证明宏。不执行启动器的生成器、既存启动器链和不透明间接配置入口
均响亮失败；runner metadata 与 wrapper postflight 共同核对构建类型、编译器和编译语义身份。

双工具链复核记录（2026-08-24）：Ubuntu GCC 13 与 Clang 20＋LLVM libomp 分别从空目录完成
Release 构建并通过 87/87 项 CTest；真实 OpenMP 团队、重定位安装消费者、Radau5 核心与适配层、
GZ18／IRW 真实短窗及资格 metadata 均在两套构建中通过，第一方新增编译告警为零。该记录只签收
上述正确性前置，不改变 `INT-07B` 的暂停状态。

### INT-08 — 双车型、跨平台与交付收口

前置 Goal：INT-07。

唯一产物：具有明确支持身份和证据边界的 Radau5 研究后端。

允许修改：资格工具、安装／离线构建、`NOTICE`、局部 README、文档索引和必要 ADR。

明确不做：不把 Radau5 改成默认，不实现 Newmark/Zhai，不要求与 CVODE 或上游逐位相同。

完成门：

1. GZ18、IRW 被动及一个 100 Hz 受控场景完成短窗和长窗；检查 `q/v/z`、轮对／轴桥横移和摇头、
   轮轨 `N/Tx/Ty`、接触切换、相位、Maxwell 状态及首次非有限点。
2. 复用 INT-07 已冻结的三档容差、独立参考、比较范数和事件／输出时钟，并把同一
   规范扩展到长窗。平滑短窗检查状态范数的加密收敛；非光滑长窗分别检查接触事件
   时刻、波形包络、峰值和物理残差，不要求未对齐逐点差在每次加密中都单调。
3. Linux GCC/Clang、项目支持的 Windows GNU 风格前端、完整 CTest、安装后消费者和离线源码包
   全部通过。
4. 许可原文、来源处置、修改记录和 `NOTICE` 随安装材料交付；`libs/integrators/README.md` 如实
   改为“CVODE 默认、Radau5 已准入研究后端”。
5. 若外部研究消费者确需选择后端，新增 ADR 和只列 `kCvodeBdf2`／`kRadau5` 等真实实现的强类型
   入口；否则保持源码树私有入口。两种结果都不得改变无参数默认构造。

## 7. 测试矩阵

### 7.1 后端中立合同

现有以下测试语义应抽取复用，而不是复制成两套逐渐漂移的断言：

- [`verify_cvode_continuous_state_advancer.cc`](../../../tests/integrators/verify_cvode_continuous_state_advancer.cc)：
  解析系统、非自治时间、稠密输出、非法输入、失败封锁和重初始化；
- [`verify_system_rhs_bridge.cc`](../../../tests/integrators/verify_system_rhs_bridge.cc)：
  `[q;v;z]` 映射、逐分量容差前提及 accepted/trial 隔离；
- [`verify_system_continuous_state_advancer.cc`](../../../tests/integrators/verify_system_continuous_state_advancer.cc)：
  公开事务、Context 局部数据同步、稠密批和真实 RHS 失败。

BDF 最大阶数、实际阶数查询和 SUNDIALS 返回码仍是 CVODE 专属测试，不应强迫 Radau5 伪造。

### 7.2 Radau5 方法测试

| 类别 | 最低覆盖 |
|---|---|
| 精确解 | 常导数、\(y'=t\)、线性衰减、谐振子 |
| 阶数 | 平滑问题以测试私有步长夹持做固定步减半的五阶终点趋势；自适应容差加密 |
| 刚性 | 刚性标量、Prothero–Robinson、Robertson、Van der Pol |
| 稠密输出 | 最近一步覆盖、两端一致、内部点收敛、失败后失效 |
| 状态机 | 拒步、Newton 失败、奇异矩阵、recoverable/fatal RHS、stop 夹持、重初始化 |
| 实例隔离 | 两个实例交错推进、并发 Jacobian worker 无共享可变状态 |

### 7.3 车辆资格

先运行 GZ18 与 IRW 短窗，再进入冻结长窗。比较至少包含：

- 连续状态和 Maxwell 力状态；
- 车体、构架、轮对／轴桥横移与摇头；
- 各轮法向力、切向力和 `Y/Q` 类派生量；
- 接触斑数量、失联与重新接触时刻；
- 成功内部步、误差检验失败、非线性收敛失败、RHS、Jacobian、Newton 和线性分解；
- 墙钟与峰值内存。

性能结论只在相近物理误差下成立。Radau5 的 L-stability、五阶身份或较少步数都不能单独证明它比
CVODE 快，也不能成为默认切换依据。

## 8. 其他积分器待做登记

### 8.1 BDF

当前 CVODE 已提供真实 BDF 后端，本路书不搬运自有 BDF 实现。INT-02 只保护它的默认构造、最大阶
身份和共同合同。若未来研究第一方 BDF，应另立来源、控制器、历史表示和与 SUNDIALS 共存边界，不能
借 Radau5 Goal 顺手进入。

### 8.2 Newmark

状态：待做，未排期。

未来启动前至少需要单独裁决二阶机械问题接口、configuration retraction、\(q/v/z\) 耦合残差、
Newton／切线、步长控制和稠密输出。实现与真实消费者同时存在后，应以带标签的
方法专属配置进入中央工厂：标签对应真实 runtime alternative，payload 明确携带
Newmark 的 \(\beta,\gamma\)、步长及已冻结策略，不把这些参数塞进后端通用参数袋。
本路书不检查或预建这些接口，也不增加 `Newmark` 枚举值。

### 8.3 Zhai

状态：待做，未排期。

未来启动前至少需要冻结“简单显式二步法”还是 predictor-corrector 家族、\(\phi,\psi\)、等步长与
事件边界、加速度历史、configuration retraction、Maxwell `z` 方案和稠密输出。本路书不检查或
预建这些接口，也不增加 `Zhai` 枚举值。未来真实实现同样必须以带标签的方法专属配置
表达 \(\phi,\psi\)、固定步长、启动历史和 `z` 策略，并与一个真实 concrete runtime alternative
同时落位；不能只增加 tag，也不应复用 Newmark payload。

## 9. 文档更新纪律

- 本路书记录 Radau5 实施状态和完成门；算法公式只维护在模型与算法文档。
- INT-01 后端行为只维护在[完整求解器合同](INT_01_RADAU5_SOLVER_CONTRACT.md)，来源身份只维护在
  [`SOURCE_DISPOSITION.txt`](../../../external/radau5/SOURCE_DISPOSITION.txt)，不建立第二份摘要账本。
- `libs/integrators/README.md` 只描述已经实现的能力。Radau5 在 INT-08 前不得被写成已准入后端。
- 根 `README.md` 不因本研究计划改变；只有未来默认或公开产品能力改变时才另行裁决。
- 每个 Goal 完成后更新本文件状态，不把临时日志、参考输出、性能快照或 `tmp/` 笔记提交为证据。
- Newmark/Zhai 未激活前只保留本节待做登记；不得建立不能调用的占位实现。
