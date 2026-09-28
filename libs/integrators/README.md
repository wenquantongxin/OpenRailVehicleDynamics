# libs/integrators

**职责**：时间推进。抽象推进器接口与 CVODE 后端（[ADR-0003](../../docs/adr/0003-abstract-advancer-cvode-first.md)）。

**对应 Goal**：G43–G46、G54–G55。

公共构造继续使用 CVODE BDF2。Radau5、基本 Newmark 与基本 Zhai 另有源码私有后端，
车辆与性能资格仍由后续消费者完成，不构成公开的方法选择入口。事务语义是这一层的核心义务：失败的公开推进不得
污染已接受的状态，成功到达边界后恰好提交一次；CVODE 内部的自适应拒步不等同于公开推进失败。

真实正长度推进中，只有后端明确列举的纯数值失败会以
`ContinuousStateNumericalFailure` 暴露。调用方按稳定的 `reason()` 分类；`backend_code()` 只保留当前
后端的原始诊断值，不能脱离后端身份解释。RHS 抛出的原始异常始终优先传播，重初始化、同步及辅助 API
失败也不经该类型改写；未知返回码继续作为普通错误响亮失败。

当前 CVODE 适配器用 `CV_ONE_STEP` 逐个接受内部步，因此 `CV_TOO_MUCH_WORK` 对应的
`kAdvanceWorkBudgetExhausted` 在现有调用模式下不会触发；该映射保留给后端语义完整性及未来可能的
`CV_NORMAL` 调用。系统层的成功内步预算现在也使用 `kAdvanceWorkBudgetExhausted`，其
`backend_code` 为 0；默认仍为 1,000,000，可由源码私有执行配置显式调整。预算耗尽保持接受态不变，
要求同步后恢复，不自动缩步，也不作为数值稳定性结论。

## C++ 中的并列后端抽象

`SystemContinuousStateAdvancer` 只负责 accepted/candidate 事务，并且只调用后端中立的
`ContinuousStateAdvancer` 接口。源码树私有的 `SystemContinuousStateBackend` 用下列闭集恰好持有
一个真实后端：

```cpp
using ConcreteRuntime =
    std::variant<std::unique_ptr<CvodeBdf2Runtime>,
                 std::unique_ptr<CvodeBdf5Runtime>,
                 std::unique_ptr<Radau5Runtime>,
                 std::unique_ptr<NewmarkRuntime>,
                 std::unique_ptr<ZhaiRuntime>>;
```

共同推进通过 `std::visit` 取得；配置身份也由实际 alternative 反推，而不是相信一枚与对象分离的
标签。因此 Radau5 不经 CVODE 兼容路径调用，CVODE 也不伪造 Radau5 语义；BDF 阶数、CVODE 并行
有限差分 Jacobian 和 Radau5 线性化失效等专属能力仍留在各自实现中。

公共构造始终选择 CVODE BDF2。其他方法只能由源码树内部的强类型配置显式选择；安装 API
不提供字符串、环境变量、任意整数或占位枚举形式的后端开关。未来方法只有在具备真实 concrete
runtime、方法专属配置／桥接和实际消费者后，才作为新的并列 alternative 加入。

G43 已落下动态连续状态 RHS、逐分量绝对容差值与极窄的抽象推进器接口。G44 已实现真实 CVODE
后端：`double + int32`、串行状态向量、BDF 与稠密数值 Jacobian；资源不足时使用 SUNDIALS
内建串行差分路径，满足既定条件的轮轨系统由私有的车型中立提供器并行计算有限差分列。后端已完成端点复制、最近成功内步
密集输出与显式“时间 + 完整状态”重初始化；接口仍不持有系统上下文，也不暴露后端内部存储。
当前 `SystemContinuousStateAdvancer` 独占主 candidate 上下文；并行 Jacobian 启用时，每个 worker 另永久独占
上下文、RHS 桥和工作缓冲，各桥只借用自己的上下文。
桥不保存已接受上下文，RHS 因而没有可写回接受态的路径。系统连续向量按冻结范围直接映射
`[q; v; z]`，不编码 GZ18 的 109 或任何车型
布局。当前桥的构造要求显式写出 `NoCallTimeAppliedForces`；模型重力、阻尼和内部力元仍参与，
但 G42 的三类调用期外力不会被假装已经接线。SUNDIALS 头只存在于私有实现，公共头不出现
任何 SUNDIALS 类型；开发期从外部前缀精确查找 7.7.0，缺失即配置失败。

当前组装图是自治的多体 q/v 与力元 z。RHS 把后端试算时间和完整状态一起写入专用试算上下文。
G45 的事务语义保持不变；INT-06 将具体数值后端、RHS bridge、CVODE 专用并行 Jacobian 资源和构造
自检统一收口到源码树私有 `SystemContinuousStateBackend`。其 concrete runtime 是只包含真实实现的
闭集，实际 alternative 决定 `cvode_bdf2`、`cvode_bdf5`、`radau5`、`newmark` 或 `zhai` 身份，避免 recipe 标签与对象类型
分离。`SystemContinuousStateAdvancer` 只保留 candidate/accepted 事务和共同 `ContinuousStateAdvancer`
调用；内部试算从不到达接受上下文，成功边界只提交一次，失败后必须从仍有效的接受端点显式同步。
上下文局部数据由唯一的 `CopyContextLocalData` / `SynchronizeContextLocalDataFrom` 路径同步，当前包含
关节阻尼、标称力与八路独立车轮保持转矩；不建立参数袋，也不在 RHS 热路径扫描状态、名称或参数。

INT-03 的纯 C++ Radau5 核心位于 `external/radau5`，只保留正向 `M=I`、full-dense、三阶段五阶
Radau IIA；以一个实 `N×N` 和一个复 `N×N` LU 求解 simplified Newton。源码树私有适配器已经运行
同一份后端中立合同。源码树只有一个 `SystemContinuousStateIntegrationAccess`，通过闭合强类型
配置构造五种真实 runtime 并查询实际身份；不为每个方法另加一套 access 类。Radau5
不使用 CVODE 并行 Jacobian provider，其 worker identity 固定为串行 `1`。公开构造、安装入口和
CVODE BDF2 默认均未改变。

BDF 最大阶数是 CVODE 实例构造时冻结的专属执行身份。现有公共构造入口保持最大二阶，不增加公共
阶数旋钮、环境变量或编译期开关；源码树资格默认配方可明确选择 BDF2 或 BDF5。BDF 最大／实际阶数
查询只保留在直接 CVODE 适配器测试，不再污染系统事务层，也不要求 Radau5 伪造诊断。资格摘要以
通用 recipe identity 为主，`maximum_bdf_order` 只是 CVODE 的 nullable 兼容字段。

Newmark 与 Zhai 的基本坐标数值核、系统坐标桥和物理态适配器均在 `src/`，已通过同一个中央
工厂接入。每种 runtime 独占其桥及求解资源，使用地址稳定的对象确保借用关系；机械后端不创建
CVODE Jacobian 资源。Newmark 的固定参数及全 Newton、Zhai 的原始启动和等步历史不进入
通用参数袋，系统事务循环与资格 runner 不增加平行调用入口。

G54 把数值后端的生产入口收窄为 `CV_ONE_STEP` 单内部步；系统层循环到公开目标，并在每个返回
端点显式安装状态、重算四个 GZ18 载体投影、推进 candidate 站位。近期冻结 WRL 路径关闭
`sdot` 预测，因此运行时只保存每载体最近 accepted `s`，不建立 `sdot/dt`、版本号或通用历史
注册表。八轮接触核只在 RHS 求值，不为站位提交额外运行。公开成功把时间、完整状态与站位一次
提交；失败保持 accepted 不变，恢复从 accepted 当前站位而不是初始静态锚点开始。

线路分支由调用方持有的站位种子标识，`TrackGeometry` 从该种子至多作两次 Newton 校正，不用
有限搜索窗反向约束积分器步长，也不扫描全线改选远根。局部分支不能在该合同内解析时属于致命
RHS 失败；`SystemRhsBridge` 不为投影异常增加可恢复旁路。线路 JSON 的作者边界不是推进失败
边界；左右两侧由 `TrackGeometry` 原生直线延长。

轮轨投影提示是 accepted 数值历史而非连续状态。每个成功内步安装 candidate 状态并更新非空提示后，
系统事务层通过后端中立通知使 Radau5 丢弃旧 Jacobian 和实／复分解；步长控制、阶段外推、接受态与
刚发布的稠密区间不重初始化。这样下一内步先在新提示下计算端点 RHS 和线性化，不把搜索分支历史
悄悄留在复用矩阵中。公共 CVODE BDF2 默认及其既有推进语义不变。

系统层不为无人消费的内部步建立回调注册表。IRW 100 Hz 控制事件是现行真实消费者；事件提交新保持转矩后，
先同步主 candidate RHS 与全部 Jacobian worker 的上下文局部数据，再执行后端重初始化；同步或重初始化失败沿用既有
`requires_synchronization` 封锁与幂等重试，不建立回滚协议。G55 的真实采样消费者已增加一次公开推进内的窄密集状态批，采样只读各成功内部步
的真实密集区间，不把 101 个样本变成停靠点或车辆级公开输出格式。事件的 `x⁻→x⁺` 与发布时序继续
由 ADR-0003 冻结。

并行 Jacobian 提供器与 SUNDIALS 接口均留在私有实现；公开面只增加窄义的“请求 worker 数”只读统计字段。该值
不是实际 OpenMP 团队规模，也不是线程策略 API。

`include/orvd/integrators/` 为公开头，`src/` 为实现；G46 将目标安装并导出为
`ORVD::integrators`。SUNDIALS 仍是外置第三方接口依赖，但离线源码包提供锁定源码并用同一
工具链静态构建，不把其类型带入 ORVD 公共头。

## 基本坐标数值核（源码私有）

`CoordinateSecondOrderProblem` 的状态为 `(q, s, z)`，其中 `s = qdot`，一次求值同时返回
`b = qddot` 与 `g = zdot`。这与公开多体状态 `(q, v, z)` 不同：源码私有的
`SystemCoordinateProblem` 负责 `s = N(q)v`、`v = Nplus(q)s` 与
`b = N(q)vdot + Ndot(q,v)v`，不能把物理广义速度直接填入 `s`。
核心不识别关节或车型，也不提交问题对象的外部接受状态。

`NewmarkCore` 固定采用平均加速度公式与 `z` 的梯形离散，联立求解完整 `(b1, z1)` 残差。
调用方显式提供名义步长、Newton 各量纲尺度和未知量差分参考尺度；每轮重建差分 Jacobian，
采用缩放后的稠密 LU，默认最多 12 次全步 Newton。`ZhaiCore` 固定采用基本两步公式与 `z` 的
AB2 离散；首次推进或传入步长不同于上次成功步时使用原始启动式。两者均不自动缩步或更换方法。

本批的非线性求解策略已固定为“每轮重建差分 Jacobian 的全 Newton”，不在步内或跨步复用
切线与分解。Newmark 时间离散与非线性方程的求解策略是两个层次；全 Newton 与修正 Newton
均可用于求解 Newmark 残差，缺少解析切线并不要求改用修正 Newton。参见 OpenSees 的
[Newton](https://opensees.github.io/OpenSeesDocumentation/user/manual/analysis/algorithm/Newton.html)
与 [Modified Newton](https://opensees.github.io/OpenSeesDocumentation/user/manual/analysis/algorithm/ModifiedNewton.html)
定义。本批保留已批准的全 Newton 实现，不增加切线复用或按残差趋势重建的策略。

后续性能报告应将该实现标为“Newmark 平均加速度＋梯形内变量＋每轮差分全 Newton”，同时
记录未知量维数、步长、Newton 尺度及迭代上限，分别报告普通 RHS、差分 RHS、迭代和分解次数。
若一次成功步完成 `k` 次 Newton 迭代，令 `m = nq + nz`、投影改变状态时 `p = 1`，否则
`p = 0`，则本步有 `k*m` 次差分 RHS、`1+k+p` 次普通 RHS、`k` 次 Jacobian 构建及分解；
初始化的那一次 RHS 另计。实际迭代数、耗时与成本占比由运行记录确定。等误差比较所得倍数
只对应被测实现与工况，不能外推为所有 Newmark 实现的固有成本。

端点投影是问题对象提供的坐标存储适配。核心保存最近成功初始化的参考坐标，初始化本身保持
合法输入。Newmark 收敛后投影，改变状态则重算接受端点导数；Zhai 投影后求值，再提交状态与
历史。四元数验证同时投影坐标与坐标速度，恢复初始化范数并保持物理角速度。

数值推进失败保留最近接受状态，后续推进要求显式重初始化；失败过程中已经发生的求值和求解
成本保留。非法调用参数不破坏可用状态。普通求值与 Jacobian 扰动求值分别统计，启动与投影
放在私有诊断中。两个核心不继承 `ContinuousStateAdvancer`；停止时刻调度与稠密输出属于
源码私有的物理态适配器。
它们通过不安装、不导出的内部 OBJECT 目标编译，产品库与 `verify_newmark_core`、
`verify_zhai_core` 使用同一批对象；测试覆盖原公式、坐标映射、内变量、收敛性与事务失败。

## 系统坐标问题桥（源码私有）

`SystemCoordinateProblem` 借用系统、编译计划与调用方专用的物理试算 context，组合现有
`SystemRhsBridge`。一次 `Evaluate` 从 `(q,s,z)` 恢复物理速度，打包原有 `(q,v,z)`，调用一次
完整 RHS，再在同一物理试算状态上把 `vdot` 映射为完整 `nq` 维 `qddot`；力元、轮轨和主动
转矩继续走既有装配。桥不保存接受态、方法历史或初始化范数，不按车型和力元名称分支。

桥独有的几何 context 与预分配转换缓冲负责初值验证、物理状态导入/导出及端点投影。这些
操作不调用 RHS，也不修改物理试算 context。初值保留安全非单位四元数，拒绝径向坐标速度；
RPY 初值经前向映射检查现有奇异域。Newton 的任意试算 `s` 可以有径向分量，此时
`Nplus(q)s` 是明示的坐标延拓，物理 RHS 的 `qdot` 是 `N(q)Nplus(q)s`，不等于任意原始 `s`。

配对投影根据模型公开的自由体范围逐块恢复参考范数并重建四元数坐标速度，保持物理速度；
其余坐标和速度逐值保留。导数输出及投影数据均在全部操作成功后写出。桥转发现有保持量同步，
不更新站位提示、不提交外部接受态、不重置核心；这些职责继续属于外层系统推进。

桥只支持当前单多体组件及其冻结的 `q/v/series-force z` 布局（允许空 `z`），构造时检查范围
覆盖与系统归属；调用方必须另建试算 context。桥本身不继承公共推进器，由中央工厂的机械
runtime 拥有并交给对应适配器。
`verify_system_coordinate_problem` 通过实际多体模型和力计划验证 RHS 等价、坐标转换、
投影与隔离，并让两个基本核完成真实刚体、Ball-RPY 和运动滑块–Maxwell 的短程验证。

## 基本方法的系统推进适配

`SystemContinuousStateIntegrationConfiguration` 的方法 alternative 分别持有三种 ODE 容差配置、
已有 `NewmarkConfiguration` 或 `ZhaiConfiguration`，另持有共同的成功内步预算。旧的
`recipe + tolerances` 构造只允许 ODE，选择机械枚举会明确拒绝，不能补造步长或 Newton 尺度。
Newmark 六组尺度继续按 q/s/b/z 的单位显式传入，s/b 为 nq 维；Zhai 不接收 ODE 容差。
本批只给测试 fixture 设定尺度，未植入车型默认值。

`NewmarkContinuousStateAdvancer` 与 `ZhaiContinuousStateAdvancer` 共享时间、事务和物理输出
适配，各自使用原基本核。名义网格由起点和整数步号通过 `fma` 计算，公式与历史始终使用原 H。
真实边界使用短步，之后从边界重新建立网格；Zhai 按实际传入步长决定是否启动，不自动重试。
核心的显式端点入口只允许舍入调整：与 t+h 的差不超过 min(h/16,16 ULP尺度)；运行时目标吸附
另限 min(H/16,8 ULP尺度)，同时通过核心检查。RHS 从开始即使用指定端点时间，不事后改钟。
原核心单参数入口仍按 t+h 推进。不能表示正向时间时报 kStepSizeUnderflow。

稠密输出冻结最近成功步的两个物理 [q;v;z] 端点，普通分量线性插值，RPY 存储坐标不包装。
桥对四元数采用同半球代表的归一化线性插值并恢复初始化范数；区间端点直接逐值复制缓存。
反号四元数在区间内部使用等价代表，末端可能有存储符号跳变，物理姿态仍连续。插值不调用
RHS 或 N/Nplus、不回灌数值状态、不改变工作计数，也不要求满足动力学微分关系。光滑状态的
误差用固定网格验证二阶；采样时刻不会变成后端停止边界。

成功端点的站位提示更新沿用同一投影返回值，因此 Newmark/Zhai 的通知为空操作，当前 B/G
及 Zhai 历史继续有效，不因此增加 RHS。此结论仅适用于同端点、同物理输入；外部状态、保持
输入或分支变化仍须显式同步并重初始化。真实接触测试覆盖提示实际变化与冷、热求值的一致性。

公开数值失败原因末尾新增 kNonFiniteState、kNonlinearConvergenceFailure、kSingularLinearSystem，
已有枚举值不变；基本方法的一次失败不伪装成多次重试。普通求值和差分求值分别统计，初始一次
RHS 保留，Zhai worker 数为 0；初始化后及同步后统计均包含新的初始求值。源码私有
`CoordinateDiagnostics` 读取启动与投影次数，ODE 返回无此诊断。任何正步失败保留最后发布端点、
失效稠密区间并要求同步，已发生工作不回滚。底层物理异常保留原原因。

安装头仅调整 SystemContinuousStateAdvancer 的私有构造，并追加上述公共失败分类；公共方法
选择入口、默认 BDF2 与物理状态布局保持原合同。安装消费者验证新分类与原公开构造，工厂能力
由源码树测试消费。本批不修改车型、工况运行器或前端，也不提供整车性能结论。
