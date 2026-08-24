# libs/integrators

**职责**：时间推进。抽象推进器接口与 CVODE 后端（[ADR-0003](../../docs/adr/0003-abstract-advancer-cvode-first.md)）。

**对应 Goal**：G43–G46、G54–G55。

CVODE 是已选定的首个且当前唯一准入后端。Radau5 已有源码树研究实现，但尚处于后续车辆与性能
资格期，不构成公开支持承诺。事务语义是这一层的核心义务：失败的公开推进不得
污染已接受的状态，成功到达边界后恰好提交一次；CVODE 内部的自适应拒步不等同于公开推进失败。

## C++ 中的并列后端抽象

`SystemContinuousStateAdvancer` 只负责 accepted/candidate 事务，并且只调用后端中立的
`ContinuousStateAdvancer` 接口。源码树私有的 `SystemContinuousStateBackend` 用下列闭集恰好持有
一个真实后端：

```cpp
using ConcreteRuntime =
    std::variant<CvodeBdf2Runtime, CvodeBdf5Runtime, Radau5Runtime>;
```

共同推进通过 `std::visit` 取得；配置身份也由实际 alternative 反推，而不是相信一枚与对象分离的
标签。因此 Radau5 不经 CVODE 兼容路径调用，CVODE 也不伪造 Radau5 语义；BDF 阶数、CVODE 并行
有限差分 Jacobian 和 Radau5 线性化失效等专属能力仍留在各自实现中。

公共构造始终选择 CVODE BDF2。Radau5 与 BDF5 只能由源码树内部的强类型 recipe 显式选择；安装 API
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
闭集，实际 alternative 决定 `cvode_bdf2`、`cvode_bdf5` 或 `radau5` 身份，避免 recipe 标签与对象类型
分离。`SystemContinuousStateAdvancer` 只保留 candidate/accepted 事务和共同 `ContinuousStateAdvancer`
调用；内部试算从不到达接受上下文，成功边界只提交一次，失败后必须从仍有效的接受端点显式同步。
上下文局部数据由唯一的 `CopyContextLocalData` / `SynchronizeContextLocalDataFrom` 路径同步，当前包含
关节阻尼、标称力与八路独立车轮保持转矩；不建立参数袋，也不在 RHS 热路径扫描状态、名称或参数。

INT-03 的纯 C++ Radau5 核心位于 `external/radau5`，只保留正向 `M=I`、full-dense、三阶段五阶
Radau IIA；以一个实 `N×N` 和一个复 `N×N` LU 求解 simplified Newton。源码树私有适配器已经运行
同一份后端中立合同。源码树只有一个 `SystemContinuousStateIntegrationAccess`，通过闭合强类型
recipe 构造三种真实 runtime 并查询实际身份；不再为 Radau5 或未来方法各加一套 access 类。Radau5
不使用 CVODE 并行 Jacobian provider，其 worker identity 固定为串行 `1`。公开构造、安装入口和
CVODE BDF2 默认均未改变。

BDF 最大阶数是 CVODE 实例构造时冻结的专属执行身份。现有公共构造入口保持最大二阶，不增加公共
阶数旋钮、环境变量或编译期开关；源码树资格默认配方可明确选择 BDF2 或 BDF5。BDF 最大／实际阶数
查询只保留在直接 CVODE 适配器测试，不再污染系统事务层，也不要求 Radau5 伪造诊断。资格摘要以
通用 recipe identity 为主，`maximum_bdf_order` 只是 CVODE 的 nullable 兼容字段。

Newmark 与 Zhai 不在当前 recipe 中预占枚举值。它们需要先冻结二阶机械状态和 `z` 状态策略；真实
实现完成后，以“真实 concrete runtime alternative ＋它的带标签方法专属配置＋自有
bridge”成套加入中央工厂。Newmark 的 \(\beta,\gamma\) 与 Zhai 的 \(\phi,\psi\)／历史策略
不进入通用参数袋；系统事务循环和资格 runner 不再增加平行调用入口。

G54 把数值后端的生产入口收窄为 `CV_ONE_STEP` 单内部步；系统层循环到公开目标，并在每个返回
端点显式安装状态、重算四个 GZ18 载体投影、推进 candidate 站位。近期冻结 WRL 路径关闭
`sdot` 预测，因此运行时只保存每载体最近 accepted `s`，不建立 `sdot/dt`、版本号或通用历史
注册表。八轮接触核只在 RHS 求值，不为站位提交额外运行。公开成功把时间、完整状态与站位一次
提交；失败保持 accepted 不变，恢复从 accepted 当前站位而不是初始静态锚点开始。

局部投影窗用于隔离线路分支，不是积分器步长上限。`ContinuousStateRhs` 默认把异常分类为致命；
系统桥只把合法局部窗内暂时无投影根这一专门异常分类为可恢复，让所选的 CVODE 或 Radau5 后端缩步重试。多根、非法
参数及成功端点投影失败继续终止公开事务，不能由诊断字符串或统一兜底降级。线路 JSON 的作者
边界不是推进失败边界；左右两侧由 `TrackGeometry` 原生直线延长。

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
