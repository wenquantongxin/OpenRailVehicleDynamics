# INT-01：Radau5 首版完整求解器合同

- 状态：**已冻结（INT-01 完成）**
- 冻结日期：2026-08-23
- 勘误日期：2026-08-24（独立正确性审查后补齐 stop、失败因果、极端尺度与隐藏投影历史合同）
- 架构决策：[ADR-0009](../../adr/0009-cvode-default-radau5-research-backend.md)
- 实施路书：[INTEGRATOR_MIGRATION_ROADMAP.md](INTEGRATOR_MIGRATION_ROADMAP.md)
- 来源账本：[`external/radau5/SOURCE_DISPOSITION.txt`](../../../external/radau5/SOURCE_DISPOSITION.txt)

本文是 INT-03–INT-05 实现和验收 Radau5 的数值行为依据。算法背景和 Butcher 表只在
[时间积分方法文档](../../models_and_algorithms/numerical_methods/TIME_INTEGRATION_METHODS.md)维护；
来源 URL、摘要和逐文件处置只在来源账本维护。实现若偏离本文，必须先修改合同并说明理由，不能以
“上游就是如此”替代 ORVD 已冻结的事务语义。

## 1. 身份与能力边界

首版固定为：

- 三阶段、五阶 Radau IIA，配点为
  \(c_1=(4-\sqrt6)/10\)、\(c_2=(4+\sqrt6)/10\)、\(c_3=1\)；方法 stiffly accurate，成功端点
  等于第三阶段状态；
- 正向时间、`double`、显式一阶 ODE \(\dot y=f(t,y)\)，质量矩阵恒为单位阵；
- 动态正维状态，标量相对容差、逐分量绝对容差；
- full-dense 数值 Jacobian、simplified Newton、一个实 \(N\times N\) 系统和一个复
  \(N\times N\) 系统；
- 自适应步长和最近一个成功内部步的三次配点稠密输出；
- 每次调用最多发布一个成功内部步，调用内部允许有限次拒绝试步。

首版不含质量矩阵／DAE、带状或稀疏 Jacobian、解析 Jacobian、Krylov、预条件器、Hessenberg、
二阶特殊结构、反向积分、用户输出回调和 GPU。INT-03–INT-06 基线不含并行
Jacobian；INT-07 只可按第 2 节冻结的保持语义规则验证一个后端中立批计算候选。
Radau5 不承担事件目录、接触
sample-and-hold、Newmark、Zhai 或历史 Radau3 的语义。

产品只编译 C++。官方 Fortran 只按来源账本中的 URL 在仓外临时核验；不提交源码、不记录其
SHA-256，也不成为构建或分发依赖。

## 2. 所有权与回调边界

1. 核心实例独占步长、阶段、Jacobian、分解、稠密系数、统计和工作区；不得使用全局或线程局部
   数值状态。两个实例交错推进时互不影响。
2. RHS 是借用对象，维数在构造期间固定。核心不得拥有或发现 ORVD System Context，也不得保留
   指向一次 RHS 输入／输出视图的指针。
3. 适配器以不抛异常的状态码回调跨越核心状态机，并单独保存 `std::exception_ptr`。C++ 异常不得
   穿过 `noexcept` 或状态码边界。
4. 所有试算写入私有 candidate 工作区；只有误差检验通过后，端点、阶段外推和稠密系数才成为
   accepted 数值历史。
5. INT-03–INT-06 基线的 finite-difference Jacobian 串行执行，
   `requested_dense_finite_difference_jacobian_worker_count` 恒为 `1`。INT-07 可以验证并行候选，
   但这是一次显式合同扩展：Radau5 核心仍拥有名义扰动公式、实际可表示增量、
   每列最多四次缩扰动重试、fresh/stale 状态和本文统计语义。后端中立层只能
   批量求值核心已指定的扰动状态；每个 worker 独占 Context、RHS bridge 和工作区，
   异常结果按最低列序确定性发布。串行路径必须保留为回退基线；候选只有在逐列数值、
   recoverable/fatal 分类、accepted 隔离和计数都一致后才能保留。

## 3. 输入、容差与误差权重

私有舍入常数固定为：

\[
u=10^{-16}.
\]

状态维数必须大于零；初始／重初始化时间和每个状态值必须有限。绝对容差向量长度必须等于状态
维数，且每项有限并严格大于零。

首版 Radau5 要求：

\[
\operatorname{rtol}>10u=10^{-15}.
\]

因此 `rtol=0` 和其他不满足该严格不等式的值在 Radau5 构造边界抛 `std::invalid_argument`。公共
`ContinuousStateErrorTolerances` 仍可表达 `rtol=0`，CVODE 行为不变；Radau5 不静默加入相对容差
下限，也不声称支持尚未资格化的纯绝对分支。

调用方容差永不改写。核心私有副本按 Hairer 变换：

\[
r^*=0.1r^{2/3},\qquad a_i^*=r^*\frac{a_i}{r}.
\]

每个试步以该步起点 accepted 状态形成：

\[
s_i=a_i^*+r^*|y_{n,i}|.
\]

`r*`、每个 `a_i*` 和每个 `s_i` 在使用前必须有限且严格大于零。构造仍按上述书写顺序先计算
`a_i/r`，不以代数重排悄悄扩大数值能力；该比值或随后乘方、乘法发生下溢、上溢或非有限结果时，
将整组容差作为当前 Radau5 能力之外的构造输入并抛 `std::invalid_argument`。这不是“已经进入正长度
推进”的 `radau5::Failure`。成功端点接受后，以新 accepted 状态更新下一步权重。

## 4. 调用状态机与 stop

### 4.1 验证次序

`AdvanceOneInternalStepToward(stop, output)` 依次检查：

1. `stop` 有限且不早于当前公开时间；
2. `output` 维数正确；
3. 后端未处于失败封锁；
4. 健康态下若 `stop == current_time`，复制当前状态并返回 `{t,t,true}`。

前三类失败不得改变调用方输出、当前端点、已有稠密区间、统计或继续推进能力。失败封锁态下即使
同刻请求也抛 `std::logic_error`；同刻 no-op 不调用 RHS、不改变统计，并保留已有稠密区间。

### 4.2 正长度推进

- 构造和每次成功重初始化后的建议步长为 `1e-6` 秒。每次正长度调用的首个 trial 若满足
  `h_suggested >= (stop-current_time)/1.0001`，一次性吸附到精确 stop；无论该 trial 接受还是放弃，
  本次调用的“首次吸附资格”均已消费，后续 trial 必须沿控制器给出的缩步继续，不得再次吸附而撤销缩步。
  不满足该条件时仍把候选裁到 `stop-current_time`。
- 后续调用保留上次控制器给出的建议步长，但每次均裁到本次剩余 stop 距离。
- 若候选终步被裁到 stop，成功后公开时间直接赋为调用方传入的 `stop`，保证精确 double 身份；
  `reached_stop` 当且仅当返回终点与该 stop 相等。
- 正长度调用进入核心前清除旧稠密区间。调用内部可拒绝多个 trial，但只可发布第一个成功 trial；
  发布的内部步必须具有严格递增时间。
- 每次正长度调用最多计算 `100000` 个 trial；允许第 `100000` 个 trial，准备第 `100001` 个时失败。
- trial counter 在选定候选 `h` 后、任何端点 RHS、Jacobian、setup 或阶段工作前增加，因此所有
  放弃路径都占用总 budget。每个 trial 还要求 `current_time+h > current_time`，并保留上游
  \(0.1|h|>|t|u\) 的最小步检查。任一失败均按后端运行失败处置。

成功端点的输出采用两阶段提交：先在私有工作区验证时间、状态和稠密元数据均有限，再一次更新公开
端点并复制到调用方输出。输出尺寸错误已在进入核心前拒绝。

### 4.3 接受端点后的 RHS 时相

官方完整循环在非末步接受后立即求一次新端点 RHS。ORVD 必须在这次 RHS **之前**返回成功端点，
让系统层先把 candidate Context 安装到该端点并更新轮轨站位历史。核心保存下一建议步长、插值历史和
Jacobian 新鲜度决策；下一次正长度调用才在更新后的系统历史下求端点 RHS。同刻 no-op 不补做该 RHS。

轮轨投影提示属于 accepted 数值历史，但不属于连续状态 `(t,y)`；它改变下一次 RHS 投影搜索的局部
定义域。系统层每次在成功内部步后更新非空投影提示，必须通过 source-private 后端通知使 Radau5 的
旧 Jacobian 和实／复分解失效。该通知不得重初始化 accepted 状态、步长控制器、阶段外推或刚发布的
稠密区间；下一次正长度调用在新提示下先求端点 RHS，再重建 Jacobian 和分解。无轮轨投影提示的系统
不触发该通知，CVODE 公共默认及其既有推进语义不因本合同扩展而改变。

## 5. Newton、Jacobian 与线性分解

### 5.1 阶段初值和 Newton

- 首个试步和成功重初始化后的首个试步以零阶段增量开始；后续试步用最近成功步的配点多项式按
  新旧步长比外推三个阶段。
- Newton 最大迭代数 `NIT=7`。每次迭代在三个配点状态各求一次普通 RHS，并利用官方变换常数把
  耦合修正化为一个实系统和一个共轭复系统。
- Newton 修正范数为三个阶段按 `s_i` 加权后的 \(3N\) 维 RMS。RMS 必须采用缩放平方和
  （LASSQ）等价算法，有限的归一化分量不得仅因先平方而产生可避免的上溢。停止阈值为
  \[
  F_{\rm newt}=\max\!\left(10u/r^*,\min(0.03,\sqrt{r^*})\right).
  \]
- `FACCON` 在构造／成功重初始化时设为 `1` 并跨 trial 保留；每个 trial 开始先令
  `FACCON=max(FACCON,u)^0.8`，`theta=0.001`。只有 `1 < NEWT < NIT`，即默认配置的第 2 至
  第 6 次修正，才执行
  收敛率预测：以 `THQ=DYNO/DYNOLD` 更新收敛率，第二次取 `theta=THQ`，第 3 至第 6 次取
  `theta=sqrt(THQ*THQOLD)`，再保存 `THQOLD=THQ`。每次修正完成预测（或跳过预测）后均更新
  `DYNOLD=max(DYNO,u)`，供下一次修正使用。
- 在第 2 至第 6 次修正的预测分支内，`theta >= 0.99` 时立即放弃 trial 并把步长减半；否则令
  `FACCON=theta/(1-theta)`，并形成 `dyth=FACCON*DYNO*theta^(NIT-1-NEWT)/Fnewt`。仅当
  `dyth >= 1` 时采用上游预测缩步
  `0.8*clamp(dyth,1e-4,20)^(-1/(4+NIT-1-NEWT))`；其他 Newton 失败把步长减半。
- 每次完成并应用修正后，实际 Newton 接受条件固定为 `FACCON*DYNO <= Fnewt`，不是
  `DYNO <= Fnewt`。`NIT=7` 是上游循环的上限参数，但默认有限算术路径下不会真正启动
  第 7 次线性修正：第 6 次预测的指数为零，因而
  `dyth=FACCON*DYNO/Fnewt`。`dyth >= 1` 时已在应用第 6 次修正前放弃；
  `dyth < 1` 则同一不等式已保证应用该修正后通过收敛判定。因此不应为一个不可达的
  “第 7 次修正成功／上限”分支伪造黑盒测试；验收应覆盖第 6 次预测的
  `dyth < 1` 接受与 `dyth >= 1` 放弃两侧。

### 5.2 数值 Jacobian

在当前 accepted 端点先求基准 RHS \(f_0=f(t,y)\)。第 `i` 列的名义正向扰动为：

\[
\delta_i=\sqrt{u\max(10^{-5},|y_i|)}.
\]

优先使用 `(y_i + delta_i) - y_i` 的实际可表示增量。若正向名义扰动被舍入回基线或溢出，则先取
`nextafter(y_i,+infinity)`；该方向不可用时取 `nextafter(y_i,-infinity)`。最终允许有符号差分分母，
但扰动状态与实际增量必须有限、增量必须非零。recoverable 后缩小名义 `delta` 时仍执行同一可表示
选择，不得把已可表示的扰动重新缩成零增量。构造期间只改变私有状态副本，并在每次 RHS 尝试后
恢复基线。一个完整矩阵构造尝试只计一次 Jacobian evaluation，各列 RHS 尝试分别计入 Jacobian
专用 RHS 计数。

### 5.3 新鲜度与分解复用

- 首个试步计算 Jacobian。一次完整 Jacobian 构造成功后标记为 fresh；接受端点发布后，同一矩阵
  对新端点标记为 stale，但仍可按 Newton 收敛率复用。
- 实 Newton 矩阵为 `(u1/h)I-J`；复矩阵为 `(alpha/h + i*beta/h)I-J`，系数采用官方 RADAU5
  变换常数。一次逻辑 setup 包含一组实分解和复分解，不构造朴素 `3N×3N` 方阵。
- 接受后先以控制器候选计算 `QT=hnew/h`。若 `theta <= 0.001` 且 `QT` 落在 `[1,1.2]`，**忽略该
  近似候选并保持 `h_next=h`**，从而复用与该 `h` 对应的现有分解；不得一边采用 `hnew` 一边复用旧
  分解。`theta <= 0.001` 但 `QT` 超出该区间时，采用 `h_next=hnew`、复用 Jacobian 并重新分解；
  `theta > 0.001` 时采用 `h_next=hnew`，重新计算 Jacobian 后再分解。
- 下一调用因 stop 夹持或任何恢复路径实际改变 `h` 时，旧实／复分解立即失效；即使 Jacobian 可以
  复用，也必须按实际 `h` 重新分解。只有实际 `h` 完全不变时才允许直接复用分解。
- Error rejection、Newton 预测失败或奇异分解后：当前 Jacobian fresh 时只按新步长重新分解，stale
  时先重算 Jacobian。这里采用官方 Fortran 语义，不继承 C++ donor 的反向条件。
- setup 必须区分非有限 `lambda/h`、矩阵形成溢出、分解产生非有限值和精确零 pivot。非有限 shift
  属不可表示步长并终止为 step-too-small；矩阵或分解的非有限值是独立 linear-system 数值失败；
  只有精确零 pivot 才算奇异、令当前 trial 以 `h *= 0.5` 放弃，并累计 singular setup 次数。自最近
  成功重初始化起第 5 次真正奇异 setup 失败终止推进。

## 6. 误差估计、接受和步长控制

误差估计保留官方常数：

\[
d_1=-(13+7\sqrt6)/3,\quad d_2=(-13+7\sqrt6)/3,\quad d_3=-1/3.
\]

以 `(d1*z1+d2*z2+d3*z3)/h + f(t_n,y_n)` 为实系统右端，使用当前实分解求误差向量，并计算按
`s_i` 加权的 RMS；该 RMS 与 Newton 范数采用相同的缩放平方和算法。最终 `err` 下限为
`1e-10`。若首个成功步之前或前一个 trial 已拒绝，且第一次
估计 `err >= 1`，在误差修正状态额外求一次普通 RHS 并重新估计；第二次结果才是最终误差检验。

最终 `err < 1` 时接受，否则拒绝。接受／拒绝前计算：

\[
fac=\min(0.9,\,0.9(1+2NIT)/(NEWT+2NIT)),
\]

\[
quot=\operatorname{clamp}(err^{1/4}/fac,\,1/8,\,5),\qquad h_{new}=h/quot.
\]

默认启用 Gustafsson predictive controller。它只在当前 trial 已满足 `err < 1`、即确定
接受，且已有前一个成功步历史时计算：

\[
fac_{gus}=\frac{h_{acc}}{h}\left(\frac{err^2}{err_{acc}}\right)^{1/4}/0.9,
\]

再令 `facgus=clamp(facgus,1/8,5)`、`quot=max(quot,facgus)`、`hnew=h/quot`。每次接受后保存
`hacc=h`、`erracc=max(1e-2,err)`。已有成功步历史但当前 `err >= 1` 的拒试仍只使用
常规 `quot`，不应用 predictive factor。因此常规控制器允许的单次步长倍率为
`[0.2,8]`。

误差拒步时不修改 accepted 历史并设置 `REJECT=true`：首次成功步之前固定 `h *= 0.1`；之后采用
`hnew`。Newton、奇异 setup、recoverable 阶段 RHS 或 recoverable 误差估计附加 RHS 导致的 trial
放弃同样设置 `REJECT=true`。
任一较小 trial 随后接受时，若 `REJECT` 仍为真，则在清除此标记前令其后续建议
`hnew=min(hnew,h_accepted)`，禁止刚恢复成功便立即放大步长。每个最终 `err >= 1` 均增加 ORVD
error-test failure，包括首次成功步之前；上游排除这部分的 `NREJCT` 不直接采用。

## 7. RHS 异常、非有限值与重试预算

每次 RHS 调用尝试均先增加所属 RHS 计数，再调用借用对象并验证输出全有限。

| 位置 | recoverable 处置 | fatal／非有限处置 |
|---|---|---|
| 已接受端点的基准 RHS | 缩时间步不能改变求值点，因此不重试；重抛原异常 | 立即失败 |
| Newton 三阶段 RHS | 放弃整个 trial，`h *= 0.5`，在总 trial budget 内重试 | 立即失败 |
| 误差估计的附加 RHS | 放弃整个 trial，`h *= 0.5`，不把未完成的误差检验计为 error failure | 立即失败 |
| Jacobian 扰动 RHS | 时间步与扰动无关；同列令 `delta *= 0.1`，最多额外重试 4 次 | 立即失败 |

Jacobian 某列的名义尝试加 4 次缩扰动均 recoverable 时，重抛最后一次原始异常；不得改为只缩时间
步。Newton 阶段或误差估计附加 RHS 的 recoverable 放弃 trial 后，Jacobian fresh 时按减半后的 `h`
只重新分解，stale 时先重算 Jacobian；这与第 5.3 节的其他放弃路径一致。阶段 recoverable 重试后来
成功时清除旧异常，不向调用方泄漏。fatal 始终保持原 C++ 异常类型。

核心为每个 trial 保留窄化的 retry cause，至少区分 stage/误差修正 recoverable、Newton 收敛拒绝、
误差拒绝和真实奇异 setup。recoverable trial 后，若下一 trial 尚未执行一次成功 RHS 就在总 trial
budget、最小步或非有限 shift 处无法继续，终止原因必须为 `kRecoverableRhsExhausted`，适配器重抛
最后一个原始异常。下一 trial 一旦完成成功 RHS，旧 recoverable 因果即清除；若后来遇到独立真实
LU 奇异，则以真实 linear failure 为准，不得重抛陈旧异常。

进入后端后的任何未恢复失败均：

- 保持最近公开时间和状态不变；
- 清除稠密区间；
- 保留失败前已经累计的统计；
- 将后端置为必须成功重初始化才能继续的封锁态。

## 8. 稠密输出

成功步保存步首／步尾时间和状态，以及官方四组 `CONT` 系数。内部点令

\[
s=(t-t_{n+1})/h
\]

并计算：

\[
y(t)=C_0+s\left[C_1+(s-(c_2-1))\left(C_2+(s-(c_1-1))C_3\right)\right].
\]

有效区间严格为最近成功正长度步的闭区间 `[t_n,t_{n+1}]`。查询时间有限、位于区间内且输出尺寸
正确后才写调用方存储；步首和步尾分别直接复制保存状态，不依赖多项式回代取得逐位一致。

新建、成功重初始化和后端失败后均无稠密区间。非法推进输入与健康同刻 no-op 保留原有区间。

## 9. ORVD 统计映射

统计字段沿用
[`ContinuousStateIntegrationStatistics`](../../../libs/integrators/include/orvd/integrators/continuous_state_advancer.h)，
自构造或最近一次成功重初始化起累计：

| 字段 | Radau5 增量事件 |
|---|---|
| `successful_internal_step_count` | 每个正长度 accepted trial 一次；same-time、所有拒试和异常不计 |
| `right_hand_side_evaluation_count` | 每个非 Jacobian RHS 调用尝试，包括抛异常或返回非有限值的尝试 |
| `linear_solver_right_hand_side_evaluation_count` | 每个仅为数值 Jacobian 服务的 RHS 尝试，包括缩扰动重试和失败列 |
| `error_test_failure_count` | 每个完成最终估计且 `err >= 1` 的 trial，包括首次成功步之前 |
| `nonlinear_solver_iteration_count` | 每次实际启动并进行线性求解的 simplified-Newton iteration |
| `nonlinear_solver_convergence_failure_count` | 每个因 Newton 上限／预测发散或奇异 setup 而放弃的 trial，单个 trial 最多一次 |
| `linear_solver_setup_count` | 每次开始一组实＋复分解；即使其中一个失败仍只计这一次逻辑 setup |
| `jacobian_evaluation_count` | 每次开始构造完整 Jacobian；部分列后失败仍计一次，不按列计 |
| `requested_dense_finite_difference_jacobian_worker_count` | INT-03–INT-06 串行基线为 `1`；若 INT-07 候选通过，为该实例冻结的请求 worker 数，不是实测 OpenMP team 数 |

每次 RHS 只能进入两个 RHS 计数之一。查询统计不改变任何数值状态；统计非有限／溢出等内部不可能
状态以运行错误报告。失败重初始化不清零，成功重初始化把八个累计数值计数清零。

## 10. 重初始化和原子性

`ReinitializeAfterExternalChange(time,state)` 允许任意有限时间，不要求晚于旧端点；状态维数必须正确
且全部有限。所有调用方参数检查先于内部修改：非法参数保持公开端点、稠密区间、统计、历史和封锁
状态全部不变。

有效输入采用两阶段提交。成功后：

- 一次发布完整新时间和状态；
- 解除失败封锁；
- 清除稠密、阶段外推、Jacobian、分解、误差控制和异常历史；
- 建议步长重设为 `1e-6` 秒；
- 清零累计统计，但构造时冻结的 worker identity 保持不变（串行基线为 `1`）；
- 不调用 RHS。

有效输入进入内部准备后若失败，保持旧公开端点，清稠密并维持／进入封锁；已发生的失败工作仍可
查询。只有后续一次成功重初始化能够恢复推进。

## 11. 上游交叉核验与已知差异

2026-08-24 已在仓外临时目录以官方 URL 当次取得的 Fortran 核心做逐步 oracle。未计算或记录
Fortran 摘要，下载源码、临时 driver、可执行文件、输出和数值快照均不提交。对
`N=1/2/4` 的线性衰减单成功步，C++ 与官方实现的端点、步中配点稠密值和归一化工作统计一致；官方
刚性 Van der Pol 与 Robertson 的末值在 double 舍入尺度内一致，普通 RHS、Jacobian、成功步、误差
拒步、分解和 Newton 求解次数按第 9 节语义归一化后逐项一致。该记录验证的是本次保留算法主链和
状态机，不提供未来字节身份或跨平台金标。

同一来源的官方 DECSOL 另由仓外临时 driver 求解三个逐级缩小谱隙的实／复移位系统；其解与永久
解析 oracle 一致。永久回归仍以生产路径的相对后向残差和可解释的前向误差界判定，不依赖 Fortran
文件、临时驱动或保存的数值输出。

同次审计确认 C++ donor 在 Newton 预测不收敛路径反转了 `CALJAC` 条件；临时按 Fortran 语义修正
后，刚性 driver 的末值与全部上游统计一致。INT-03 仍须逐段审计全部保留路径，不能从这一案例外推
整包正确。

相对上游的首版有意差异冻结为：

1. 调用方容差只读，失败路径也不发生临时改写；
2. 一次调用只发布一个成功步，并把接受端点 RHS 延迟到系统层更新站位历史之后；
3. 终步以调用方 stop 的精确 double 身份发布，dense 两端直接复制；
4. `error_test_failure_count` 包含首次成功步之前的拒步；
5. recoverable/fatal 异常、Jacobian 缩扰动、trial budget、失败封锁和原子 reinit 是 ORVD 新策略；
6. 采用官方 Fortran 的 Jacobian fresh/stale 控制流和“允许第 `NMAX` 次、准备下一次才失败”的
   比较边界；ORVD trial 的增量时点按第 4.2 节覆盖所有放弃路径，不继承 C++ donor 的 `>=` 漂移；
7. 产品只保留首版能力分支，不搬用户输出、DAE、带状和其他排除代码。

## 12. 后续验收约束

INT-03/04 至少覆盖固定步五阶趋势、常导数、非自治多项式、线性衰减、振子、刚性标量、
Prothero–Robinson、Robertson、Van der Pol、误差拒步、Newton 失败、奇异分解、recoverable/fatal
RHS、缩扰动耗尽、稠密两端、极小 stop、统计和双实例交错。INT-05 再运行全部后端中立合同。
进入 INT-07 前还必须定向覆盖“已有成功步历史后的误差拒试不应用 Gustafsson factor”。
Newton 验收应覆盖第 6 次预测的可达两侧，不为不可达的第 7 次线性修正分支伪造黑盒测试。

精确零 pivot 继续采用官方 DECSOL 策略，不增加经验条件数阈值。生产矩阵形成、实／复分解与求解
由同一条内部实现供数值核心和源码树资格入口复用；资格以三个逐级缩小谱隙、且确实可由实 Jacobian
形成的实块和复块检查相对后向残差，并用解析解及仓外官方 DECSOL 临时 oracle 交叉核对。条件数只
解释前向误差界，不作为产品拒绝阈值，也不用于从车辆物理稳定性推断线性系统是否“近奇异”。

这些测试验证数学性质和已冻结行为，不要求与 Fortran、C++ donor 或 CVODE 逐位相同，也不把一次
参考输出提交成跨平台永久金标。
