# Radau5 派生源码修改记录

本文件随每次派生 C++ 源码修改更新。来源身份和逐文件处置只由
[`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt)维护，完整数值行为由
[`INT_01_RADAU5_SOLVER_CONTRACT.md`](../../docs/planning/integrator_migration/INT_01_RADAU5_SOLVER_CONTRACT.md)
维护。

## 当前状态

INT-03 已落入 `include/orvd/radau5/radau5_core.h` 与 `src/radau5_core.cc`。源码保留官方 RADAU5
三阶段变换常数、误差公式、Newton 收敛预测、配点稠密输出和主要控制器次序；其余工程结构按下列
记录收窄。Fortran oracle 仍只保留 URL，不成为提交或构建输入。

## 搬运前已知的 donor 差异

- `StiffIntegratorT.cpp` 在 Newton 预测不收敛路径把 Fortran 的“新鲜 Jacobian 只重分解、陈旧
  Jacobian 重新计算”反译成了相反条件。INT-03 必须以官方 Fortran 控制流为准并增加针对性测试。
- 官方 Fortran 在 `NSTEP > NMAX` 时退出，C++ donor 使用 `nstep >= nmax`。首版采用合同冻结的
  trial budget 语义，不继承 donor 的边界漂移。
- donor 使用旧式头文件、全局回调、裸指针、控制台输出和一次运行到 `xend` 的 `Integrate()`；这些
  工程结构不属于可保留接口。

## INT-03 已实施处置

- 只派生 `M=I`、正向时间、full-dense、一阶 ODE、三阶段 Radau IIA 和其误差／稠密输出路径；删除
  DOPRI5、质量矩阵／DAE、带状、Hessenberg、二阶特殊结构、用户输出回调和示例程序。
- 用实例独占的 Eigen 向量、矩阵与 `PartialPivLU` 替代 donor 的裸数组和 `dec/sol/decc/solc`；仍只
  分解一个实 `N×N` 系统和一个复 `N×N` 系统，不形成朴素 `3N×3N` 方阵。奇异判据只拒绝精确零或
  非有限 LU 对角，不另引入经验条件数阈值。
- 用借用的 `noexcept` 成功／可恢复／致命状态码回调替代全局函数；核心不认识 ORVD Context，也不让
  C++ 异常越过该边界。ORVD 薄适配器单独保存并按失败类别重抛原始 `exception_ptr`。
- 用 RAII、结构化异常、失败封锁和显式重初始化替代控制台输出、返回整数和进程式 `Integrate()`；
  每次正长度调用内部可拒试，但至多发布第一个成功内步，并在接受端点的下一次 RHS 前返回。
- 调用方容差保持只读；核心保存 Hairer 变换后的私有相对／绝对容差。公开端点、稠密系数和下一步
  建议均在有限性检查后一次提交。绝对容差严格先形成并检查 `atol/rtol`，再乘变换后的相对容差，
  不以代数重排掩盖合同要求观察的中间上溢或下溢。
- 按官方 Fortran 语义实现 Jacobian fresh/stale 拒试路径：fresh Jacobian 只按新步长重分解，stale
  Jacobian 在失败后重算；未继承 donor 已知的 `CALJAC` 反向条件。聚焦测试分别锁定两条路径的
  Jacobian 与 setup 计数。
- trial budget 允许第 `100000` 次尝试，只有准备第 `100001` 次才失败；未继承 donor 的 `>=` 边界。
- 稠密输出另存接受 trial 使用的原始 `h`，内部点不从浮点端点差重新推导步长；步首／步尾仍直接
  复制保存状态。
- 所有公开统计计数使用溢出受检增量，Newton 计数在实际线性求解开始前记录，因非有限求解而失败的
  已发生工作也不会漏计；聚焦测试直接构造非有限 Newton 修正并验证该失败时相的计数与端点原子性。
- 增加测试专用的建议步长夹持，以及可缩小 trial budget 的边界测试入口；前者隔离固定步五阶测量，
  后者以小预算验证“允许第 N 次、拒绝第 N+1 次”。产品适配器不调用这两个入口，构造和成功重初始
  化后的建议步长仍固定为 `1e-6` 秒，产品 trial budget 仍固定为 `100000`。

## INT-07 前数值勘误

- 把常规误差控制器与 Gustafsson predictive controller 分成两个内部路径。误差拒步只使用常规
  `hnew`；predictive quotient 只在 `err < 1` 的接受路径、且已有一次成功步历史时参与下一建议步长。
  此处修正了首版把 predictive quotient 也误用于已有成功步后的 error rejection 的偏差。新增确定性
  分段四次时间 RHS 回归，以解析构造 `err=2` 的拒试，锁定常规控制器重试步长；同一回归在旧路径下
  会得到约 `0.020125` 秒而不是应有的约 `0.070951` 秒。
- 复核官方默认 `NIT=7` 边界：第 6 次修正的预测指数为零，故在正常有限算术下，该次预测与紧随其后
  的实际收敛判定形成同一 accept-or-reject 边界，第 7 次修正和迭代上限守卫不能通过稳定的黑盒 RHS
  用例到达。实现继续逐字保留官方 `1 < NEWT < NIT` 控制流，并把同一生产判定抽成 source-tree-only
  纯函数，以确定性测试锁定第 6 次 `dyth<1` 接受及 `dyth==1` 预测拒绝两侧；没有为了覆盖率而擅自
  改成非官方预测范围。
- 增加 Jacobian 扰动以及第一轮三个配点各自返回 fatal 状态的定位测试，锁定失败分类、已发生 RHS／
  setup 工作计数、端点原子性和稠密输出清除。复矩阵的精确奇异分支仍不以依赖浮点精确抵消的脆弱
  用例强行覆盖；现有实矩阵奇异、累计第五次奇异失败及恢复路径继续承担奇异策略资格。

## 独立正确性审查后的状态机修复

- stop 吸附改为每次正长度调用的首个 trial 一次性裁决。构造、重初始化、大绝对时间和后续内步均
  可把足够接近的候选直接发布到调用方 stop；一旦该 trial 被拒，本次调用内沿控制器缩步，不再次
  吸附。该修复同时消除了 100 Hz 边界前一个 double 尾差导致的伪 `step-too-small`。
- trial 保存窄化的失败因果，区分阶段／误差修正 recoverable、Newton、误差拒绝和真实奇异 setup。
  持续 recoverable 因预算、最小步或不可表示 shift 耗尽时保留原异常类型；后续成功 RHS 会清除旧
  因果，随后独立发生的真实 LU 奇异不会被陈旧异常覆盖。
- 线性建立结果拆成精确奇异、非有限 shift、非有限矩阵和非有限分解。只有精确零 pivot 累计
  singular；步长尺度导致的 `lambda/h` 非有限归入不可表示步长，矩阵／分解溢出使用独立失败原因。
- Newton 与误差 WRMS 改用缩放平方和，避免有限的大归一化分量在直接平方时提前上溢。数值 Jacobian
  扰动在名义正向增量不可表示时依次选择正向、负向 `nextafter`，允许有限有符号分母；正负大状态和
  `DBL_MAX` 的负向回退均有定向覆盖。
- 保留 `atol/rtol` 中间上溢为已冻结能力边界，但构造失败统一为 `std::invalid_argument`，不再错误
  使用只属于正长度数值状态机的 `radau5::Failure`。
- 增加只丢弃 Jacobian 与实／复分解的 source-private 失效通知。系统层每个成功内步更新非空轮轨
  投影提示后调用该通知，避免下一内步在已变化的隐藏 RHS 历史上复用旧线性化；接受态、控制器、
  阶段外推和刚发布的稠密区间保持不变。
- 将生产使用的移位矩阵形成、实／复分解和求解收敛为同一组内部实现，并以源码树专用入口直接调用
  这条实现。三个逐级缩小谱隙、且可由实 Jacobian 形成的实／复块同时检查解析前向误差界和相对后向
  残差；没有把条件数或经验阈值加入产品拒绝逻辑。
- 核心翻译单元拒绝 fast-math、finite-math-only 和不满足 binary64 基本条件的构建；完整资格构建还在
  顶层配置守门，并由唯一编译启动器审计展开后的每条实际编译命令、在通过后注入证明宏，最后由
  runner 工件身份复核同一严格浮点语义。
- 官方 Fortran 仅在仓外临时运行。`N=1/2/4` 单步、刚性 Van der Pol 与 Robertson 的端点／稠密值
  及按合同归一化的工作统计与本核心一致；近奇异实／复移位求解另与官方 DECSOL 临时驱动一致。
  未提交源码、驱动或输出，也未计算或记录 Fortran 摘要。
