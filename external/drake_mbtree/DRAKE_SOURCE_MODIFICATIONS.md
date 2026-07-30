# 对 vendored Drake 源码的修改

本文件记录 `drake/` 下的源码相对上游的**每一处**改动,以及替代上游实现的 ORVD
第一方源码。上游修订与逐文件处置见
[`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt),此处不复述。

不记行号:行号随任何一次上游同步而漂移,漂移后的记录比没有记录更坏。每条记的是
**改了哪个符号、改成什么、为什么**,这些在源码里可以直接搜到。

未列入本文件的差异即为缺陷。**任何对 vendored 源码的修改,必须在同一次提交里更新本
文件**——分两次做,中间那段历史就是一份说谎的记录。

本文件是 vendored 源码**修改说明**的权威,不承载全部分发义务:许可证正文、逐文件处置与
来源事实见 [`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt),第三方与再分发说明见仓库
根的 [`NOTICE`](../../NOTICE)。

## G13 — vendor 刚性 topology 源码

### `multibody/topology/link_joint_graph.h`

**删除** `LinkJointGraph::GenerateGraphvizString()` 与 `LinkJointGraph::MakeGraphvizFiles()`
的完整声明及其说明块,连同随之失去对象的调试 API 注释,以及只为 `MakeGraphvizFiles`
存在的 `#include <filesystem>`。

**原因**:这两个函数的唯一实现在 `multibody/topology/link_joint_graph_debug.cc`,该文件
已裁为 `discard`——它把图写成 `.dot` 并调用 `system()` 去跑 Graphviz 的 `dot`,与刚体
动力学求解无关。留下声明而不留实现,会造出一个"能编译、一调用就缺符号"的半支持 API:
调用方看得见它、写得出调用、直到链接才发现它不存在。删实现就同步删声明。

**未改动**:`SpanningForest::GenerateGraphvizString()` 保留。它是同名但不同类的另一个
函数,其实现在**准入**的 `spanning_forest_debug.cc` 内,该文件因 `spanning_forest.cc`
经 `DRAKE_ASSERT_VOID` 调用 `SanityCheckForest()` 而必须准入。

### `multibody/topology/spanning_forest.h`

**删除**指向已删除的 `LinkJointGraph::GenerateGraphvizString()` 的 `@see` 引用。

**原因**:上一处删除使该引用悬空。文档里指向不存在的东西,读者会去找,找不到,然后
怀疑是自己漏了。

## G15 — vendor 剩余刚性 tree 源码

### `orvd_implementations/double_rotation_and_rigid_transform_composition.cc`(新增,**非上游文件**)

**新增** ORVD 自研的 `ComposeRR` / `ComposeRinvR` / `ComposeXX` / `ComposeXinvX`,满足上游
`math/fast_pose_composition_functions.h` 的声明。上游同名 `.cc` 已裁 `first_party`,**不复制**。

**原因**:上游实现无条件 `#include <hwy/foreach_target.h>` 与 `<hwy/highway.h>`,经 Google
Highway 分派。那是额外的第三方依赖,产品不准入(G12)。抽取上游的 portable 分支再称作
第一方实现,等于复制上游代码却改署名,已明令禁止。因此按数学定义重写:

```
ComposeRR:    R_AC = R_AB R_BC
ComposeRinvR: R_AC = R_BAᵀ R_BC
ComposeXX:    R_AC = R_AB R_BC,  p_AC = p_AB + R_AB p_BC
ComposeXinvX: R_AC = R_BAᵀ R_BC, p_AC = R_BAᵀ (p_BC - p_BA)
```

`ComposeXX` 刻意不做 4×4 齐次矩阵乘:刚体变换的末行在结构上是常量,乘出来是花算力
重算一个已知的行。取转置而非通用求逆,是因为声明的前置条件就是输入为合法旋转,正交矩阵
的转置即其逆。

声明允许输出与任一或两个输入重叠,因此每个结果先算进定长局部量,读完全部输入后**一次**
写回;不依赖 `noalias()`——那是对调用方明确允许的行为作出的保证。不重复校验、不重新
正交化、不做空指针检查:内部调用方在进入热路径前保证旋转合法且输出非空。

### `math/fast_pose_composition_functions.h`

**删除**「以平台 SIMD 指令实现以求速度」「as quickly as possible」等描述,改为说明本实现
由 ORVD 按数学定义提供、不含 SIMD 分派、且不作性能承诺。

**原因**:上游那些描述针对的是被我们拒绝的 Highway 实现。留着它们,读者会以为这里有向量化
分派,而本实现没有。文件名沿用上游。G17 在仓库外把本实现与 Drake 的 Highway 路径现场
比较过一次,结论只用于裁定把位姿组合列入 G47 的专项测量;那次比较的耗时不写进源码,也不
构成任何性能承诺。

### `math/rotation_matrix.h` 与 `math/rigid_transform.h`

**删除或改写**只适用于上游 Highway 实现的 SIMD 与速度描述。保留可由公式直接说明的事实:
轴向旋转专门路径运算更少,逆组合使用旋转转置与刚体变换结构。

**原因**:当前实现没有 SIMD 分派;不把速度承诺留在公共 API 文档中。性能是否需要针对性
处理由 G47 在真实调用比例下判断,不由公共头的描述预先声明。

### `multibody/tree/element_collection.cc`

**删除** `#include "drake/multibody/tree/deformable_body.h"` 与三组
`ElementCollection<*, DeformableBody, DeformableBodyIndex>` 显式实例化。

**原因**:`deformable_body.h` 是禁入类别(FEM + geometry + MultibodyPlant 三条边同在一个
头文件里)。这是准入集合里唯一一条禁入边,删除它使产品边界干净——`forbidden_prefix` 与
逐文件禁入的裁决在此兑现,而不是停留在清单上。

### `multibody/tree/multibody_tree_indexes.h`

**删除** `MultibodyConstraintId`、`DeformableBodyId`、`DeformableBodyIndex` 三个别名及其
说明,连同随之无用的 `#include "drake/common/identifier.h"`。

**原因**:三者都在刚体边界之外——前者属 MultibodyPlant 的约束,后两者属 deformable。它们是
`common/identifier.h` 进入闭包的唯一原因;删掉后该文件改判 `discard` 并从落位树移除。

### `multibody/topology/link_joint_graph_loop_constraint.h`

**删除**引用已删除的 `MultibodyConstraintId` 与 MultibodyPlant 的 TODO 注释。

**原因**:上一处删除使它悬空,且它指向的是产品不准入的 plant 概念。

### `multibody/tree/element_collection.h`、`multibody/tree/multibody_element.h`
与 `multibody/tree/joint_actuator.h`

**删除**文档中对 `DeformableBody` 的引用:前者把它列在 `ElementCollection` 的实例化类型
表里,后者两处以 `DeformableBody::DoDeclareDiscreteStates()` /
`DoDeclareCacheEntries()` 作为用法示例。另从 `MultibodyElement` 删除
`DeformableModel` 的前置声明与 friend,并删除只有禁入 plant 翻译单元提供实现的
`GetParentPlant()` 声明；`JointActuator` 的相关说明改为它实际使用的 parent tree。

**原因**:deformable 与 plant 都不在准入边界内,这些引用与 API 因此悬空。第一条还是
**事实错误**——对应的显式实例化已在 `element_collection.cc` 删除,类型表若不同步就会
说谎。保留 `GetParentPlant()` 会暴露一个可调用声明,但产品中没有也不会提供它的定义。

### `multibody/tree/unit_inertia.cc`

**删除**未被使用的 `#include "drake/common/text_logging.h"`。

**原因**:全文没有一处 `log()` 或 `DRAKE_LOGGER_*` 调用。留着它会把 spdlog 拖进闭包,
而拖进来的理由是一行谁都没在用的 include。

### `multibody/tree/frame.cc`

**删除**未被使用的 `#include "drake/common/identifier.h"`。

**原因**:同上,该文件不构造任何 `Identifier`。

## G01–G15 深度复核收口

### `common/drake_assert.h`

**删除** `ThrowWithValues()` 上无条件的 GNU `__attribute__((noinline, cold))`。

**原因**:该属性只影响异常路径的编译布局，不参与语义；MSVC 不接受 GNU 属性语法，而该
公共头已经进入 topology 的真实构建边界。直接删除比为一个非必要性能提示建立编译器宏
兼容层更轻，也避免把尚未验证的 MSVC 兼容性问题拖到交付阶段。

## G16 — 删除非 double 标量路径

机械且同形的改动按**机制**记(改了什么构造、依据什么等价性),语义或公共边界的改动仍按
**文件与符号**记。两类都可以在源码里直接搜到。

贯穿全部机制的等价性只有一条:**`T` 恒为 `double`**。凡是仅当 `T` 是 AutoDiffXd 或
symbolic::Expression 才会被选中的分支、重载与类型,在这棵树里不可实例化。

### 机制一:标量克隆链(整链删除)

**删除** `MultibodyTree::CloneToScalar()` / `Clone()` / `ToAutoDiffXd()`、六个
`Clone*AndAdd()` 助手、`get_variant()` 的四个 const 重载与 `get_mutable_variant()`、
基类 `Frame` / `Joint` / `Mobilizer` / `ForceElement` / `JointActuator` / `RigidBody` 的
`CloneToScalar()` NVI 与 `DoCloneToScalar()` 纯虚三元组、`Joint::FindMobilizerToScalarClone()`,
以及落位树中每一个派生 Frame / Joint / Mobilizer / ForceElement 类型的 `DoCloneToScalar()`
覆盖与 `TemplatedDoCloneToScalar()` 助手。同时删除
只为跨标量访问而存在的 `template <typename U> friend class X;`(RotationMatrix、
RigidTransform、RotationalInertia、ArticulatedBodyInertia、RigidBodyFrame、
LinearBushingRollPitchYaw、PrismaticSpring、RevoluteSpring、九个 Joint、MultibodyTree、
JointActuator),以及 `JointActuator` 那个只被 `DoCloneToScalar()` 调用的私有克隆构造函数。
删除 `Joint` 中只为跨标量克隆开放的友元,以及 `ElementCollection::ResizeToMatch()` 和仅被
它调用的 `AppendNull()`。

**原因**:整条链的存在理由是把一棵树复制成**另一个标量**的树。`Clone()` 与 `ToAutoDiffXd()`
在落位活动源码中零调用方,`Clone*AndAdd()` 只被 `CloneToScalar()` 自己调用,`get_variant()`
只在 `TemplatedDoCloneToScalar()` 体内出现——是一条自封闭的死代码链,留下任何一段都会让
调用方看得见一个链接期才缺符号的 API。

**未删** `ShallowClone()` / `DoShallowClone()`:它是**同标量**的浅克隆,与标量转换无关。
改动前后其非注释声明、定义与调用集合逐行相同。

### 机制二:符号随机状态接口(整链删除)

**删除** `MobilizerImpl::random_state_distribution_`(类型
`std::optional<Vector<symbolic::Expression, kNx>>`)及其
`set_random_position_distribution()` / `set_random_velocity_distribution()` /
`get_random_state_distribution()`;`Mobilizer::set_random_state()` 纯虚与
`MobilizerImpl::set_random_state()` 实现;`QuaternionFloatingMobilizer` 与
`RpyFloatingMobilizer` 的四个分布 setter;九个 Joint 的分布 setter;
`MultibodyTree::SetRandomState()` 与三个 `SetFreeBodyRandom*DistributionOrThrow()`。

**原因**:这条链的根是一个 symbolic 表达式向量,每一层 setter 的形参都是
`symbolic::Expression`,而 `set_random_state()` 的全部工作就是求值那个表达式。删掉表达式
之后 `set_random_state()` 退化为 `set_default_state()` 的别名,属于明令禁止的兼容别名。

**连带出界**:`math/random_rotation.h`、`math/quaternion.h`、`common/random.h` 因此失去
全部消费方,改判 `discard`;`quaternion_floating_mobilizer.cc` 中一条不使用其任何符号的
`math/quaternion.h` include 一并删除。

### 机制三:标量断言与分支(取 double 侧)

**折叠** 16 处 `if constexpr (scalar_predicate<T>::is_bool)`(保留真分支,删除 else)、
3 处 `if (scalar_predicate<T>::is_bool && …)` 的空守卫、4 对 `std::enable_if_t` 重载
(保留 `is_bool` 侧并去掉 SFINAE,删除 `!is_bool` 侧):`RollPitchYaw::ThrowIfNotValid`、
`RotationalInertia::ThrowIfMultiplyByNegativeScalar` /
`ThrowIfDivideByZeroOrNegativeScalar`、`ArticulatedBodyInertia::IsPhysicallyValid`、
`RotationMatrix::RotationMatrixToUnnormalizedQuaternion`。

**替换** 44 处 `boolean<T>` 为 `bool`,7 处 `if_then_else(c, a, b)` 为 `c ? a : b`
(`roll_pitch_yaw.cc` 中折角归一的四处写成 `if`,读起来更直接),
`SpatialInertia::IsNaN` / `IsZero` 中的 `any_of` / `all_of` 写成 Eigen 自己的
`.array().isNaN().any()` 与 `(.array() == 0.0).all()`。

**原因**:`scalar_predicate<T>::is_bool` 问的是"比较两个 T 是否得到普通 `bool`",对
`double` 恒真;`boolean<double>` 就是 `bool`;无分支的 `if_then_else` 之所以存在,是为了让
symbolic 表达式避开控制流,对 `double` 它就是条件运算符。保留这层只会让读者以为还有第二个
标量。**连带出界**:`common/drake_bool.h`、`common/cond.h`、`common/double_overloads.h`。

**注意**:`RotationMatrix::RotationMatrixToUnnormalizedQuaternion` 的两个特化在上游即注明
"两处数学必须保持同步"。保留的是 if-elseif 版本;被删的 `if_then_else` 版本按上游自己的说明
与之等价,不引入数值差异。

**折叠** `RotationMatrix` 与 `RigidTransform` 位姿组合运算中的六个
`if constexpr (std::is_same_v<T, double>)` 分支,无条件调用 G15 的 `ComposeRR()` /
`ComposeRinvR()` / `ComposeXX()` / `ComposeXinvX()`。原 else 分支是明确的非 double
执行路径,保留它与本 Goal 的边界相冲突。

### 机制四:标量转换与提取(改写为直接表达式)

**删除** `RotationMatrix` / `RigidTransform` / `RotationalInertia` / `UnitInertia` /
`ArticulatedBodyInertia` / `SpatialInertia` 的 `cast<Scalar>()` 成员模板,以及
`PiecewiseConstantCurvatureTrajectory` 的标量转换构造函数、`ScalarValueConverter` 别名与
`ScalarConvertStdVector()` 助手(连同 `systems/framework/scalar_conversion_traits.h` 的
include——该头不在准入边界内)。

**改写** 13 处 `ExtractDoubleOrThrow(x)` 为 `x`、2 处 `DiscardGradient(v)` 为 `v`、16 处
落在 double 上的恒等 `cast<T>()` 为对象本身。

**原因**:`cast<U>()` 的约束 `requires is_default_scalar<U>` 直接引用已裁的
`common/default_scalars.h`;`ExtractDoubleOrThrow` 对 `double` 实参原样返回;
`DiscardGradient` 在没有梯度时同理。路书完成门点名不得再消费标量转换与 `ExtractDouble`。
**连带出界**:`common/extract_double.h`。

### 机制五:默认标量实例化与文档

**删除** 58 行 `#include "drake/common/default_scalars.h"`、4 行
`#include "drake/common/autodiff.h"`、1 行 `#include "drake/common/symbolic/expression.h"`
(三个头都不在落位树内,这些 include 是 G11 编译探针报出 55 处失败的直接原因),
60 个 `@tparam_default_scalar` 文档标签,以及 `element_collection.cc`、`mobilizer_impl.cc`、
`body_node_impl.cc`、`body_node_impl_mass_matrix.cc`、`prismatic_mobilizer.{h,cc}`、
`revolute_mobilizer.{h,cc}` 中 AutoDiffXd 与 symbolic 的显式实例化。

**改写**三十余处描述非 double 标量行为的说明(诸如"当 T 为 symbolic 时不做校验"、
"cast 到 AutoDiffXd 合法而反向不合法"),它们在这棵树里已经是假陈述。

### 机制六:`M_PI` 改用 `std::numbers`

**替换** 7 个文件中的 34 处 `M_PI` 与 6 处 `M_PI_2` 为 `std::numbers::pi` 与
`std::numbers::pi / 2`,各文件加 `#include <numbers>`;`common/fmt_eigen.h` 的文档示例同步。

**原因**:`M_PI` 不是标准 C 或 C++ 的一部分,MSVC 需要 `_USE_MATH_DEFINES` 才提供。
C++23 的 `std::numbers` 是标准途径,不需要为编译器差异建宏兼容层。

### 逐文件记:`math/linear_solve.h`(整档重写)

**重写**为只保留 `LinearSolver` 类与其求解器类型别名 `internal::EigenLinearSolver`。
**删除** `is_symbolic` / `is_symbolic_v` / `is_double_or_symbolic_v` / `is_autodiff` /
`is_autodiff_v`、五个自由函数 `SolveLinearSystem` 重载、`GetLinearSolver()`、
`Promoted` / `Solution` 别名、Eigen 5 的 `ColPivHouseholderQR` / `PartialPivLU` 兼容别名,
以及 `LinearSolver` 内保存原矩阵的 `A_` 成员和 `Solve()` 里的三路 `if constexpr` 分派。

**原因**:该文件的绝大部分是隐函数定理路径——当 A 或 b 含 AutoDiffScalar 时,用 double 分解
配合 ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ − ∂A/∂zᵢ·x) 求梯度。没有 AutoDiff 就没有这条路径。落位活动源码
只经 `body_node.cc` 与 `body_node_impl.cc` 使用 `LinearSolver<Eigen::LLT, MatrixUpTo6<T>>` 的
默认构造、矩阵构造、`eigen_linear_solver()` 与 `Solve()`;其余全部无消费方。删除后
`Solve()` 直接返回 `eigen_linear_solver_.solve(b)`,与原先 double 分支逐字相同。
它原先 include 的 `math/autodiff.h` 与 `math/autodiff_gradient.h` 都不在落位树内,
改为直接 include `<Eigen/Dense>` 使该头自足。

### 逐文件记:其余语义改动

- **`multibody/tree/curvilinear_mobilizer.h`**:删除 `math/wrap_to.h` 的 include。
  该 include 自 G15 起即无用,处置账本已记明可删。
- **`math/wrap_to.h`**:删除 `common/double_overloads.h` 的 include,其唯一用途是一段被
  注释掉的替代实现;该注释同步改写,不再声称 fmod 受标量类型限制。
- **`common/trajectories/piecewise_trajectory.{h,cc}`**:删除
  `PiecewiseTrajectory<T>::RandomSegmentTimes()` 及其 `#include <random>` 与
  `using std::uniform_real_distribution;`。它在落位树中零调用方,属完成门点名的"无人消费
  helper"。
- **`common/eigen_types.h`**:删除指向未落位的 `eigen_autodiff_types.h` 的 `@see`。
- **`common/drake_assert.h`**:删除 `ConditionTraits` 说明中指向未落位的
  `common/symbolic/expression/formula.h` 的举例。
- **`common/nice_type_name.cc`**:删除把 Eigen AutoDiff 类型名改写成 `drake::AutoDiffXd`
  的两条正则。产品里不存在这些类型,规则永不命中。
- **`common/hash.h`**:删除以 `symbolic::Expression` 为例的 `std::hash` 特化说明。
- **`common/drake_deprecated.h`**:删除。弃用流接口的声明与定义已同步移除,落位源码不再
  消费 `DRAKE_DEPRECATED`;对应头文件、死 include 与已完成事项的 TODO 不保留。
- **`multibody/tree/element_collection.{h,cc}`**:删除只服务跨标量树克隆的
  `ResizeToMatch()` 与其私有依赖 `AppendNull()`。
- **`multibody/tree/frame.cc` / `multibody_tree.{h,cc}` / `unit_inertia.h`**:删除标量
  裁剪后已失真的实例化、variant、克隆与转换说明。

### G16 未声称的事

本 Goal 的产物是**边界**,不是可构建的树。以钉死的 g++ 与 C++23 对落位源码逐 TU 编译,
能脱离运行时的翻译单元产出对象;其余翻译单元的首个错误均是缺少
`drake/multibody/tree/multibody_tree_system.h`,即 G20–G28 要处理的 systems 运行时依赖。
标量相关的错误一个不剩。该次编译是一次性探针,结论吸收后即删,不留归档。

## G18 — 分发义务

### `common/nice_type_name.cc`

**新增**位于原 Stanford / Apache-2.0 声明**正下方**的一段改动声明:

```
/* This file has been modified by the OpenRailVehicleDynamics project.
   See external/drake_mbtree/DRAKE_SOURCE_MODIFICATIONS.md for details. */
```

**原因**:该文件带 Apache-2.0 条款,而 G16 修改了它(删除了把 Eigen AutoDiff 类型名改写成
`drake::AutoDiffXd` 的两条正则)。Apache-2.0 第 4(b) 条要求"被修改的文件须携带显著声明,
说明你改动了它"。这与本文件末尾"没有添加逐文件版权头"那条不冲突:那条禁止的是给**本无
声明**的文件凭空发明版权头(误述来源);这里是一个**已经带着 Apache-2.0** 的文件按其自身
条款欠下的声明。原声明一字未改,也没有添加任何 ORVD 版权归属。

`common/copyable_unique_ptr.h` 同为 Apache-2.0 但与上游逐字节相同,因此不欠这项声明。
BSD-3-Clause 没有对应条款,故另外 133 个被修改的文件在文件内无须任何声明,其义务由随仓库
携带的 `LICENSE.TXT` 承担。

这项义务由 `tools/drake_source_boundary/verify_landed_drake_source_provenance.py` 直接
对固定 Git 对象核验；克隆工作区的未提交改动不会改变结论，也不靠人记住。

## 未作的修改

- **没有改写任何 `#include "drake/..."` 前缀。** 源码按上游相对路径落位在 `drake/` 下,
  include 行一字未动。改前缀是对每个文件的机械编辑,且每次上游同步都要重做一遍。
- **没有添加逐文件版权头。** Drake 的 tree/topology 候选文件本就没有;人工发明是
  误述来源,不是保全来源。支撑源码原有的 Stanford/Apache-2.0 声明原样保留。
- **没有改写 vendored Drake 的刚体算法主体。** 边界/API/include/说明的裁剪均逐项列在
  上文；四个位姿组合函数是明确分列的 ORVD 第一方实现。
