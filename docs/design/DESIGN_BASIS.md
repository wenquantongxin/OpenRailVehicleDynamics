# OpenRailVehicleDynamics：零-Drake 依赖重构方案 — 对抗性审查请求

> 交付对象：CodeX
> 请求类型：**对抗性审查（adversarial review）**，不是征求同意
> 撰写者：Claude（基于对 `wheel-rail-lab` 与 Drake v1.54.0 源码的只读静态勘察）

---

## 0. 给审查者的任务定义

请对下述"目标—观察—落地方案"三段做**对抗性审查**。评审规范沿用本项目既有纪律：

1. **承重数字必须第一手复算**。本文所有数字都标注了出处，但请**不要引用本文的数字**，请自己重新测。凡是你复算不出来的，直接判"不可复现"。
2. **禁止美化**。不要为了让方案成立而放宽口径。
3. **只读勘察**。审查阶段不要修改 `wheel-rail-lab` 的任何文件（该仓库有大量有效的未提交改动，正在被并行修改）。
4. **明确区分"我不同意"与"我无法复现"**。前者是判断分歧，后者是事实争议，请分开陈述。
5. 若你发现本文的**结论方向**是对的但**理由**是错的，请照样指出——理由错会在后续实施中产生错误的设计决策。

**输出请按此结构**：
- A. 事实层：本文哪些陈述你复算后**不成立**（附你的复算方法与结果）
- B. 判断层：哪些结论你**不同意**（附替代方案与代价）
- C. 遗漏层：本文**没看到**但会影响方案成立的东西
- D. 风险层：若按本方案实施，最可能在第几阶段以什么方式失败
- E. 裁决：方案可否作为实施基线；若可，需要先改哪几处；若否，替代路线是什么

**第 7 节列出了我自认最脆弱的 8 个点，请优先攻击那里**——但不要被它限制，第 7 节之外的漏洞更有价值。

---

## 1. 项目目标与硬约束

### 1.1 目标

在 `/home/yaoyao/Documents/myProjects/OpenRailVehicleDynamics/`（当前为空）建立一套**对 Drake 零运行期依赖**的 C++ 轨道车辆动力学求解软件，替换 `wheel-rail-lab` 中 GZ18 刚性轮对的仿真核心。

### 1.2 硬约束（均由项目负责人明确指定，不是我的推断）

| # | 约束 | 说明 |
|---|---|---|
| C1 | **零 Drake 运行期依赖** | 最终产物不链接 `libdrake.so` |
| C2 | **跨平台优先级高于自包含** | Eigen / SUNDIALS CVODE / Ceres 允许作为外置第三方按需调用，不强制 vendor |
| C3 | **保真基准是 Drake，不是 SIMPACK** | 与 SIMPACK 的对齐是并行的、由 CodeX 负责的另一条线。本项目的验收基准是"复现 Drake 当前行为"。**本项目只做当下 Drake 线的自包含化，不承担任何 SIMPACK 复现/对齐工作**；文中凡引用 SIMPACK 对齐记录，仅作背景事实 |
| C4 | **入口车型只做 GZ18 刚性轮对** | IRW 的抽象层是仓库自写代码、非 Drake 主导，暂不纳入 |
| C5 | **积分器保留 CVODE + 数值 Jacobian** | 不复现 AutoDiffXd |
| C6 | **保留 LeafSystem 等价的整车组装层** | 理由：便于不同车型的模板化实现，以及与 CVODE 接口的便捷连接。**不采用"丢掉框架、直写单体 RHS 主循环"的方案** |
| C7 | `rigid_wheelset_linearize_upper_vehicle` **排除在范围外** | 实验性、无下游消费者；且它算的是"刨去车体的系统线性化"，非线性临界速度，不必用 Drake 原生实现 |

### 1.3 引用基准（已锁定）

- 源仓库：`/home/yaoyao/Documents/myProjects/wheel-rail-lab`，HEAD `9355d17`（**只读引用**）
- Drake 安装：`/opt/drake`，版本 1.54.0
- Drake 源码：上游仓库、固定 commit 与 tag 的唯一权威是 `external/drake_mbtree/SOURCE_DISPOSITION.txt`；本文件不复述这些值，克隆位置按机器提供、不写入仓库

---

## 2. 已核实的观察 — Drake 侧

> 标注：`[实测-C]` = 我本人第一手测得；`[实测-W]` = 多 agent 工作流实测并经独立对抗验证。**两者都请你重测。**

### 2.1 结构性发现：RHS 不在 MultibodyPlant 里，在 MultibodyTree 里 `[实测-W]`

所有前向动力学例程都定义在 `multibody/tree/multibody_tree.cc`，而非 plant：

| 例程 | 位置 |
|---|---|
| `CalcPositionKinematicsCache` | multibody_tree.cc:1364 |
| `CalcVelocityKinematicsCache` | multibody_tree.cc:1462 |
| `CalcInverseDynamics` | multibody_tree.cc:1800, 1818 |
| `CalcForceElementsContribution` | multibody_tree.cc:1916 |
| `CalcMassMatrixViaInverseDynamics` / `CalcMassMatrix` | multibody_tree.cc:2057 / 2088 |
| `CalcArticulatedBodyAccelerations` | multibody_tree.cc:3912, 3942 |
| `DoCalcTimeDerivatives` | multibody_tree_system.cc:376 |

### 2.2 几何耦合是 plant 层的产物，物理层与几何无关 `[实测-C]`

我用传递 include 闭包实测（脚本对 `/opt/drake/include` 递归展开）：

```
multibody_tree.h  闭包 = 131 头，其中 geometry 头 =  0
multibody_plant.h 闭包 = 240 头，其中 geometry 头 = 42
multibody_tree.h  闭包中 symbolic 相关头 = 12
```

**含义（REV2 修正）**：上表只对**声明头闭包**成立。第二轮实测**实现闭包**（tree+topology 全部 63 个非测试编译单元）= **283 头，含 geometry 46、FEM 30、plant 1**（`constraint_specs.h`，header-only）。直接肇事文件仅两个——`deformable_body.cc` 与 `geometry_spatial_inertia.cc`——外加 `element_collection.cc` 对 `DeformableBody` 的三标量实例化（:184-187）。因此"物理头文件层无几何依赖"成立，但**"原样 vendor 两个目录"不成立**：Stage 1 必须做 **GZ18 刚体源码允许清单**（排除上述文件、修补 element_collection、携带 constraint_specs.h），手术面小但不可省略。

> ⚠️ 这一条推翻了我早期的判断（"几何与物理焊死、躲不掉"）。请重点复验闭包脚本的正确性——如果我的闭包算法有 bug（例如漏跟 `.cc` 里的 include、或把条件编译分支算错），整个路线的前提就动摇。

### 2.3 连续路径无优化求解器 `[实测-W]`

`grep -rln MathematicalProgram multibody/tree/ multibody/plant/` 返回**零文件**。`solvers::` 仅出现在离散接触驱动（sap_driver / tamsi_driver / compliant_contact_manager / discrete_update_manager / deformable_driver / contact_jacobians）。**连续 RHS 是纯 O(n) Featherstone ABA，无优化。**

### 2.4 抽取的真实阻断点：Context 耦合 `[实测-W]`

`MultibodyTreeSystem<T>` **本身就是** `systems::LeafSystem`（multibody_tree_system.h:72），且每个树数学函数的第一个参数都是 `systems::Context<T>`——仅 multibody_tree.h 内就有 109 处 `systems::Context<T>`，mobilizer.h 30 处，body_node.h 4 处。Context 既是唯一的状态容器，又是记忆化入口。

**这是最重的真阻断点，严重度评级 hard——REV2 已证其为双向耦合**：树还持有 `tree_system_` 反向指针并经它求值缓存（multibody_tree.h:2211-2215 `tree_system_->EvalFrameBodyPoses(context)`），不是替换缓存声明即可切断的单向依赖。其余耦合评级：
- MultibodyPlant → geometry：**clean**（见 2.2）
- default_scalars 三标量实例化：**moderate**（`libdrake.so` 导出 1299 个含 `MultibodyTree<` 的符号：449 double / 520 AutoDiff / 426 symbolic，80.8 MB）
- Value/AbstractValue 类型擦除：**clean**（是 Context 耦合的搭车项，非独立阻断）
- solvers/MathematicalProgram：**无耦合**（clean negative）
- `MultibodyPlant` 声明为 `final`（multibody_plant.h:970）：**moderate**——无法子类化渐进替换，没有增量接缝

### 2.5 规模实测 `[实测-W，我复验了 tree/topology 两项]`

| 模块 | 原始 LOC | 备注 |
|---|---:|---|
| `multibody/tree` | 40,710 | 核心；119 文件 |
| `multibody/topology` | 5,654 | **拥有深度优先 mobod 重编号与 q/v 分配**，第一轮遗漏了它 |
| `multibody/math` | 1,471 | 仅 473 NCLOC，注释占 68%；100% 需要，零死重 |
| `math`（位姿核心） | 5,448 | 17 文件合计：五大件 rotation_matrix 1716 / rigid_transform 1102 / roll_pitch_yaw 1083 / fast_pose_composition 855 / quaternion 415（计 5,171）+ 12 个小文件（unit_vector、cross_product、normalize_vector、convert_time_derivative 等约 277 行） |
| `multibody/plant` | 28,447 | **58% 可丢**；真正依赖的注入机制极小：`externally_applied_spatial_force.h` 仅 25 行 |
| `systems/framework` | 28,747 | 需要子集 24,436（REV2：无文件允许清单，降为参考值） |
| `systems/analysis` | 14,736 | 严格子集 7,310（REV2：同上，降为参考值） |
| `common/symbolic` | 10,738 | 由 DEFAULT_SCALARS 强制传递引入，可干净切除 |
| **主动放弃的模块合计** | **187,191** | geometry 87,610 / solvers 43,147 / contact_solvers 22,932 / parsing 17,143 / primitives 9,473 / fem 6,886 |

### 2.6 许可证 `[实测-C]`

`drake-src/LICENSE.TXT` 首行："All components of Drake are licensed under the BSD 3-Clause License"。**宽松、非传染，vendoring 合法。**

### 2.7 缓存依赖追踪器的真实粒度 `[实测-C]`

读 `multibody_tree_system.cc:199+` 的 **15 处** `DeclareCacheEntry`（`CacheIndexes` 同为 15 字段，multibody_tree_system.h:556；初稿误记 20，REV2 已复数），粒度确为 q/v 分离：

| 缓存项 | 前置 ticket |
|---|---|
| `PositionKinematicsCache` | **{q, 参数}** |
| `ArticulatedBodyInertiaCache` | **{q, 参数}**（内含 ABA 的 LLT 分解） |
| `H_PB_W` / `M_B_W` / 复合体惯量 / `BlockSystemJacobian` | {位置缓存} |
| `VelocityKinematicsCache` | {q, v, 参数} |
| `Fb_Bo_W(q,v)` | {M_B_W 缓存, 速度缓存} |

---

## 3. 已核实的观察 — 仓库侧

### 3.1 GZ18 拓扑与状态布局 `[实测-W]`

7 × `QuaternionFloatingJoint` + 8 × `RevoluteJoint` + 2 × `WeldJoint`
→ **nq = 57，nv = 50，基础 N = 107，用户刚体 17 个（车体 1 + 构架 2 + DUM 2 + 轮对 4 + 轴箱载体 8×循环添加；`num_bodies()=18` 含 world），作动器 0 个**（初稿"刚体 16 个"有误，REV2 按构建器逐行复数）
（依据：cvode_driver.cc:63-75 的注释与 `DRAKE_DEMAND(N >= 107)`；构建器 gz18_vehicle_builder.cc:63-153。注意守卫是 **N ≥ 107** 而非等于：注释明言 "RHS state-eval performance core may append continuous Maxwell force states"——**自定义连续状态并入 CVODE 向量的先例已存在**，这正是 mini-LeafSystem 层要泛化的模式）

状态布局：`x = [q 块 ; v 块]`；每个浮动关节 `q=[qw,qx,qy,qz,px,py,pz]`、`v=[wx,wy,wz,vx,vy,vz]`（**角速度在前**）。**REV2 关键修正**：深度优先分配使轮对浮动关节与其轴箱转动关节**交错**（carbody 0/0、frame_front 7/6、frame_rear 14/12、wheelset_ff 21/18、rev_ff 28,29/24,25、wheelset_fr 30/26……），而 `BuildAbsTolVector` 假定"7 浮动排完再排 8 转动"——**当前即错位：语义 43/107 项、按现默认值实际 30/107 项（q18+v12，本方独立复算与 CodeX 一致）**。初稿"代码从不硬编码全局 DOF 下标"不成立（该函数即反例）；GZ18 路径亦无 `MapQDotToVelocity` 调用（全仓唯一命中在范围外的 IRW 回放 contact_replay_from_afs.cc:1057），q̇↔v 映射由 Drake 树在 RHS 内部完成。

### 3.2 接触已完全外置 `[实测-C]`

接触力在 `rwc_core`（该库 CMake 只链 `Eigen3::Eigen` 与 `OpenMP`，**不链 drake**）计算，经 `ExternallyAppliedSpatialForce` 通过 `get_applied_spatial_force_input_port` 注入。**Drake 的碰撞检测、SceneGraph、hydroelastic 在物理路径上零参与**；SceneGraph/Meshcat 仅可视化。

### 3.3 需要复现的 Drake 接口面 `[实测-C，但我早期低估过]`

多体数学部分（约 20 个方法）：建模期 `AddRigidBody/AddJoint/AddFrame/AddForceElement/Finalize`；状态 `Set/GetPositionsAndVelocities`、`SetFreeBodyPose`、`SetFreeBodySpatialVelocity`、`Set/GetPositions`、`Set/GetVelocities`；运动学 `EvalBodyPoseInWorld`、`EvalBodySpatialVelocityInWorld`、`EvalBodySpatialAccelerationInWorld`、`CalcJacobianSpatialVelocity`；动力学 `CalcTimeDerivatives`、`CalcInverseDynamics`、`CalcForceElementsContribution`。

**外加端口面（我早期漏掉，是实质工作量）**：`get_state_output_port`（25 处）、`get_applied_spatial_force_input_port`（9 处）、`get_actuation_input_port`（6 处）、`get_reaction_forces_output_port`（1 处）。我按放宽模式计 32 个唯一名字，工作流按更宽模式计 56 个——**这个计数分歧本身请你裁决**。

### 3.4 积分层已自建，且唯一 Drake 触点只有一行 `[实测-C]`

`rigid_wheelset/src/cvode_driver.cc`（1552 行）直接用 SUNDIALS C API。RHS：

```cpp
int CvodeRhsFn(sunrealtype t, N_Vector y, N_Vector ydot, void* user_data) {
    // 1. Write time + continuous state (auto-invalidates Drake caches)
    ctx.SetTime(t);
    ctx.get_mutable_continuous_state().SetFromVector(...);      // :45-48
    d->system->CalcTimeDerivatives(ctx, d->derivatives.get());  // :51  ← 唯一 Drake 触点
    ...
}
```

线性求解器为 `SUNLinSol_Dense`（:410-411），**全仓无 `CVodeSetJacFn`**，即用 CVODE 内部差商 Jacobian。

**但**：GZ18 路径**同时**在用 Drake `Simulator::AdvanceTo` 与 `ResetIntegratorFromFlags`（main.cc:518；simulation_runner.cc:935, 1105）。**是否砍掉这条路径只留 CVODE，是一个尚未决定的范围开关**（见 5.1）。

### 3.5 LeafSystem 层要托住的特性清单 `[实测-C]`

跨 `drake_sim/src` + `rigid_wheelset/src` + `control_core` 的 `Declare*` 统计：

| 类别 | 特性 × 次数 |
|---|---|
| 端口 | VectorOutput 21、VectorInput 14、AbstractOutput 11、AbstractInput 4 |
| 状态 | Discrete 15、Abstract 5、**Continuous 3** |
| 事件 | PeriodicUnrestrictedUpdate 4、InitializationUnrestrictedUpdate 4、PeriodicPublish 2 |
| 其他 | CacheEntry 4、NumericParameter 3 |

最重的两个：`wheel_rail_contact_system`（6 离散 + 2 抽象 + 4 缓存 + 3 参数 + 周期更新）与 `motor_pid_control_system`（7 离散 + 1 连续 + 周期更新）。**Continuous 状态只有 3 处**，故展平进 CVODE `N_Vector` 的工作量很小。
**GZ18 范围提示**：此表是跨车型模板层的**超集**设计目标；其中 `motor_pid_control_system`、`motor_bridge_proxy`、`torque_applier_system` 属 IRW 线（GZ18 无作动器），GZ18 首版只需实现 GZ18 实际使用的子集，接口按超集设计、实现按需。

### 3.6 力元现状好于预期 `[实测-W]`

- live 路径上**唯一**的 Drake 力元是 `UniformGravityFieldElement`（一个 9.81 常数）
- Drake 的 `LinearBushingRollPitchYaw` **只在 Legacy 装配路径**被实例化：gz18_force_elements.cc:28 的 `AddBushing` 仅被 `AddGz18ForceElementsLegacy`（:216）使用，**该函数无任何非测试调用方**（已 grep 全仓核实）；irw_force_elements.cc:480-483 同理。live GZ18 路径是 simulation_runner.cc:438-440/644-646 → `BuildActiveSuspensionConfigs` → `AddForceElement<SpecializedSuspensionForceElement>`（仓库自有 ForceElement 子类，数学在仓库内）
- 运行期全部弹性/耗散力**已在仓库代码内**：`SpecializedSuspensionForceElement` 自行重导了整套 bushing 闭式（半角坐标系见 specialized_suspension.cc:71-81）

### 3.7 autodiff 与 Ceres 的真实占用 `[实测-C]`

- `AutoDiffXd` 全仓仅两处：① `specialized_suspension.h:86-137` 的 `DoCloneToScalar` **样板**（Drake 强制的三标量转换，无任何自动微分运算）；② `linearize_upper_vehicle.cc`（**已按 C7 排除**）。→ **运行期零 autodiff**
- `Ceres` 仅用于 `static_equilibrium.cc`（`DynamicNumericDiffCostFunction<..., CENTRAL>` + `DENSE_QR`），**数值差分而非自动微分**

### 3.8 接触求解次数是物理可观测量 ⚠️ `[实测-C]`

`wheel_rail_contact_system.cc:216-245` 存在 warm-start 提示：`driver_cache_s_hint_`、`driver_cache_s_dot_hint_`、`driver_cache_dt_hint_`，另有 `kNumericParameter` 提示模式以 `numeric_parameter_ticket` 为前置。

**含义**：接触求解**被执行了多少次、按什么顺序**会通过 warm-start 序列影响数值结果。因此"复现 Drake"必须复现**求值次数与顺序**，而不只是数学公式。

---

## 4. 我的路线判断与推荐

### 4.1 两条候选

**方案 A — 全自研重写**：自己写 Featherstone 核 + 空间数学 + 力元 + mini-systems 层。
**方案 B — 分期：先 vendor MultibodyTree，再逐 pass 替换**（**我推荐**）：
- **Stage 1（REV2 修订）**：按 **GZ18 刚体源码允许清单**把 `multibody/tree` + `multibody/topology` 以 double 标量 vendor 进新仓库（排除 `deformable_body.cc`/`geometry_spatial_inertia.cc`、修补 `element_collection.cc` 的 DeformableBody 实例化、携带 header-only 的 `constraint_specs.h`），自写 Context/cache 对象（REV2 估 **4–8 kLOC** 生产代码，初稿 2–4k 为乐观下界）顶掉那 **15 处** `DeclareCacheEntry` 并消解 `tree_system_` 反向指针，外面套自有 facade。→ 达成 C1（零运行期 Drake）；**vendor 提供高可信起点，逐位一致仍须由分层预言机实测证明**（初稿"由构造保证"为过度主张，已撤回）；这份 in-tree 代码同时永久充当 oracle。
- **Stage 2**（可选、可无限期推迟）：在冻结接口后逐个 pass 换自研实现，每次以金标向量门控。

Stage 1 同时**免除** `systems/framework`(28,747) + `multibody_plant`(11,378) + `common/symbolic`(10,738) 三块负担。symbolic 可干净切除的依据：`multibody/tree` 中每一处 symbolic 引用都是可删除的 `DoCloneToScalar<symbolic::Expression>` 重载（例：revolute_mobilizer.cc:205-207）。

### 4.2 工作量预估（GZ18-only、复现 Drake、不含 SIMPACK 再基线）

| 组件 | 方案 A | **方案 B** |
|---|---:|---:|
| MBS 动力学核 | 12–18（Featherstone）+ 5–8（facade） | **8–14**（vendor + 自写 context/cache + facade） |
| 空间数学/惯量 | 5–8 | 0（随 vendor 自带） |
| 力元 | 3–5 | 3–5 |
| 自有 mini-LeafSystem 层（GZ18 + CVODE-only） | 6–10 | 6–10 |
| 静平衡 re-host（Ceres 外置） | 2–3 | 2–3 |
| 构建图 + 跨平台（MSVC/flag 面） | 4–8 | 4–8 |
| Drake 差分 oracle 测试台 | 4–6 | 含在 Stage 1 |
| **合计（人周）** | **41–66** | **20–35** |

Stage 2 全部自研化另计 +20–30 人周，**不阻塞** C1。

> **REV2 工作量基线**：第二轮独立估算（含本表未列的基线冻结/分层预言机 5–8、快照/输出/CLI 8–12、打包 CI 5–8、集成余量 8–13 等工作包）为 **M0–M3 合计 50–80 人周**；与本表重叠的核心包约 24–39 人周，与本表同量级——差额主要来自**范围补全**而非同项重估。**规划以 50–80 人周为准**，本表保留作为分项参考。Stage 2 另计约 25–40 人周；若首版需替代 Meshcat/回放再加 4–8 人周。

> ⚠️ 这些数字来自一次多 agent 评估的对抗验证结果，两轮估算相差 1.7 倍（初判 28–44 人周，对抗验证上修至 45–75 人周全范围）。**请把工作量当作最不可靠的一栏来审。**

### 4.3 缓存依赖追踪器：为什么可以不复现

- 它捆绑了两个功能：**(a) 一次求值内的记忆化**、**(b) 跨求值的选择性失效**。
- **(a) 必须保留**：一次 RHS 内 `PositionKinematicsCache` 被 ≥5 个下游 pass 读，接触结果被 8 个输出端口读。但定拓扑下"算一次读多次"用**静态求值顺序 + 预分配缓冲结构体**即可，不需要依赖图。
- **(b) 在本项目大幅退化但未归零（REV2 修正）**：`CvodeRhsFn` 每次写入时间 + **整个**连续状态（注释自陈 "auto-invalidates Drake caches"），15 项缓存中依赖状态/时间/输入的 **12 项**每次失效；但**仅依赖参数的 3 项**（reflected inertia、joint damping、frame body poses）跨 RHS 调用持续复用——静态调度替代品必须把这 3 项实现为"参数变更时才重算"，不能每步全算。差商 Jacobian 逐分量扰动走同一整体写状态 shim，对 12 项状态缓存同样无选择性收益。
- **速度**：去掉 (b) 中性到略快（省下失效通知传播、每次读取的 out-of-date 检查与 `AbstractValue` 向下转型，且预分配连续缓冲局部性更好）；量级是**个位数百分比的常数因子，不是数量级**。
- **会变慢的具名场景**：若将来加入"只变 v 的解析/结构化 Jacobian"，Drake 的细粒度设计会赢。

---

## 5. 代码落地方案

### 5.1 范围开关（三项均已拍板，审查时作为既定前提）

| 开关 | 选项 | 影响 |
|---|---|---|
| **S1** | Drake `Simulator::AdvanceTo` 路径的去留 | **已由项目负责人拍板**：整车推进循环定义**抽象推进器接口**（须捕获的语义：事件日历/`CalcNextUpdateTime`、步长夹持到事件时刻、x⁻→x⁺ 挂起更新、拒步事务性回滚、Initialize 派发初始化事件），**首版只实现 CVODE 后端**（移植现有 cvode_driver 的迷你调度，其 update→publish→条件 ReInit 顺序见 cvode_driver.cc:1259-1282）；**Drake 积分器族（radau/implicit-euler，实测 5,615 LOC）不实现、仅预留接口**。背景事实（保留供参考）：`SIMPACK-Drake对齐列表.md:85` 记有"宽松 BDF2 的约 26° 相位色散已识别；严格 BDF2/BDF5 历史差约 0.05°；正式 30 秒全程收敛尚未测"，四处配置 `cvode_max_order` 默认 2——该关切属 SIMPACK 对齐线（本项目范围外），保留接口即保留日后补第二积分器族的通道 |
| **S2** | 验收口径 | **已由项目负责人拍板**:同平台同编译选项下对 Drake 逐位一致;跨平台只要求"构建可跑 + 工程容差回归",**不追求(也追求不了)跨系统逐位相等**。金标向量按平台各自生成。审查时请把 S2 当作既定前提,不再作为开放问题 |
| **S3** | `linearize_upper_vehicle` | 已按 C7 排除。确认后 autodiff 需求归零 |

### 5.2 目录结构提案

```
OpenRailVehicleDynamics/
├── third_party/
│   └── drake_mbtree/        # Stage 1: vendored multibody/tree + multibody/topology
│                            #   double-only；删除全部 DoCloneToScalar<AutoDiffXd|Expression>
│                            #   保留 BSD-3 许可证与出处声明
├── core/
│   ├── context/             # 自写 Context/cache（顶掉 15 处 DeclareCacheEntry + 消解 tree_system_ 反向指针，约 4-8 kLOC）
│   └── schedule/            # Finalize 期算出的静态求值顺序 + 预分配缓冲
├── systems/                 # mini-LeafSystem 运行时（3.5 节的 12 项特性）
│   ├── system.h             #   Declare{Continuous,Discrete,Abstract}State / {Vector,Abstract}Port
│   ├── diagram.h            #   固定拓扑组装 + 端口路由 + 状态展平
│   └── events.h             #   Periodic / Initialization 更新事件调度
├── vehicles/
│   └── gz18/                # 整车组装模板（其他车型按同一模板扩展）
├── integrators/
│   ├── advancer.h           # 抽象推进器接口（S1：事件日历/步长夹持/挂起更新/拒步回滚/Initialize 语义）
│   ├── cvode/               # 首版唯一后端：移植 cvode_driver.cc，RHS 改指自有核
│   └── drake_family/        # 预留目录，不实现（radau/implicit-euler 日后按需在接口后补）
├── forces/                  # 力元：重力 + 从仓库搬迁的 Maxwell/anti-hunting/specialized_suspension
├── equilibrium/             # 静平衡（Ceres 外置）
└── parity/                  # Drake 差分 oracle 测试台 + 金标向量
```

### 5.3 里程碑与验收门

| 阶段 | 内容 | 验收门 |
|---|---|---|
| **M0** | 建 parity 测试台：同 (q,v,外力) 双跑 Drake 与新核 | 台子本身能对 Drake vs Drake 复跑给出逐位一致；**并测出 Drake 的 RHS/接触求解调用计数基准** |
| **M1** | mini-LeafSystem 层落地，后端仍挂 Drake plant | 现有 GZ18 算例结果**逐位不变**（此时仍依赖 Drake，纯粹验证组装层无副作用） |
| **M2** | Stage 1 vendor：tree+topology double-only + 自写 context/cache + facade | `ldd` 不再出现 `libdrake.so`；GZ18 30s 算例对 M1 结果逐位一致；**调用计数与 M0 基准一致** |
| **M3** | 力元/静平衡/构建图收尾 | 全测试通过；跨平台构建（Linux + MSVC）可跑，回归在工程容差内（按 S2 口径，不要求跨平台逐位） |
| **M4** | （可选）Stage 2 逐 pass 自研化 | 每个 pass 单独以金标向量门控，任一 pass 不达标即回滚该 pass |

**M2 是 C1 达成点。** M4 可无限期推迟。

### 5.4 迁移策略要点

- **不做大爆炸切换**：每个里程碑都可运行、可与前一里程碑逐位比对。
- **facade 先冻结**：3.3 节的接口面在 M1 就定死，后续所有替换都在 facade 之后发生，仓库侧调用点不再改动。
- **状态布局必须与 Drake 一致**：`q=[quat4,pos3]`、`v=[angvel3,vel3]`、深度优先编号（浮动与转动**交错**，非"浮动排完再排转动"）。`cvode_driver.cc:68-99` 的 abstol 向量**在当前布局下已错位 43/107**（守卫只有 `DRAKE_DEMAND(N>=107)`）——迁移期照抄冻结，修正另行立项（见陷阱 9）。

---

## 6. 跨平台的硬上限（S2 已据此拍板，本节保留作为依据）

**逐位跨平台复现基本不可达，瓶颈不是编译选项：**

1. **主导发散源是 libm**：仅 `rwc_core` 就有 33×`std::sin`、33×`std::cos`、10×`std::atan`、7×`std::tan`/`hypot`/`atan2` 等，**没有任何 CMake 选项能钉住 libm 实现**。
2. **Drake 自己就不是平台不变的**：`math/fast_pose_composition_functions.cc`（855 LOC）按构建在 Google Highway 的 SIMD/FMA 路径与可移植路径之间切换。
3. **`-ffast-math` 与 `-march=native` 是已发布的构建预设**（`scripts_cpp/CMakePresets.json:42-48` 的 `ninja-fastmath`，`RWC_CORE_FAST_MATH=ON`；另有 `ninja-native`），且存在两棵活构建树。任何 parity 工作必须先关掉它们。
4. **SUNDIALS 必须精确钉版**：现构建为 7.7.0 / `SUNDIALS_DOUBLE_PRECISION` / `SUNDIALS_INT32_T`。默认 int64 的 vendored 构建会静默改变行为。

→ 故建议 S2 取"同平台同选项逐位一致 + 跨平台 ulp 有界"。若坚持全平台位同一，需 vendor 一个正确舍入的数学库，代价另计。

---

## 7. 请优先攻击的 8 个点（我自认最脆弱处）

1. **闭包测量的正确性**（2.2）。我的 131 / 0 vs 240 / 42 若因脚本 bug 而错，方案 B 的前提崩塌。请独立重测，注意 `.cc` 内的 include 与条件编译。
2. **"Stage 1 vendor 即可保证逐位复现"这个断言**（REV2 已裁决：**攻击成立**——改述为"高可信起点 + 分层预言机实测证明"；Context/cache 实为 15 处声明 + `tree_system_` 反向指针消解，估 4–8 kLOC）。
3. **接口面计数分歧**（3.3）：32 还是 56？漏计的部分是否包含无法用 facade 屏蔽、必须改仓库调用点的项？
4. **S1 接缝设计的完备性**（5.1，已拍板"留接口不实现"）：抽象推进器接口列出的五项语义（事件日历、步长夹持、x⁻→x⁺ 挂起更新、拒步回滚、Initialize）**够不够**？现有 cvode_driver 迷你调度捕获了哪些 AdvanceTo 语义、漏了哪些（如 witness 函数、同刻多事件的处理顺序）？只有一个后端的抽象接口会不会腐化成形同虚设的间接层（YAGNI 反对意见）？请给出接口的最小充分集，或论证现在根本不该定接口。
5. **工作量估算**（4.2）。两轮估算差 1.7 倍。请给出你自己的独立估算，特别是"自写 Context/cache 2–4 kLOC"这一项是否严重低估。
6. **`multibody/topology` 的深度优先重编号**。Drake 是先广度建森林、再 `CreateDepthFirstReordering`、最后 `AssignCoordinates`。vendor 时若连带 topology 就没问题，但若 Stage 2 要自研替换，这个编号顺序是隐藏的强约束。请评估它对 Stage 2 的锁定效应。
7. **C6 与方案 B 的相容性**。项目负责人要求保留 LeafSystem 等价层以便车型模板化；而 vendored 的 `MultibodyTreeSystem` **本身就是** LeafSystem。自写 mini-systems 层与 vendored tree 的 Context 之间会不会出现**两套 Context 概念打架**？这是我没有想透的地方。
8. **本方案完全没有覆盖的东西**。例如：`irw_env` 的 pybind 绑定、`startup_snapshot`、NPZ/CSV 序列化契约、`.gitignore` 把 `*.npz` 排除导致启动快照无法作为版本化二进制产物冻结（若属实，则"新算例永远需要 Drake 才能生成"）。请补齐我漏掉的范围。

---

## 8. 已知的保真陷阱清册（供审查时对照，请补充与证伪）

以下每条都可能静默破坏"复现 Drake"：

1. **DOF 编号是"先广度建森林、再深度重排"**，不是 BFS。
2. **四元数状态不做归一化写回**（REV2 精化措辞）：`N(q)=0.5·Q(q)` 用原始可能非单位的 q，`N⁺` 是带 `(I₄−q̂q̂ᵀ)/|q|` 投影的真伪逆；但姿态转换（`RotationMatrix::ToQuaternion`）与 `N⁺` 内部会**局部**归一化——不能笼统说"从不归一化"。**切勿加 Baumgarte/全局归一化稳定化。**
3. **ABA 铰接惯量分解顺序**：`D_B=HᵀPH` 只写 `triangularView<Lower>`，反射惯量加对角，用 **LLT**（非 LDLT、非显式求逆），Kalman 增益取 `llt.Solve(U).transpose()`。
4. **反射惯量在 4 处用 4 个不同公式**（逆动力学加到 τ、质量阵加对角、ABA 加进 D_B 对角、……）。
5. **`RollPitchYaw` 的矩阵→RPY 不是教科书 atan2**，是 Mitiguy 半角混合式；规范化到 w≥0 并按 `1/‖q‖` 缩放的是 **`RotationMatrix::ToQuaternion`**（rotation_matrix.cc:419-431，非 Eigen 行为）——`RollPitchYaw::ToQuaternion`（roll_pitch_yaw.cc:150）则是纯半角三角式，不归一化也不定号（REV2 修正初稿的归属混淆）。
6. **万向锁 THROW 在 RHS 上承重**：`kGimbalLockToleranceCosPitchAngle = 0.008`（roll_pitch_yaw.h:562），而 `specialized_suspension.cc` 在力计算内部调用 `CalcRpyDtFromAngularVelocityInParent` **6 次**（:263/:307/:316/:400/:456/:476）。
7. **SpatialForce 与 SpatialVelocity 的 `Shift` 符号相反**。
8. **`LinearBushingRollPitchYaw` 用半角中点坐标系**，不是原点对原点的弹簧。
9. **CVODE abstol 向量按位置硬编码**（cvode_driver.cc:68-99）——**REV2 升级：不是"布局一改才错"，是当前即错**（语义 43/107、按现默认值 30/107，双方独立复算一致，见 §3.1）。按 C3"复现当前行为"：C1 达成前**原样冻结这条错误向量**；修正是之后单独命名、单独重建基准的行为变更，不得夹在迁移中静默完成。
10. **被拒步必须事务性回滚**：Drake 在误差控制拒步时回滚 Context；普通 C++ 成员不会。warm-start 提示、PID 积分/抗饱和累加器等"向量外状态"都要显式回滚。
11. **`rwc_core` 接触批在 OMP 下**：按轮位独立写输出、每槽提示、无跨轮归约（contact_batch.cc:94+）——**"线程数必然改变结果"的机制未被证实，初稿断言撤回（REV2）**；降级为 1/2/4/8 线程逐位试验项。注意线程数会改变轮位→worker 映射，若 worker 工作区携带跨调用温启动状态则可能可观测。
12. **Ceres 数值差分步长**：as-built 头文件实测默认**相对步长 `1e-6`、近零参数回退 `sqrt(eps)`**（.cppdeps/include/ceres/numeric_diff_options.h:42；初稿 `cbrt(eps)≈6.06e-6` 有误，REV2 修正），它决定 t=0 初始条件——任何自研 Newton/LM 替换都会选到不同的扰动尺度。
13. **`RotationMatrix` 只在 debug 构建下按 `128*eps` 校验正交性，且从不重投影**。**不要加 Drake 没有的正交化。**

---

## 9. 我请求的裁决

请明确回答：

1. **方案 B（先 vendor MultibodyTree 再逐 pass 替换）能否作为实施基线？** 若否，替代路线及其代价。
2. **第 7 节 8 个攻击点中，哪几个你确认成立、哪几个你证伪？**
3. **S1/S2/S3 均已拍板（S1=留抽象推进器接口、首版只实现 CVODE 后端；S2=同平台同选项逐位、跨平台仅容差回归；S3=线性化出范围）。请改为审查：S1 抽象接口的语义集是否最小且充分？**
4. **M0–M4 的里程碑顺序与验收门是否可执行？** 特别是 M0 的"调用计数基准"该怎么测才可靠。
5. **你独立估算的工作量区间是多少？**

---

## 10. REV2：第二轮对抗审查后的裁决记录（2026-07-24）

第一轮审查（CodeX）总裁决为"方案 B 有条件通过，文稿不能直接冻结为基线"。本节记录 Claude 对其全部承重反驳的**第一手复测结果**与双方收敛后的**新增约束**。以上正文已按此修订（标注 REV2 处）。

### 10.1 复测记分（Claude 逐项第一手重测）

| CodeX 主张 | 复测结果 | 处置 |
|---|---|---|
| A1 实现闭包 281 头/geo 46/FEM 30/plant 1 | **283/46/30/1，同向确认**（±2 为计数口径）；肇事文件恰为 `deformable_body.cc`、`geometry_spatial_inertia.cc` + `element_collection.cc` 实例化 | 接受；Stage 1 改允许清单制。注：Claude 首次自测此项时脚本因源码树无 `drake/` 前缀而空转返回 0——恰好证明"闭包脚本可错"的攻击点 1 警告 |
| A2 缓存 15 项非 20；3 项纯参数缓存跨 RHS 复用 | **grep 确认 15**；3 项 `{all_parameters_ticket}` 确不随全状态写入失效 | 接受；§4.3 已改"12 项失效 + 3 项参数缓存必须按需重算" |
| A2 `tree_system_` 反向指针 | **multibody_tree.h:2211-2215 确认** | 接受；"Context 唯一阻断点"改"双向耦合最重阻断点" |
| A3 刚体 17+world=18 | **构建器逐行复数确认**（轴箱 8 个在循环内，首轮 grep 漏计） | 接受修正 |
| A3 abstol 当前错位 43/107（语义）、30/107（数值） | **独立枚举复算，两数逐位一致** | 接受；冻结错误向量入 M0，修正另行立项 |
| A3 GZ18 无 `MapQDotToVelocity` | **全仓唯一命中在 IRW 回放** | 接受修正 |
| A4 符号 449/520/426 非互斥 | 算术即证：449+520+426=1395>1299 | 接受（未逐符号重跑其 445/426/422 分类） |
| A4 位姿核心 5,448≠五件之和 5,171 | 小文件实测 269 行，5,171+269=5,440≈5,448——**是清单呈现不全，非算术错** | 表格已补全口径；差 8 行不再追 |
| A5 Ceres 步长 1e-6 + sqrt(eps) 下限 | **as-built 头文件确认** | 接受，陷阱 12 已改 |
| A5 `RollPitchYaw::ToQuaternion` 纯半角式 | **源码确认**；规范化属 `RotationMatrix::ToQuaternion` | 接受，陷阱 5 归属已改 |
| A5 OpenMP 机制无据 | **contact_batch.cc 确认按轮独立/每槽提示** | 接受撤回断言，降级为试验项 |
| B4 Drake publish 在 x⁻、仓库 CVODE 在 x⁺ | **双侧源码确认**（simulator.cc 挂起更新次循环执行 vs cvode_driver.cc:1259+ 先更新后发布） | 接受：S1 语义集扩为 B4 七项；**该分歧本身是"复现当前 CVODE 行为"的一部分,接口文档必须记录两族语义不等价** |
| C2 正式入口强制 `--startup_snapshot` | **main.cc:411-415 确认** | 接受：快照冻结入 M0 前置 |

### 10.2 双方收敛后的新增硬约束（并入实施基线）

1. **允许清单 vendor**：禁入 geometry/FEM/plant（`constraint_specs.h` 除外），CI 加 include 禁入检查。
2. **单一权威 Context**：根上下文唯一持有 t/x/参数/输入版本；子系统与树均为零复制视图 + 独立求值缓冲；**禁止双 Context 各存一份 q/v**（消解攻击点 7）。
3. **版本化缓存语义保留**：状态/参数/输入/时间版本号 + 按需记忆化 + 试算/接受双态 + 参数缓存按需重算；"每步全算"仅为一种执行模式。
4. **S1 语义集采用 B4 七项**（初始化次序与失败传播、同刻事件稳定序与 x⁻/x⁺ 可见性、发布/更新时刻区分与积分边界夹持、试算纯净/提交/回滚、CVODE 重初始化通知、失败与终止、不支持项显式报错）；witness/逐步事件/监视器 v1 显式不支持。
5. **M0 重排**：冻结工作区内容哈希/构建身份/启动快照/错误 abstol 向量；金标为**有序轨迹**（阶段序号 + t/q/v/外力/温启动逐位哈希 + 缓存命中记录 + CVODE 步统计），调用计数降为辅助指标;Drake 自重放跨进程逐位一致为台架验收门。
6. **零 Drake 判据**：链接图 + `readelf`/符号扫描 + 干净环境安装测试,不以 `ldd` 单证。
7. **交付物补齐**：输出契约(CSV/NPZ/JSON 列序/命名/采样)、两套整车图功能矩阵、第三方全清单(Highway/fmt/spdlog/CLI11/nlohmann-json/miniz 逐项裁定)、许可证 NOTICE/来源/修改记录、可视化与回放显式划出 C1。
8. **工作量基线 50–80 人周（M0–M3）**,Context/cache 兼容层按 4–8 kLOC 计。

### 10.3 仍保留的少数分歧（记录在案，不阻塞）

- 实现闭包总数 283 vs 281：计数口径差,无实质影响。
- "5,448"：呈现不全而非算术错,已补口径。
- OpenMP：双方一致认为需 1/2/4/8 线程逐位试验;Claude 补充轮位→worker 映射 + worker 工作区温启动状态为**候选**机制,待试验裁决,不作断言。
