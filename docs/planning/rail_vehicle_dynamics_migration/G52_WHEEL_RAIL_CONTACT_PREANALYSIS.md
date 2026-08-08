# G52 轮轨接触：接口边界与迁移清单

## 文档职责

本文件起源于 G52「落地轮轨型面与串行接触核心」开工前的只读侦查，现继续承载接口边界、活动迁移清单
和阶段审查后的证据订正。它**不是**实施路书，**不**授权代码改动，**不**自行修订路书；
结论转入 [DISCUSSION_AND_DECISION_LOG.md](DISCUSSION_AND_DECISION_LOG.md) 裁决后才进
[MIGRATION_ROADMAP.md](MIGRATION_ROADMAP.md)。零散只读观察归
[MIGRATION_OBSERVATIONS.md](MIGRATION_OBSERVATIONS.md)，参考模型与本仓模型的差异归
[SIMPACK_ORVD_MODEL_DIFFERENCE_WATCHLIST.md](SIMPACK_ORVD_MODEL_DIFFERENCE_WATCHLIST.md)。

WRL 提交标识、工件路径、结果哈希、输出快照与历史数值语料不得成为 ORVD 的运行时身份或完成门；
**经源码消费链核实的物理参数、算法选择与离散参数可以且必须进入 ORVD 的类型化产品配置**——那正是
G52 完成门 4 要求的 all-and-only 迁入。引用 WRL 行号是登记出处，不是把该处数值升格为期望值。

WRL 路径相对 `wheel-rail-lab/` 仓根书写，本文件不含机器绝对路径。数值标 ✅ 表示本轮已由第一手
测量或复算得到，标 ◻ 表示尚未测量。

### 扫尾标记

本文件中以「**项目负责人抉择／偏好（CodeX 记录，2026-08-05）**」开头的段落，是项目负责人本轮明确给出的
取舍，不是 Claude 或 CodeX 从源码反推的结论；以「**CodeX 复核（2026-08-05）**」开头的段落，是 CodeX 在
Claude 实施前按代码、现有运行工件与现行路书做出的扫尾校准。正式实施前，二者均须同步写入
`MIGRATION_ROADMAP.md` 的 G52 条目；本文件本身仍不构成开工授权。**CodeX 收口判断（2026-08-08）**
标记的是 G52b 后续修复与 G52c 开工侦查形成的最终合同校准；实施状态只以路书为准。

---

## K0　型面资产与本地 SIMPACK 互操作

`LM.prw`、`UIC60.prr` 的注释抬头逐字为（✅ 已逐字读取）：

> `! Copyright Dassault Systemes Simulia Corp.`
> `! This is a Simpack example wheel profile.`
> `! This file should not be modified.`

✅ 全文对 redistribut / distribut / licen / publish 的检索为零命中——文件对再分发**未作任何表述**，
既非许可亦非禁止。**本仓不作法律判断。**

**项目负责人抉择（CodeX 记录，2026-08-05）**：

1. Dassault Systèmes Simulia 的 `.prw/.prr` 文件不进入 Git、不上传 GitHub、不随安装包发布，只留在本地用于
   科研核对与向 SIMPACK 回灌验证。
2. ORVD 自行整理并随包发布**自包含、格式清晰、由本项目维护的 JSON 型面资产**。这些 JSON 是 ORVD 的唯一
   运行时型面真源；项目不依赖 CONTACT，也不把 CONTACT 的资产或代码引入依赖链。项目负责人关于型面坐标数据
   公共性的判断在此登记为项目取舍，不扩写成法律结论。
3. 第一方型面设施须同时提供：严格 JSON 读写、受支持的 SIMPACK `.prw/.prr` 读写，以及二者之间的**语义转换**。
   SIMPACK 兼容通路服务于本地开发／验证，不进入接触热路径，也不得让 `.prw/.prr` 成为第二份运行时权威。
4. 转换验证比较坐标系、单位、角色和点列语义，不要求注释、空白、头字段顺序或字节级往返。测试使用临时生成的
   合成 `.prw/.prr`，仓内不得提交供应商型面或由其直接生成的固化夹具。
5. 不固化型面 SHA、外部绝对路径或供应商文件名为运行身份。G52c 以无扩展名的逻辑标识绑定安装后的 JSON 资产，
   文件路径由安装数据根解析。

**CodeX 复核（2026-08-05）**：运行时 JSON 加载器与本地 SIMPACK 转换器应共享一个经过校验的型面值对象，
但入口职责分开：前者只接受正式 JSON；后者解析／输出 `.prw/.prr`。对 SIMPACK 头部的单位、型面角色、反向、
平移、旋转、缩放等声明，只能按已查明的语义执行，尚未支持者必须响亮拒绝，不能沿用 WRL 的“读取点列后静默
忽略全部头部变换”。

---

## 一、GZ18 已晋级人格

权威清单：`scripts_cpp/rigid_wheelset/src/gz18_startup_identity.cc:29-60` 的 `ValidateContactPersonality`；
数值出处 `scripts_cpp/generated/gz18_contact_config.gen.h`。✅ 逐项复核。

**策略位**：`delta_mode=kCommonNormal(4)`、`profile_interpolation_mode=kNaturalCubic(1)`、
`centroid_mode=kVertical(0)`、`primary_patch_rule=kMinAbsDelta(0)`、`l_mode=kInterpenetration3d(3)`、
FASTSIM 网格 `kKalker1982`、`shape_correction=kNone`、`creepage.transport_mode=kRigidProfile`、
`reference_velocity_radius_mode=kSimpackLocalContactRadius`、
`kalker_coefficient_table_mode=kSimpackQuadraticPoisson`、`profile_origin_mode=kProfileCoordinate`、
`use_rail_reference_contact_frame=true` 与 `force_frame_mode=kRailReference`、
直接斑内蠕滑矩 `kSimpackMethod6DoNotComputeOrApply`。

**数值**（gen.h 行号）：`nominal_radius_m=0.42`(83)、`lateral_wheel_distance_m=0.7465`(84)、
`vref_min=0.01`(88)、`eps_contact=0.0`(89)、`rail_cant_rad=0.024994792`(104)、`gap_merge_tol=1e-06`(105)、
`dy_bin=2e-05`(106)、`s_samples=1000`(107)、`n_int_centroid=120`(108)、历史配置
`n_int_delta=80`(109)、
`softmax_temperature=1e-05`(110)、`wheel_profile_rediscretization_m=0.0`(111)、
`wheel_profile_arc_rescan_second_spline_m=0.0005`(112)、两个 spline mode(113-114)、
`eec_k_pen=1.8181818181818181`(120-121)、`fastsim_mx=fastsim_my=21`(131-132)、
`polach_mu/A/B=0.3/0.4/0.55`(133-135)、`kalker_weight=1.0`(139)、`kalker1982_tolerance=0.01`(140)。

✅ **一处活着的常量不自洽**：`rail_cant_rad=0.024994792` 与位姿侧 `pose_phi_rail=atan(1/40)=0.02499479361892016`
相差 `1.618920e-09 rad`，是两个独立字面量。**CodeX 复核（2026-08-05）**：G52 首批按两个具名用途原样
承载，不擅自归一成 `atan(1/40)`；等真实消费者证明二者应是同一物理量后，再单独裁决是否统一。这里保存的是
活动模型的两项输入语义，不把该差值升格为数值金标。

✅ **`kalker1982_tolerance`**：`src/fastsim.cc:310-313` 在该值非正时回退 `1/(5·(my-1))`，`my=21` 时位级等于
`0.01`，所以 GZ18 的 0.01 与 IRW 的结构体默认 0.0 今天等价纯属巧合。G52 应把它作为**显式、必填、严格为正**的
配置项承载 GZ18 的 0.01，并**不迁移**那个非正哨兵回退。它是数值参数，不是第三条策略轴。

**CodeX 收口判断（2026-08-08）**：`n_int_delta` 虽存在于生成配置，却不属于已晋级共同法向人格。
WRL 在 `kCommonNormal` 下仍运行旧的加权接触角积分，随后无条件用共同法向结果覆盖它；该细分数因而没有
到达接触几何输出。ORVD 已直接实现共同法向并以解析／几何性质门覆盖，不迁移这段无消费者计算，也不把
`n_int_delta` 纳入 G52c 来源清单分母。`softmax_temperature` 同样只服务于本 Goal 明确不迁移的软极值
形心分支，不属于 GZ18 接触人格，也不进入 G52c 的 all-and-only 分母。

---

## 二、参数分五类

一、**共性物理参数**（`E`、`nu`、`polach_*`、`mu`）；二、**车型策略位**（第一节的策略位）；
三、**显式离散参数**（`s_samples`、`n_int_centroid`、`dy_bin`、`gap_merge_tol`、`mx/my`、`eps_contact`、
`kalker1982_tolerance`、两个型面预处理步长）；四、**算法内部常量**
（`kMaxPatches=4`、`kMaxCandidatePatches=16`、`kMaxSegments=64`、弧长重扫 64 步二分、Gauss-16 阶数、
`SolveCircleSegmentHeight` 的 `tol=1e-14`／`max_iter=80`——✅ 后两者是默认实参，唯一调用点从不覆盖）；
五、**资产派生常量**。

第五类是新增的，且已有实证：✅ 用 `ComputeGaugeDelta` 在冻结的 UIC60 数组上以
`gauge_measuring_depth_m=0.016` 复算，得到的 `pose_y0_r` 与冻结字面量**位级相等**（差为 0.0，左右皆然）。
它不是自由参数，而是（轨型 × 轨距 × 测量深度 × 轨底坡）的函数；测量深度改 2 mm，`y0_r` 移动
`1.113e-04 m`。正式型面既然在运行时从本项目 JSON 加载，这类常量必须随型面重新求解；本地 `.prw/.prr`
核对也须走同一派生函数，不能把冻结字面量当普通配置项抄进来。

**CodeX 复核（2026-08-05）**：路书完成门 4 的 all-and-only 分母应包括前三类里由车型／接触人格选择的
物理量、策略位与离散参数，以及第五类的**派生公式和输入**；第四类仍是实现内部常量，由解析性质或结构测试
约束，不为凑齐清单而全部升格成产品配置。开工前须把这一边界写入路书，避免把轻量实现变成参数袋。

---

## 三、路书需要修订的四处

**修订 1 — 完成门 1 的 `‖F_t‖ ≤ μ·N` 会否决忠实移植。**
✅ FASTSIM 只逐单元按 `μ_eff·p_z` 钳位。在 GZ18 实际使用的 Kalker-1982 网格上，实测
`‖(Tx,Ty)‖/(μ_eff·N) − 1 = +1.362163186812e-03`；均匀网格上是 `−2.895619001886e-03`。两者与
`Σ(μ_eff·p_z·dA)/(μ_eff·N) − 1` **同位相等**——超出量正是压力求积误差，且**与 a、b、N 精确无关**
（a/b 从 0.15 到 21、N 从 1 到 1e7 位级不变）。六万样本中 15838 个超出 `μ_eff·N`。
另：压力律是**抛物面** `p_z=(2N/πab)(1−x̄²−ȳ²)`，不是 Hertz 半椭球，文中措辞需一并改正。

建议门：逐单元 `hypot(p_curr) ≤ μ_eff·p_z`（实测最差 2 ULP）；合力 `‖F_t‖ ≤ Σ(μ_eff·p_z·dA)·(1+1e-14)`
（实测最差相对 `5.55e-15`，32 ULP，约 2.5 倍余量）。不可写 1 ULP。另需注意：`pmax_area_integral`
只在 `capture_diagnostics` 打开时累加；测试应从单元输入独立重算该上界，不为完成门给产品输出新增诊断字段。

**修订 2 — 完成门 1 必须区分平移蠕滑与自旋蠕滑。**

**CodeX 复核（2026-08-05）**：`nu_x=nu_y=0` 并不表示“没有蠕滑”；只要 `phi_z!=0`，它就是合法的
**纯自旋蠕滑辨识点**。`src/fastsim.cc` 的两个牵引梯度仍显式包含 `phi_z`，Kalker `C23` 耦合与滚动历史
可以产生非零 `Ty`。因此完成门只能规定 `nu_x=nu_y=phi_z=0` 时切向合力为零，并另用纯自旋夹具证明
`phi_z` 通道没有被误删。

原稿的 `phi_z=-0.186423 → Ty=4294.59 N` 是**使用 GZ18 冻结参数构造的 FASTSIM 性质探针**，不是 GZ18
生产轨迹曾精确达到 `nu_x=nu_y=0` 的证据。项目负责人已澄清：仓内两份历史 AFS 长期未被当前仿真调用，
与最近两三周的 Git 固化和主要数值实验无关；下述结论**不使用这两份 AFS**。

**CodeX 外置卷实查（2026-08-05）**：最近一周的 GZ18 误差归因与生产验收工件给出了更完整的答案。

- P007 是改正公共轮速前的 60 km/h 八斑零时审计（仓内索引见 WRL
  `docs/模型对齐进展记录/GZ18宏观动力学误差闭合路书.md:51`）。其 SIMPACK／C++ 八斑都有 `nu_y==0`，但
  `nu_x` 分别约为 `1.1960e-4` 与 `1.0026e-4`，并有 `phi_z≈±0.1862/±0.1897 1/m`。它定位了
  “冻结轮速低于局部纯滚所需轮速”，不代表当前公共轮速身份。
- P035 是采用当前公共轮速 `39.659715290819015 rad/s` 与移动 Type-78 偏移的 0–10 ms
  SIMPACK／Drake 成对移动启动实验（`GZ18_P035_type78_moving_offset_lifecycle_10ms_release/analysis/`
  `P035_CROSS_PLATFORM_101POINT_ARRAYS.npz`），共 `101×8` 轮—时刻样本。SIMPACK 的
  `nu_x∈[-6.5880e-7,3.5460e-8]`、`|nu_y|≤1.7516e-11`；Drake 的
  `nu_x∈[-8.5853e-7,-6.9898e-8]`、`|nu_y|≤2.4390e-11`；两端 `|phi_z|≈0.1862 1/m`。它们已经是
  **近纯自旋的实际移动启动态**，只是平移蠕滑在浮点上不是逐位零。
- P057 是 8 月 2 日的当前生产人格、平直基础轨道＋AAR6、20 s 正时 SIMPACK／Drake 验收。Drake 长程
  归档未写出三项蠕滑率，但同批 SIMPACK SBR 已抽取为
  `GZ18_P057_full_envelope_aar6_production_acceptance/analysis/SIMPACK_AAR6_20S_NATIVE_CORE_F32.npz`：
  8 轮各 40001 点，共 **320008 个活动轮—时刻样本**。
  实查中没有一个样本的 `nu_x` 与 `nu_y` 在 binary32 上同时精确为零；但有 **7180 个**
  样本满足 `hypot(nu_x,nu_y)<1e-12`，时间覆盖约 `0.719–1.835 s`，此时
  `|phi_z|=0.186205685 1/m`；八轮的最小平移蠕滑模为 `1.43e-14–2.09e-14`。这是**实际生产轨迹中
  数值上可达的纯自旋极限**，不是旧 AFS 或合成语料。
- 作为对照，P046 的 R300+AAR5 16 s 动态误差分解在 6 个选定状态、2 侧车轮、3 个同态评估臂中共保存
  36 条详细记录，同样无一条两项同时为零；SIMPACK 原生 12 条为
  `nu_x∈[-6.8680e-3,4.5483e-3]`、`nu_y∈[6.1264e-3,1.1632e-2]`。这说明强曲线／不平顺动态与平直纯滚区的蠕滑结构不同。

因此，对“GZ18 真的出现过 `nu_x=nu_y=0` 吗”的严格答案分两层：**按浮点位模式逐位判断，本次核查的近期工件没有；
按物理与数值精度判断，P035 已是近纯自旋启动态，P057 更在数千个实际样本中把平移蠕滑压到 `1e-12` 以下，而自旋蠕滑保持约
`0.1862 1/m`**。所以 G52 使用精确 `nu_x=nu_y=0, phi_z!=0` 的 FASTSIM 夹具是为了隔离并辨识自旋通道；它是物理可达极限的理想化，
不应被写成已观测到的逐位恒等式。本节数值只登记近期实验事实，不进入 G52 完成门或产品运行身份。

**修订 3 — 轮／轨反力的合同与完成门必须分开。**

✅ `rwc_core` 只产出轮侧扳手。把轨侧反力定义成轮侧取负可以是正确合同，但拿同一个数现场构造再检查
`-x == -x` 没有判别力。**CodeX 复核（2026-08-05）**：G52 冻结以接触点 `p_GP` 同点约化的规范轮侧扳手，
并给出等大反向的**规范轨侧反扳手**；G53 在真实钢轨接线出现时再裁定是否把轨侧扳手搬到钢轨材料点。
若轨侧点 `R` 与轮侧点 `P` 不同，则必须做空间扳手搬运：`F_R=-F`，
`tau_R(R)=-tau+(R-P)×F`。完成门应独立从 `N/Tx/Ty` 与直接力偶重建轮侧扳手，再把两侧扳手搬到同一原点
验证合扳手为零；不能只验由产品输出取负得到的值。由同点取负定义轨侧输出是接口合同，不单独冒充物理证据。

方法 6 的语义也需准确登记：它保留自旋对 `Tx/Ty` 的影响，却有意令直接接触斑自旋力矩 `Mz_direct=0`。
因此等大反向扳手的两体虚功关系仍可成立；缺掉的是相对于完整牵引分布一阶矩的通道，量纲上为
`Delta P = Mz_direct*wrel_z = vref*Mz_direct*phi_z`。这是 GZ18 已资格化人格的模型截断，不得用“完整牵引
分布功率守恒”反过来否决忠实复现。原稿未附完整输入的 `14.617 W / 61.44 W` 不进入路书或完成门。

**修订 4 — 「两套核心策略在本 Goal 以直接几何夹具获得真实消费者」须收窄。**
✅ GZ18 冻结了 `kCreepageTransportMode=kRigidProfile` 与 `kProfileOriginMode=kProfileCoordinate`，实测在
`has_rigid_profile_transform=false` 时内核**直接抛出**「rigid-profile transport requires a rail-profile
transform and wheel rigid-body state」。要做一次 GZ18 接触求值，必须同时给出 `wheel_origin_T`、
`wheel_velocity_T`、`wheel_omega_T`、`rail_profile_origin_T` 与 `R_T_from_rail_profile`。**由型面加位姿标量
构成的「直接几何夹具」无法驱动促进路径**，这直接决定 G52 的内部阶段边界。

关于侧滚 y-z 输运的分层：✅ `use_profile_track_roll_yz_transport` 在 `rwc_core` 里零读取（仅
`contact_kernel.h:45` 的字段声明），读它的是 `drake_sim/src/wheel_rail_contact_system.cc:756` 与
`irw/src/irw_static_equilibrium.cc:489`。但产生三个偏移的数学 `pose_builder.cc:265-289` 与消费它们的
`contact_kernel.cc:130-138, 226` **都是 G52 文件**，且实测该数学只吃两组 `Vec3d/Mat3d`、不需要线路——
WRL 自带的夹具 `tests/test_profile_track_roll_yz_offsets.cc` 用三个平移量、无型面无线路即可独立运行。
**CodeX 复核（2026-08-05）**：数学与类型化策略留在 G52，由手工构造的完整刚体运动学输入形成合成消费者；
真实线路位姿获取与开关判断归 G53，核心里不保留无人读取的布尔字段。G52 的合成消费者只能证明核心合同，
不能在 G52b 就宣称 GZ18 随包型面和真实轨道已经接通。
需要在计划里写明的一处细节：✅ 三个偏移是**部分施加**的——只作用于位姿阶段与 `rail_contact_angle_T`，
而蠕滑输入与接触点约化仍用原始 `phi/psi/y_ws`；实测 `1e-5 rad` 的滚动偏移把 `n_G` 恰好转过 `1e-5`、
`f_G` 移动 `0.181 N` 而 `N` 位级不变。这个非对称合同必须有主人。

---

## 四、G52→G53 的系统冻结断口

✅ `AssembleVehicleSystem(const VehicleDefinition&, double gravity)` 只有两个参数，在一个函数体内依次
建模型、力计划、`SystemAssemblyDescription`、`SystemInstance`、`CompiledSystemPlan` 后返回
（`libs/configuration/src/assembled_vehicle_system.cc:74-101`）；`CompiledSystemPlan` 不可移动且构造时
就存下 `const VehicleForcePlan*`。线路晚一次调用才出现，`AssembleResolvedInitialContext` 收到的是
**已冻结**的 bundle。✅ `libs/system_assembly/` 与 `libs/forces/` 中**没有任何** Add/Register/Append API，
也**没有任何** `TrackGeometry` 引用；路书 583 行「G53 消费 G50 的类型化计划扩展点」高估了 G50 的交付——
那是一句约定，不是可调用的接缝。

两侧都要动：一、G53 的允许修改列表（路书 576-577 行）必须加入 `libs/configuration/`；
二、G52 的产物定义要写明：接触模型**由型面加类型化配置构造，不需要线路、不需要试算状态**，
并在求值时接受调用方给出的轨道位姿。缺了第二半，G53 无从下手。

**项目负责人考虑（CodeX 记录，2026-08-05）**：系统冻结时序不阻塞 G52，按依赖的自然顺序建设。
**CodeX 复核（2026-08-05）**：必须区分两层对象。G52 只交付不可变的 `WheelRailContactModel`（名称可在
实施时按仓内规则调整）、求值输入输出与调用方工作区；它不知道八个轮体、线路或系统索引。G53 才以车型、
线路和该模型构造系统绑定的 `WheelRailContactForcePlan`，并按“多体模型和 `VehicleForcePlan` + 接触力计划 →
`SystemAssemblyDescription` → `SystemInstance` → `CompiledSystemPlan` → 运行时上下文”的顺序一次冻结。
不得先冻结编译计划，再提供 Append/Register 之类的运行期补接接口。接触工作区属于每个运行时上下文；
不可变型面和接触模型可被只读共享。

---

## 五、所有权与分配

**所有权三分**：不可变、拥有型面数据的接触模型（不照搬 WRL 的 `span` 寿命合同——
`contact_geometry.h:62-68` 明写型面数组须由调用方保活，而 ORVD 的型面来自运行期加载）；
调用方独占、构造期预分配的**几何**工作区（WRL 已经是这样，`Evaluate` 收 `ContactGeometryWorkspace&`）；
接受步站位历史留到 G54。✅ 更正一处：`FastSimWorkspace` 只有 `{yb, dyb}`，由 `MakeFastSimWorkspace(my)`
建一次、作为求值器成员、以 `const&` 传入，是**不可变模型数据**，不是调用方 scratch。

**分配事实**（✅ 全部本轮实测，热工作区、诊断全关、真实 GZ18 配置）：

- `ContactKernelOutput` 构造即 **4 次 / 32 字节**——四个 `..._offsets{0}` 是含一个元素的初始化列表，
  在任何物理计算与无接触早退**之前**付出。无接触求值就是 4/32。
- 有接触时取决于 FASTSIM 自旋极点位置：极点在斑外 **5 次 / 368 字节**；极点在斑内 **10 次 / 1040 字节**；
  极点距侧边缘 < TOL 时 **11 次 / 2656 字节**。**GZ18 的真实平衡态是「极点在斑内」**
  （实测 `ȳ=0.6197`，`r²=0.384`），要到斑外需要 `|nu_x| > 1.7e-3`，那是大滑行不是正常运行。
  按 8 轮计，每次接触更新 **80 次 / 8320 字节**。此前记录的「5 次 / 368 字节」低估了真实稳态。
- 主因是 `MakeKalker1982LateralSlices` 细化分支**没有 `reserve()`**（`fastsim.cc:56/97`），6 次增长装 28 个切片。
  ✅ 该分区是 `(my, tolerance, 极点)` 的纯函数，而极点每次求值重算，**不能提到构造期**；须写进调用方缓冲。
  ✅ 上界与 `my` 无关，等于 `floor(1/TOL)+1`：GZ18 是 **101 个切片 / 1616 字节**。
- ✅ 冷启动另计且很大：求值器构造 168 次 / 675832 字节，`MakeGeometryWorkspace` 40 次 / 1717528 字节，
  首次 `Evaluate` 24 次 / 1822784 字节。

**几何工作区容量**：✅ 修复前的 `envelope_capacity_ = 4951` 只按未旋转横向跨度推导，名义姿态已需 4979 箱，
会让工作区首次求值扩容。该数只是预定容提示；修复前的 `GrowEnvelope`／`GrowUnion` 在写入前按实际需求扩容，
所以问题是热路径合同而不是内存安全。原结构还把三类不同规模的数组绑成一个容量：逐分箱数组、只保存
占用箱的包络数组、以及轮轨并集数组。

**项目负责人裁决（CodeX 记录，2026-08-08）**：不得为了节省容量把真实车辆响应限制为某个横移、侧滚或
偏航范围。三类容量分开：

- `bin_highest`／`bin_argument` 按全部姿态严格成立的几何界定容。令构造后轮廓样本
  `r_max=max_i|R+h_i|`、`span=max(y_i)-min(y_i)`，任意单位投影方向上的跨度不超过
  `D=sqrt((2·r_max)^2+span^2)`。按产品实际消费的弧长重扫样条复算，GZ18 为
  `D=0.90085634911737333 m`，20 µm 分箱的 `ceil(D/step)=45043`；实现再加 3 格机械浮点裕量，
  故 `bin_capacity=45046`。不得拿作者原始节点集推得的近似值替代产品实际预处理结果。
- 逐占用包络数组按 `outline_sample_count` 定容，因为每个轮廓样本最多贡献一个占用箱。
- 并集数组按 `outline_sample_count + rail_node_count` 定容；无需为省两个元素引入更复杂的紧界证明。

拆分后每轮工作区有效载荷为 665,376 B，八轮合计 5.323 MB（5.076 MiB）；相对旧预定容约增加
1.573 MB（1.500 MiB），不构成车辆状态限制的理由。严格几何界只加少量机械尺度浮点裕量；仍被突破
说明实现不变量已被破坏，可以响亮失败。不得再把近期观测到的姿态最大值
写成许可域，也不得为容量核验增加第二次状态求值。`ContactKernelOutput` 的初始化列表、FASTSIM 切片和
几何暂存是本轮应消除的三处真实热路径分配。

---

## 六、ORVD 侧现在没有的输入

✅ 全仓 `libs/` 检索：**没有轨距**（`gauge`），没有轨距测量深度，没有不平顺族，没有轮对相对运动学。
线路只提供 `rail_reference_lateral_span_meters`，随包线路取 `1.5`，而 GZ18 的 `gauge_m=1.435`——
照搬会把每根钢轨放偏 32.5 mm。而轨距与测量深度正是第五类常量 `pose_y0_r` 的输入。
GZ18 冻结的 `kPsiMode=kIrregY` 定义为 `psi − atan2(dy_ds·s_dot, s_dot)`，其输入在 ORVD 也不存在。
**CodeX 复核（2026-08-05）**：`gauge_m` 与 `gauge_measuring_depth_m` 属接触型面装配输入，G52 必须类型化
承载并由它们和轨型派生左右 `pose_y0_r`；`rail_reference_lateral_span_meters` 是线路参考几何，不得冒充轨距。
轨道不平顺属后续接线输入；GZ18 随包演示在 G53 显式传零并登记适用边界，G52 不预建一套不平顺系统。

---

## 七、G51 交付接口与 G52 的绑定责任

**CodeX 复核（2026-08-05）**：原稿把两项正确接口误判成形状问题，现予纠正。

1. `StartupWheelRailBinding` 的车辆级轮型、轨型和接触策略标识是 GZ18 当前正确粒度：四根轮对、左右两侧
   共用同一组不可变型面资产与接触人格。G52 完成门 5 要把标识解析成真实加载对象并比对人格，而不是把三个
   字符串机械扩成逐轮／逐侧数组。将来出现逐侧不同资产的真实消费者时再扩展。
2. `rail_profile_reference_vertical_offset_meters` 的单值也应保留。活动的 WRL 资格化启动身份会把同一
   `rail_offset_m` 写入左右 `pose.z0_r`；生成配置里相差 `1e-8 m` 的两个默认量不是该移动启动状态的权威。
   G52 不得借此拆成两份启动偏移。
3. 左右 `pose_y0_r` 不是启动输入，而是轨型、轨距、测量深度和轨底坡的派生量，必须在 G52 加载型面后重算。
4. G51 现有 `LM.prw`／`UIC60.prr` 是来源侧文件名。按 K0 的项目负责人抉择，G52c 应把运行时绑定迁成
   与文件格式无关的逻辑标识（例如 GZ18 的轮型标识和 UIC60 轨型标识），再由安装数据根解析本项目 JSON；
   标识与路径是两个概念，不保留扩展名作为产品身份。

---

## 八、明确不做（订正版）

不迁 `src/contact_batch.cc`（并行驱动整支）；不迁 `profiling.h` 的阶段计时与 `RWC_CONTACT_STAGE_TIMING`；
不迁 `RWC_IRW_*` 的环境开关（IRW 完整人格不在 G52 范围）。✅ 无并行/无环境依赖需要**三个**清除点而非两个：
OpenMP 源码用法只在 `contact_batch.cc`，环境读取在 `contact_batch.cc:22` 与 `contact_kernel.cc:23`，
**构建侧** `rwc_core/CMakeLists.txt:29-33` 无条件 `PUBLIC` 链接 `OpenMP::OpenMP_CXX`。

✅ **三维长度的 WRL 事实与 QCH 证据边界必须分开。** WRL 中解析弦长不是独立人格：
`normal_force_eec.cc:200-202` 无条件先算 `L_base = 2·sqrt(2·R_eff·pen)` 并作为 `L_i` 初值；有限正值的
三维长度解析成功后才覆盖，失败时保留基线并置回退标志，`contact_kernel.cc:286-290` 累计尝试与回退。
QCH FileId 38410／38415 没有公开三维纵向长度解析失败时应回退还是拒绝，因此这不是 QCH 与 WRL 的
明文冲突，而是 QCH 沉默、近期 WRL 给出执行事实。

**项目负责人最终裁决（CodeX 记录，2026-08-08；取代 2026-08-06 的强失败裁决）**：ORVD 恢复近期 WRL
兼容语义。2026-08-02 平顺 30 s 工件记录 302/10,934,468 次回退／尝试，AAR5 30 s 工件记录
4,686/23,936,829 次；这些计数包含重复 RHS 与被拒步试算，不能解释成唯一接受态样本，但足以证明回退是
正式 RHS 路径真实消费的行为。G52b 后续修复已恢复解析弦长基线、三维成功覆盖、失败置定长标志／计数；不得为统计
另做状态求值，不得在热路径格式化日志或动态分配。通用 CVODE 可恢复错误分类不随本修复引入；其余真正的
配置错误、编程错误与内部不变量破坏仍按现有致命事务处理。

✅ **`kComputeAndApply` 不是死分支**：它是结构体默认值，且 SH17 因**未设置该字段**而继承它
（`sh17_contact_model_config.cc:97-108`）。应称「GZ18/IRW 未选中的非目标人格分支」。
✅ 同理 `ShapeCorrection::kPkBeta` 由 SH17 正选（`sh17_contact_config.gen.h:49`）。
**CodeX 复核（2026-08-05）**：PREF-013 已明确当前迁移不考虑 SH17，因此这些分支在 G52 中不迁移，
但不得在文档里称为“全局已死”；它们是被本批 GZ18／后续 IRW 范围排除的活动 WRL 分支。无需再次上呈
“SH17 是否在范围内”。
✅ 可干净删除的只有两支：`KPenMode::kStripes`（连带 `StripesEpsilon`、`SolveHertzEccentricity`、
`k_pen_fallback`）与 `kalker_weight≠1`。✅ 删 `kMaxYZ/kMaxYN` 时应连带删除
`contact_geometry.cc:1367-1382` 那个**无门控**的生产循环——它对每个斑每步的每个积分采样做两次 sqrt，
而没有任何人格读它。

✅ **`io/track_ext_loader.cc`（rwc_core 十五个 `.cc` 之一）是轨道不平顺文本加载器**，消费者只有
`drake_sim`，不属于接触，不得被通配符扫进 `libs/wheel_rail_contact/`。

---

## 九、执行分段：G52a／G52b／G52c 三个强制审查子步

**项目负责人执行抉择（CodeX 记录，2026-08-05）**：G52 保持一个总 Goal，但正式拆成 G52a、G52b、G52c
三个可独立审查的子步。这是对既有“一 Goal 一个聚焦提交”纪律的显式项目负责人例外，不得再写成“最终一个
聚焦提交”。Claude 每完成一个子步，必须依次：

1. 对该子步做两遍独立自检，第二遍不得只是重复同一条命令；
2. 仅把该子步相关文件形成一个原子本地提交，提交信息遵循 `type(scope): 中文 / English`，不 push；
3. 立即暂停，不得预写下一子步产品代码；
4. 由项目负责人交给 CodeX 做阶段性对抗审查，待明确放行后才进入下一子步。

正式开工前必须先把这一执行纪律、每个子步的允许修改面和完成门写回 `MIGRATION_ROADMAP.md`，并把当前 Goal
从“暂停于 G52 前”切换到 G52a。该状态翻转可形成一笔仅含规划文档的启用提交，不计作 G52a 产品交付，
且不得夹带产品代码；随后以干净工作区开始 G52a。阶段边界按**依赖**而非行数划分：

### G52a — 型面值对象、互操作与低层几何策略

- 允许修改：`libs/wheel_rail_contact/`、型面值对象确需的 `libs/configuration/` 窄入口、
  `tools/profile_conversion/`、`tests/wheel_rail_contact/`、`tests/configuration/`、`tests/CMakeLists.txt`、
  根／模块 CMake、根 `README.md`、本迁移文档与对应窄 ADR。
- 交付：拥有点列的型面值对象；正式 JSON 模式和严格读写；本地 SIMPACK `.prw/.prr` 读写与 JSON 语义转换
  工具；样条原语；两套车轮型面预处理及构造期互斥；`ComputeProfileTrackRollYzOffsets` 数学与类型化策略。
- 夹具：只使用仓内合成点列和临时生成的 `.prw/.prr`；验证 JSON 语义往返、JSON ↔ SIMPACK 语义往返、
  两策略确实分歧且绑定明确。不得提交供应商型面或复制外部资产作为夹具。
- 完成门：JSON 与临时 SIMPACK 输入落到同一规范型面值对象；语义往返不改变坐标系、单位、角色和点列；
  两套预处理在辨识夹具上给出不同结果且同时启用被拒；侧滚 y-z 两策略的数学夹具各命中自己的类型化绑定。
- 边界：运行时正式入口只加载 JSON；SIMPACK 兼容读写服务本地科研转换，不进入接触求值热路径。
- 两轮自检：第一轮做全新开发构建、既有回归与 G52a 单元门；第二轮从另一全新构建树做 JSON／临时 SIMPACK
  语义往返、公共头自包含和 `git diff --check`。二者通过后才提交并暂停。
- 文档：更新根 `README.md`，至少补齐已完成的 G50/G51 能力、当前 G52a 状态、型面 JSON／本地 SIMPACK
  互操作边界。根 README 当前仍停在 G49，不能继续拖到总 Goal 结束。

### G52b — 串行接触核心与合成完整促进路径

- 允许修改：`libs/wheel_rail_contact/`、其 CMake 与公共头安装清单、`tests/wheel_rail_contact/`、
  `tests/installation/`、`tests/CMakeLists.txt`、本迁移文档；不借本子步改配置资产或系统装配。
- 交付：接触几何、共同法向、三维互穿、EEC 法向力、蠕滑率、Kalker 表、FASTSIM、在 `p_GP` 同点约化的
  规范轮侧／轨侧成对扳手，以及
  不可变接触模型、每上下文独占的预分配工作区和完整刚体运动学输入。
- 夹具：以 G52a 的合成型面和手工构造的非退化刚体位姿／速度跑通完整促进路径；覆盖三个蠕滑量全零、
  纯自旋、方法 6 截断、两端空间扳手搬运所需的轮侧输出与热路径零分配。此时尚未宣称真实 GZ18 型面已接通。
- 完成门：全零三蠕滑产生零切向力、纯自旋仍进入 `Tx/Ty`；逐单元摩擦钳位与积分上界成立；独立重建的轮侧
  扳手与规范成对扳手一致；方法 6 令直接 `Mz` 为零但不抹掉自旋力；促进路径不发生堆分配。
- 边界：不修改 `libs/forces/`、`libs/system_assembly/` 或多体装配；接触进入编译计划留到 G53。
- 两轮自检：第一轮全新开发／发布构建与接触完成门；第二轮做独立判别力变异、热路径分配探针和迁移安装下的
  公共库消费者。还原后全绿才提交并暂停。

### G52c — 自有型面资产、GZ18 人格与安装绑定

**CodeX 阶段记录（2026-08-08）**：本节合同已完成实施、两轮自检与阶段性对抗审查；G52 已签收，
但本次签收不授权 G53。完成状态仍以路书为准。

- 允许修改：`track_library/rail_profiles/`、`vehicle_library/gz18/wheel_profiles/`、
  `vehicle_library/gz18/startup_states/` 的逻辑标识、`libs/configuration/`、安装规则、
  `tests/{configuration,installation,wheel_rail_contact}/`、`tests/CMakeLists.txt`、根 `README.md`、路书与窄 ADR。
- 交付：本项目维护并随包发布的 GZ18 轮型／轨型 JSON；格式无关的逻辑标识；轨距、测量深度及
  `pose_y0_r` 派生；GZ18 类型化接触人格；安装数据根解析；G51 启动绑定 all-and-only；迁移安装消费者。
- 夹具：从**安装后**数据根按逻辑标识加载 JSON，组装 GZ18 人格并跑一次完整核心求值；本地 `.prw/.prr`
  只作为仓外一次性语义互操作核对的输入，不成为产品资产、运行时输入或第二份权威；本地科研源文件可以
  保留，临时探针与输出用后删除。
- 完成门：两个逻辑标识 all-and-only 解析到随包 JSON；轨距／测量深度与轨型独立派生左右 `pose_y0_r`，不得
  使用线路参考半距代替；GZ18 类型化人格逐项消费活动参数；迁移安装消费者从非默认数据根完成加载和求值；
  真实 GZ18 资产和人格须验证 G52b 后续修复已经建立的三类工作区定容、促进路径不扩容及三维长度同次
  回退报告；Kalker 表外保持近期 WRL 的渐近硬切换，不引入姿态许可域或未经对拍的连续化。
- 来源核对直接读取 P035 冻结状态与 SIMPACK 原生样本 0 接触元组，不在 G52c 重新求静平衡。临时程序
  验证八轮 all-and-only、左右镜像和单斑，再把 ORVD、近期 WRL 与 SIMPACK 三列并排；数值元组、外部绝对
  路径、输出与 SHA 用后删除，不抄入代码、测试、资产或仓内数值金标。P035 只资格化 GZ18、直线、零不平顺、
  60 km/h、Type-78 移动偏移启动附近，不外推到轮缘、高攻角、表外 `a/b` 或完整动态范围。
- **CodeX 收口判断（2026-08-08）**：SIMPACK Type-80 原生 `Tx/Ty` 是轨侧端点分量；与 ORVD 的
  `rail_on_wheel` 结果比较前须取负。ORVD 轨型系 `+z_T` 向下且轮侧力
  `F_w=Tx·t_x+Ty·t_y-N·n`；P035 平直轨道上 `T` 与惯性系同向，所以正的世界竖向支承幅值为
  `Q=-e_{z_T}^T·F_w=N·n_{z_T}-Tx·(t_x)_{z_T}-Ty·(t_y)_{z_T}`，不得以 `N·cos(delta)` 代替。SIMPACK 独立通道是
  binary32 写出，重建投影只按物理容差比较，不要求逐位相同。
- Kalker 表外现阶段为端到端复现保留近期 WRL `kAsymptotic` 与 `a/b=0.1/10` 硬切换；两端有限表与
  渐近式存在约 0.08%–5.64% 的分量跳跃。**CodeX 收口判断（2026-08-08）**：当前实现与观察表沿用
  Kalker 1990 Appendix E／Table E3 的文献归属，但本轮没有独立持书逐式复核；SIMPACK 精确切换语义也
  未资格化。不得冒称 QCH／SIMPACK 合同，未经另立模型实验不得平滑，也不得以此建立车辆姿态许可域。
- 两轮自检：第一轮三套全新构建及 GZ18 绑定门；第二轮使用非默认 `CMAKE_INSTALL_DATADIR` 做迁移安装消费，
  再做一次仓外 `.prw/.prr` → JSON → 核心的科研对拍，删除探针和外部工件后才提交并暂停。
- 文档：原子提交时更新根 `README.md` 的稳定能力边界，并同步路书、窄 ADR 与安装目录说明；阶段
  审查通过后在路书中标记 G52 完成。

G52c 才允许写入真实型面 JSON 目录；G52a/G52b 不得抢跑资产绑定。若实施发现必须越出上述允许面，Claude
须在该子步内停下上呈，不得以“总 Goal 已授权”为由跨层修改。

### 型面互操作合同

✅ WRL 现有摄取只读第一个 `point.begin…point.end` 块、取每行前两个浮点并按 y 排序，且静默忽略头部变换；
11 份文件里 7 份声明 `inversion=1`，之所以未改变当前点列，只是因为这些点在文件内已经严格按 y 升序。
这只能解释历史行为，**不能作为 ORVD 新解析器继续忽略元数据的理由**。

**CodeX 复核（2026-08-05）**：正式 JSON 必须显式携带模式版本、型面角色、坐标轴／单位和有限有序点列。
本地 SIMPACK 兼容器至少识别文件角色、长度单位、点块和已查明的变换声明；按 SIMPACK 已确认语义执行，未知或
未支持的非恒等变换响亮拒绝。输出采用 SIMPACK 可导入的规范简式，不承诺注释和字节往返。UIC60 的 y 步长并非
严格常数，不得用 `index=(y-y0)/dy` 假装 O(1) 均匀查表。测试只验证语义，不保存哈希或逐位外部金标。

---

## 十、仍然未知

1. ◻ 接触站位是否由投影获得。ORVD 的 `ProjectPointNearSeed` 只接受种子加窗口、拒绝越域窗口、
   且要求窗口内恰有一个内部极小；WRL 的 `ComputeSEff` 在 `rwc_core` 之外有三份逐字副本。
2. ◻ G53 的轨侧反力最终保留在规范点 `p_GP`，还是搬运到钢轨材料点——这是有了真实钢轨接线才能裁定的
   建模问题，不阻塞 G52a/G52b；G52 冻结同点约化的规范成对扳手和正确的空间扳手搬运原语。
3. ◻ SIMPACK 在 Kalker 表两端的精确切换语义仍未对拍。该未知项不改变 G52c 当前复现 WRL
   `kAsymptotic` 的执行合同，也不得转化成车辆横移、偏航或 `a/b` 许可边界。

**CodeX 收口判定（2026-08-08）**：容量已由全部姿态严格几何界解决，不再依赖资格化偏航范围；Kalker
表外当前行为也已由项目负责人裁定为复现近期 WRL，不以车辆响应边界规避。上述未知项分别属于 G53 接线或
后续 SIMPACK 同输入模型核验，不得拖延 G52c，也不得给热路径增加第二次求值、动态诊断或日志。
