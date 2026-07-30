# 多体运行时契约

落位的刚性树要脱离 `systems/framework`,先得说清楚它到底**消费**了什么。本文件逐项列出
那些消费点,并为每一项指定处置与承接 Goal。它是 G20 的产物,是 G21–G28 的输入。

## 本文件是什么、不是什么

**是**:一次现场审计的结论。证据锚点是源码路径与符号名;行号只用于定位,会随上游同步
和本仓库改动漂移,读到对不上时以符号名为准。

**不是**:防漂移机制。本轮不建长期扫描器——契约的价值在裁决内容,而防漂移由能执行的
东西承担:G26 的纯落位编译、G27 的冷热上下文对照与重算计数。

文中的计数是**本轮审计证据**,不是长期门槛。铁律第 7 条:数量是某次测量的结果,不是判据。

三件事必须分开记,混起来就会得出错误的依赖结论:

1. **Drake 声明的新鲜度先决条件** —— 写在 `MultibodyTreeSystem::Finalize` 里的 ticket 集合;
2. **计算时实际读取的上游** —— 写在 `MultibodyTree::Calc*` 函数体里的 `Eval*` 调用;
3. **ORVD 采用的语义根依赖与处置** —— 本文件的裁决。

三者不等价。例如 `Accelerations` 的**声明**只提根 ticket,而它的**连续模式计算**会读
`EvalArticulatedBodyForceCache()`;`position kinematics` 的声明是 `{q, 全部参数}`,
而它的计算读 `EvalFrameBodyPoses()`——后者只依赖参数,所以声明经传递仍然正确。

## 一、固定来源的证据

来源:`multibody/tree/multibody_tree_system.cc`(**未落位**),固定于处置账本声明的
commit。落位树里没有任何具体缓存声明,只有 `MultibodyElement::DeclareCacheEntry` 这个
通用转发器,所以声明侧只能从固定来源取。

`position_ticket` / `velocity_ticket` 在离散模式下是 `xd_ticket()`,连续模式下是
`q_ticket()` / `v_ticket()`。ORVD 只有连续模式,故下表一律记 q / v。

Drake 在该处留有自己的 TODO:**"Create more granular parameter tickets for finer …"**
——粗粒度参数 ticket 是上游的已知待办,不是正确答案。

| # | 语义名称 | 值类型 | 计算函数 | Drake 声明先决条件 | 计算时实际读取的上游 |
|---|---|---|---|---|---|
| 1 | reflected inertia | `VectorX` | `CalcReflectedInertia` | 全部参数 | 无 |
| 2 | joint damping | `VectorX` | `CalcJointDamping` | 全部参数 | 无 |
| 3 | frame pose in link and body frames | `FrameBodyPoseCache` | `CalcFrameBodyPoses` | 全部参数 | 无 |
| 4 | position kinematics | `PositionKinematicsCache` | `CalcPositionKinematicsCache` | q、全部参数 | 3 |
| 5 | system Jacobian | `BlockSystemJacobianCache` | `CalcBlockSystemJacobianCache` | 4 的 ticket | 4、9 |
| 6 | mobod spatial inertia in world (M_B_W) | `std::vector<SpatialInertia>` | `CalcSpatialInertiasInWorld` | 4 的 ticket | 3、4 |
| 7 | composite mobod inertia in world (K_BBo_W) | `std::vector<SpatialInertia>` | `CalcCompositeBodyInertiasInWorld` | 4 的 ticket | 4、6 |
| 8 | velocity kinematics | `VelocityKinematicsCache` | `CalcVelocityKinematicsCache` | q、v、全部参数 | 4、9 |
| 9 | H_PB_W(q) | `std::vector<Vector6>` | `CalcAcrossNodeJacobianWrtVExpressedInWorld` | 4 的 ticket | 3、4 |
| 10 | mobod dynamic bias (Fb_Bo_W) | `std::vector<SpatialForce>` | `CalcDynamicBiasForces` | 6 的 ticket、8 的 ticket | 6、8 |
| 11 | Articulated Body Inertia | `ArticulatedBodyInertiaCache` | `CalcArticulatedBodyInertiaCache` | q、全部参数 | 1、4、6、9、锁定布尔(当前) |
| 12 | spatial acceleration bias (Ab_WB) | `std::vector<SpatialAcceleration>` | `CalcSpatialAccelerationBias` | q、v、全部参数 | 3、4、8 |
| 13 | ABI force bias cache (Zb_Bo_W) | `std::vector<SpatialForce>` | `CalcArticulatedBodyForceBias` | q、v、全部参数 | 11、12 |
| 14 | ABA force cache | `ArticulatedBodyForceCache` | `CalcArticulatedBodyForceCache` | q、v、全部参数、**时间、精度、全部输入端口** | 3、4、8、9、10、11、13、锁定布尔(当前)；力元与派生系统注入的力 |
| 15 | Accelerations | `AccelerationKinematicsCache` | `CalcForwardDynamics` | q、v、全部参数、**时间、精度、全部输入端口** | 连续模式读 14，并在加速度 pass 读取 4、9、11、12、锁定布尔(当前) |

第 14、15 两条共用同一个 `force_and_acceleration_prereqs` 集合。上游在该处的注释说明了
原因:力与加速度不只是状态的函数,还依赖输入,并且用户可经 `MultibodyElement` 与
`ForceDensityFieldBase` 注入额外依赖。时间、精度和输入端口是 Drake 外层 system 为派生
实现保留的**保守声明包络**,落位 tree 本身对它们零命中；它们不能反推为 ORVD 刚性树内核
的实际输入。ORVD 内核只接收显式的调用期施力,由后续系统组装层决定这些力是否来自时间或
端口；数值精度不进入多体树契约。

### 落位侧的路由入口

`MultibodyTree` 声明 16 个 `Eval*`,其中 **13 个**转发到缓存;另外 3 个
(`EvalLinkPoseInWorld`、`EvalLinkSpatialVelocityInWorld`、
`EvalLinkSpatialAccelerationInWorld`)是 per-link 访问器,路由到聚合缓存,**不是新的缓存
实体**。15 条声明中未经 `MultibodyTree` 转发的两条是 system Jacobian 与 ABA force cache,
由其他路径访问。外层 `RigidBody` / `Frame` 的公共 API 同样只是路由。

## 二、ORVD 的目标处置

### 根版本压平的适用范围

对**仅由模型状态与物理参数决定**的长期缓存,新鲜度判定直接保存 q、v 与语义化参数版本的
快照即可:每条缓存的传递闭包都落在这些根上,因此不需要 ticket 图,也不需要通用依赖图。
ADR-0002 的"不预造通用依赖图"因此不是精度上的妥协。

**但压平只适用于新鲜度判定,不删除静态计算数据流**:下游计算仍然显式求值上游缓存。
上表第五列与第六列因此都要保留。

**第 14、15 两条不适用**:Drake 的声明为派生 system 保守纳入时间、精度和全部输入端口,
而 ORVD 刚性树内核真正新增的输入是本次调用施加的力。把一次求值的施力藏进长期缓存的
先决条件里,就等于让缓存记住它不该记住的东西。`Accelerations` 是前向动力学结果；
规定加速度属于逆动力学调用,不是该缓存的输入。

| # | 语义名称 | 传递后的语义根依赖 | ORVD 处置 | 承接 Goal |
|---|---|---|---|---|
| 1 | reflected inertia | 转子惯量、传动比 | 保留缓存 | G27 |
| 2 | joint damping | 关节阻尼 | **删除**;保留类型化阻尼参数,直接计算阻尼力 | G26 删,G35 计算 |
| 3 | frame pose in link and body frames | 固定坐标系位姿、刚体惯量 | 保留缓存 | G27 |
| 4 | position kinematics | q、固定坐标系位姿 | 保留缓存 | G27 |
| 5 | system Jacobian | q、固定坐标系位姿 | **不预分配缓存**,首版直接计算;只有实测证明需要才优化 | G33 |
| 6 | mobod spatial inertia in world | q、固定坐标系位姿、刚体惯量 | 保留缓存 | G27 |
| 7 | composite mobod inertia in world | q、固定坐标系位姿、刚体惯量 | 保留缓存 | G27 |
| 8 | velocity kinematics | q、v、固定坐标系位姿 | 保留缓存 | G27 |
| 9 | H_PB_W(q) | q、固定坐标系位姿 | 保留缓存 | G27 |
| 10 | mobod dynamic bias | q、v、固定坐标系位姿、刚体惯量 | 保留缓存 | G27 |
| 11 | Articulated Body Inertia | q、固定坐标系位姿、刚体惯量、转子惯量、传动比 | 保留缓存 | G27 |
| 12 | spatial acceleration bias | q、v、固定坐标系位姿 | 保留缓存 | G27 |
| 13 | ABI force bias | q、v、固定坐标系位姿、刚体惯量、转子惯量、传动比 | 保留缓存 | G27 |
| 14 | ABA force cache | q、v、固定坐标系位姿、刚体惯量、转子惯量/传动比、关节阻尼、上下文可变力元参数、最终化模型力元数据、调用期外力 | **调用期工作区**,不进长期缓存;锁定分支由 G26 删除 | G36 |
| 15 | Accelerations | 同 14 | **直接计算或调用期工作区**,不进长期缓存;锁定分支由 G26 删除 | G36 |

`FrameBodyPoseCache` 同时装有固定坐标系位姿与刚体惯量,但这只是值类型的聚合,不是一个
不可分割的版本根。第 4、5、8、9、12 条只读取其中的位姿字段,修改质量或惯量不得使它们
失效；第 6、7、10、11、13 条才沿实际读取传播刚体惯量。

## 三、缓存 × 参数类别

Drake 用单一 `all_parameters_ticket()`,于是改任何一个参数会让**全部**缓存过期。下表是
从各计算函数现场导出的实际读取,用于把参数拆成有证据的类别。

| 参数类别 | 读取它的入口 | 长期缓存或直接计算消费者 |
|---|---|---|
| 固定坐标系位姿 | `Frame::CalcPoseInBodyFrame(context)` | 3、4、5、6、7、8、9、10、11、12、13 |
| 刚体惯量 | `RigidBody::CalcSpatialInertiaInBodyFrame(context)` | 3、6、7、10、11、13 |
| 执行器转子惯量与传动比 | `JointActuator::calc_reflected_inertia(context)` | 1(以及经 1 传递的 11、13) |
| 关节阻尼 | `Joint::GetDampingVector(context)` | **无**(2 已删除,阻尼力在 G35 直接计算) |
| `RevoluteSpring` 与 `LinearBushingRollPitchYaw` 的上下文参数 | 力元自身的参数访问 | **无**(力元直接计算) |
| 锁定布尔 | `Mobilizer::is_locked(context)` | 当前 11、14、15；G26 删除后目标消费者无 |

**分工**:G20 只定语义类别(本表);G21 定这些参数的类型化存储与布局;G22 决定版本字段
如何合并——**只有长期缓存消费者集合相同的类别才可以共享版本**,没有长期缓存消费者的
参数不为对称性凭空获得版本源。

按本表,改一个刚体的质量只影响 3、6、7、10、11、13；位置、速度、跨节点 Jacobian 与
空间加速度偏置不应重算。在 Drake 的粗粒度下它会让全部 15 条过期。

## 四、`systems::Context` 消费面的处置分组

`systems::Context` 在落位树中出现 881 处,分布于 63 个文件(全部 `systems::` 类型合计
分布于 66 个文件)。逐处列出会得到一张没人读的表,但只写"全部替换"等于没有裁决。
按**消费形态**分八组,每组处置不同:

| 组 | 形态 | 处置 | 承接 Goal |
|---|---|---|---|
| 1 | 形参完全未被使用 | 删除该形参 | G26 |
| 2 | 仅向下透传,自身不读不写 | 改为具名状态/求值视图类型 | G26 |
| 3 | 读 q / v | 接 G21–G22 的类型化多体状态 | G26 |
| 4 | 写 q / v 与默认状态设置 | 同上,经先验证后一次写入的接口 | G26 |
| 5 | 类型化参数的读、写与默认值 | 接 G21 的类型化参数存储 | G26 |
| 6 | 保留缓存的求值路由 | 接 G23–G25 的槽与 G27 的工作区 | G27 |
| 7 | 直接计算或调用期工作区 | 不经缓存,改为显式参数或调用期工作区 | G33、G35、G36 |
| 8 | framework 生命周期、连续/离散脚手架、所有权校验、attorney、ticket、锁定抽象参数 | 按下表替换或整链删除 | G21、G22、G23、G26、G27 |

第 1 组与第 2 组必须分开统计:透传只随签名改型,不涉及语义;未使用形参则是直接删除。

**第 8 组的连续/离散双路径是活代码,不是空壳。** 元素级的 `DoDeclareDiscreteState` 确实
是空函数体且无派生元素覆盖(`multibody_element.h:152` 声明、`.cc:68` 空定义、`.cc:37`
调用),但 tree 自身持有 `discrete_state_index_` 成员,并有 `set_discrete_state_index` /
`get_discrete_state_index` / `get_discrete_state_vector`(三个重载)/
`extract_qv_from_continuous`,且 `multibody_tree-inl.h` 里按 `is_discrete()` 分支。
G26 要同时删掉元素级空声明与 tree 级离散分支。

### 十二种 `systems::` 类型的逐项处置

| 类型 | 处置 | 承接 Goal |
|---|---|---|
| `Context` | 改为类型化状态视图或内部求值上下文 | G26、G27 |
| `State` | 改为单一所有者的 q/v 状态实例 | G21、G26 |
| `Parameters` | 改为类型化参数存储 | G21、G26 |
| `BasicVector` | q/v 直接用 Eigen 存储,参数直接用物理类型；删除通用向量包装 | G21、G26 |
| `VectorBase` | 改为 q/v 的只读或受控写入视图 | G21、G26 |
| `LeafContext` | 改为内部多体求值上下文 | G27 |
| `NumericParameterIndex` | 只保留类型化参数布局需要的稳定句柄或偏移 | G21、G26 |
| `AbstractParameterIndex` | 随锁定抽象参数整链删除 | G26 |
| `DiscreteStateIndex` | 随离散状态双路径整链删除 | G26 |
| `CacheEntry` | 改为 G23 的类型化槽和 G27 的具体缓存目录 | G23、G27 |
| `ValueProducer` | 改为具名计算器,不保留运行时类型擦除 | G25、G27 |
| `DependencyTicket` | 改为 q、v 和相关参数的语义根版本快照 | G22–G24 |

`ValidateContext` / `ValidateCreatedForThisSystem` 的 systems 实现会删除,但"状态是否属于本
模型"这一正确性条件不能静默消失。G21 的状态布局记录最终化模型身份,G26–G27 在接收状态
或求值上下文的入口核对该身份；不建立代际句柄或回退路径。

## 五、计算输入的四类来源

不能只审计 `Context`。落位树的计算输入必须分成四类,它们的生命周期与所有权都不同:

1. **q / v** —— 每个求值上下文独占,可变,有版本。
2. **每上下文的类型化物理参数** —— 每个求值上下文独占且可变；只有第三节证明存在长期
   缓存消费者的类别才有版本。
3. **最终化后不可变的模型数据** —— 由多个求值上下文共享,**没有版本**。
4. **调用期外力或规定加速度** —— 不进上下文,不进缓存,只作为调用参数或调用期工作区。

`UniformGravityFieldElement::set_gravity_vector`(`uniform_gravity_field_element.h:52`)
当前**直接修改模型成员且没有最终化检查**。裁决:首版把重力向量归入第 3 类——建模期设置、
最终化后不可变的模型数据,**不为它建立缓存版本**;最终化后的修改入口由 G30 收口。
`LinearSpringDamper`、`PrismaticSpring` 等直接存于力元对象的常量同属第 3 类,不搬入每
上下文参数。刚体惯量在建模期可用 NaN 作未完成模型的占位,但最终化必须拒绝非有限惯量；
不把 `parameter_conversion.h` 的全 NaN 回退带入运行时。

## 六、明确记录的不支持:运行时关节锁定

**首版多体运行时不支持运行时关节锁定。** 其 API、抽象参数存储以及 ABA 算法分支在 G26
整链删除。未来若出现已批准需求,必须作为新的证据化产品决策,连同存储、API、算法与测试
完整重引入。

删除范围:

- `RigidBody::Lock` / `Unlock` / `is_locked`;
- `Joint::Lock` / `Unlock` / `is_locked`;
- `Mobilizer::Lock` / `Unlock` / `is_locked` 及其 `is_locked_parameter_index_` 抽象参数;
- ABA 中三处 `mobilizer_->is_locked(context)` 分支(`body_node_impl.cc`)。

**`Mobilizer::Lock()` 当前还会清零速度**,这个入口也必须整链消失,不能只删布尔存储。

依据:锁定状态存放在 `systems::AbstractParameter` 里,而 ADR-0002 明确首版不复制通用
抽象参数容器;为锁定单独造一个容器,是为一项没有已批准需求的能力付架构代价。已批准的
场景契约(`tests/contract/scenario_definition.h`)只有 `LinkDefinition` 与
`GeneralizedForceComponentKind{kForceNewtons, kTorqueNewtonMetres}`,不含任何锁定概念。

## 七、目标依赖方向

```
运行时基础层   状态实例(q、v、类型化参数)+ 版本 + 不依赖具体值类型的类型化缓存槽
      │                                                          G21–G25
      ▼
刚性树接入层   具体缓存工作区(位置、速度、惯量、跨节点 Jacobian H_PB_W …)+ 内部求值上下文
      │                                                          G26–G27
      ▼
公共门面       模型中立的拓扑构建、最终化与查询
                                                                 G29–G30
```

单向。基础层**不**引用 `PositionKinematicsCache` 一类 vendored 值类型,也不拥有刚性树的
具体缓存目录;接入层单向依赖基础层并组成具体工作区;门面单向依赖接入层。

**调用方向是 tree 接收上下文**,不是上下文反向定位 tree——那正是 ADR-0002 要消解的
`tree_system_` 反向指针。

最终化后的模型不可变,可由多个求值上下文共享;每个上下文独占自己的状态、版本与缓存。
