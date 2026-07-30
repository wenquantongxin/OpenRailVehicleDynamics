# libs/multibody_runtime

**职责**：多体状态、缓存与刚性树求值运行时。

- 单一权威运行时上下文：状态只有一个所有者，其余一律为零复制视图（[ADR-0002](../../docs/adr/0002-single-authoritative-context.md)）
- 广义位置、广义速度与实际参数的版本戳，以及上下文之间的隔离
- 固定类型的缓存存储、静态先决条件与惰性求值
- vendored 刚性树与第一方运行时契约的内部绑定

**对应 Goal**：G20–G28。

运行时契约与实际缓存依赖由 G20 从 G16 后的 landed 源码和固定 Drake 来源中的
`MultibodyTreeSystem` 职责现场裁决。缓存条目、值类型与先决条件都不在此处预先写死。
最终化模型不可变并可被多个求值上下文共享；每个上下文独占 q、v、类型化参数、版本和
缓存工作区。运行时基础层只提供状态、版本和不依赖具体值类型的类型化缓存槽；刚性树接入层
组成具体缓存工作区并接收上下文求值，基础层不反向依赖 vendored tree 或缓存类型。
时间、事件、输入端口与离散状态在出现实际多体消费者前不进入本模块。
模型中立的公共建模门面属于 G29–G30，不在运行时尚未成形时提前占接口。

`include/orvd/multibody_runtime/` 为公开头，`src/` 为实现。公共类型必须用具体职责命名，
不以裸 `Context`、`Cache` 或上游内部术语占据接口。

## 已落地(G21)

`orvd_multibody_runtime` 静态库,只链 Eigen 与标准库,零 `drake/` include、零 `drake::`。

- `MultibodyStateLayoutDescription` / `MultibodyStateLayout`：最终化模型的规模。基础层
  不读最终化模型本身——那是 vendored 刚性树,读它会把依赖方向反过来——而是由接入层填写
  描述,布局是校验它之后剩下的东西。状态按 layout 的**对象身份**绑定,不按内容相等:
  两个数值相同的 layout 仍描述不同的模型。
- `MultibodyStateInstance`：广义位置、广义速度与上下文可变类型化物理参数的**唯一所有者**。
  读出 const 视图;写入取整值、全部校验通过后才落盘,因此被拒绝的写入让活状态一字不动,
  也不存在「先改活数据、随后提交」的编辑器——那个窗口里的状态既不是旧值也不是新值。
- 参数记录按物理职责命名(`RigidBodyInertiaParameters`、`FixedFramePoseParameters`、
  `JointDampingParameters`、`JointActuatorParameters`、`RevoluteSpringParameters`、
  `LinearBushingRollPitchYawParameters`),不照搬上游把空间惯量打包成十个匿名数的传输格式。
- 校验只做这一层能判定的:维数、有限性、阻尼/刚度/转子惯量非负、固定坐标系旋转确为旋转
  (校验而不修补)、惯量满足三角不等式且**不误拒合法的零惯量**。哪些位置分量是四元数属于
  模型知识,本层不猜,既不拒绝也不静默归一化。

不含时间、离散状态或事件字段:落位树对它们零命中,而为不存在的消费者预留字段,迟早会有人
为错误的理由把它填上。
