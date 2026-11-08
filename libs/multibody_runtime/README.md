# libs/multibody_runtime

提供多体状态、类型化物理参数、写入版本和惰性缓存求值，安装目标为 `ORVD::multibody_runtime`。

## 状态与所有权

- `MultibodyStateLayout` 描述最终化模型的状态及参数布局。
- `MultibodyStateInstance` 独占广义位置、广义速度与可变物理参数，按布局对象身份绑定模型。
- 写入先校验再提交；读出为只读视图。联合状态写入同时更新位置与速度。
- 模型接入层负责安装默认位置与参数；基础状态存储不推断四元数分段。

最终化模型可以共享，每个求值上下文保有独立状态和工作区。

## 版本与缓存

广义位置、广义速度、固定架位姿、刚体惯量和执行器参数构成五类版本源。
执行器参数包括转子惯量与传动比。关节阻尼、转动弹簧和内核线性衬套参数由力求值直接读取。

`VersionedCacheSlot<Value, Sources...>` 在类型中声明依赖来源，保存预分配值及成功求值时的版本。
`EvaluateVersionedCacheSlot` 仅在冷或过期时调用计算器；成功后提交，失败保持过期。
返回值借用缓存存储，不能跨状态变化或再次重算持有。

具体缓存目录由刚性树接入层定义，基础层只依赖 Eigen 和标准库。
具体缓存及其依赖见[刚性树缓存工作区](../../external/drake_mbtree/include/orvd/rigid_multibody_tree/rigid_multibody_tree_cache_workspace.h)。
