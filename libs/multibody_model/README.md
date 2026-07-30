# libs/multibody_model

**职责**：模型中立的程序化多体建模门面。

G29 只提供构建所需的第一方类型与加入期失败语义：刚体、直接附着刚体的固定坐标系、转动/移动/焊接关节，以及显式自由体关系。公共头不出现 Drake 类型、上游索引或 mobilizer；PIMPL 内部独占 landed 刚性树。

`RigidBodyHandle`、`FrameHandle` 与 `JointHandle` 只用于把已经加入同一模型的元素交回模型，不暴露 ordinal，也不能由公共消费者伪造。稳定查询范围、最终化后生命周期和广义位置/速度区段统一由 G30 定义，G29 不提前建立一套可漂移的查询接口。

公共 fixed frame 与关节端点映射到 landed tree 中同一个真实 frame，不复制 pose 去生成隐藏的 joint frame。自由关系在内部使用 quaternion floating joint 实现，但该实现类型不对外，且不列入公共关节名称、句柄或计数面。

全局 World 可达性、只允许一次的最终化和最终化后的不可变性属于 G30。当前目标不安装、不导出；交付边界仍归 G46。
