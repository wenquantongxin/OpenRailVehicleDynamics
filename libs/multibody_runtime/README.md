# libs/multibody_runtime

**职责**：多体状态、缓存与刚性树求值运行时。

- 单一权威运行时上下文：状态只有一个所有者，其余一律为零复制视图（[ADR-0002](../../docs/adr/0002-single-authoritative-context.md)）
- 广义位置、广义速度与实际参数的版本戳，以及上下文之间的隔离
- 固定类型的缓存存储、静态先决条件与惰性求值
- vendored 刚性树与第一方运行时契约的内部绑定

**对应 Goal**：G20–G28。

运行时契约与实际缓存依赖由 G20 从 G16 后的 landed 源码和固定 Drake 来源中的
`MultibodyTreeSystem` 职责现场裁决。缓存条目、值类型与先决条件都不在此处预先写死。
模型中立的公共建模门面属于 G29–G30，不在运行时尚未成形时提前占接口。

`include/orvd/multibody_runtime/` 为公开头，`src/` 为实现。公共类型必须用具体职责命名，
不以裸 `Context`、`Cache` 或上游内部术语占据接口。
