# libs/core

**职责**：运行时地基。

- 单一权威 Context：状态只有一个所有者，其余一律为零复制视图（[ADR-0002](../../docs/adr/0002-single-authoritative-context.md)）
- q、v 与参数的版本戳，以及 Context 之间的隔离
- 固定类型的缓存存储、静态先决条件与惰性求值
- 覆盖 vendored 多体树的门面

**对应 Goal**：G20–G28。

运行时契约与实际缓存依赖由 G20 从当时的 Drake 源码现场导出——缓存条目、值类型与
先决条件都不在此处预先写死。

`include/orvd/core/` 为公开头，`src/` 为实现。
