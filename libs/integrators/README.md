# libs/integrators

**职责**：时间推进。抽象推进器接口与 CVODE 后端（[ADR-0003](../../docs/adr/0003-abstract-advancer-cvode-first.md)）。

**对应 Goal**：G43–G45。

CVODE 是首个也是当前唯一的后端。事务语义是这一层的核心义务：被拒绝的步不得污染
已接受的状态，被接受的步恰好提交一次。

`include/orvd/integrators/` 为公开头，`src/` 为实现。SUNDIALS 为外置第三方。
