# 架构决策记录（ADR）

每份 ADR 记录一个有长期影响、且有替代方案被否决的决策。格式：背景 / 决策 / 后果 / 状态。
决策一旦 `Accepted` 不原地改写；要推翻则新增一条并把旧条标 `Superseded by ADR-NNNN`。

## 索引

| # | 决策 | 状态 |
|---|---|---|
| [0001](0001-vendor-tree-behind-allowlist.md) | 先按允许清单 vendor Drake `multibody/tree`+`topology`，再逐 pass 自研（方案 B） | Accepted |
| [0002](0002-single-authoritative-context.md) | 单一权威 Context，树与子系统均为零复制视图 | Accepted |
| [0003](0003-abstract-advancer-cvode-first.md) | 抽象推进器接口 + 首版只实现 CVODE 后端 | Accepted |

这三条决策的详尽论证、被否方案与复测证据见 [../design/DESIGN_BASIS.md](../design/DESIGN_BASIS.md)。
