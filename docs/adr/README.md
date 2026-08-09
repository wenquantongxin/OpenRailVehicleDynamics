# 架构决策记录（ADR）

每份 ADR 记录一个有长期影响、且有替代方案被否决的决策。格式：背景 / 决策 / 后果 / 状态。
已接受的架构决策若被推翻，则新增 ADR 并把旧条标为已取代；不改变决策的事实纠错与澄清
可以原地修正，由 Git 历史追溯。

## 索引

| # | 决策 | 状态 |
|---|---|---|
| [0001](0001-vendor-tree-behind-allowlist.md) | 先按允许清单 vendor Drake `multibody/tree`+`topology`，再逐 pass 自研（方案 B） | Accepted |
| [0002](0002-single-authoritative-context.md) | 单一权威 Context，树与子系统均为零复制视图 | Accepted |
| [0003](0003-abstract-advancer-cvode-first.md) | 抽象推进器接口 + 首版只实现 CVODE 后端 | Accepted |
| [0004](0004-focused-dynamics-qualification.md) | 多模型本地性质门 + 单个高耦合在线漂移门 | Accepted |
| [0005](0005-bind-wheel-rail-low-level-strategies-by-vehicle.md) | 按车型绑定两处轮轨低层策略，禁止运行期混搭 | Accepted |
| [0006](0006-deterministic-wheel-contact-openmp-batch.md) | 八接口轮轨接触采用每接口独占工作区的确定性 OpenMP 批求值 | Accepted |

早期调研背景见 [../design/DESIGN_BASIS.md](../design/DESIGN_BASIS.md)；它不是现行证据，
其中承重事实使用前必须按当前源码重新验证。
