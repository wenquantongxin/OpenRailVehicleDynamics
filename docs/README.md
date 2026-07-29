# ORVD 文档索引

| 文档 | 内容 |
|---|---|
| [design/DESIGN_BASIS.md](design/DESIGN_BASIS.md) | **设计基线**：目标与硬约束、Drake/仓库侧第一手观察、路线判断（方案 B）、代码落地方案、保真陷阱清册，以及两轮对抗审查的完整裁决记录（§10 REV2）。这是项目的权威依据。 |
| [planning/PROJECT_PHASES.md](planning/PROJECT_PHASES.md) | **规划书**：M0–M4 分阶段的难点、测试点、工时与摆动因子。 |
| [adr/](adr/) | **架构决策记录**：已锁定的关键决策，每条一份，含背景、决策、后果、状态。 |
| [review/](review/) | 对抗审查往来存档（供追溯，不作为当前依据；当前结论已并入设计基线 §10）。 |

## 阅读顺序

1. 新加入者：先读根 [README.md](../README.md) 建立全局，再读 `design/DESIGN_BASIS.md` §0–§6。
2. 要动手实施：读 `planning/PROJECT_PHASES.md` 定位当前里程碑，再回设计基线 §5（落地方案）与 §8（陷阱）。
3. 要理解某个架构取舍：读对应 `adr/` 条目。

## 文档纪律

- 设计基线中所有承重数字都标注了出处（path:line）。修改基线时，新数字须第一手复测，并在 §10 记分表登记。
- 决策变更走 ADR：新增一条 `adr/NNNN-*.md`（状态 `Proposed`→`Accepted`），旧决策标 `Superseded by ADR-NNNN`，不原地改写历史。
