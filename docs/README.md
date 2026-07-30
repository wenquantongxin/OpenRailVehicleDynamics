# ORVD 文档索引

## 当前权威文件

| 文档 | 内容 |
|---|---|
| [planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md](planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md) | **唯一实施路书**：18 个子目标、50 个原子 Goal、依赖关系与完成门。 |
| [engineering/FIRST_PARTY_ENGINEERING_RULES.md](engineering/FIRST_PARTY_ENGINEERING_RULES.md) | **第一方工程约束**：命名、兼容层、输入解析、检查深度、热路径与验收依据；即刻生效。 |
| [adr/](adr/) | **架构决策记录**：方案 B、单一 Context、CVODE 优先等已接受决策。 |
| [design/MULTIBODY_RUNTIME_CONTRACT.md](design/MULTIBODY_RUNTIME_CONTRACT.md) | **多体运行时契约**：落位树对 systems 类型、状态、参数与缓存的逐项消费与处置，以及目标依赖方向。G20 的产物，G21–G28 的输入。 |

## 调研输入

| 文档 | 内容 |
|---|---|
| [design/DESIGN_BASIS.md](design/DESIGN_BASIS.md) | 早期对抗审查请求、源码观察与旧 REV2 裁决；仅作调研输入，不是现行计划或验收依据。 |
| [review/](review/) | 审查往来说明；不作为实现依据。 |

## 阅读顺序

1. 开始工作前读唯一实施路书，确认当前 Goal、前置产物和明确不做事项。
2. 写第一方代码前读第一方工程约束；它即刻生效，不随 Goal 变化。
3. 遇到长期架构取舍时读对应 ADR。
4. 只有追查某个 Drake 源码事实时才回看旧设计基线，并重新以当前源码验证承重事实。

## 文档纪律

- 任何实施状态、顺序和完成门只写在唯一实施路书中，不建立平行规划。
- 决策改变走新 ADR；Git 历史承担旧内容追溯，不保留废弃兼容说明。
- 不把文件数、符号数、哈希、历史输出或某次性能结果写成永久验收依据。
