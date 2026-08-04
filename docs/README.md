# ORVD 文档索引

## 当前权威文件

| 文档 | 内容 |
|---|---|
| [planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md](planning/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md) | **已完成底座路书**：17 个子目标、46 个原子 Goal、依赖关系与完成门；全部 Goal 已完成，执行边界止于 G46。 |
| [planning/rail_vehicle_dynamics_migration/DISCUSSION_AND_DECISION_LOG.md](planning/rail_vehicle_dynamics_migration/DISCUSSION_AND_DECISION_LOG.md) | **车辆迁移讨论与裁决账本**：逐轮记录分歧、裁决、项目负责人的总规划与待决问题；不定义实施顺序。 |
| [planning/rail_vehicle_dynamics_migration/MIGRATION_OBSERVATIONS.md](planning/rail_vehicle_dynamics_migration/MIGRATION_OBSERVATIONS.md) | **车辆迁移协作观察簿**：Codex、Claude 等共同维护的源码事实、规划初期五组只读清单快照、假设和开放问题。 |
| [planning/rail_vehicle_dynamics_migration/SIMPACK_ORVD_MODEL_DIFFERENCE_WATCHLIST.md](planning/rail_vehicle_dynamics_migration/SIMPACK_ORVD_MODEL_DIFFERENCE_WATCHLIST.md) | **参考模型差异观察表**：登记 GZ18／IRW 的 SIMPACK 参考模型与 ORVD 动力学模型之间可能被宏观闭合掩盖的差异及其筛查结论。 |
| [planning/rail_vehicle_dynamics_migration/MIGRATION_ROADMAP.md](planning/rail_vehicle_dynamics_migration/MIGRATION_ROADMAP.md) | **现行车辆迁移路书**：6 个子目标、9 个原子 Goal（G47–G55）；已由项目负责人启用，G51 已完成，当前暂停于 G52 前；它是唯一带「当前 Goal」的进度权威。 |
| [engineering/FIRST_PARTY_ENGINEERING_RULES.md](engineering/FIRST_PARTY_ENGINEERING_RULES.md) | **第一方工程约束**：命名、兼容层、输入解析、检查深度、热路径与验收依据；即刻生效。 |
| [adr/](adr/) | **架构决策记录**：方案 B、单一 Context、CVODE 优先等已接受决策。 |
| [design/MULTIBODY_RUNTIME_CONTRACT.md](design/MULTIBODY_RUNTIME_CONTRACT.md) | **多体运行时契约**：落位树对 systems 类型、状态、参数与缓存的逐项消费与处置，以及目标依赖方向。G20 的产物，G21–G28 的输入。 |

## 调研输入

| 文档 | 内容 |
|---|---|
| [design/DESIGN_BASIS.md](design/DESIGN_BASIS.md) | 早期对抗审查请求、源码观察与旧 REV2 裁决；仅作调研输入，不是现行计划或验收依据。 |
| [review/](review/) | 审查往来说明；不作为实现依据。 |

## 阅读顺序

1. 先读已完成底座路书，确认 ORVD 已有能力及 G46 的边界。
2. 车辆迁移讨论阶段依次读裁决账本和协作观察簿；不得把其中的假设当作实施命令。
3. 按现行车辆迁移路书的「当前 Goal」推进；只实施该 Goal 的唯一产物，并逐条满足其完成门。
4. 写第一方代码前读第一方工程约束；它即刻生效，不随 Goal 变化。
5. 遇到长期架构取舍时读对应 ADR。
6. 只有追查某个 Drake 源码事实时才回看旧设计基线，并重新以当前源码验证承重事实。

## 文档纪律

- G01–G46 的状态只在已完成底座路书中维护；新的车辆迁移在路书激活前没有实施状态。
- 讨论裁决与技术观察分别进入账本和观察簿；任何实施状态、顺序和完成门只写在激活后的唯一迁移路书中，不建立平行规划。
- 决策改变走新 ADR；Git 历史承担旧内容追溯，不保留废弃兼容说明。
- 不把文件数、符号数、哈希、历史输出或某次性能结果写成永久验收依据。
