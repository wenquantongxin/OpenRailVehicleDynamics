# ORVD 文档索引

## 模型与算法

| 文档 | 内容 |
|---|---|
| [models_and_algorithms/README.md](models_and_algorithms/README.md) | **模型与算法入口**：轨道几何、轮轨接触、车辆动力学、控制与估计、数值方法的物理和数学说明。 |
| [models_and_algorithms/track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.md](models_and_algorithms/track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.md) | **轨道竖向剖面建模**：恒坡段、竖曲线、平面线形和超高的三维耦合，以及首轮产品与整车资格合同。 |

## 当前权威文件

| 文档 | 内容 |
|---|---|
| [planning/drake_multibody_runtime_decoupling/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md](planning/drake_multibody_runtime_decoupling/DRAKE_MULTIBODY_RUNTIME_DECOUPLING_ROADMAP.md) | **已完成底座路书**：17 个子目标、46 个原子 Goal、依赖关系与完成门；全部 Goal 已完成，执行边界止于 G46。 |
| [planning/rail_vehicle_dynamics_migration/DISCUSSION_AND_DECISION_LOG.md](planning/rail_vehicle_dynamics_migration/DISCUSSION_AND_DECISION_LOG.md) | **车辆迁移讨论与裁决账本**：逐轮记录分歧、裁决、项目负责人的总规划与待决问题；不定义实施顺序。 |
| [planning/rail_vehicle_dynamics_migration/MIGRATION_OBSERVATIONS.md](planning/rail_vehicle_dynamics_migration/MIGRATION_OBSERVATIONS.md) | **车辆迁移协作观察簿**：Codex、Claude 等共同维护的源码事实、规划初期五组只读清单快照、假设和开放问题。 |
| [planning/rail_vehicle_dynamics_migration/SIMPACK_ORVD_MODEL_DIFFERENCE_WATCHLIST.md](planning/rail_vehicle_dynamics_migration/SIMPACK_ORVD_MODEL_DIFFERENCE_WATCHLIST.md) | **参考模型差异观察表**：登记 GZ18／IRW 的 SIMPACK 参考模型与 ORVD 动力学模型之间可能被宏观闭合掩盖的差异及其筛查结论。 |
| [planning/rail_vehicle_dynamics_migration/MIGRATION_ROADMAP.md](planning/rail_vehicle_dynamics_migration/MIGRATION_ROADMAP.md) | **已完成车辆迁移路书**：覆盖 GZ18 首次短窗与不平顺长窗、IRW 被动、IRW 100 Hz 全状态控制与转矩指令调理，以及 G82 现有库跨平台构建与可重定位安装验证；现已封存。 |
| [performance/README.md](performance/README.md) | **计算性能文档入口**：统一计时术语、证据存放方式和直接替换纪律。 |
| [performance/PERFORMANCE_MIGRATION_ROADMAP.md](performance/PERFORMANCE_MIGRATION_ROADMAP.md) | **现行计算性能路书**：把 GZ18 专项分支中可复用的轮轨接触、Jacobian 和观测优化重新落位到现行主线，按候选性质分层资格并以一个 GZ18、两个 IRW 代表长窗收口。 |
| [performance/GZ18_PERFORMANCE_BRANCH_ARCHIVE.md](performance/GZ18_PERFORMANCE_BRANCH_ARCHIVE.md) | **GZ18 专项性能分支档案**：压缩记录历史最快组合、热点、已接受／否决方向及向 IRW 迁移的边界；不代表现行主线速度。 |
| [engineering/FIRST_PARTY_ENGINEERING_RULES.md](engineering/FIRST_PARTY_ENGINEERING_RULES.md) | **第一方工程约束**：命名、兼容层、输入解析、检查深度、热路径与验收依据；即刻生效。 |
| [adr/](adr/) | **架构决策记录**：方案 B、单一 Context、CVODE 优先等已接受决策。 |
| [design/MULTIBODY_RUNTIME_CONTRACT.md](design/MULTIBODY_RUNTIME_CONTRACT.md) | **多体运行时契约**：落位树对 systems 类型、状态、参数与缓存的逐项消费与处置，以及目标依赖方向。G20 的产物，G21–G28 的输入。 |

## 调研输入

| 文档 | 内容 |
|---|---|
| [design/DESIGN_BASIS.md](design/DESIGN_BASIS.md) | 早期对抗审查请求、源码观察与旧 REV2 裁决；仅作调研输入，不是现行计划或验收依据。 |
| [review/](review/) | 审查往来说明；不作为实现依据。 |

## 阅读顺序

1. 理解某项物理模型或算法时先读 `models_and_algorithms/`；其中会区分 ORVD 当前合同、外部参考
   语义和待验证问题。
2. 再读已完成底座路书，确认 ORVD 已有能力及 G46 的边界。
3. 追查车辆迁移裁决时依次读裁决账本和协作观察簿；不得把其中的假设当作实施命令。
4. 车辆能力与既有实施边界从两份已完成路书查阅；计算加速只按现行性能路书推进，不能把新范围
   回填到已封存 Goal。
5. 追查性能候选的历史证据时读专项分支档案；不得把历史单次最快数字写成现行产品能力。
6. 写第一方代码前读第一方工程约束；它即刻生效，不随 Goal 变化。
7. 遇到长期架构取舍时读对应 ADR。
8. 只有追查某个 Drake 源码事实时才回看旧设计基线，并重新以当前源码验证承重事实。

## 文档纪律

- 模型与算法文档维护当前物理、数学和参考语义，不记录 Goal 流水；外部软件观察必须标明来源，
  未裁决内容不得写成 ORVD 产品合同。
- G01–G46 与 G47–G82 的历史状态分别由两份已完成路书维护；本索引不重复逐 Goal 完成记录。
- 计算性能实施状态、顺序和完成门只进入性能路书；历史研究结论进入性能档案，不建立平行规划。
- 车辆讨论裁决与技术观察分别进入账本和观察簿；新的车辆功能须另立权威路书，不重新打开旧 Goal。
- 决策改变走新 ADR；Git 历史承担旧内容追溯，不保留废弃兼容说明。
- 不把文件数、符号数、哈希、历史输出或某次性能结果写成永久验收依据。
