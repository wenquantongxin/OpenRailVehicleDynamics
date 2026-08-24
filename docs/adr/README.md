# 架构决策记录（ADR）

每份 ADR 记录一个有长期影响、且有替代方案被否决的决策。格式：背景 / 决策 / 后果 / 状态。
已接受的架构决策若被推翻，则新增 ADR 并把旧条标为已取代；不改变决策的事实纠错与澄清
可以原地修正，由 Git 历史追溯。若既有 ADR 已明确预登记迁移期的收口条件，则完成该条件时在
同一 ADR 中记录最终收口，并保留原阶段的背景与 Git 历史。

## 索引

| # | 决策 | 状态 |
|---|---|---|
| [0001](0001-vendor-tree-behind-allowlist.md) | 先按允许清单 vendor Drake `multibody/tree`+`topology`，再逐 pass 自研（方案 B） | Accepted |
| [0002](0002-single-authoritative-context.md) | 单一权威 Context，树与子系统均为零复制视图 | Accepted |
| [0003](0003-abstract-advancer-cvode-first.md) | 抽象推进器接口 + 首版只实现 CVODE 后端 | Accepted |
| [0004](0004-focused-dynamics-qualification.md) | 多模型本地性质门 + 单个高耦合在线漂移门 | Accepted |
| [0005](0005-bind-wheel-rail-low-level-strategies-by-vehicle.md) | 轮轨低层策略由分车型绑定收口为 0.5 mm 等弧长重扫、镜像同相位和抑制侧滚输运 | Accepted |
| [0006](0006-deterministic-wheel-contact-openmp-batch.md) | 八接口轮轨接触采用每接口独占工作区的确定性 OpenMP 批求值 | Accepted |
| [0007](0007-finite-typed-vehicle-control-event-session.md) | 车辆周期控制采用有限类型化事件会话，不建立通用事件总线 | Accepted |
| [0008](0008-build-installed-archives-for-shared-module-consumers.md) | 安装归档统一支持普通 C++ 与进程内共享模块消费者 | Accepted |
| [0009](0009-cvode-default-radau5-research-backend.md) | 保持 CVODE BDF 默认，增加纯 C++ Radau5 研究后端 | Accepted |

早期调研背景见 [../design/DESIGN_BASIS.md](../design/DESIGN_BASIS.md)；它不是现行证据，
其中承重事实使用前必须按当前源码重新验证。
