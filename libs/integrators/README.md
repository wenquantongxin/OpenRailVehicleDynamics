# libs/integrators

**职责**：时间推进。抽象推进器接口 + CVODE 后端（见 ADR-0003）。

```text
include/orvd/integrators/
  advancer.h          # 抽象推进器接口（B4 七项语义：事件日历/步长夹持/挂起更新/拒步回滚/Initialize/ReInit 通知/不支持项报错）
src/
  cvode/              # 首版唯一后端：移植 cvode_driver.cc，RHS 改指自有核；SUNDIALS 外置
  drake_family/       # 预留（不实现）——radau / implicit-euler 日后按需在接口后补
```

**里程碑**：M2（后端接轨）。CVODE 驱动的大部分（容差、stats、reinit 逻辑）原样保留，只换 RHS 指向。

**必须复现的陷阱**：abstol 向量当前按位置硬编码且**已错位 43/107**——迁移期照抄冻结，
修正另行立项（设计基线 §3.1、陷阱 9）；CVODE 内部差商 Jacobian，无 `CVodeSetJacFn`；
SUNDIALS 须精确钉版 7.7.0 / double / int32。
