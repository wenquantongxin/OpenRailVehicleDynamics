# external/drake_mbtree — vendored Drake 多体树（允许清单）

本目录**尚为空**（预实施）。M1 阶段将在此按**允许清单**放置 vendored 自 Drake 的
`multibody/tree` + `multibody/topology` 源码，仅 double 标量。

## 来源（Provenance）

| 项 | 值 |
|---|---|
| 上游 | Robot Locomotion Group @ CSAIL — Drake |
| 版本 | v1.54.0 |
| commit | `231c260201ee2f7d101a8d9ccede78626f7ca13a` |
| 许可证 | **BSD-3-Clause**（宽松、非传染） |
| 源码位置（只读参考） | `../../drake-extraction-study/drake-src` |

## 允许清单策略（vendor 边界）

**纳入**：`multibody/tree`、`multibody/topology` 的核心多体源码 + header-only `multibody/plant/constraint_specs.h`。

**排除**（实现闭包会拉回 geometry 46 / FEM 30 头的肇事者）：
- `deformable_body.{h,cc}`
- `geometry_spatial_inertia.cc`
- `element_collection.cc` 中对 `DeformableBody` 的三标量模板实例化（须修补）

**double-only 化**：删除全部 `DoCloneToScalar<AutoDiffXd | symbolic::Expression>` 重载与
`DEFAULT_SCALARS` 三标量实例化宏。

**CI 守卫**：include 禁入检查——本目录任何文件不得 `#include "drake/geometry/..."`、
`"drake/multibody/fem/..."` 或 `"drake/multibody/plant/..."`（`constraint_specs.h` 除外）。

## 分发义务（放置代码时必须补齐）

1. 保留 Drake `LICENSE.TXT` 原文于本目录。
2. `MANIFEST.md`：逐文件的 vendored 清单 + 来源 commit + 修改记录（哪些行被删/改）。
3. `NOTICE`：BSD-3 版权声明；审计源码中标注的其他宽松来源并一并列入。
4. 源码与二进制分发都须携带上述许可证与 NOTICE。

> 详细论证见 [../../docs/adr/0001-vendor-tree-behind-allowlist.md](../../docs/adr/0001-vendor-tree-behind-allowlist.md)
> 与设计基线 §2.6、§10.2。
