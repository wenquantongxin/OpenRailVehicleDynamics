# ADR-0001：先按允许清单 vendor Drake 树，再逐 pass 自研（方案 B）

- 状态：**Accepted**（REV2 确认）
- 日期：2026-07
- 相关：ADR-0002、ADR-0003

## 背景

复现 Drake 是硬要求（C3）。两条候选路线：
- **方案 A**：从零自研 Featherstone 核 + 空间数学 + 力元 + mini-systems。
- **方案 B**：先 vendor Drake 的 `multibody/tree` + `multibody/topology`（BSD-3，合法），只按 double 标量，
  自写 Context/cache 与 facade 达成零运行期依赖；之后按需逐 pass 换自研实现。

关键事实（第一手实测，见设计基线 §2、§10）：
- RHS 定义在 `multibody/tree`（非 plant）；连续路径纯 O(n) Featherstone ABA，无优化求解器。
- `multibody_tree.h` **声明头**闭包 131 头、零 geometry；但 tree+topology 全部 63 个编译单元的**实现闭包**
  为 283 头、含 geometry 46 / FEM 30 / plant 1，肇事者仅 `deformable_body.cc`、`geometry_spatial_inertia.cc`
  与 `element_collection.cc` 的 DeformableBody 实例化。
- 保真陷阱密集（DFS 交错编号、四元数不归一化、ABA 的 LLT 分解顺序、反射惯量四处四式……），
  自研复现这些细节的成本极高。

## 决策

采用**方案 B**，但 Stage 1 必须是**允许清单 vendor**，而非"原样复制两个目录"：
排除可变形体与几何惯量源文件、修补 `element_collection.cc`、携带 header-only `constraint_specs.h`，
并以 CI 的 include 禁入检查守住边界。逐位一致**由分层预言机实测证明**，而非"由构造保证"
（后者是被 REV2 推翻的过度主张）。

## 后果

- 正面：以约 8–14 pw（Stage 1 核心）达成零运行期 Drake，且 vendored 代码同时永久充当 oracle。
  自研（M4）降级为可无限期推迟、按 pass 隔离的增量工作。
- 负面/义务：承担 vendored 源码的许可证交付物（文件清单、来源 commit、修改记录、NOTICE，见 `external/drake_mbtree/`）；
  须消解树的 `tree_system_` 反向指针（见 ADR-0002）。
- 被否：方案 A 全自研，估 41–66 pw，其中大块是在重新赚取 vendoring 免费提供的数值保真。

## 出处

设计基线 §2.1–2.6、§4.1、§10.1（A1 记分）、§10.2（新增硬约束 1）。
