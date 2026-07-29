# ADR-0001：先按允许清单 vendor Drake 刚性多体树，再逐步替换（方案 B）

- 状态：**Accepted**
- 决策日期：2026-07
- 相关：ADR-0002、ADR-0003

## 背景

复现 Drake 的多体行为是既定目标。两条候选路线：

- **方案 A**：从零自研 Featherstone 核、空间数学、力元与运行时。
- **方案 B**：先 vendor Drake 的 `multibody/tree` + `multibody/topology`（BSD-3，仅
  `double`），自写 Context 与缓存达成零 Drake 运行期依赖；之后按需逐 pass 换成自研实现。

支持方案 B 的事实：前向动力学定义在 `multibody/tree` 而非 plant；连续路径是无优化求解器
的 Featherstone 递推；刚性树的头文件层不依赖 geometry。而保真细节密集——深度优先坐标
分配、四元数不做归一化写回、ABA 的分解顺序、反射惯量的多处不同形式——自研复现这些细节
的成本远高于 vendor。

## 决策

采用**方案 B**，且 vendor 必须走**逐文件语义处置清单**，而不是整目录复制：清单在 G09
确定，闭包由现场解析工具计算（G10），并由编译探针验证（G11）。

**vendored 源码是实现基底，不是 oracle。** 它承载着正在被替换的那些修改，因此不能用来
证明自己正确。正确性由**独立的 Drake 参考进程与 ORVD 候选进程**在模型中立场景下按工程
容差比较来验证：连续量 `1e-3` 相对误差，近零量按单位声明的绝对限，旋转用 SO(3) 角度。

两端始终位于不同进程。`libdrake.so` 导出的符号与 vendored 副本同处 `namespace drake`，
同进程链接构成 ODR 违规，其最可能的症状是一次看起来通过的比较。

## 后果

- vendored 代码达成零运行期 Drake 依赖，逐 pass 自研化降级为可推迟的增量工作。
- 承担 vendored 源码的分发义务：文件清单、来源 commit、修改记录、NOTICE。
- 须消解树对 `systems/framework` 的依赖（见 ADR-0002）。
- 被否：方案 A 的大部分工作是在重新赚取 vendor 免费提供的数值保真。
