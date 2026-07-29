# ORVD 分阶段规划书：难点 · 测试点 · 工时

> 基线：两轮对抗审查收敛后的 REV2（见 [../design/DESIGN_BASIS.md](../design/DESIGN_BASIS.md) §10）。
> 工时单位：人周（pw），按单名有经验 C++ 多体开发者计，已含集成与保真余量的分摊。
> 所列难点均对应设计基线中已第一手验证的代码事实。

## 总览

| 阶段 | 名称 | 工时(pw) | 核心交付 | 风险 |
|---|---|---:|---|---|
| **M0** | 基线冻结与分层预言机 | 5–8 | 冻结的 oracle 身份 + 三层金标 + 有序轨迹台架 | 中 |
| **M1** | 允许清单抽取 + 被动门面 | 6–10 | vendored double-only tree/topology 独立编译链接；门面零行为变化 | 中高 |
| **M2** | 单 Context + mini-systems + CVODE 切换 | 22–35 | **C1 达成**：零 Drake 运行期，30s 逐位 | **高（成败在此）** |
| **M3** | 外围收口与跨平台 | 13–20 | 快照/静平衡/输出契约/CLI/Linux+MSVC 打包 | 中 |
| **M0–M3 合计** | | **50–80** | 可交付的自包含软件 | |
| M4（可选，不阻塞） | 逐 pass 自研化 | +25–40 | 摆脱 vendored Drake 源码 | 按 pass 隔离 |

日历换算：单人约 12–19 个月；两人（一人主线、一人从 M2 中期起并行 M3 打包/输出契约）约 **8–12 个月**。
关键路径 M0→M1→M2 严格串行（每步验收依赖前步金标）；M3 打包/输出契约可从 M2 中期并行。

---

## M0 · 基线冻结与分层预言机（5–8 pw）

**内容**：冻结"当前 Drake 行为"的完整身份；搭差分测试台。

**难点**
1. **oracle 本体是活动目标**：`wheel-rail-lab` 工作区有大量未提交改动且被并行修改——HEAD 哈希不能标识行为，须做内容哈希清单 + 补丁快照。政治难点大于技术难点：需与 SIMPACK 对齐线协商冻结时点。
2. **启动快照不在版本库**：正式入口强制 `--startup_snapshot`（main.cc:411），而 `*.npz` 全被 gitignore——金标输入须先抢救性归档，否则新算例永远需要 Drake 才能生成。
3. **构建身份多义**：两棵活构建树 + `ninja-fastmath`/`ninja-native` 预设；parity 只能定义在关掉它们的构建上，须显式声明并在 CI 强制。
4. **有序轨迹 schema 一次定对**：阶段序号、t/q/v/外力/温启动哈希、缓存命中、CVODE 步统计——后三个阶段都消费它。

**测试点（验收门）**
- Drake 自重放**跨进程**逐位一致（同平台同构建）——台架自证。
- 三层金标就位：① 树 pass 级（序列化 q/v/参数/外力 → 各 pass 输出）；② 整车 RHS 级（x → ẋ）；③ 短时积分级（含事件 0.1–1s）。
- 冻结清单完备性检查：内容哈希、`libdrake.so` 哈希、SUNDIALS config、CPU 特性掩码、OMP/浮点环境、编译选项。
- **错误的 107 维 abstol 向量照抄冻结**（43/107 语义错位是当前行为的一部分，见设计基线 §3.1）。

---

## M1 · 允许清单抽取 + 被动门面（6–10 pw）

**内容**：把 `multibody/tree`+`topology` 按 GZ18 允许清单抽出、double-only 化、独立构建；仓库侧上被动转发门面。**不建第二套 Context**。

**难点**
1. **允许清单边界**：排除 `deformable_body.cc` / `geometry_spatial_inertia.cc`、修补 `element_collection.cc` 的 DeformableBody 三标量实例化、携带 header-only `constraint_specs.h`——手术面小，但漏一个 include 就把 geometry 46 头 / FEM 30 头拉回来。
2. **double-only 化**：删除全部 `DoCloneToScalar<AutoDiffXd|Expression>` 重载与 DEFAULT_SCALARS 宏展开——量大（69/119 文件涉及）但机械。
3. **首次独立链接是最确定的早期硬失败点**：Bazel 依赖图 translate 到 CMake 时的隐藏依赖（`common/` 的 fmt/nice_type_name 等）在此集中爆发。宁可在 M1 爆，不要拖到 M2。

**测试点**
- **include 禁入 CI**：vendored 树内任何文件不得引 geometry/FEM/plant（`constraint_specs.h` 除外）。
- vendored 库独立编译链接，产物不含任何 drake 库。
- **逐 pass 位回归**：vendored pass vs Drake pass，在 M0 序列化输入上逐位一致（此时代码相同，应平凡通过——不通过即说明抽取改变了什么）。
- 门面中性：仓库 GZ18 仿真全部改走门面（仍链 Drake），结果与 M0 金标逐位一致。
- 许可证交付物：vendored 文件清单、来源 commit、修改记录、NOTICE。

---

## M2 · 单 Context + mini-systems + CVODE 切换（22–35 pw）

**内容**：项目的心脏。自写单一权威 Context（根持有 t/x/参数/输入版本，树与子系统均为零复制视图）、15 处 `DeclareCacheEntry` 替换、`tree_system_` 反向指针消解、12 特性 mini-LeafSystem 层、B4 七项语义的事件调度器、CVODE 后端接轨，摘除 Drake。

**难点**（按杀伤力排序）
1. **`tree_system_` 反向指针消解**：树经它求值缓存（multibody_tree.h:2211+），替换必须保证**求值次数与顺序不变**——接触求解带温启动提示，次数可观测，错一次整条轨迹漂移。
2. **缓存语义**：12 项状态缓存每 RHS 失效、**3 项纯参数缓存跨调用复用**（参数变更才重算）——"每步全算"会直接改变数值路径。
3. **事件语义**：仓库 CVODE 线是"先更新后发布"（publish 见 x⁺），与 Drake AdvanceTo（publish 见 x⁻）不等价——**复现目标是前者**，B4 七项（同刻事件稳定序、试算/提交/回滚、ReInit 通知、失败传播、不支持项显式报错）逐项落地。
4. **向量外状态的事务性**：CVODE 拒步时，温启动提示、PID 积分/抗饱和累加器必须回滚——普通成员变量不会自己回滚。
5. **DFS 交错坐标分配**：浮动与转动关节交错（wheelset_ff 21/18、rev 28,29/24,25…），照抄 `multibody/topology` 可保；自写任何辅助索引时别按"浮动排完再排转动"的直觉走——abstol 教训在前。

**测试点**
- **有序轨迹 parity**（主指标）：阶段序列、接触求解次数与顺序、缓存命中记录与 M0 逐位对齐；调用计数只作辅助。
- 短时（1s）→ 30s 正式算例，对 M0 金标**逐位一致**（冻结 abstol、冻结快照输入）。
- **零 Drake 判据**：链接图 + `readelf`/符号扫描 + 干净环境安装测试，不以 `ldd` 单证；libdrake/mosek/tbb/lcm 全部消失。
- 语义单元测试：注入强制拒步 → 断言提示/PID 状态回滚；同刻多事件稳定序；参数缓存"参数变才重算、状态变不重算"。
- witness/逐步事件/监视器：显式报"不支持"，有测试锁住。

---

## M3 · 外围收口与跨平台（13–20 pw）

**内容**：静平衡 re-host（Ceres 外置）、快照生成/加载（schema 6）、输出契约、CLI、资产哈希、Linux+MSVC 打包与 CI。

**难点**
1. **快照双向兼容**：无 Drake 生成新快照 + 加载旧快照逐位——快照是每个正式跑的初始条件，差一点就是全程无法归因的常量偏移。
2. **静平衡数值身份**：Ceres `relative_step_size=1e-6` / `sqrt(eps)` 下限、DENSE_QR，且 as-built libceres 链着 LAPACK——重建的 Ceres 若切到 Eigen dense 后端，t=0 就变；须钉死后端。
3. **MSVC 是未开垦地**：现构建零 Windows 处理；Highway 指令集分派、符号可见性、SUNDIALS 精确钉版（7.7.0/double/int32）、OpenMP MSVC 方言都在此第一次见面。跨平台验收按 S2 只到容差回归，不追逐位。
4. **输出契约**：CSV/NPZ/JSON 的列序、命名、采样时刻、异常态字段——只比状态向量会漏。

**测试点**
- 快照：新生成（Drake-free）与旧快照加载，同平台逐位；schema/配置哈希校验通过。
- 静平衡：t0 状态对冻结基线同平台零差。
- 输出契约：对基线做字段级（必要处字节级）比对。
- MSVC：构建+运行+容差回归；OMP 1/2/4/8 线程逐位试验；parity CI 强制 fastmath/native 关闭。
- 干净机器安装测试。

---

## M4（可选，不阻塞 C1）· 逐 pass 自研化（+25–40 pw）

**内容**：在冻结接口后，把 vendored pass 逐个换成自研实现，每 pass 独立门控、可回滚。推荐顺序：空间代数/惯量 → mobilizer → RNEA/质量阵 → ABA 最后。

**难点**：全是已验证的数值细节——ABA 的 LLT 分解顺序与 `triangularView<Lower>`、反射惯量四处四式、Mitiguy 半角 RPY、`RotationMatrix::ToQuaternion` 的 w≥0 规范化、四元数不归一化写回与 N⁺ 伪逆、`fast_pose_composition` 的 SIMD/可移植双路径。（清单见设计基线 §8）

**测试点**：每 pass 序列化输入位回归 + 30s 轨迹零漂移预算；任一不达标回滚该 pass，不影响 C1。

---

## 摆动因子（会实质改变总数的决策）

| 决策 | 影响 |
|---|---|
| M2 验收从逐位放宽到工程容差 | **−30~40%**（CodeX 与工作流两轮独立同向折减） |
| MSVC 推迟到 C1 之后 | M3 减 5–8 pw |
| 普通图 / RHS 性能图只支持一套进 v1 | ±3–5 pw（两套整车图功能矩阵是 REV2 新增义务） |
| 旧快照只重生成、不做加载兼容 | ±2–4 pw |

**最大单点风险**始终是 M2 的"求值次数与顺序不变"——所以 M0 的有序轨迹台架值得做扎实，它是后面每个阶段的裁判。
建议在 M1 末加一个约一周的**探针里程碑**：用 vendored 树 + 临时粗糙 Context 跑通单步 RHS 并对金标，提前暴露 M2 的未知未知。
