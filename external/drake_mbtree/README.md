# external/drake_mbtree

本目录已落位刚性 topology、tree 与必要支撑源码。五个 OBJECT owner 各自唯一编译 common
support、topology、double 位姿数学、tree + trajectories 与第一方位姿组合实现；三个窄
静态库继续服务各自消费者，内部产品目标 `orvd_rigid_multibody_tree` 则把全部 landed
对象装入一个归档。G26 已用类型化状态替换 systems 状态表面，G27 已删除 tree-system
反向指针并接入具名缓存，G28 已完成全对象链接与最小模型运行。

## 来源与逐文件处置

**确切的 commit、tag、许可证标识与每个候选文件的处置，唯一权威是
[`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt)。** 本文件不复述这些值——同一事实
写两处必然漂移，而漂移的那一份会被当成事实使用。

## 准入边界

只 vendor 刚性多体树与拓扑；产品边界仅支持 `double`，源码中的非 `double` 路径由
G16 删除。明确排除 geometry、FEM、plant、contact、solver 与 deformable。

准入边界由处置清单确定（G09），闭包由
[`tools/drake_source_boundary/`](../../tools/drake_source_boundary/) 的解析工具从源码
现场计算（G10），并由编译探针验证（G11）。**不以文件数、头文件数或符号数作为边界**
——数量是某次测量的结果，不是判据；用它当门会在上游变动时给出错误的通过。

处置词汇中 `forbidden` 与 `discard` 不可混用：前者表示违反既定架构边界，闭包一旦触达
必须失败；后者只表示当前用不到，将来可以重新裁决。

## 外置第三方的处置

准入源码需要的外部库，按实际使用裁决，不按上游依赖清单照搬：

| 第三方 | 处置 | 依据 |
|---|---|---|
| Eigen | 准入 | 每一趟多体运算都是 Eigen 算术；已由顶层 `find_package(Eigen3 3.4 CONFIG REQUIRED)` 落地 |
| fmt | 准入 | 准入源码直接 include `fmt/format.h` 与 `fmt/ranges.h`，并用 `DRAKE_FORMATTER_AS` 特化 `fmt::formatter`；G11 探针在不提供 fmt 时链接失败 |
| Abseil | 不查找、不构建 | 当前准入源码不直接 include 或使用它 |
| Highway | **不准入首版产品** | 声明头作为 vendored 文件保留；上游 Highway `.cc` 裁为 `first_party`，由 G15 的 ORVD 实现按数学定义提供四个函数，不复制上游 portable 分支 |

不锁定 fmt 版本。Drake 可以使用模块提供的 fmt，也可以经无版本的
`find_package(fmt CONFIG REQUIRED)` 使用外部 fmt；参考端的 ABI 不能外推成候选端约束。
G13 已用 topology 目标验证所配置 fmt；G17 在真实编译前沿验证 tree 源码的 fmt 消费，
G28 又以完整 tree 目标及真实消费者验证其兼容性。只有具体 API 提供证据时才声明最低版本。

`cxxabi.h` 不在此表内：它是 GNU C++ ABI 的平台头而非第三方库，include 与调用都在
`__GNUG__` 守卫内，非 GNU 前端直接返回原始 `typeid` 名称。

产品中的 vendored common support 已通过 `fmt::fmt` 显式声明该依赖，并公开传递给
topology 与 double 位姿数学目标。

## 分发义务

1. 上游 `LICENSE.TXT` 与 Apache-2.0 正文已随源码放置，准入支撑文件已有的逐文件版权与
   许可证声明原样保留。**没有声明的文件不人工补写版权头**——凭空发明是误述来源。但一个
   已经带着 Apache-2.0 的文件被我们修改时，第 4(b) 条要求的改动声明是它自身条款欠下的，
   与前一句不冲突：声明只说"我们改过"，不主张所有权。
2. 逐文件清单就是 [`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt)：它逐行记录每个
   vendored 文件的上游相对路径，头部记录上游仓库、commit 与 tag，落位布局保留上游路径。
   **不另立第二份清单**——两份必然漂移，而漂移的那份会被当成事实。
3. 修改记录见 [`DRAKE_SOURCE_MODIFICATIONS.md`](DRAKE_SOURCE_MODIFICATIONS.md)：改了哪些
   符号或构造、为什么改。任何 vendored 源码修改必须在同一次提交里更新它。
4. 仓库根的 [`NOTICE`](../../NOTICE) 记录再分发的第三方材料及其许可证正文位置，并明写它
   不授予第一方代码的许可。

来源审计工具
`tools/drake_source_boundary/verify_landed_drake_source_provenance.py` 直接读取固定 Git
对象而不信任克隆工作区，核验 commit 与 tag、每个 vendored 上游路径、落位集合、Drake
许可证原文、Apache 文件元数据及头部改动声明。修改说明是否完整解释实际差异，以及根
`NOTICE` 是否准确，仍由同次变更的人审阅，不伪装成该工具已经理解散文。
**不用冻结哈希清单作身份**——来源身份是仓库、commit 与路径；逐字比较只在审计当次用于
判断许可证原文是否保留、文件是否被修改。

源码分发须携带 vendored 源码对应的材料；二进制包按其实际包含的代码与库在 G50 重新收集
许可证和 NOTICE。第一方代码自身的许可证尚未选定，见仓库根 `README.md`。

## 边界闸门

两道闸门,分别守住边界的两侧。都不以文件数或符号数作判据——数量是某次测量的结果,不是判据。

**链接侧**由 `cmake/OrvdProductBoundaryGate.cmake` 在**配置期**把关:递归走每个产品目标的
链接闭包,发现 `drake::` 目标、名为 `libdrake…` 的库文件、或 `-ldrake` 一类链接选项即
`FATAL_ERROR`。显式库选择和整库链接同样受检；目标命名空间按词边界识别，不误伤名称里
恰好含有 `drake` 的其他目标。判据是库自身的身份，**不匹配 `/opt/drake` 这类搜索或
运行时目录**——装在意料之外位置的 Drake 恰恰是路径匹配漏掉的那种。

**哪些目标算产品目标,由顶层的 `ORVD_PRODUCT_MODULE_DIRECTORIES` 按目录决定**:已列模块
目录及其后代目录中的每一个非 imported 目标都受管。模块内新增目标无须登记;新增顶层产品
模块时必须把模块根加入该列表。产品目录中手写 imported target 必须是 `GLOBAL`,否则闸门
无法检查其真实库位置,配置会直接失败。`tests/` 不在产品模块列表中,因此 Drake 参考端可以
正常链接 Drake;闸门本身**无条件启用**,与
`ORVD_BUILD_DRAKE_REFERENCE_TESTS` 无关——那个选项开启时正是 Drake 在图中的时候,也正是
最该检查的时候。完整 tree 目标与其 OBJECT owners 都位于产品目录中，已经自动进入同一道
闸门。

**源码侧**由 `tools/drake_source_boundary/verify_product_source_drake_boundary.py` 把关:
产品源码既不得**是**禁入文件,也不得**include** 禁入文件;`forbidden_prefix` 优先于逐文件
处置,一行 `vendor` 不能为跨越架构边界背书。它**只管** `forbidden` 与 `forbidden_prefix`——
`discard`、未分类头或其他未落位但并非禁入的 include 都不由这道门判失败，那些分别是闭包
分析器与编译前沿工具的职责；当前完整 tree 的编译前沿已经无缺口。把不同问题混成一道会在
预期状况下报警的门，只会让门很快被关掉。第一方源码采用常见的头、模板实现和翻译单元后缀
时都进入扫描，不沿用只覆盖上游 Drake 现有文件形态的窄后缀集。
