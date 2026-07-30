# external/drake_mbtree

本目录已落位刚性 topology、tree 与必要支撑源码。common support、topology 与 double
位姿数学已有独立构建目标；其余 tree 已完成 G16 的 `double`-only 裁剪并由 G17 界定
编译前沿，G28 建立完整 tree 目标。

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
G28 再以完整 tree 目标验证其兼容性。只有具体 API 提供证据时才声明最低版本。

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

以上四项由 `tools/drake_source_boundary/verify_landed_drake_source_provenance.py` 对着
钉死的上游现场核验：上游 HEAD 与账本一致、每个 vendored 路径在上游同路径存在、落位集合与
账本准入集合精确相等、许可证正文在位、被修改且其许可证要求改动声明的文件确实带着声明。
**不用文件哈希作身份**——哈希只说两个文件是否逐字节相同，不说任何一个来自哪里，而且它在
我们行使许可证授予的修改权那一刻就失效。

源码与二进制分发都须携带上述材料。第一方代码自身的许可证尚未选定，见仓库根 `README.md`。

## 边界闸门（G19）

构建期检查：故意引入禁入头或链接 `libdrake` 必须失败。检查针对产品的实际边界，不针对
固定的文件数或符号数。
