# external/drake_mbtree

本目录尚未放入 vendored Drake 源码；当前只有边界与来源说明。

## 来源与逐文件处置

**确切的 commit、tag、许可证标识与每个候选文件的处置，唯一权威是
[`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt)。** 本文件不复述这些值——同一事实
写两处必然漂移，而漂移的那一份会被当成事实使用。

## 准入边界

只 vendor 刚性多体树与拓扑，仅 `double` 标量。明确排除 geometry、FEM、plant、
contact、solver 与 deformable。

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
| Highway | **不准入首版产品** | 在当前准入闭包中，它只经 `math/fast_pose_composition_functions` 进入，而该文件已裁为 `first_party`；四个位姿组合函数由 G15 按数学定义独立实现，不复制上游的 portable 分支 |

不锁定 fmt 版本。Drake 可以使用模块提供的 fmt，也可以经无版本的
`find_package(fmt CONFIG REQUIRED)` 使用外部 fmt；参考端的 ABI 不能外推成候选端约束。
G13/G15/G17 用真实源码验证所配置 fmt 的兼容性；只有具体 API 提供证据时才声明最低版本。

`cxxabi.h` 不在此表内：它是 GNU C++ ABI 的平台头而非第三方库，include 与调用都在
`__GNUG__` 守卫内，非 GNU 前端直接返回原始 `typeid` 名称。

产品中 vendored topology 目标对 fmt 的显式查找与直接链接在 G13 落地。此刻建空目标或
选项，就是 G05 已经拒绝过的那种占位技术债。

## 分发义务（放置源码时一并完成，G18）

1. 保留上游 `LICENSE.TXT`，并原样保留准入支撑文件已有的逐文件版权与许可证声明；
   tree/topology 候选文件没有的声明不人工补写。
2. 逐文件清单：每个 vendored 文件可追溯到上游路径与来源 commit。
3. 修改记录：改了哪些行、为什么改。
4. `NOTICE`：仓库级 BSD-3 版权声明、清单记录的 Apache-2.0 支撑文件，以及其他实际
   进入产品的第三方许可证。

源码与二进制分发都须携带上述材料。

## 边界闸门（G19）

构建期检查：故意引入禁入头或链接 `libdrake` 必须失败。检查针对产品的实际边界，不针对
固定的文件数或符号数。
