# external/drake_mbtree

本目录**尚为空**。它是 vendored Drake 刚性多体树与拓扑源码的落位。

## 来源与逐文件处置

上游是 Robot Locomotion Group @ CSAIL 的 Drake，许可证 BSD-3-Clause（宽松、非传染）。

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

## 分发义务（放置源码时一并完成，G18）

1. 保留上游 `LICENSE.TXT`。Drake 不写逐文件版权头，因此履约靠随附仓库级许可证，
   不给源码人工补写上游本来不存在的版权声明。
2. 逐文件清单：每个 vendored 文件可追溯到上游路径与来源 commit。
3. 修改记录：改了哪些行、为什么改。
4. `NOTICE`：BSD-3 版权声明，以及源码中标注的其他第三方许可证。

源码与二进制分发都须携带上述材料。

## 边界闸门（G19）

构建期检查：故意引入禁入头或链接 `libdrake` 必须失败。检查针对产品的实际边界，不针对
固定的文件数或符号数。
