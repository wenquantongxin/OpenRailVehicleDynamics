# external/drake_mbtree

本目录**尚为空**。它是 vendored Drake 刚性多体树与拓扑源码的落位。

## 来源

| 项 | 值 |
|---|---|
| 上游 | Robot Locomotion Group @ CSAIL — Drake |
| 版本 | v1.54.0 |
| commit | `231c260201ee2f7d101a8d9ccede78626f7ca13a` |
| 许可证 | BSD-3-Clause（宽松、非传染） |

## 准入边界

只 vendor 刚性多体树与拓扑，仅 `double` 标量。明确排除 geometry、FEM、plant、
contact、solver 与 deformable。

准入边界由**逐文件语义处置清单**确定（G09），闭包由现场解析工具从源码计算（G10），
并由编译探针验证（G11）。**不以文件数、头文件数或符号数作为边界**——数量是某次测量
的结果，不是判据；用它当门会在上游变动时给出错误的通过。

## 分发义务（放置源码时一并完成，G18）

1. 保留上游 `LICENSE.TXT`。
2. 逐文件清单：每个 vendored 文件可追溯到上游路径与来源 commit。
3. 修改记录：改了哪些行、为什么改。
4. `NOTICE`：BSD-3 版权声明，以及源码中标注的其他第三方许可证。

源码与二进制分发都须携带上述材料。

## 边界闸门（G19）

构建期检查：故意引入禁入头或链接 `libdrake` 必须失败。检查针对产品的实际边界，不针对
固定的文件数或符号数。
