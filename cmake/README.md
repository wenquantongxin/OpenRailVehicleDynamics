# cmake/

CMake 辅助模块，尚为空。

规划中的内容由后续 Goal 建立：

- **G04**：构建入口与目标级编译策略——统一告警选项，且第一方告警不污染 vendored 或
  第三方代码；预设格式与 CMake 最低版本的一致性也在此解决。
- **G05**：Eigen 接口目标与 CTest 注册入口。其余依赖定位模块（SUNDIALS、Ceres）留到
  第一个真实消费者出现时再建立。
- **G19**：vendor 边界闸门——禁入头检查与"产品目标不链接 `libdrake`"检查。

`CMAKE_MODULE_PATH` 已在顶层 CMakeLists 指向本目录。
