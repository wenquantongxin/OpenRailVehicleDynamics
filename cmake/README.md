# cmake/

CMake 辅助模块，预实施占位。规划内容：

- `FindSUNDIALS.cmake` — CVODE 定位（wheel-rail-lab 因 SuiteSparse/KLU 链在 macOS 上 config 失败，故手写 find；须钉 7.7.0 / double / int32）。
- `orvd_warnings.cmake` — 统一告警选项；集中定义 parity 构建对 `-ffast-math` / `-march=native` 的**禁用**守卫。
- `toolchains/` — 交叉/平台工具链（`linux-gcc.cmake`、`windows-msvc.cmake`）。
- `orvd_no_drake_guard.cmake` — release 构建断言产物不链接 Drake 的辅助。

`CMAKE_MODULE_PATH` 已在顶层 CMakeLists 指向本目录。
