# cmake/

| 模块 | 内容 |
|---|---|
| `OrvdFirstPartyTargetPolicy.cmake` | `orvd_configure_first_party_target(<target>)`：按目标声明 C++23、关闭语言扩展并设置第一方告警 |
| `OrvdPlatformToolchainBootstrap.cmake` | 原生 macOS 在 `project()` 前初始化 macOS SDK 与 arm64 目标架构，配置后检查支持边界，并为 AppleClang 自动定位 Homebrew libomp；同一模块随安装包导出 |
| `OrvdProductBoundaryGate.cmake` | 递归纳管已列产品模块的目标；配置期拒绝产品对 `libdrake` 的链接依赖 |
| `OrvdStrictFloatingPointQualification.cmake` | 配置期审计严格浮点资格选项、生成器和 launcher 所有权，并为受资格化目标接入最终命令检查 |
| `OrvdStrictFloatingPointCompileLauncher.cmake` | 编译期检查完全展开的编译命令；拒绝 fast／finite-math 和不透明 forwarding，只窄允许 AppleClang 的 OpenMP 参数对 |
| `OpenRailVehicleDynamicsConfig.cmake.in` | 安装包入口：恢复精确 Eigen、fmt、SUNDIALS 与 OpenMP 依赖，再载入导出的 ORVD 目标 |

## 为什么按目标而不是全局

全局 `add_compile_options()` 或修改 `CMAKE_CXX_FLAGS` 会同时作用于 vendored Drake 源码
与外置第三方，产生我们无权修改的告警。无法处理的告警最终会被忽略，而这会连带淹没本可
处理的那些。因此策略只按目标施加，第三方通过 imported target 引入并以系统头方式处理。

告警不设为错误：在构建系统里启用 `-Werror` 会让一次上游编译器升级变成未改动代码的构建
中断。需要更严格时由 CI 在此之上叠加。

`CMAKE_MODULE_PATH` 已在顶层 CMakeLists 指向本目录。
