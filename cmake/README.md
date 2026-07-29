# cmake/

| 模块 | 内容 |
|---|---|
| `OrvdFirstPartyTargetPolicy.cmake` | `orvd_configure_first_party_target(<target>)`：按目标声明 C++23 与第一方告警 |

## 为什么按目标而不是全局

全局 `add_compile_options()` 或修改 `CMAKE_CXX_FLAGS` 会同时作用于 vendored Drake 源码
与外置第三方，产生我们无权修改的告警。无法处理的告警最终会被忽略，而这会连带淹没本可
处理的那些。因此策略只按目标施加，第三方通过 imported target 引入并以系统头方式处理。

告警不设为错误：在构建系统里启用 `-Werror` 会让一次上游编译器升级变成未改动代码的构建
中断。需要更严格时由 CI 在此之上叠加。

## 由后续 Goal 建立

- **G05**：Eigen 与 CTest 地基。其余依赖定位模块（SUNDIALS、Ceres）留到第一个真实
  消费者出现时再建立。
- **G19**：vendor 边界闸门——禁入头检查与"产品目标不链接 `libdrake`"检查。

`CMAKE_MODULE_PATH` 已在顶层 CMakeLists 指向本目录。
