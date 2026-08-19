# 官方源码包组装工具

`assemble_source_bundle.py` 是开发者侧的组包工具。它接收四份已经取得的官方源码归档，按
`distribution/dependencies/dependency_sources.cmake` 中的具名版本与归档文件名组装包，复制 ORVD 源码，
并从归档中提取声明的许可证材料。它只接受干净的 Git checkout、只复制 Git 已跟踪
文件，不会覆盖已有输出目录，也不会把未跟踪的机器文件、Git 元数据或编译产物带入包中。

依赖声明采用唯一的严格 CMake 记录格式，不携带数字格式编号。离线超级构建和 Python 组包器共同执行
`OrvdDependencySources.cmake`：字段缺失、重复、未知或为空，依赖集合不完整，归档／许可证路径逃逸，
以及许可证输出重名都会直接失败。许可证采用可重复、单值的 `LICENSE_PATH` 字段，不使用可吞并未知
字段的末尾变参。Python 只读取该校验器发布的私有类型化结果，不再维护第二套 JSON 解析或兼容路径。

示例：

```sh
python3 tools/package_distribution/assemble_source_bundle.py \
  --git-executable /path/to/git \
  --output-directory /tmp/orvd-source-bundle \
  --eigen-archive /path/to/eigen-3.4.0.tar.gz \
  --fmt-archive /path/to/fmt-9.1.0.tar.gz \
  --nlohmann-json-archive /path/to/nlohmann-json-3.12.0.tar.xz \
  --sundials-archive /path/to/sundials-7.7.0.tar.gz
```

`--git-executable` 可选，默认为 `git`。它同时用于确认工作树根、查看状态和列出
跟踪文件。Windows 上若工作树由 Git for Windows 创建，应显式传入该
`git.exe`，避免另一个 MSYS2 Git 用不同路径或换行规则重新解释同一工作树。

组包是开发者工作，因此使用 Python、Git 与 CMake；生成物的配置与构建只需要 CMake、平台编译工具链
和构建器，不需要 Python、Git、submodule 或网络。脚本不自行下载归档，避免把网络状态混入
来源选择与组包结果。下载时需要的一次性完整性核对在仓库外完成，组包工具不保存或重复校验归档哈希。
