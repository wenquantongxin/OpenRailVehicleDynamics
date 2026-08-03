# 官方源码包组装工具

`assemble_source_bundle.py` 是开发者侧的组包工具。它接收四份已经取得的官方源码归档，按
`distribution/dependencies/dependency_sources.json` 中的具名版本与归档文件名组装包，复制 ORVD 源码，
并从归档中提取声明的许可证材料。它只接受干净的 Git checkout、只复制 Git 已跟踪
文件，不会覆盖已有输出目录，也不会把未跟踪的机器文件、Git 元数据或编译产物带入包中。

示例：

```sh
python3 tools/package_distribution/assemble_source_bundle.py \
  --output-directory /tmp/orvd-source-bundle \
  --eigen-archive /path/to/eigen-3.4.0.tar.gz \
  --fmt-archive /path/to/fmt-9.1.0.tar.gz \
  --nlohmann-json-archive /path/to/nlohmann-json-3.12.0.tar.xz \
  --sundials-archive /path/to/sundials-7.7.0.tar.gz
```

组包是开发者工作，因此使用 Python 与 Git；生成物的配置与构建只需要 CMake、平台编译工具链
和构建器，不需要 Python、Git、submodule 或网络。脚本不自行下载归档，避免把网络状态混入
来源选择与组包结果。下载时需要的一次性完整性核对在仓库外完成，组包工具不保存或重复校验归档哈希。
