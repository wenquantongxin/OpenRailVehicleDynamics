# 构建、安装与平台支持

本页说明共同依赖与构建机制。平台命令见 [Linux](LINUX.md)、[Windows](WINDOWS.md) 和
[macOS](MACOS.md)。

## 平台与工具链

| 平台 | 工具链 | 注册测试 | 验证结果 |
|---|---|---:|---|
| Ubuntu 24.04 x86-64 | GCC 13.3.0＋libgomp | 105 | CMake 3.28.3、Release，`105/105` |
| Ubuntu 24.04 x86-64 | Clang 20＋LLVM libomp | 历史 87 | 历史资格快照为 `87/87` |
| Windows 10／11 x86-64 | MSYS2 UCRT64 GCC；MSYS2 CLANG64 Clang＋LLVM libomp | 99 | Windows 10 Pro，两套均 `99/99`；版本见 [Windows 手册](WINDOWS.md) |
| macOS Apple silicon arm64 | AppleClang 21＋Homebrew libomp；Homebrew GCC 15 | 历史 87 | 历史资格快照两套均为 `87/87` |

各平台的构建、测试与安装命令见对应手册。

## 共同要求

- CMake 3.24 或更高版本；
- Ninja，或使用 GNU 风格命令行的 Makefile generator；
- 支持 C++23 的已列工具链；
- 测试和组包需要 Python 3.10 或更高版本及 Git；
- Eigen 3.4.0、fmt 9.1.0、nlohmann/json 3.12.0、SUNDIALS 7.7.0；
- 能被 CMake `find_package(OpenMP COMPONENTS CXX)` 找到的 OpenMP 编译语义和运行时。

四个 C/C++ 依赖使用精确版本。OpenMP 由工具链提供：GCC 使用 libgomp，Clang／AppleClang
使用 LLVM libomp。下游需提供兼容工具链与 `OpenMP::OpenMP_CXX`。

依赖版本、归档名和上游 URL 的唯一权威是
[`distribution/dependencies/dependency_sources.cmake`](../../distribution/dependencies/dependency_sources.cmake)。
平台文档中的命令是该声明的可执行副本；两者不一致时以声明文件为准。

## CMake 如何区分平台

CMake 根据目标系统、编译器和命令行前端配置平台差异：

| CMake 条件或能力 | 用途 |
|---|---|
| `WIN32` | 拒绝 MSVC／clang-cl 风格 frontend；当前资格支持限定为 MSYS2 UCRT64 GCC 与 CLANG64 Clang；运行期依赖检查识别 Windows API-set |
| `UNIX`＋GNU frontend＋非交叉编译 | 注册两个直接调用本机构建编译器的开发期源码探针 |
| `CMAKE_SYSTEM_NAME STREQUAL Linux` | 可选 SIMPACK Realtime adapter 和 `state_stream` UDP 发送组件 |
| `CMAKE_CXX_COMPILER_ID` | 选择 GNU／Clang／AppleClang 告警策略，并识别 AppleClang OpenMP forwarding |
| `CMAKE_CXX_COMPILER_FRONTEND_VARIANT` | 区分 GNU 风格和 MSVC 风格命令行 |
| `find_package(... CONFIG REQUIRED)` | 通过 imported target 恢复 Eigen、fmt、SUNDIALS 和 JSON |
| `find_package(OpenMP REQUIRED)` | 检测编译旗标、头文件和运行时 |
| `CMAKE_OSX_*` | macOS 设置 arm64 目标架构，按显式 sysroot、`SDKROOT`、`xcrun` 的顺序选择 SDK，并传给离线子构建 |

`scene_stream` 编解码默认在各平台构建安装。

## 获取固定依赖归档

POSIX shell 下载示例，归档保存在源码树外：

```sh
export ORVD_SOURCE_ROOT=/absolute/path/OpenRailVehicleDynamics
export ORVD_WORK_ROOT=/absolute/path/orvd-work
export ORVD_ARCHIVE_ROOT="$ORVD_WORK_ROOT/archives"
export ORVD_BUNDLE_ROOT="$ORVD_WORK_ROOT/bundle"

mkdir -p "$ORVD_ARCHIVE_ROOT"

curl --fail --location \
  --output "$ORVD_ARCHIVE_ROOT/eigen-3.4.0.tar.gz" \
  https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz

curl --fail --location \
  --output "$ORVD_ARCHIVE_ROOT/fmt-9.1.0.tar.gz" \
  https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.tar.gz

curl --fail --location \
  --output "$ORVD_ARCHIVE_ROOT/nlohmann-json-3.12.0.tar.xz" \
  https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz

curl --fail --location \
  --output "$ORVD_ARCHIVE_ROOT/sundials-7.7.0.tar.gz" \
  https://github.com/LLNL/sundials/releases/download/v7.7.0/sundials-7.7.0.tar.gz
```

从干净 checkout 组装离线源码包：

```sh
python3 "$ORVD_SOURCE_ROOT/tools/package_distribution/assemble_source_bundle.py" \
  --source-root "$ORVD_SOURCE_ROOT" \
  --output-directory "$ORVD_BUNDLE_ROOT" \
  --eigen-archive "$ORVD_ARCHIVE_ROOT/eigen-3.4.0.tar.gz" \
  --fmt-archive "$ORVD_ARCHIVE_ROOT/fmt-9.1.0.tar.gz" \
  --nlohmann-json-archive \
    "$ORVD_ARCHIVE_ROOT/nlohmann-json-3.12.0.tar.xz" \
  --sundials-archive "$ORVD_ARCHIVE_ROOT/sundials-7.7.0.tar.gz"
```

组包要求源码工作区干净，输出目录位于源码树外且事先不存在。
bundle 使用所附的四份依赖归档离线构建，OpenMP 由所选工具链提供。

## 两种构建入口

### 离线 bundle

这是资格化和首次安装的推荐入口。超级构建按同一工具链依次构建四个依赖和 ORVD，并安装到一个
隔离 prefix。Linux、Windows 和 macOS 的完整命令见各自平台页。

开发时先在工作树干净时组包，构建四个 `orvd_dependency_*` 目标及 `orvd_dependency_licenses`，
再编辑并从工作树直建。bundle 保留组包时的产品快照，默认 `orvd_product` 关闭测试；
当前修改的完整 CTest 由工作树构建执行。

### 已有依赖 prefix 的源码直建

开发者已有由 bundle 产生的工具链匹配 prefix 时，可以直接配置仓库根：

```sh
cmake -S "$ORVD_SOURCE_ROOT" -B /absolute/path/to/product-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=/absolute/path/to/cxx \
  -DCMAKE_PREFIX_PATH=/absolute/path/to/dependency-prefix \
  -DCMAKE_INSTALL_PREFIX=/absolute/path/to/orvd-prefix \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF \
  -DORVD_BUILD_SIMPACK_REALTIME_TOOLS=OFF \
  -DORVD_BUILD_STATE_STREAM=OFF

cmake --build /absolute/path/to/product-build
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir /absolute/path/to/product-build \
        --output-on-failure -j1
cmake --install /absolute/path/to/product-build
```

macOS 自动选择 SDK 和 arm64 目标架构，AppleClang 从 Homebrew libomp 获取 OpenMP。
Windows 命令见 [Windows 手册](WINDOWS.md)。

使用空目录作为安装前缀。五种积分方法共用 `SystemContinuousStateAdvancer` 和
`SystemIntegrationConfiguration`，见[积分器说明](../../libs/integrators/README.md)。重定位安装测试运行八个
消费者，包含五种积分方法和场景记录读写，并逐个检查运行期依赖；Drake marker 单独作阳性对照。

## 构建树和本机配置纪律

同一份源码按工具链分别构建：

```text
同一源码快照
├── Linux GCC：独立 build + dependency/install prefix
├── Linux Clang：独立 build + dependency/install prefix
├── Windows UCRT64：独立 build + dependency/install prefix
├── Windows CLANG64：独立 build + dependency/install prefix
├── macOS AppleClang：独立 build + dependency/install prefix
└── macOS GCC：独立 build + dependency/install prefix
```

不得跨操作系统、编译器、C++ ABI 或 OpenMP runtime 复用 `CMakeCache.txt`、build tree、静态库或
install prefix。工具链或关键 CMake 逻辑变化后，用空 build tree 重新资格；普通源码增量修改可在
同一工具链树中增量构建，最终发布前仍应做一次干净配置。

机器绝对路径只进入命令行、环境变量或被 `.gitignore` 排除的 `CMakeUserPresets.json`。仓库中的
`CMakePresets.json` 只提供通用配置，默认 build 和 install 分别位于源码同级的
`<源码目录名>-build/<预设名>` 与 `<源码目录名>-install/<预设名>`。预设不准备依赖，也不选择本机
编译器；调用方必须通过命令行或 user preset 提供匹配的 compiler 和 `CMAKE_PREFIX_PATH`。
同机验证多个工具链时，为每套建立独立命名的 user preset（例如 `ucrt-release`、`clang-release`），
继承 `release` 并填写各自的 compiler、dependency prefix、binaryDir 和 installDir；不要用同一个
`release` 构建目录轮换编译器。Windows 的完整验证仍以 [Windows 手册](WINDOWS.md) 的显式命令
为入口。不要把 user preset、build 目录或依赖 prefix 提交到 Git。

## 跨平台开发

跨平台改动遵循以下约定：

1. 产品算法优先使用标准 C++23 和 imported targets；新增操作系统宏必须有真实 API／ABI 差异。
2. 工具链路径和 dependency prefix 只作 configure 输入；macOS SDK 与 AppleClang libomp 由 CMake
   自动发现。任何显式覆盖仍只进入命令行或 toolchain file，不写进项目源码。
3. 一次提交若修改公共头、数值实现、OpenMP、安装导出或 CMake，应在可用平台上运行各自完整门。
4. 分别记录配置后的测试清单、实测通过数量和验证环境，旧快照的结果标为历史记录。
5. 平台间不要求逐位一致；同一二进制的固定槽并行合同仍必须逐位一致。
6. 依赖来源 URL、归档名和版本以 `dependency_sources.cmake` 为准；升级时同步根项目与安装
   Config 的消费约束。
