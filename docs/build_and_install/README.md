# 构建、安装与平台支持

本目录是 ORVD 面向使用者和 code agent 的构建入口。共同依赖、CMake 机制和跨平台纪律只在本页
维护；操作系统命令分别进入 [Linux](LINUX.md)、[Windows](WINDOWS.md) 和
[macOS](MACOS.md)，避免把一台机器的绝对路径或包管理器假设带到另一台机器。

## 当前支持面与证据

| 平台 | 当前支持的工具链 | 默认注册测试 | 最近完整实测与当前状态 |
|---|---|---:|---|
| Ubuntu 24.04 x86-64 | GCC 13；Clang 20＋LLVM libomp | 87 | 最近两套工具链均为 `87/87`；本轮 macOS/CMake 收口后待复验 |
| Windows 10／11 x86-64 | MSYS2 UCRT64 GCC；MSYS2 CLANG64 Clang＋LLVM libomp | 85 | 当前注册数为 85；本次 macOS 收口后仍需在 Windows 复验 |
| macOS Apple silicon arm64 | AppleClang 21＋Homebrew libomp；Homebrew GCC 15 | 87 | 两套工具链均为 `87/87` |

这里把“支持边界”“当前 CMake 会注册多少测试”和“最近在哪台机器实测”分开记录。没有运行某平台
就不能把推导出的测试数写成已通过。当前不支持原生 MSVC、clang-cl、Intel Mac、universal2、
Xcode generator 或 Visual Studio generator；项目只发布源码，不发布预编译二进制。

## 共同要求

- 项目 CMake 语法下限为 3.24；各平台最近资格使用的版本见对应手册；
- Ninja，或使用 GNU 风格命令行的 Makefile generator；
- 支持 C++23 的已列工具链；
- 测试和组包需要 Python 3.10 或更高版本及 Git；
- Eigen 3.4.0、fmt 9.1.0、nlohmann/json 3.12.0、SUNDIALS 7.7.0；
- 能被 CMake `find_package(OpenMP COMPONENTS CXX)` 找到的 OpenMP 编译语义和运行时。

四个 C/C++ 依赖使用精确版本。OpenMP 属于编译器工具链，不是第五份源码归档：GCC 使用
libgomp，Clang／AppleClang 使用 LLVM libomp。ORVD 安装包不会复制 OpenMP 动态运行时；下游
consumer 必须使用兼容工具链，并能再次解析 `OpenMP::OpenMP_CXX`。

依赖版本、归档名和上游 URL 的唯一权威是
[`distribution/dependencies/dependency_sources.cmake`](../../distribution/dependencies/dependency_sources.cmake)。
平台文档中的命令是该声明的可执行副本；两者不一致时以声明文件为准。

## CMake 如何区分平台

ORVD 不复制三套目标图，也不在产品算法中按操作系统维护三份实现。CMake 先识别目标系统、编译器身份
和命令行前端，再只在确有不同语义的边界处分支：

| CMake 条件或能力 | 用途 |
|---|---|
| `WIN32` | 拒绝 MSVC／clang-cl 风格 frontend；当前资格支持限定为 MSYS2 UCRT64 GCC 与 CLANG64 Clang；运行期依赖检查识别 Windows API-set |
| `UNIX`＋GNU frontend＋非交叉编译 | 注册两个直接调用本机构建编译器的开发期源码探针 |
| `CMAKE_SYSTEM_NAME STREQUAL Linux` | 可选 SIMPACK Realtime adapter 只在 Linux 构建 |
| `CMAKE_CXX_COMPILER_ID` | 选择 GNU／Clang／AppleClang 告警策略，并识别 AppleClang OpenMP forwarding |
| `CMAKE_CXX_COMPILER_FRONTEND_VARIANT` | 区分 GNU 风格和 MSVC 风格命令行，不根据操作系统猜编译器 |
| `find_package(... CONFIG REQUIRED)` | 通过 imported target 恢复 Eigen、fmt、SUNDIALS 和 JSON |
| `find_package(OpenMP REQUIRED)` | 以能力检测得到编译旗标、头文件和运行时，不在源码中手写 |
| `CMAKE_OSX_*` | 原生 macOS 在启用编译器前设置 arm64 目标架构，并按显式 sysroot、`SDKROOT`、`xcrun` 的顺序选择 macOS SDK，再传给离线子构建；Windows／Linux 不会收到这些参数 |

第一方产品代码没有 `_WIN32`、`__APPLE__` 或 `__linux__` 算法分支。少数平台条件位于构建、测试
夹具或可选工具边界；这比让每次平台开发都改一遍数值实现更容易审查。

## 获取固定依赖归档

POSIX shell 可使用以下示例。目录必须在源码树外；第三份归档在下载时直接保存为声明名称，不能
保留上游的 `json.tar.xz` 文件名。

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

组包器要求可见的 tracked 和 untracked 状态都干净，输出目录必须位于源码树外且事先不存在。生成
bundle 后，四份固定归档不联网、不更新、也不回退到系统同名开发包；OpenMP 是明确的工具链依赖，
仍由 CMake 从所选 compiler/runtime 环境发现。仓库根 `tmp/` 已被 Git 忽略，
短期放归档不会污染状态；长期工作目录仍推荐使用仓库外的 `orvd-work/archives`。

## 两种构建入口

### 离线 bundle

这是资格化和首次安装的推荐入口。超级构建按同一工具链依次构建四个依赖和 ORVD，并安装到一个
隔离 prefix。Linux、Windows 和 macOS 的完整命令见各自平台页。

### 已有依赖 prefix 的源码直建

开发者已有由 bundle 产生的工具链匹配 prefix 时，可以直接配置仓库根：

```sh
cmake -S "$ORVD_SOURCE_ROOT" -B /absolute/path/to/product-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=/absolute/path/to/cxx \
  -DCMAKE_PREFIX_PATH=/absolute/path/to/dependency-prefix \
  -DCMAKE_INSTALL_PREFIX=/absolute/path/to/orvd-prefix \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build /absolute/path/to/product-build
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir /absolute/path/to/product-build \
        --output-on-failure -j1
cmake --install /absolute/path/to/product-build
```

macOS 根项目会自动选择 macOS SDK并设置 arm64 目标架构，AppleClang 会从已安装的 Homebrew libomp 公式恢复 OpenMP；
Windows 的 compiler、Git 和路径规则仍不同，不能把这段概念命令直接当作平台页的替代。

## 构建树和本机配置纪律

同一 Git commit 应形成彼此隔离的构建：

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
`CMakePresets.json` 只提供通用目标；每台机器可在自己的 user preset 中继承它并填写 compiler、
dependency prefix、install prefix，以及必要时的显式工具链覆盖。不要把 user preset、build 目录或
依赖 prefix 提交到 Git。

## 防止跨平台反复修改源码

平台文档和条件化 CMake 可以消除“到一台机器就改一次绝对路径或编译旗标”，但没有运行过的平台
无法凭文档获得数学保证。当前不建立 macOS CI，因此采用以下纪律：

1. 产品算法优先使用标准 C++23 和 imported targets；新增操作系统宏必须有真实 API／ABI 差异。
2. 工具链路径和 dependency prefix 只作 configure 输入；macOS SDK 与 AppleClang libomp 由 CMake
   自动发现。任何显式覆盖仍只进入命令行或 toolchain file，不写进项目源码。
3. 一次提交若修改公共头、数值实现、OpenMP、安装导出或 CMake，应在可用平台上运行各自完整门。
4. Windows 当前快照未复验时明确写“待复验”，不能把推导出的 85 项冒充 `85/85`。
5. 平台间不要求逐位一致；同一二进制的固定槽并行合同仍必须逐位一致。
6. 依赖来源 URL、归档名和版本决策以 `dependency_sources.cmake` 为权威；升级时同步根项目与安装
   Config 的精确消费约束，平台文档不建立另一套版本裁决。
7. 若将来需要远端自动阻止回归，可增加 Linux＋Windows CI matrix；macOS 仍可维持人工资格，除非
   项目负责人改变“不建立 macOS CI”的决定。

这意味着 Windows／Linux 拉取包含 macOS 支持的提交后，应只需要选择本平台工具链和新建本平台
build tree，不应修改产品源码“改回去”。若确实需要修改源码，应该被当作可移植性缺陷修复，并由
所有平台共同保留，而不是形成来回翻转的补丁。
