# macOS：Apple silicon 源码构建与安装

本页描述 macOS 的当前源码支持边界和可复制命令。共同依赖、归档下载、组包与跨平台纪律见
[构建、安装与平台支持](README.md)。

## 支持边界

- 当前资格平台是 Apple silicon arm64；
- 首选工具链是 AppleClang＋Homebrew LLVM libomp；
- 第二工具链是 Homebrew GCC 15＋其自带 libgomp；
- 不支持 Intel Mac 或 universal2；
- 不发布预编译二进制，也不建立 macOS CI；
- 支持 Ninja／Makefile generator，不支持 Xcode generator；
- generic Release 不添加 `-mcpu=native`；本机优化必须使用独立 build tree 和 prefix。

最近实测环境为 macOS 26.6.2 arm64、AppleClang 21、Homebrew GCC 15.2、CMake 4.2.3。
AppleClang 和 GCC 的默认 Release 测试均为 `87/87`。版本号记录证据，不表示未来只能使用这一个
补丁版本。

## 安装工具链

先安装 Apple Command Line Tools；完整 Xcode 同样可以提供 `xcrun`、AppleClang 和 SDK：

```sh
xcode-select --install
```

通过 Homebrew 安装构建工具和 AppleClang 所需的 keg-only libomp：

```sh
brew install cmake ninja libomp python
```

Homebrew 的 libomp 安装入口见
[Homebrew libomp formula](https://formulae.brew.sh/formula/libomp.html)。第二工具链再安装 GCC 15：

```sh
brew install gcc@15
```

检查宿主和工具：

```sh
test "$(uname -m)" = arm64
cmake --version
ninja --version
xcrun clang++ --version
brew list --versions libomp
```

项目会以 `cmake_minimum_required(VERSION 3.24)` 拒绝更早版本；macOS 最近资格使用 CMake 4.2.3，
3.24–4.1 的 AppleClang／Homebrew libomp 自动发现组合尚未逐版本资格。Homebrew 提供的 Eigen、fmt
或 SUNDIALS 版本不是 ORVD 的资格依赖来源；仍应使用项目声明的四份固定源码归档。

## 共同路径

先按 [共同归档与组包流程](README.md#获取固定依赖归档) 生成 bundle，然后在当前 shell 设置：

```sh
export ORVD_SOURCE_ROOT=/absolute/path/OpenRailVehicleDynamics
export ORVD_WORK_ROOT=/absolute/path/orvd-work
export ORVD_BUNDLE_ROOT="$ORVD_WORK_ROOT/bundle"
```

源码、bundle、build tree 和 install prefix 应位于不同目录。AppleClang 和 GCC 也必须使用不同
build/prefix；静态库不能在 libc++ 与 libstdc++ 之间复用。ORVD 的根项目和离线超级构建在
`project()` 启用编译器前默认设置 arm64 目标架构；未显式提供 sysroot 时先采用 `SDKROOT`，否则
调用 `xcrun` 选择 macOS SDK。显式 toolchain file、sysroot 或 architecture 仍由调用者控制并接受
同一支持边界检查。

## AppleClang：推荐路径

### 完整离线超级构建

```sh
export ORVD_APPLE_CC="$(xcrun --find clang)"
export ORVD_APPLE_CXX="$(xcrun --find clang++)"
export ORVD_APPLE_BUNDLE_BUILD="$ORVD_WORK_ROOT/appleclang-bundle-build"
export ORVD_APPLE_PREFIX="$ORVD_WORK_ROOT/appleclang-prefix"

cmake -S "$ORVD_BUNDLE_ROOT" -B "$ORVD_APPLE_BUNDLE_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER="$ORVD_APPLE_CC" \
  -DCMAKE_CXX_COMPILER="$ORVD_APPLE_CXX" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_APPLE_PREFIX"

cmake --build "$ORVD_APPLE_BUNDLE_BUILD"
```

超级构建把自动解析的 SDK 和 arm64 架构传播给四个依赖和产品。嵌套 ORVD 产品识别 AppleClang
后调用 `brew --prefix libomp`，只在 `find_package(OpenMP)` 期间把该 prefix 作为末位搜索提示，
随后恢复原 `CMAKE_PREFIX_PATH`，不会让 Homebrew prefix 污染 Eigen、fmt、JSON 或 SUNDIALS 的
精确依赖搜索。配置输出应包含：

```text
Found OpenMP_CXX: -Xclang -fopenmp
Found OpenMP: TRUE
```

不要手写 `OpenMP_CXX_FLAGS`、include directory 或 `libomp.dylib` 文件路径。严格浮点启动器只在
编译器身份为 AppleClang 时允许相邻的 `-Xclang -fopenmp`，其他 `-Xclang` forwarding 仍拒绝。

### 当前源码的完整测试

bundle 的默认产品构建关闭测试。开发者应再以同一个依赖 prefix 配置当前源码：

```sh
export ORVD_APPLE_TEST_BUILD="$ORVD_WORK_ROOT/appleclang-tests"
export ORVD_APPLE_TEST_INSTALL="$ORVD_WORK_ROOT/appleclang-test-install"

cmake -S "$ORVD_SOURCE_ROOT" -B "$ORVD_APPLE_TEST_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER="$ORVD_APPLE_CXX" \
  -DCMAKE_PREFIX_PATH="$ORVD_APPLE_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_APPLE_TEST_INSTALL" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build "$ORVD_APPLE_TEST_BUILD"

OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_APPLE_TEST_BUILD" \
        --output-on-failure -j1
```

预期注册 87 项。`verify_openmp_parallel_execution` 必须观察到真实团队，
`verify_relocated_install_consumer` 必须完成安装迁移、独立 consumer 构建和运行期闭包检查。

### 下游 consumer

ORVD 安装包通过 `find_dependency(OpenMP COMPONENTS CXX)` 恢复 OpenMP，并复用同一 Homebrew
libomp 自动发现逻辑。AppleClang consumer 只需提供包含 ORVD 与四个固定依赖的安装 prefix：

```sh
cmake -S /absolute/path/to/consumer \
      -B /absolute/path/to/consumer-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER="$ORVD_APPLE_CXX" \
  -DCMAKE_PREFIX_PATH="$ORVD_APPLE_PREFIX"
cmake --build /absolute/path/to/consumer-build
```

不要把安装后的静态 ORVD 归档与另一个 C++ 标准库或 OpenMP runtime 混用。

## Homebrew GCC 15：第二工具链

Homebrew GCC 的 Darwin 驱动不会自行补上 Apple SDK。ORVD 因此在根项目和 bundle 的
`project()` 之前自动初始化 `CMAKE_OSX_SYSROOT`：保留显式值，或依次采用 `SDKROOT` 与 `xcrun`
解析结果。没有这层 bootstrap 时，compiler test 或链接会报：

```text
ld: library 'System' not found
```

设置工具链路径：

```sh
export ORVD_GCC_FORMULA_PREFIX="$(brew --prefix gcc@15)"
export ORVD_GCC_CC="$ORVD_GCC_FORMULA_PREFIX/bin/gcc-15"
export ORVD_GCC_CXX="$ORVD_GCC_FORMULA_PREFIX/bin/g++-15"
export ORVD_GCC_BUNDLE_BUILD="$ORVD_WORK_ROOT/gcc15-bundle-build"
export ORVD_GCC_PREFIX="$ORVD_WORK_ROOT/gcc15-prefix"

test -x "$ORVD_GCC_CC"
test -x "$ORVD_GCC_CXX"
```

完整离线超级构建：

```sh
cmake -S "$ORVD_BUNDLE_ROOT" -B "$ORVD_GCC_BUNDLE_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER="$ORVD_GCC_CC" \
  -DCMAKE_CXX_COMPILER="$ORVD_GCC_CXX" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_GCC_PREFIX"

cmake --build "$ORVD_GCC_BUNDLE_BUILD"
```

测试当前源码：

```sh
export ORVD_GCC_TEST_BUILD="$ORVD_WORK_ROOT/gcc15-tests"
export ORVD_GCC_TEST_INSTALL="$ORVD_WORK_ROOT/gcc15-test-install"

cmake -S "$ORVD_SOURCE_ROOT" -B "$ORVD_GCC_TEST_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER="$ORVD_GCC_CXX" \
  -DCMAKE_PREFIX_PATH="$ORVD_GCC_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_GCC_TEST_INSTALL" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build "$ORVD_GCC_TEST_BUILD"

OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_GCC_TEST_BUILD" \
        --output-on-failure -j1
```

GCC 路径应由 CMake 解析为 `-fopenmp`＋libgomp，不需要 Homebrew libomp。安装消费者测试继承
父构建自动确定的 `CMAKE_OSX_SYSROOT`，不依赖运行 `ctest` 的 shell 是否导出 `SDKROOT`。任意第三方
工程若直接以 Homebrew GCC 启用自己的首个语言，仍须在其 `project()` 前通过 toolchain file 或
等价 bootstrap 提供 SDK；ORVD 的安装 Config 在该 compiler test 之后加载，无法追溯修复它。

## generic 与本机性能构建

上述命令是 generic arm64 Release 支持证明，不加入 `-mcpu=native`。超级构建当前不传播任意
`CMAKE_CXX_FLAGS`；本机优化应在 generic bundle 已生成的依赖 prefix 上另建源码直建产品树，例如
AppleClang：

本节只维护怎样产生和验证这些构建；AppleClang／GCC 的 `8/12/16/32` worker、四工况计时和
内存结果见
[macOS Apple silicon 计算评价](../performance/platform_evaluations/MACOS_APPLE_SILICON.md)。

```sh
export ORVD_APPLE_NATIVE_BUILD="$ORVD_WORK_ROOT/appleclang-native-tests"
export ORVD_APPLE_NATIVE_PREFIX="$ORVD_WORK_ROOT/appleclang-native-prefix"

cmake -S "$ORVD_SOURCE_ROOT" -B "$ORVD_APPLE_NATIVE_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER="$ORVD_APPLE_CXX" \
  -DCMAKE_CXX_FLAGS=-mcpu=native \
  -DCMAKE_PREFIX_PATH="$ORVD_APPLE_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_APPLE_NATIVE_PREFIX" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build "$ORVD_APPLE_NATIVE_BUILD"
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_APPLE_NATIVE_BUILD" \
        --output-on-failure -j1
```

若要求四个依赖也带本机旗标，应使用一份独立 toolchain file、独立 bundle build/prefix 并重新做完整
资格，不能假定外层 `CMAKE_CXX_FLAGS` 会进入所有 `ExternalProject`。native 二进制还要重新运行
`1/4/8/12/16/32` 固定槽矩阵。generic 与 native 二进制之间不要求逐位一致；同一二进制内部仍要求
固定槽逐位一致。

本机 16 核中的 12 个性能核不等于 macOS 提供了 12 核硬 affinity。`OMP_NUM_THREADS=12` 只是线程
预算，不能证明调度器只使用性能核。32 worker 在 16 核机器上是超额订阅的正确性／容量探针，不是
本机性能默认配置。

## 已知边界与排障

- `run_qualification_with_metrics.py` 依赖 Linux `sched_setaffinity` 和 POSIX 资源统计；macOS 会
  fail closed。它不属于默认 87 项 CTest 的执行路径。
- 功能测试不要以 `OMP_PLACES`／`OMP_PROC_BIND` 为前提；Homebrew GCC 的 macOS libgomp affinity
  支持与 Linux 不同。
- `FindOpenMP` 找不到 AppleClang OpenMP 时，先检查 `brew --prefix libomp` 和其中的 `include/omp.h`
  与 `lib/libomp.dylib`；不要改写编译器旗标绕过。
- GCC 报 `library 'System' not found` 时，确认 Apple Command Line Tools 可由 `xcrun` 访问，并检查
  根构建和嵌套 `*-cfgcmd.txt` 是否收到同一 `CMAKE_OSX_SYSROOT`。
- `CMAKE_PREFIX_PATH`、SDK、compiler、build tree 或 prefix 改变后，不要继续使用另一工具链生成的
  `CMakeCache.txt`。
