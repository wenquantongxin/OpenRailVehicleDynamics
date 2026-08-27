# Linux／Ubuntu：源码构建与安装

本页给出 Linux 的资格化构建路径。共同依赖、归档下载、CMake 平台机制和跨平台纪律见
[构建、安装与平台支持](README.md)。最近完整验证宿主为 Ubuntu 24.04 x86-64；这不是对所有 Linux
发行版和版本的无条件承诺。

## 支持边界

- GCC 13＋libgomp；
- Clang 20＋LLVM libomp；
- CMake 3.24 或更高版本；
- Ninja／Makefile generator；
- 默认产品和测试不需要 SIMPACK；可选 SIMPACK Realtime adapter 只支持 Linux；
- 默认配置关闭 Drake reference tests，注册 87 项 CTest。

## 安装工具链

Ubuntu GCC 示例：

```sh
sudo apt update
sudo apt install git python3 cmake ninja-build gcc-13 g++-13
```

Clang 构建还需要同一主版本的 LLVM libomp：

```sh
sudo apt install clang-20 llvm-20-dev libomp-20-dev
```

不同 Ubuntu 版本的默认仓库未必同时提供这些主版本。Clang 20 可从发行版仓库或
[LLVM 官方 APT 仓库](https://apt.llvm.org/) 取得；无论来源如何，都必须保证 clang、头文件和
LLVM libomp 属于同一工具链，不与 GCC libgomp 混用。

检查版本：

```sh
cmake --version
ninja --version
gcc-13 --version
g++-13 --version
clang-20 --version
python3 --version
```

## 共同路径和离线 bundle

按 [共同归档与组包流程](README.md#获取固定依赖归档) 生成 bundle，然后设置：

```sh
export ORVD_SOURCE_ROOT=/absolute/path/OpenRailVehicleDynamics
export ORVD_WORK_ROOT=/absolute/path/orvd-work
export ORVD_BUNDLE_ROOT="$ORVD_WORK_ROOT/bundle"
```

GCC 与 Clang 必须各用自己的 dependency prefix、product build、test build 和 install prefix。

## GCC 13

### 完整离线超级构建

```sh
export ORVD_GCC_BUNDLE_BUILD="$ORVD_WORK_ROOT/linux-gcc13-bundle-build"
export ORVD_GCC_PREFIX="$ORVD_WORK_ROOT/linux-gcc13-prefix"

cmake -S "$ORVD_BUNDLE_ROOT" -B "$ORVD_GCC_BUNDLE_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc-13 \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_INSTALL_PREFIX="$ORVD_GCC_PREFIX"

cmake --build "$ORVD_GCC_BUNDLE_BUILD"
```

### 当前源码完整测试

```sh
export ORVD_GCC_TEST_BUILD="$ORVD_WORK_ROOT/linux-gcc13-tests"
export ORVD_GCC_TEST_INSTALL="$ORVD_WORK_ROOT/linux-gcc13-test-install"

cmake -S "$ORVD_SOURCE_ROOT" -B "$ORVD_GCC_TEST_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_PREFIX_PATH="$ORVD_GCC_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_GCC_TEST_INSTALL" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build "$ORVD_GCC_TEST_BUILD"

OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_GCC_TEST_BUILD" \
        --output-on-failure -j1
```

预期 `87/87`。额外查看真实 OpenMP 团队探针：

```sh
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_GCC_TEST_BUILD" \
        -R '^verify_openmp_parallel_execution$' -V
```

输出必须表明 pragma 形成了真实多线程团队，而不是只成功链接 libgomp。

## Clang 20＋LLVM libomp

使用完全独立的目录：

```sh
export ORVD_CLANG_BUNDLE_BUILD="$ORVD_WORK_ROOT/linux-clang20-bundle-build"
export ORVD_CLANG_PREFIX="$ORVD_WORK_ROOT/linux-clang20-prefix"

cmake -S "$ORVD_BUNDLE_ROOT" -B "$ORVD_CLANG_BUNDLE_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=clang-20 \
  -DCMAKE_CXX_COMPILER=clang++-20 \
  -DCMAKE_INSTALL_PREFIX="$ORVD_CLANG_PREFIX"

cmake --build "$ORVD_CLANG_BUNDLE_BUILD"
```

如果 LLVM 安装在非系统默认 prefix，且已安装提供 `llvm-config-20` 的开发包，可把
`llvm-config-20 --prefix` 的结果作为外层
`CMAKE_PREFIX_PATH`；超级构建只把该提示传播给 ORVD 产品的 package discovery：

```sh
export ORVD_LLVM_PREFIX="$(llvm-config-20 --prefix)"
```

然后在配置命令中增加：

```text
-DCMAKE_PREFIX_PATH="$ORVD_LLVM_PREFIX"
```

当前源码测试：

```sh
export ORVD_CLANG_TEST_BUILD="$ORVD_WORK_ROOT/linux-clang20-tests"
export ORVD_CLANG_TEST_INSTALL="$ORVD_WORK_ROOT/linux-clang20-test-install"

cmake -S "$ORVD_SOURCE_ROOT" -B "$ORVD_CLANG_TEST_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=clang++-20 \
  -DCMAKE_PREFIX_PATH="$ORVD_CLANG_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_CLANG_TEST_INSTALL" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build "$ORVD_CLANG_TEST_BUILD"

OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_CLANG_TEST_BUILD" \
        --output-on-failure -j1
```

如果 libomp 位于非默认 prefix，测试配置的 `CMAKE_PREFIX_PATH` 应同时包含 ORVD 依赖 prefix 和
LLVM prefix，例如：

```sh
"-DCMAKE_PREFIX_PATH=/path/to/linux-clang20-prefix;/path/to/llvm-20"
```

不要让 Clang 构建加载 libgomp，也不要让 GCC 构建加载 LLVM libomp。

## 安装与下游 consumer

超级构建已经把 ORVD 安装到工具链专属 prefix。源码直建可显式安装：

```sh
cmake --install "$ORVD_GCC_TEST_BUILD"
```

下游工程使用同一 ABI 工具链和 prefix：

```sh
cmake -S /absolute/path/to/consumer \
      -B /absolute/path/to/consumer-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=g++-13 \
  "-DCMAKE_PREFIX_PATH=$ORVD_GCC_TEST_INSTALL;$ORVD_GCC_PREFIX"
cmake --build /absolute/path/to/consumer-build
```

ORVD package config 会恢复 fmt、SUNDIALS、Eigen 和 OpenMP 的安装依赖。不要跨未经资格化的
compiler、C++ runtime 或 OpenMP runtime 复用静态 prefix。Clang 源码直建的 consumer 同样需要同时
提供 test-install 与 dependency prefix；若 LLVM libomp 使用非默认 prefix，再把
`$ORVD_LLVM_PREFIX` 追加为第三项。

## Linux 并行与性能资格

功能测试只固定 `OMP_NUM_THREADS` 和 `OMP_DYNAMIC`。正式性能实验还应冻结 placement、binding 和
active levels，例如：

```sh
taskset -c 0-11 \
env OMP_NUM_THREADS=12 \
    OMP_DYNAMIC=FALSE \
    OMP_PLACES=cores \
    OMP_PROC_BIND=spread \
    OMP_MAX_ACTIVE_LEVELS=1 \
./program
```

多 NUMA 节点机器可改用 `numactl --physcpubind=... --membind=...`。不要同时依赖未记录的 shell
环境；资格工件必须保存实际 OpenMP 环境和 CPU affinity。

`tools/dynamics_qualification/run_qualification_with_metrics.py` 是 Linux research wrapper，会使用
POSIX child resource accounting 和 `sched_setaffinity`。它不代表 Windows/macOS 的功能构建入口。

## 可选 SIMPACK Realtime adapter

该工具默认关闭，只在确有本机 SIMPACK 2021x Realtime SDK 时启用：

```sh
cmake -S "$ORVD_SOURCE_ROOT" -B /absolute/path/to/simpack-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_PREFIX_PATH="$ORVD_GCC_PREFIX" \
  -DBUILD_TESTING=OFF \
  -DORVD_BUILD_SIMPACK_REALTIME_TOOLS=ON \
  -DORVD_SIMPACK_ROOT=/absolute/path/to/Simpack-2021x

cmake --build /absolute/path/to/simpack-build
```

CMake 在非 Linux 宿主会直接拒绝这一选项。默认 ORVD 产品、安装包和 87 项测试不依赖 SIMPACK。
