# Windows 10／11：MSYS2 源码构建与安装

本页描述当前 Windows 支持面。共同依赖、CMake 机制和跨平台纪律见
[构建、安装与平台支持](README.md)。Windows 构建使用 MSYS2 的 GNU 风格工具链，不使用 Visual
Studio generator、MSVC 或 clang-cl。

## 支持边界

| MSYS2 环境 | C++ 编译器 | OpenMP runtime |
|---|---|---|
| UCRT64 | GCC／g++ | libgomp |
| CLANG64 | clang／clang++ | LLVM libomp |

从 MSYS2 安装目录中的 `ucrt64.exe` 或 `clang64.exe` 启动对应官方 shell。不要从 Git Bash 再启动
另一个 `bash.exe`，也不要在一套 build tree 或 prefix 中混用两个环境。

当前源码默认应注册 85 项测试：默认关闭四项 Drake reference tests，并且 Windows 不注册两个只
服务 UNIX GNU 命令行的开发源码探针。本次 macOS 收口后尚未在 Windows 实机重跑，因此这里记录
“当前注册数 85”，不把它冒充未经运行的 `85/85`；合并或发布前应在两套 MSYS2 工具链复验。

## 安装工具链

在 UCRT64 shell：

```sh
pacman -S --needed git \
  mingw-w64-ucrt-x86_64-gcc \
  mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-ninja \
  mingw-w64-ucrt-x86_64-python
```

在 CLANG64 shell：

```sh
pacman -S --needed git \
  mingw-w64-clang-x86_64-clang \
  mingw-w64-clang-x86_64-llvm-openmp \
  mingw-w64-clang-x86_64-cmake \
  mingw-w64-clang-x86_64-ninja \
  mingw-w64-clang-x86_64-python
```

检查当前 shell 没有混入另一环境。UCRT64 shell：

```sh
command -v cmake ninja python git
command -v gcc g++
```

CLANG64 shell：

```sh
command -v cmake ninja python git
command -v clang clang++
```

只检查当前环境实际应有的一组 compiler。CMake、Ninja、Python 和 compiler 路径应落在同一 MSYS2
环境；GCC 构建不得加载 LLVM libomp，Clang 构建不得加载 libgomp。

## 路径和 Git 规则

源码、bundle、build tree 和 prefix 使用短的 ASCII 物理路径，并始终彼此分离，例如：

```text
C:/src/OpenRailVehicleDynamics
C:/orvd-work/archives
C:/orvd-work/bundle
C:/orvd-work/ucrt-build
C:/orvd-work/ucrt-prefix
C:/orvd-work/clang-build
C:/orvd-work/clang-prefix
```

不用目录联接隐藏过长路径。一份工作树只由一个 Git 解释跟踪状态：

- 如果 checkout 由 Git for Windows 创建，组包和测试继续显式使用同一个
  `C:/Program Files/Git/cmd/git.exe`；
- 如果 checkout 由 MSYS2 Git 创建，后续统一使用 `/usr/bin/git`；
- 不要交替使用两套 Git 解释换行、safe-directory 和 untracked 状态。

Git for Windows 在 MSYS2 shell 中按 MSYS2 的 home 解析全局配置。若遇到 dubious ownership／
safe-directory，必须在该 shell 实际读取的配置中登记同一物理路径。更详细的 bundle 机制说明见
[`distribution/dependencies/README.md`](../../distribution/dependencies/README.md)。

## 获取归档并组装 bundle

四个文件名必须精确为：

```text
eigen-3.4.0.tar.gz
fmt-9.1.0.tar.gz
nlohmann-json-3.12.0.tar.xz
sundials-7.7.0.tar.gz
```

URL 以 [`dependency_sources.cmake`](../../distribution/dependencies/dependency_sources.cmake) 为权威。
第三份上游下载名是 `json.tar.xz`，保存时必须改成上述声明名称。

在使用 Git for Windows checkout 的 MSYS2 shell 中组包：

```sh
python tools/package_distribution/assemble_source_bundle.py \
  --source-root C:/src/OpenRailVehicleDynamics \
  --git-executable "C:/Program Files/Git/cmd/git.exe" \
  --output-directory C:/orvd-work/bundle \
  --eigen-archive C:/orvd-work/archives/eigen-3.4.0.tar.gz \
  --fmt-archive C:/orvd-work/archives/fmt-9.1.0.tar.gz \
  --nlohmann-json-archive C:/orvd-work/archives/nlohmann-json-3.12.0.tar.xz \
  --sundials-archive C:/orvd-work/archives/sundials-7.7.0.tar.gz
```

若工作树从始至终使用 MSYS2 Git，可省略 `--git-executable`。组包器要求 checkout 没有可见的
tracked/untracked 改动，且输出目录事先不存在。生成 bundle 后，普通构建不需要 Git、Python 或网络。

## UCRT64 GCC

### 完整离线超级构建

在 UCRT64 shell：

```sh
cmake -S C:/orvd-work/bundle -B C:/orvd-work/ucrt-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc \
  -DCMAKE_CXX_COMPILER=g++ \
  -DCMAKE_INSTALL_PREFIX=C:/orvd-work/ucrt-prefix

cmake --build C:/orvd-work/ucrt-build
```

### 当前源码测试

```sh
cmake -S C:/src/OpenRailVehicleDynamics \
      -B C:/orvd-work/ucrt-tests -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=g++ \
  -DCMAKE_PREFIX_PATH=C:/orvd-work/ucrt-prefix \
  -DCMAKE_INSTALL_PREFIX=C:/orvd-work/ucrt-test-install \
  -DGIT_EXECUTABLE="C:/Program Files/Git/cmd/git.exe" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build C:/orvd-work/ucrt-tests

OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir C:/orvd-work/ucrt-tests \
        --output-on-failure -j1
```

若 checkout 使用 MSYS2 Git，把 `GIT_EXECUTABLE` 改为 `/usr/bin/git`。

## CLANG64 Clang＋LLVM libomp

使用完全独立的目录，在 CLANG64 shell：

```sh
cmake -S C:/orvd-work/bundle -B C:/orvd-work/clang-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=clang \
  -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_INSTALL_PREFIX=C:/orvd-work/clang-prefix

cmake --build C:/orvd-work/clang-build

cmake -S C:/src/OpenRailVehicleDynamics \
      -B C:/orvd-work/clang-tests -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_PREFIX_PATH=C:/orvd-work/clang-prefix \
  -DCMAKE_INSTALL_PREFIX=C:/orvd-work/clang-test-install \
  -DGIT_EXECUTABLE="C:/Program Files/Git/cmd/git.exe" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF

cmake --build C:/orvd-work/clang-tests

OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir C:/orvd-work/clang-tests \
        --output-on-failure -j1
```

## 必查平台门

两套工具链都应单独运行真实 OpenMP probe：

```sh
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir C:/orvd-work/ucrt-tests \
        -R '^verify_openmp_parallel_execution$' -V
```

测试必须观察到至少两个真实 worker。还应确认：

- `verify_relocated_install_consumer` 通过；
- 普通 DLL 全部可解析；
- candidate 不加载共享 Drake；
- Drake marker 阳性对照仍能被运行期依赖检查拒绝；
- `verify_serial_qualification_comparison_contract` 通过，其 POSIX resource、affinity、subprocess 和
  execute-permission 路径使用测试代理，不依赖 Windows 提供 Linux API。

`run_qualification_with_metrics.py` 的真实执行模式仍是 Linux research wrapper；Windows 默认 CTest
只验证其参数布局和隔离后的执行合同，不把 Windows 冒充正式 affinity 性能平台。

## 安装和运行期

超级构建已经把 ORVD 安装到所选 prefix。下游 consumer 必须在同一 MSYS2 环境运行 CMake，并保持
该环境的 `bin` 位于 `PATH`，使 libgomp／LLVM libomp 和其他普通 DLL 可加载。不要把 UCRT64 静态库
或 DLL 与 CLANG64 prefix 混合，也不要把任一 prefix 复制到 MSVC／clang-cl 工程中使用。

若切换 shell、compiler、Git 或 MSYS2 environment，删除旧 build tree 并重新配置；只拉取新的源码
commit 不要求修改 CMakeLists 中的平台路径。
