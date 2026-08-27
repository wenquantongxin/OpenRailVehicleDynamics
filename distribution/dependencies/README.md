# 离线依赖超级构建

面向使用者的 Linux、Windows 与 macOS 完整安装命令以仓库中的 `docs/build_and_install/` 为入口；
生成 bundle 后，同一手册位于 `OpenRailVehicleDynamics/docs/build_and_install/`。本页会原样复制到
bundle 根，因此不使用只在其中一个位置成立的相对链接。本页维护离线源码包内部合同和 Windows
历史背景；若出现平台操作重复，以分平台手册为准。

本目录是 ORVD 离线源码包根 `CMakeLists.txt`、严格依赖声明及其共享校验器的源模板。它不直接作为仓库根构建入口；
开发者使用 `tools/package_distribution/assemble_source_bundle.py` 生成完整源码包。选择了对应平台
工具链后，下列命令是共同最小入口：

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

原生 macOS bundle 会在启用编译器前设置 arm64 目标架构，并按显式 sysroot、`SDKROOT`、`xcrun`
的顺序选择 macOS SDK；AppleClang 产品配置会自动
解析已安装的 Homebrew libomp 公式。显式 toolchain、sysroot、architecture 和 prefix 仍优先，完整
工具链选择与排障命令见 bundle 内的 `OpenRailVehicleDynamics/docs/build_and_install/MACOS.md`。

默认安装到 `build/install`；也可在配置时显式设置 `CMAKE_INSTALL_PREFIX`。生成后的包已携带
Eigen 3.4.0、fmt 9.1.0、nlohmann/json 3.12.0 与 SUNDIALS 7.7.0 的具名源码归档和许可证，
超级构建只读取这些本地文件，不含下载、更新或补丁步骤。四个依赖与 ORVD 由同一工具链构建，
ORVD 仍通过产品根中
唯一的 `find_package()` 路径消费安装前缀。

`dependency_sources.cmake` 是依赖来源的唯一人工维护格式；它不携带数字格式编号，也没有历史读取分支。
组包器和生成包根共同调用 `OrvdDependencySources.cmake`，因此依赖字段、许可证列表、路径约束和四项
依赖全集只定义一次。每个许可证文件使用一个可重复、单值的 `LICENSE_PATH` 字段，未知字段不会被
末尾列表吞并。生成包仍不需要 Python。

G59 起，构建工具链还必须提供标准 OpenMP C++ 编译与运行时支持；它与 C/C++ 运行时一样由
编译器工具链提供，不作为第五份源码归档捆绑。ORVD 通过 CMake `FindOpenMP` 建立并在安装包中
恢复该依赖，不手写编译器旗标或运行库路径。

## Windows 10／11

Windows 支持 [MSYS2](https://www.msys2.org/docs/installer/) 的两套 GNU 风格工具链：
UCRT64 的 GCC/g++ 与 CLANG64 的 clang/clang++。分别从 MSYS2 安装目录中的
`ucrt64.exe` 与 `clang64.exe` 启动官方环境；不从 Git Bash 内再启动一个
`bash.exe`。两套环境的前缀、编译器和包名差异见
[MSYS2 环境说明](https://www.msys2.org/docs/environments/)。

首先在对应的官方 shell 中安装同一环境的工具：

```sh
# UCRT64 shell
pacman -S --needed git \
  mingw-w64-ucrt-x86_64-gcc \
  mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-ninja \
  mingw-w64-ucrt-x86_64-python

# CLANG64 shell
pacman -S --needed git \
  mingw-w64-clang-x86_64-clang \
  mingw-w64-clang-x86_64-llvm-openmp \
  mingw-w64-clang-x86_64-cmake \
  mingw-w64-clang-x86_64-ninja \
  mingw-w64-clang-x86_64-python
```

不得在同一构建中混用 UCRT64 与 CLANG64 的程序或库。`command -v cmake ninja python`
应全部指向当前环境前缀；组包时也使用该环境的 Python。GCC 构建只能加载
libgomp，Clang 构建只能加载 LLVM libomp。CMake `FindOpenMP` 必须从所选环境
解析编译语义、头文件与唯一运行时。

源码、源码包、构建树与安装树使用短的 ASCII 物理路径，且构建树始终放在源码树外；
例如在 `C:\orvd-work` 下分别建立 `bundle`、`ucrt-build`、`ucrt-prefix`、
`clang-build` 与 `clang-prefix`。不用目录联接隐藏过长或虚拟路径。

### 组装离线源码包

一份工作树只使用一个 Git 解释其跟踪状态。如果工作树由 Git for Windows 创建，
组包器和 CMake 测试都显式传入这一个 Git，不用 MSYS2 Git 重新解释其换行策略：

```sh
python tools/package_distribution/assemble_source_bundle.py \
  --git-executable "C:/Program Files/Git/cmd/git.exe" \
  --output-directory C:/orvd-work/bundle \
  --eigen-archive C:/orvd-work/archives/eigen-3.4.0.tar.gz \
  --fmt-archive C:/orvd-work/archives/fmt-9.1.0.tar.gz \
  --nlohmann-json-archive C:/orvd-work/archives/nlohmann-json-3.12.0.tar.xz \
  --sundials-archive C:/orvd-work/archives/sundials-7.7.0.tar.gz
```

实际归档文件名必须与 `dependency_sources.cmake` 一致。如果工作树本就由 MSYS2
Git 创建，可省略 `--git-executable`，但后续 CMake 配置也必须使用同一
`/usr/bin/git`。生成的源码包只依赖四份本地归档，不需要 Git、Python 或网络。

Git for Windows 在 MSYS2 会话内按 MSYS2 的 `HOME`（默认 `db_home: cygwin desc`，
即 `/home/<用户>`）解析其全局配置；只写在 `%USERPROFILE%\.gitconfig` 里的
`safe.directory` 授权在该会话中不可见，`git rev-parse` 会以退出码 128 拒绝
工作树，组包器随即中止。`GIT_CONFIG_GLOBAL` 会被 MSYS2 登录 shell 清除，不能
作为替代；把 Windows 侧 `[safe]` 条目镜像到 MSYS2 家目录的 `~/.gitconfig` 即可。

### 离线构建与完整测试

在 UCRT64 shell 中执行：

```sh
cmake -S C:/orvd-work/bundle -B C:/orvd-work/ucrt-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ \
  -DCMAKE_INSTALL_PREFIX=C:/orvd-work/ucrt-prefix
cmake --build C:/orvd-work/ucrt-build

cmake -S C:/src/OpenRailVehicleDynamics -B C:/orvd-work/ucrt-tests -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=g++ \
  -DCMAKE_PREFIX_PATH=C:/orvd-work/ucrt-prefix \
  -DCMAKE_INSTALL_PREFIX=C:/orvd-work/ucrt-test-install \
  -DGIT_EXECUTABLE="C:/Program Files/Git/cmd/git.exe" \
  -DBUILD_TESTING=ON -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF
cmake --build C:/orvd-work/ucrt-tests
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir C:/orvd-work/ucrt-tests --output-on-failure -j1
```

在 CLANG64 shell 中使用独立的 `clang-build`、`clang-prefix`、`clang-tests`
和 `clang-test-install` 目录；依赖超级构建使用 `clang` 与 `clang++`，ORVD
产品与测试构建只需指定 `CMAKE_CXX_COMPILER=clang++`。ORVD 顶层项目只启用
C++，不向其传入无效的 C 编译器选项。不在两套工具链之间复用 CMake 构建树或
安装前缀。Windows 不注册两个只服务开发期 GNU 命令行的源码工具自检；其余当前测试全部进入
Windows 集合。当前源码默认注册 85 项；本次 macOS 收口后尚未在 Windows 复验，因此不能把该
注册数写成已实测的 `85/85`，也不应继续沿用历史的 `79/79`。

每套工具链都要额外查看真实并行门：

```sh
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir C:/orvd-work/ucrt-tests \
        -R '^verify_openmp_parallel_execution$' -V
```

测试必须观测到至少两个线程，资格运行元数据在 `OMP_NUM_THREADS=8` 时必须报告
团队规模 `8`。可重定位安装消费者和运行期依赖检查必须同时通过：所有普通 DLL
都能解析，不得加载共享 Drake，Drake 阳性对照仍必须能被拒绝。原生 MSVC 和
clang-cl 不属于当前支持面，顶层配置会直接拒绝。

SUNDIALS 配置只启用 CVODE 软件包，关闭其全部当前不消费的并行后端和第三方线性代数依赖。
SUNDIALS 7.7.0 上游仍无条件构建一组基础矩阵、线性/非线性求解模块；本项目不维护私有补丁
裁剪这些上游强制模块。ORVD 只链接 `cvode`、`nvecserial`、`sunmatrixdense`、
`sunlinsoldense` 与 `core` 目标。
ORVD 自身的 OpenMP 消费者是八接口轮轨接触批，与 SUNDIALS 的向量后端无关。

`nlohmann/json` 只参与配置静态库的编译；其类型不会进入 ORVD 公共头或安装导出依赖。已安装
消费者无需再次查找该包。

依赖声明只记录版本、上游 URL、归档文件名和许可证路径，不在仓库中固化或校验归档哈希。
如需核对下载完整性，由开发者在取得归档时于仓库外单次完成，不保存校验值或结果。
