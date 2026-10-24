# Windows 10／11：MSYS2 源码构建、测试与安装

本页是 Windows 操作入口。共同依赖及平台机制见[构建总览](README.md)。当前支持
MSYS2 UCRT64 GCC／libgomp 和 CLANG64 Clang／LLVM libomp，使用同一套 CMake、C++23
产品代码和数值判据，构建采用 GNU 风格编译器前端与 Ninja。

默认构建包含五种积分方法、场景采样、场景记录读写和三个源码树车辆运行器。
`ORVD_BUILD_SIMPACK_REALTIME_TOOLS`、`ORVD_BUILD_STATE_STREAM` 是默认关闭的 Linux 专属
可选项，Windows 保持关闭。浏览器查看器独立使用 Node，不需要启用 UDP state_stream。

## 更新环境并预检

从 MSYS2 安装目录的 `ucrt64.exe` 或 `clang64.exe` 打开官方环境。自动化也可从 PowerShell
调用官方入口，例如 `C:/msys64/msys2_shell.cmd -ucrt64 -defterm -no-start -c '命令'`；
Clang 使用 `-clang64`。交互和自动化均从对应 MSYS2 官方入口启动，工具链 `bin` 由当前会话载入 PATH。

先按 [MSYS2 官方更新流程](https://www.msys2.org/docs/updating/)完整更新：

```sh
pacman -Syu
```

若核心更新要求关闭 shell，关闭全部 MSYS2 终端，再打开官方环境重新执行 `pacman -Syu`，
直到完整更新成功。网络失败时修复连接并重试完整更新，随后按 ORVD 现行语言标准和依赖版本构建。

UCRT64 所需包：

```sh
pacman -S --needed git \
  mingw-w64-ucrt-x86_64-gcc \
  mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-ninja \
  mingw-w64-ucrt-x86_64-python
```

CLANG64 所需包：

```sh
pacman -S --needed git \
  mingw-w64-clang-x86_64-clang \
  mingw-w64-clang-x86_64-llvm-openmp \
  mingw-w64-clang-x86_64-cmake \
  mingw-w64-clang-x86_64-ninja \
  mingw-w64-clang-x86_64-python
```

在每个官方 shell 中设置下面这一组变量。例子使用 `F:/orvd-work`；可整体换成另一条短 ASCII
物理路径。源码、bundle、依赖、构建和产品安装目录彼此分离。

```sh
export ORVD_SOURCE_ROOT=F:/myProjects/OpenRailVehicleDynamics
export ORVD_WORK_ROOT=F:/orvd-work
export ORVD_GIT='C:/Program Files/Git/cmd/git.exe'
case "$MSYSTEM" in
  UCRT64) ORVD_FLAVOR=ucrt; ORVD_CC=gcc; ORVD_CXX=g++ ;;
  CLANG64) ORVD_FLAVOR=clang; ORVD_CC=clang; ORVD_CXX=clang++ ;;
  *) echo '请使用 UCRT64 或 CLANG64 shell'; exit 1 ;;
esac
export ORVD_TOOLCHAIN="$(cygpath -m "$MINGW_PREFIX")"
export ORVD_ARCHIVE_ROOT="$ORVD_WORK_ROOT/archives"
export ORVD_BUNDLE_ROOT="$ORVD_WORK_ROOT/bundle"
export ORVD_DEPS_BUILD="$ORVD_WORK_ROOT/$ORVD_FLAVOR-deps-build"
export ORVD_DEPS="$ORVD_WORK_ROOT/$ORVD_FLAVOR-deps"
export ORVD_BUILD="$ORVD_WORK_ROOT/$ORVD_FLAVOR-build"
export ORVD_INSTALL="$ORVD_WORK_ROOT/$ORVD_FLAVOR-install"
export CMAKE_BUILD_PARALLEL_LEVEL=8

printf 'MSYSTEM=%s MINGW_PREFIX=%s\n' "$MSYSTEM" "$MINGW_PREFIX"
command -v cmake ninja python "$ORVD_CC" "$ORVD_CXX"
"$ORVD_CXX" --version
cmake --version
ninja --version
python --version
"$ORVD_GIT" --version
"$ORVD_GIT" -C "$ORVD_SOURCE_ROOT" status --short
```

CMake、Ninja、Python、C/C++ 编译器应来自当前 `/ucrt64/bin` 或 `/clang64/bin`。
GCC 使用 libgomp，Clang 使用 LLVM libomp；旧 TDM-GCC 不满足当前构建要求。
清理旧编译器时先备份安装信息及用户／系统 PATH，再使用其已登记卸载器，精确删除对应路径。
系统 PATH 需要管理员权限；清理后重新打开终端，已有进程可能仍持有旧 PATH。

一份 checkout 始终使用同一个 Git。上述例子使用 Git for Windows；若 checkout 一直由
MSYS2 Git 管理，可把 `ORVD_GIT` 设为 `/usr/bin/git`。如出现 dubious ownership，先确认目录
归属，再在当前 shell 实际使用的 Git 配置中登记准确的物理路径。

## 共享归档与干净 bundle

四份归档的版本、文件名及 URL 以
[`dependency_sources.cmake`](../../distribution/dependencies/dependency_sources.cmake) 为唯一权威。
下列准备只需执行一次，两套工具链共享归档和 bundle：

```sh
mkdir -p "$ORVD_ARCHIVE_ROOT" "$ORVD_WORK_ROOT/logs"
curl --fail --location -o "$ORVD_ARCHIVE_ROOT/eigen-3.4.0.tar.gz" \
  https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
curl --fail --location -o "$ORVD_ARCHIVE_ROOT/fmt-9.1.0.tar.gz" \
  https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.tar.gz
curl --fail --location -o "$ORVD_ARCHIVE_ROOT/nlohmann-json-3.12.0.tar.xz" \
  https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
curl --fail --location -o "$ORVD_ARCHIVE_ROOT/sundials-7.7.0.tar.gz" \
  https://github.com/LLNL/sundials/releases/download/v7.7.0/sundials-7.7.0.tar.gz

python "$ORVD_SOURCE_ROOT/tools/package_distribution/assemble_source_bundle.py" \
  --source-root "$ORVD_SOURCE_ROOT" \
  --git-executable "$ORVD_GIT" \
  --cmake-executable "$ORVD_TOOLCHAIN/bin/cmake.exe" \
  --output-directory "$ORVD_BUNDLE_ROOT" \
  --eigen-archive "$ORVD_ARCHIVE_ROOT/eigen-3.4.0.tar.gz" \
  --fmt-archive "$ORVD_ARCHIVE_ROOT/fmt-9.1.0.tar.gz" \
  --nlohmann-json-archive "$ORVD_ARCHIVE_ROOT/nlohmann-json-3.12.0.tar.xz" \
  --sundials-archive "$ORVD_ARCHIVE_ROOT/sundials-7.7.0.tar.gz"
```

组包要求可见的 tracked／untracked 状态全部干净，且输出目录尚不存在。开发时应先从干净
checkout 组包、准备依赖，再编辑源码；随后直接构建工作树，不必为准备依赖临时提交。
已有改动时复用此前准备好的匹配依赖，或使用另一个干净 checkout 准备 bundle。
bundle 保留组包时的源码快照，后续开发与完整测试使用当前工作树；正式发行包从干净源码制作。

## 分工具链准备依赖

分别在 UCRT64、CLANG64 shell 中执行，先完成一套再运行另一套。
每套依赖 build 和 install 目录首次运行时必须为空或不存在。

```sh
cmake -S "$ORVD_BUNDLE_ROOT" -B "$ORVD_DEPS_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER="$ORVD_TOOLCHAIN/bin/$ORVD_CC.exe" \
  -DCMAKE_CXX_COMPILER="$ORVD_TOOLCHAIN/bin/$ORVD_CXX.exe" \
  -DCMAKE_MAKE_PROGRAM="$ORVD_TOOLCHAIN/bin/ninja.exe" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_DEPS"

for target in orvd_dependency_eigen orvd_dependency_fmt \
              orvd_dependency_nlohmann_json orvd_dependency_sundials \
              orvd_dependency_licenses; do
  cmake --build "$ORVD_DEPS_BUILD" --parallel 8 --target "$target" || exit
done
```

上述目标构建四个依赖及许可证。
依赖构建不需要网络；OpenMP 由所选工具链提供。直接构建 bundle 的默认目标则会连同其产品
快照一起安装，且该产品默认关闭测试；当前源码的完整测试按下一节执行。

## 当前源码的干净构建与完整测试

每套工具链使用独立且空的 `ORVD_BUILD`、`ORVD_INSTALL`。切换编译器或 ABI 时重新配置空目录；
清理前确认绝对路径及 CMakeCache 的源码归属，保留已有结果和研究文件。
ORVD 根项目只启用 C++，C 编译器仅传给上面的依赖构建。

```sh
cmake -S "$ORVD_SOURCE_ROOT" -B "$ORVD_BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER="$ORVD_TOOLCHAIN/bin/$ORVD_CXX.exe" \
  -DCMAKE_MAKE_PROGRAM="$ORVD_TOOLCHAIN/bin/ninja.exe" \
  -DPython3_EXECUTABLE="$ORVD_TOOLCHAIN/bin/python.exe" \
  -DGIT_EXECUTABLE="$ORVD_GIT" \
  -DCMAKE_PREFIX_PATH="$ORVD_DEPS" \
  -DEigen3_DIR="$ORVD_DEPS/share/eigen3/cmake" \
  -Dfmt_DIR="$ORVD_DEPS/lib/cmake/fmt" \
  -Dnlohmann_json_DIR="$ORVD_DEPS/share/cmake/nlohmann_json" \
  -DSUNDIALS_DIR="$ORVD_DEPS/lib/cmake/sundials" \
  -DCMAKE_INSTALL_PREFIX="$ORVD_INSTALL" \
  -DBUILD_TESTING=ON \
  -DORVD_BUILD_DRAKE_REFERENCE_TESTS=OFF \
  -DORVD_BUILD_SIMPACK_REALTIME_TOOLS=OFF \
  -DORVD_BUILD_STATE_STREAM=OFF

cmake --build "$ORVD_BUILD" --parallel 8
ctest --test-dir "$ORVD_BUILD" -N
ctest --test-dir "$ORVD_BUILD" --show-only=json-v1 \
  > "$ORVD_WORK_ROOT/logs/$ORVD_FLAVOR-test-list.json"
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_BUILD" -j1 --output-on-failure
```

CTest 完整输出位于构建目录 `Testing/Temporary/LastTest.log`。

`verify_openmp_parallel_execution` 必须观测到至少两个真实 worker。
`verify_relocated_install_consumer` 会安装到测试专用临时前缀、迁移前缀，再构建和运行八个独立
消费者，覆盖五种积分方法、配置与资产、线路、不平顺、轮轨接触、力元、场景记录与资源读写、可移植状态流编解码。
八个程序均须通过普通 DLL 完整解析和无共享 Drake 检查；Drake marker 阳性对照须因含共享 Drake
而被拒绝。需要详细诊断时可单独运行这两项：

```sh
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  ctest --test-dir "$ORVD_BUILD" -V \
    -R '^(verify_openmp_parallel_execution|verify_relocated_install_consumer)$'
```

## 显式安装与下游消费

通过完整测试后，显式安装到干净的产品目录：

```sh
cmake --install "$ORVD_BUILD" --prefix "$ORVD_INSTALL" --config Release
```

使用空安装目录，使安装的公共头与当前接口保持一致。
当前公共积分入口是 `SystemContinuousStateAdvancer` 加 `SystemIntegrationConfiguration`，显式
选择 CVODE BDF2／BDF5、Radau5、Newmark 或 Zhai。具体 CVODE 类属于源码私有实现，
公共接口的调用方式见[积分器说明](../../libs/integrators/README.md)。

下游项目使用 `find_package(OpenRailVehicleDynamics CONFIG REQUIRED)` 和 `ORVD::` imported
targets。依赖前缀必须来自同一工具链；运行程序时继续在对应官方 shell 内，使普通运行时 DLL
可解析。下面直接复用仓库的独立消费者作为可运行示例，不依赖产品源码内部头：

```sh
cmake -S "$ORVD_SOURCE_ROOT/tests/installation/consumer" \
  -B "$ORVD_WORK_ROOT/$ORVD_FLAVOR-consumer-build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER="$ORVD_TOOLCHAIN/bin/$ORVD_CXX.exe" \
  -DCMAKE_MAKE_PROGRAM="$ORVD_TOOLCHAIN/bin/ninja.exe" \
  -DCMAKE_PREFIX_PATH="$ORVD_INSTALL;$ORVD_DEPS"
cmake --build "$ORVD_WORK_ROOT/$ORVD_FLAVOR-consumer-build" --parallel 8
"$ORVD_WORK_ROOT/$ORVD_FLAVOR-consumer-build/orvd_installation_smoke.exe"
"$ORVD_WORK_ROOT/$ORVD_FLAVOR-consumer-build/scene_record_installation_smoke.exe"
```

消费者夹具中的 Drake marker 为依赖检查提供阳性对照。UCRT64 与 CLANG64 各自使用对应的
静态库、DLL、安装前缀和 `CMakeCache.txt`。

## 运行器与浏览器查看器

三个车辆运行器位于构建树 `tools/dynamics_qualification/`，不安装到产品前缀。下面在未使用过的
输出目录生成一段 30 ms IRW 场景记录，时间参数单位为整数纳秒：

```sh
OMP_NUM_THREADS=8 OMP_DYNAMIC=FALSE \
  "$ORVD_BUILD/tools/dynamics_qualification/orvd_irw_passive_scenario.exe" \
  irw_r300_no_irregularity_v60_passive \
  "$ORVD_SOURCE_ROOT/vehicle_library/irw/vehicle_definition.json" \
  "$ORVD_SOURCE_ROOT/vehicle_library/irw/startup_states/moving_startup_60kmh.json" \
  "$ORVD_SOURCE_ROOT/track_library/geometries/r300_centerline_superelevation_1100m.json" \
  "$ORVD_SOURCE_ROOT" none "$ORVD_WORK_ROOT/$ORVD_FLAVOR-irw-scene" \
  30000000 10000000 --scene-record
```

成功输出目录包含 `COMPLETE` 完成标记文件、数值表及 `scene_record/`。三个 CLI 共用 `--integration-config PATH`
选择积分配置；缺省时使用各场景的明确配方。完整参数和配置示例见
[运行器说明](../../tools/dynamics_qualification/README.md)。Windows 的 `cpu_affinity_at_start`
元数据为 `null`，运行器不设置 CPU affinity。

该例初始时有轴桥位于线路定义起点之前，运行器会报告使用三维边界切线延拓，并继续完成积分与
记录发布。每次运行使用新的输出目录。

在普通 Windows PowerShell 中，进入 `apps/scene_viewer_web` 后使用现有 Node 24 环境：

```powershell
npm.cmd install --package-lock=false
npm.cmd test
npm.cmd run build
npm.cmd run dev
```

`npm.cmd` 可直接在 PowerShell 中执行；安装依赖时使用 `--package-lock=false`。
生产构建包含 `tsc --noEmit`，输出到 `dist/`；浏览器可直接选择上述 `scene_record` 文件夹。
更多加载方式见[查看器说明](../../apps/scene_viewer_web/README.md)。

## Windows 验证记录

验证系统为 Windows 10 Pro 22H2 x64，版本 `10.0.19045.7663`。
MSYS2 完整更新后，两套工具链串行完成四个依赖及许可证目标构建，再从空产品构建／安装目录
验证同一工作树。使用 Release、C++23、`BUILD_TESTING=ON`，上述三个可选项均关闭。

| 环境 | 编译器及 OpenMP runtime | CTest 注册／通过 | 重定位正常消费者 | 正式产品安装目录 |
|---|---|---|---|---|
| UCRT64 | GCC 16.2.0（MSYS2 Rev6），libgomp 16.2.0 | 99／99 | 7／7 | `F:/orvd-work/ucrt-review-install` |
| CLANG64 | Clang 23.1.3，LLVM libomp 23.1.3 | 99／99 | 7／7 | `F:/orvd-work/clang-review-install` |

共同工具为 CMake 4.4.4、Ninja 1.13.2、Python 3.14.8、Git for Windows 2.56.0.windows.1。
复验使用 `release` 预设并显式指定工具链、依赖和安装前缀，构建目录为
`F:/orvd-work/ucrt-review-build` 与 `F:/orvd-work/clang-review-build`。
构建并行度为 8；CTest 使用 `OMP_NUM_THREADS=8`、`OMP_DYNAMIC=FALSE` 和 `-j1` 执行全部注册测试。
两套 OpenMP 探针均通过真实双 worker 的静态批和动态列循环检查（报告最大线程数 8）；
GCC 的 `_OPENMP=202111`，Clang 的 `_OPENMP=202011`。
安装后的五种积分方法均通过解析转子检查，场景记录写入并读回成功；七个消费者的普通 DLL 均可
解析且无共享 Drake，Drake marker 均因正确原因被拒绝。正式安装后的独立消费者也成功构建运行。

上面的 30 ms IRW 命令在两套环境中均生成 4 帧记录和 `COMPLETE`；查看器的 `check-record`
命令均成功解析这些记录及显示绑定。

前端使用 Node 24.21.0／npm 11.19.0，48／48 测试和包含 TypeScript 检查的生产构建通过。
CLANG64 构建输出 `wheel_spin_sampling.h` 未使用私有字段告警；Vite 输出主包超过 500 kB 的
体积提示。

日志保存在仓库外 `F:/orvd-work/logs`：`ucrt-review-*`／`clang-review-*` 包含复验的配置、构建、
CTest 清单、完整测试输出和安装／下游消费记录；`viewer-*` 包含前端安装、测试、构建与实际依赖版本。
TDM-GCC 的登记卸载项、安装目录及用户／系统 PATH 对应条目均已清除，备份位于
`F:/orvd-work/system-backup`。旧 `build/release` 已移至该备份目录，结果和研究脚本保留。
