# 离线依赖超级构建

面向使用者的 Linux、Windows 与 macOS 完整安装命令以仓库中的 `docs/build_and_install/` 为入口；
生成 bundle 后，同一手册位于 `OpenRailVehicleDynamics/docs/build_and_install/`。本页会原样复制到
bundle 根，因此不使用只在其中一个位置成立的相对链接。本页维护离线源码包内部合同；平台操作
统一进入对应手册。

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

## 开发依赖与产品验证

完整默认超级构建会安装 bundle 中的产品快照，并且关闭其测试。要验证当前工作树中的产品修改，
先从干净 checkout 组包，再只构建四个依赖及许可证目标，最后直接配置工作树并启用测试。
这样依赖准备不需要临时提交，组包器的干净状态要求仍然保留。

```sh
# 在已经选择并显式配置工具链、安装前缀的 bundle build 中：
for target in orvd_dependency_eigen orvd_dependency_fmt \
              orvd_dependency_nlohmann_json orvd_dependency_sundials \
              orvd_dependency_licenses; do
  CMAKE_BUILD_PARALLEL_LEVEL=8 \
    cmake --build /absolute/path/to/dependency-build --parallel 8 --target "$target" || exit
done
```

UCRT64 与 CLANG64 共享归档和 bundle，分别使用独立的依赖 build、依赖安装、产品 build 和产品
安装目录。产品安装目录应为空；覆盖安装不会清除已退出公共接口的旧头。当前安装验证实际运行
七个正常消费者，覆盖五种积分方法及场景记录读写，逐个检查普通运行期依赖完整且不含共享 Drake；
Drake marker 另作必须因正确原因被拒绝的阳性对照。

## Windows 10／11

Windows 操作的唯一入口是仓库内 `docs/build_and_install/WINDOWS.md`；生成 bundle 后位于
`OpenRailVehicleDynamics/docs/build_and_install/WINDOWS.md`。该手册包含完整 MSYS2 更新、
UCRT64 GCC／libgomp 和 CLANG64 Clang／LLVM libomp 包名、环境预检、显式 Python／Git／编译器
路径、依赖准备、完整测试、显式安装、下游消费和车辆运行器示例。

使用官方 `ucrt64.exe`／`clang64.exe` 环境或 `msys2_shell.cmd` 入口，不混用两个环境的库，
不向全局 PATH 追加其工具链。一个 checkout 始终使用同一个 Git；Git for Windows 创建的工作树
在组包器和 CMake 测试中继续显式指定该 Git。如遇 dubious ownership，核实目录归属，并在当前
shell 实际使用的 Git 配置中登记准确路径，不以信任所有目录代替排查。

当前默认 Windows 配置注册 99 项 CTest，以配置后的 `ctest -N` 或 `--show-only=json-v1`
清单为准，分别记录注册数量与实测通过数量。Drake reference、Linux
专属 SIMPACK Realtime 和 state_stream 在默认 Windows 配置中关闭。真实 OpenMP 团队、七个
重定位安装消费者和共享 Drake 排除均须实际运行通过，版本和结果统一记录在 Windows 手册。

SUNDIALS 配置只启用 CVODE 软件包，关闭其全部当前不消费的并行后端和第三方线性代数依赖。
SUNDIALS 7.7.0 上游仍无条件构建一组基础矩阵、线性/非线性求解模块；本项目不维护私有补丁
裁剪这些上游强制模块。ORVD 只链接 `cvode`、`nvecserial`、`sunmatrixdense`、
`sunlinsoldense` 与 `core` 目标。
ORVD 自身的 OpenMP 消费者是八接口轮轨接触批，与 SUNDIALS 的向量后端无关。

`nlohmann/json` 参与配置和场景记录静态库的编译；其类型不会进入 ORVD 公共头或安装导出依赖。已安装
消费者无需再次查找该包。

依赖声明只记录版本、上游 URL、归档文件名和许可证路径，不在仓库中固化或校验归档哈希。
如需核对下载完整性，由开发者在取得归档时于仓库外单次完成，不保存校验值或结果。
