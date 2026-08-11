# 离线依赖超级构建

本目录是 ORVD 离线源码包根 `CMakeLists.txt`、严格依赖声明及其共享校验器的源模板。它不直接作为仓库根构建入口；
开发者使用 `tools/package_distribution/assemble_source_bundle.py` 生成完整源码包后，普通用户在
该包根运行：

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

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

Windows 上可使用 Visual Studio 生成器：

```powershell
cmake -S . -B build -G "Visual Studio 16 2019" -A x64 `
  -DCMAKE_INSTALL_PREFIX=<orvd-prefix>
cmake --build build --config Release
```

也可在 Visual Studio 开发者命令环境中使用较新的独立 clang-cl；C 与 C++ 编译器必须属于同一
工具链，且仍使用所选 Visual Studio 提供的 Windows SDK 与标准库。Visual Studio 2019 随附的
Clang 12 不满足本项目的 C++23 编译前沿；其 MSVC 19.29 标准库也不足以构建少数使用较新 C++23
设施的测试。因此该环境只资格 Release 产品库和独立安装消费者，不应通过降低语言级别来伪造完整
测试通过。

Visual Studio 2019 的 FileTracker 仍受嵌套工程路径长度约束，即使系统已经启用长路径也可能
失败。源码包、构建树与安装树应直接放在短的真实物理目录中；不要用目录联接掩盖路径问题，因为
MSBuild／FileTracker 仍可能解析到较长的实体路径。

SUNDIALS 配置只启用 CVODE 软件包，关闭其全部当前不消费的并行后端和第三方线性代数依赖。
SUNDIALS 7.7.0 上游仍无条件构建一组基础矩阵、线性/非线性求解模块；本项目不维护私有补丁
裁剪这些上游强制模块。ORVD 只链接 `cvode`、`nvecserial`、`sunmatrixdense`、
`sunlinsoldense` 与 `core` 目标。
ORVD 自身的 OpenMP 消费者是八接口轮轨接触批，与 SUNDIALS 的向量后端无关。

`nlohmann/json` 只参与配置静态库的编译；其类型不会进入 ORVD 公共头或安装导出依赖。已安装
消费者无需再次查找该包。

依赖声明只记录版本、上游 URL、归档文件名和许可证路径，不在仓库中固化或校验归档哈希。
如需核对下载完整性，由开发者在取得归档时于仓库外单次完成，不保存校验值或结果。
