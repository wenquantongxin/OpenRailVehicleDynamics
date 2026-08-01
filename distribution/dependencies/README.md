# 离线依赖超级构建

本目录是官方源码包根 `CMakeLists.txt` 与依赖锁表的源模板。它不直接作为仓库根构建入口；
开发者使用 `tools/package_distribution/assemble_source_bundle.py` 生成完整源码包后，普通用户在
该包根运行：

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

默认安装到 `build/install`；也可在配置时显式设置 `CMAKE_INSTALL_PREFIX`。生成后的包已携带
Eigen 3.4.0、fmt 9.1.0 与 SUNDIALS 7.7.0 的锁定源码归档和许可证，超级构建只读取这些本地
文件，不含下载、更新或补丁步骤。三个依赖与 ORVD 由同一工具链构建，ORVD 仍通过产品根中
唯一的 `find_package()` 路径消费安装前缀。

Windows 上可使用 Visual Studio 生成器：

```powershell
cmake -S . -B build -G "Visual Studio 16 2019" -A x64 `
  -DCMAKE_INSTALL_PREFIX=<orvd-prefix>
cmake --build build --config Release
```

Visual Studio 2019 的 FileTracker 仍受嵌套工程路径长度约束，即使系统已经启用长路径也可能
失败。源码包、构建树与安装树的实体目录可以统一放在项目辅助目录中；若该根路径较长，构建
期间给该根建立一个短目录联接并从短路径配置，完成后删除联接即可，实体文件不必搬散。

SUNDIALS 配置只启用 CVODE 软件包，关闭全部当前不消费的并行后端和第三方线性代数依赖。
SUNDIALS 7.7.0 上游仍无条件构建一组基础矩阵、线性/非线性求解模块；本项目不维护私有补丁
裁剪这些上游强制模块。ORVD 只链接 `cvode`、`nvecserial`、`sunmatrixdense`、
`sunlinsoldense` 与 `core` 目标。

锁表中的 SHA-256 只校验源码归档在搬运过程中是否保持完整。版本、上游 URL 和归档内许可证
共同记录来源身份；哈希不参与 ORVD 功能正确性判定。
