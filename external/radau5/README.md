# Radau5 外部来源边界

本目录承载 ORVD 研究型 Radau5 后端的第三方来源身份、许可、逐文件处置和派生的纯 C++ 数值
核心。它不是一个由 `find_package()` 解析的外部依赖，也不建立 Fortran 产品身份。

## 当前内容

INT-03 已落位收窄核心和不安装的 `orvd_radau5_core_objects` 目标：

- [`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt)是来源 URL、C++ donor 摘要、作者和逐文件
  处置的唯一权威；
- [`LICENSE.UNIGE.txt`](LICENSE.UNIGE.txt)是官方 Windows-1252 许可文本的 UTF-8 等文转录，只规范化
  首尾空行和行末空格；官方原始字节摘要仍只在来源账本记录；
- [`RADAU5_SOURCE_MODIFICATIONS.md`](RADAU5_SOURCE_MODIFICATIONS.md)逐项记录相对 donor 的实际修改；
- [`include/orvd/radau5/radau5_core.h`](include/orvd/radau5/radau5_core.h)与
  [`src/radau5_core.cc`](src/radau5_core.cc)构成 build-only 核心，不进入安装头；
- 核心对象由独立数值测试和 `orvd_integrators` 复用，不分别编译漂移副本。

完整数值合同见
[`INT_01_RADAU5_SOLVER_CONTRACT.md`](../../docs/planning/integrator_migration/INT_01_RADAU5_SOLVER_CONTRACT.md)，
“CVODE 默认不变、Radau5 为研究后端”的长期决策见
[`ADR-0009`](../../docs/adr/0009-cvode-default-radau5-research-backend.md)。

## Fortran oracle

项目负责人裁决：官方 Fortran oracle 只记录 URL，不提交源码，也不在仓库记录其 SHA-256。需要
交叉核验时，可把处置表列出的文件临时下载到仓外目录并在那里编译；下载物、驱动、可执行文件和
输出均不进入本仓库。

因此 Fortran 不属于：

- ORVD 的 CMake 语言或产品 target；
- `orvd_integrators` 的外部链接、运行或安装依赖；
- 离线产品源码包；
- 本目录派生 C++ 核心的编译输入。

## 实现边界

核心只支持正向显式一阶 ODE、`M=I`、动态正维 `double` 状态、full-dense 数值 Jacobian、一个实
`N×N` 与一个复 `N×N` LU、三阶段 Radau IIA、单接受内步、自适应误差控制和最近成功步的配点稠密
输出。质量矩阵／DAE、带状和稀疏 Jacobian、解析 Jacobian、Hessenberg、二阶特殊结构、用户输出
框架和 Fortran 均不在目标内。

根 `NOTICE` 与安装文档收集 UNIGE 许可、来源账本和修改记录；核心 target 自身不安装或导出。源码树
内的 Radau5 适配器和系统 recipe 仍是资格期研究入口，公共构造继续选择 CVODE BDF2。
独立正确性审查后的 stop、异常因果、线性建立分类、极端尺度范数、轮轨投影历史、近奇异移位求解
资格及严格浮点编译门记录见
[`RADAU5_SOURCE_MODIFICATIONS.md`](RADAU5_SOURCE_MODIFICATIONS.md)；这些修复不授权性能排名或
公开后端选择。
