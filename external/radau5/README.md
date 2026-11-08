# Radau5 数值核心

本目录保存 Radau IIA 算法的纯 C++ 数值核心、来源和许可。
`orvd_radau5_core_objects` 由积分器适配器与数值测试共用，公共调用通过 `ORVD::integrators` 提供。

| 文件 | 内容 |
|---|---|
| [SOURCE_DISPOSITION.txt](SOURCE_DISPOSITION.txt) | 上游来源、作者与逐文件处置 |
| [LICENSE.UNIGE.txt](LICENSE.UNIGE.txt) | UNIGE 许可 |
| [RADAU5_SOURCE_MODIFICATIONS.md](RADAU5_SOURCE_MODIFICATIONS.md) | 派生实现的修改说明 |
| [radau5_core.h](include/orvd/radau5/radau5_core.h) | 核心接口 |
| [radau5_core.cc](src/radau5_core.cc) | 数值实现 |

核心处理正向显式一阶 ODE，采用三阶段五阶 Radau IIA、稠密数值 Jacobian、实／复线性分解、
自适应步长与配点稠密输出。算法推导见[时间积分](../../docs/models_and_algorithms/numerical_methods/TIME_INTEGRATION_METHODS.md)，
方法选择见[积分器说明](../../libs/integrators/README.md)。

核心头与对象目标供源码树内部使用。安装包包含积分器库以及来源、许可和修改说明。
官方 Fortran 实现可通过来源文件中的 URL 获取，供独立数值对照。
