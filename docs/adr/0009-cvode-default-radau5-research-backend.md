# ADR-0009：CVODE 默认配方与 Radau5 参考后端

- 状态：**Accepted**
- 相关：[ADR-0003](0003-abstract-advancer-cvode-first.md)

## 背景

车辆推进需要高效的默认配方，以及采用不同数值方法的刚性问题对照。
数值后端共用单接受步、稠密输出、统计、失败恢复和重初始化接口；系统层管理接受态与控制事件。

## 决策

1. 默认积分配方由运行场景选择，使用 CVODE BDF2 或 BDF5。Radau5 通过同一强类型配置显式选择。
2. Radau5 采用纯 C++ 实现：三阶段五阶 Radau IIA、单位质量矩阵、正向显式一阶 ODE、
   `double` 状态、稠密数值 Jacobian、simplified Newton 及实／复线性系统。
3. 一次推进至多发布一个接受内步。端点 RHS 在系统层提交轮轨投影历史后求值；后端异常和
   重初始化遵守公共事务合同。
4. 来源、许可与派生修改分别保存在
   [来源记录](../../external/radau5/SOURCE_DISPOSITION.txt)、
   [许可](../../external/radau5/LICENSE.UNIGE.txt)和
   [修改说明](../../external/radau5/RADAU5_SOURCE_MODIFICATIONS.md)。
5. 官方 Fortran 实现用于独立数值对照，产品构建使用 C++ 核心。
6. 方法推导见[时间积分](../models_and_algorithms/numerical_methods/TIME_INTEGRATION_METHODS.md)，
   配置与推进合同见[积分器说明](../../libs/integrators/README.md)。

## 后果

CVODE 与 Radau5 可在相同车辆、载荷和系统事务下对照。方法配置和精度要求由调用方明确给出，
稠密采样与控制事件的语义保持一致。核心对象供积分器适配器与数值测试共用，安装接口由
`ORVD::integrators` 提供。
