# ORVD 文档索引

## 构建与使用

| 入口 | 内容 |
|---|---|
| [构建与安装](build_and_install/README.md) | 依赖、CMake 配置与安装消费；[Linux](build_and_install/LINUX.md)、[Windows](build_and_install/WINDOWS.md)、[macOS](build_and_install/MACOS.md) 命令 |
| [车辆运行工具](../tools/dynamics_qualification/README.md) | 被动与受控运行、积分配置、结果输出 |
| [场景查看器](../apps/scene_viewer_web/README.md) | 离线回放、实时观察与资源加载 |
| [计算性能](performance/README.md) | 计时定义与 Linux、macOS 实测 |

## 模型与接口

| 入口 | 内容 |
|---|---|
| [模型与算法](models_and_algorithms/README.md) | 线路、不平顺、轮轨接触、力元、多体动力学与积分方法；含中英文索引 |
| [坐标与记号](models_and_algorithms/CONVENTIONS_AND_NOTATION.md) | 坐标系、正号、状态布局与单位 |
| [车辆与编组资产](../vehicle_library/README.md) | 车辆定义、起动状态、编组配置与参考模型 |
| [车辆配置](../libs/configuration/README.md) | 显式加载、机械装配与具名启动量 |
| [IRW 参考模型](../vehicle_library/irw/reference_models/simpack/README.md)与[GZ18 参考模型](../vehicle_library/gz18/reference_models/simpack/README.md) | SIMPACK 文件、工况与初始化 |
| [力元](../libs/forces/README.md) | 本构、载荷作用点与内力状态 |
| [多体模型](../libs/multibody_model/README.md) | 刚体、关节、坐标架与模型拓扑 |
| [系统装配](../libs/system_assembly/README.md) | 模型、上下文、接受态与右端求值 |
| [积分接口](../libs/integrators/README.md) | 方法配置、推进、步内采样与同步 |
| [场景记录](../libs/scene_record/README.md)与[状态流](../libs/scene_stream/README.md) | 描述、静态资源、样本与通信协议 |

## 实现约定

- [按目标推进的开发方法](engineering/GOAL_DRIVEN_DEVELOPMENT.md)：目标定义、证据核查与独立复核。
- [SIMPACK 模型对照](engineering/SIMPACK_MODEL_COMPARISON.md)：GZ18 实测、比较口径与模型差异。
- [多体运行时](../libs/multibody_runtime/README.md)：状态所有权、参数与缓存依赖。
- [架构决策](adr/README.md)：模块边界与接口取舍。
- [第一方工程约束](engineering/FIRST_PARTY_ENGINEERING_RULES.md)：命名、输入与实现规则。
