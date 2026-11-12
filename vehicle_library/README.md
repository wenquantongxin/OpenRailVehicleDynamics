# 车辆与编组资产

每个车型目录保存机械定义、起动状态、轮型、显示几何和参考模型。线路与轨道不平顺由运行入口独立选取。

| 资产 | 内容 |
|---|---|
| `vehicle_definition.json` | 单车刚体、关节、安装坐标架与力元 |
| `startup_states/` | 具名初始位姿、速度、串联内力及配套参数 |
| `wheel_profiles/`、`visualization/` | 轮型与显示几何 |
| `consists/` | 车辆实例、相对站位及具名端点间的力元 |
| `reference_models/` | SIMPACK 参考模型与运行说明 |

## 车辆

| 车型 | 机械定义 | 起动状态 | 参考模型 |
|---|---|---|---|
| GZ18 | [刚性轮对车辆](gz18/vehicle_definition.json) | [60 km/h 移动初态](gz18/startup_states/moving_startup_60kmh.json) | [SIMPACK](gz18/reference_models/simpack/README.md) |
| IRW | [独立旋转车轮车辆](irw/vehicle_definition.json) | [60 km/h 移动初态](irw/startup_states/moving_startup_60kmh.json) | [SIMPACK](irw/reference_models/simpack/README.md) |

## 编组

[GZ18 三节编组](gz18/consists/three_car.json)复用同一车辆定义，记录各实例的名称、车辆身份与相对站位。
车辆定义和起动记录仍由运行入口提供，全列共享线路与轨道不平顺。

力元按本构族配置。车辆记录中的端点使用局部坐标架名，编组记录使用包含实例前缀的完整名称，
例如 `car_1.rear_coupler_mount`。实例内与实例间的连接采用相同装配规则。

三节配置、迟滞车钩参数和对照结果见 [GZ18 参考模型说明](gz18/reference_models/simpack/README.md#三节编组)。
命令行用法见[运行工具](../tools/dynamics_qualification/README.md#run-a-consist)。
