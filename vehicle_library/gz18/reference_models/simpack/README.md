# GZ18 SIMPACK 参考模型

用于与 ORVD 进行整车动力学对比。ORVD 运行参数位于 `vehicle_library/gz18/`。

## 文件与运行

- `main_model/gz18_straight_aar6.spck`：主模型，内置 60 km/h 移动初态。
- `database/mbs_db_substructure/bogie_motor.spck`：构架子结构。
- `database/mbs_db_substructure/wheelset_motor.spck`：轮对子结构。

使用 SIMPACK 2021x 普通时间积分（`simpack-slv --integration`），由
`vehicle.applystartvel=1` 设置启动速度。模型采用双精度结果输出，子结构路径为
`../database`。本机须提供 `LM.prw` 和 `UIC60.prr` 型面。
