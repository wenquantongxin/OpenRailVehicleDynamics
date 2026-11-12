# GZ18 SIMPACK 参考模型

用于与 ORVD 进行整车动力学对比。ORVD 运行参数位于 `vehicle_library/gz18/`。

## 文件与运行

- [gz18_straight_aar6.spck](main_model/gz18_straight_aar6.spck)：单车主模型，内置 60 km/h 移动初态。
- [gz18_three_car.spck](main_model/gz18_three_car.spck)：三节主模型，重复引用同目录的单车模型。
- `database/mbs_db_substructure/bogie_motor.spck`：构架子结构。
- `database/mbs_db_substructure/wheelset_motor.spck`：轮对子结构。

使用 SIMPACK 2021x 普通时间积分（`simpack-slv --integration`），由
`vehicle.applystartvel=1` 设置启动速度。模型采用双精度结果输出，子结构路径为
`../database`。本机须提供 `LM.prw` 和 `UIC60.prr` 型面。

## 三节编组

三节同向 GZ18 的车体中心初始站位为 0、−23.6、−47.2 m。相邻车通过前车后端与后车前端
的 Hook 标记连接，两处均采用 105 号点对点迟滞力元。原生配置见
[three_car.json](../../consists/three_car.json)，命令见[运行工具](../../../../tools/dynamics_qualification/README.md#run-a-consist)。

| 参数 | 值 |
|---|---:|
| 名义长度 | 0.80 m |
| 中线刚度 | 8 MN/m |
| 迟滞比 | 0.25 |
| 过渡速度 | 1 mm/s |

上下包络围绕弹性中线对称，在过渡速度带内按伸缩速度线性插值，零速取中线。
轴力沿两安装点的瞬时连线作用，拉伸为正。本构与耗散关系见
[点对点迟滞弹簧](../../../../docs/models_and_algorithms/force_elements/POINT_TO_POINT_HYSTERETIC_SPRING.md)。

默认线路为直线四通道 AAR6，初速 60 km/h。下表采用相同已导出的空间不平顺，比较原生默认
BDF2 与 SIMPACK 普通时间积分；SIMPACK 最大步长为 1 ms，共同输出间隔为 0.5 ms。
RMS 差在完整工况窗口内对全列十二个轮对或两处车钩的同类通道合并计算。

| 工况 | 横移 RMS 差 / µm | 摇头 RMS 差 / µrad | 车钩轴力 RMS 差 / kN |
|---|---:|---:|---:|
| 直线＋AAR6，60 km/h，20 s | 13.04 | 4.81 | 0.178 |
| R300＋AAR5，60 km/h，50 s | 37.64 | 16.89 | 0.561 |

两软件的车钩拉压变化及轮对横移、摇头响应在整体幅值、相位和进出曲线过程上接近。
