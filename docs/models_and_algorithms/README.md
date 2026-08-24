# 模型与算法文档

本目录说明 ORVD 代码背后的轨道车辆领域模型、数学关系和计算方法。它既不是软件架构目录，
也不是实施路书或实验档案。

## 与其他文档目录的边界

| 目录 | 职责 |
|---|---|
| `models_and_algorithms/` | 物理模型、坐标约定、数学公式、算法语义和外部参考实现比较 |
| `design/` | 软件架构、模块边界和接口契约 |
| `engineering/` | 第一方代码、输入、热路径和验收纪律 |
| `planning/` | 实施顺序、阶段裁决和迁移观察 |
| `performance/` | 计时证据、性能候选和优化实施计划 |

模型与算法文档不维护实施进度，也不把一次实验结果写成永久规律。外部软件或文献中的做法必须
标明来源；尚未通过源码、文档或受控试验确定的行为必须保留为开放问题，不能静默变成 ORVD 合同。

## 主题分类

主题目录按实际文档逐步建立，不预建空目录：

| 分类 | 内容 |
|---|---|
| `track_geometry/` | 线路平纵断面、超高、轨道坐标系、站位和轨道不平顺的分层关系 |
| `track_irregularity_spectra/` | 轨道不平顺 PSD、有限空间频带、随机实现和多方向相关性 |
| `wheel_rail_contact/` | 接触几何、EEC、Kalker、FASTSIM、多斑接触及 CONTACT 等软件比较 |
| `vehicle_dynamics/` | 车辆拓扑、悬挂、载荷、刚性轮对和独立旋转轮对模型 |
| `control_and_estimation/` | 控制器、观测器、状态估计、采样和执行器语义 |
| `numerical_methods/` | 时间积分、非线性求解、误差传播及数值稳定性 |

数值方法文档解释算法与误差机理；机器计时、实时因子和具体优化实验仍归
[`performance/`](../performance/README.md)。

## 现有文档

- [轨道竖向剖面建模及其三维耦合](track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.md)：
  定义恒坡段、PL2、CIR、竖向接缝及其与平面线形和超高的三维耦合，并给出首轮产品与整车资格合同。
- [轨道不平顺谱及其空间随机实现](track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md)：
  定义空间频率单位、简化 FRA/AAR 单截止谱、有限频带、随机种子和空间序列资格。
- [BDF、Radau5、Newmark 与 Zhai 时间积分方法](numerical_methods/TIME_INTEGRATION_METHODS.md)：
  说明四类方法的离散公式、单步计算、误差与稳定性，并区分完整一阶后端和二阶机械方法在
  ORVD `[q;v;z]` 状态上的适用边界。
