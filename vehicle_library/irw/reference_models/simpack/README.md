# IRW SIMPACK 参考模型

本目录提供 IRW 整车动力学对照的 SIMPACK 参考模型。

## 模型组成

- [`main_model/irw_vehicle.spck`](main_model/irw_vehicle.spck)：整车装配、线路、输入输出及已保存的平衡位姿与名义预载；
- [`ref_files/Bogie_R300_FREE_PROFILE_RESOLUTION.spck`](ref_files/Bogie_R300_FREE_PROFILE_RESOLUTION.spck)：构架、定位拉杆与悬挂子结构；
- [`ref_files/IRW_R300_FREE_PROFILE_RESOLUTION.spck`](ref_files/IRW_R300_FREE_PROFILE_RESOLUTION.spck)：轴桥、左右独立车轮及轮轨接触子结构；
- [`ref_files/subvars_OptBase.subvar`](ref_files/subvars_OptBase.subvar)：悬挂、几何及车辆参数；
- `ref_files/几何模型_STL版本_构架.STL` 与 `ref_files/几何模型_STL版本_轴桥.STL`：模型显示几何。

## 运行

默认线路为 R300 右曲线，初速 60 km/h，包含横向、竖向、轨距和侧滚四通道 AAR5 激励。
线路与激励由模型内的 Track、功率谱密度及 Type-108 元素定义。

主模型采用 `S1002.prw` 车轮型面与 `UIC60.prr` 钢轨型面；轴桥子结构另有 `LM.prw` 默认引用。
SIMPACK 型面搜索路径须提供这三个原生型面文件。

从主模型已保存的平衡位姿开始，采用普通初始化。SIMPACK 根据名义预载重算轮轨接触的名义
轨面偏移，存入描述态；`Set vehicle velocities` 设置车辆初速与车轮转速。新运行须执行此初始化，
`--continuation-run` 用于续接已有运行。

八轮转矩经外部输入施加。零转矩被动运行将八路输入持续置零；受控运行由外部控制器更新并保持转矩。

## ORVD 读取入口

SIMPACK 加载上述主模型及子结构。ORVD 原生运行器读取
[`vehicle_definition.json`](../../vehicle_definition.json) 与
[`moving_startup_60kmh.json`](../../startup_states/moving_startup_60kmh.json)，
启动文件中的轨面偏移作为固定接触几何输入使用。命令见
[动力学运行工具](../../../../tools/dynamics_qualification/README.md)。
