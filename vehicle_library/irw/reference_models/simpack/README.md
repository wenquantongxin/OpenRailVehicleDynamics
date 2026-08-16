# IRW SIMPACK 参考模型

本目录保存 ORVD 迁移与资格工作采用的 IRW 平衡名义预载 SIMPACK 车型。它用于来源审计和在具备
SIMPACK 的环境中复查模型，不是 ORVD 运行时车型参数的第二份权威；ORVD 计算仍只读取
`vehicle_library/irw/` 下的产品 JSON。

## 模型组成

- `main_model/irw_vehicle.spck`：平衡名义预载车型主模型，包含多条可切换线路；
- `ref_files/Bogie_R300_FREE_PROFILE_RESOLUTION.spck`：构架子结构；
- `ref_files/IRW_R300_FREE_PROFILE_RESOLUTION.spck`：轴桥与左右独立车轮子结构；
- `ref_files/subvars_OptBase.subvar`：由优化结果经工程化收敛得到的活动参数集；
- `ref_files/几何模型_STL版本_构架.STL` 与 `ref_files/几何模型_STL版本_轴桥.STL`：模型显示几何。

车型、子结构与活动参数源自 WRL 提交 `d7e272df8c29a2074d70eee7cd5a24cfede78d83` 的平衡名义
预载人格；当前主模型采用 SIMPACK 默认的 SODASRT2 求解器。该人格的每轮名义预载约为
`49,287.489 N`，已接受初始压缩轮载约为 `49,292.951 N`，竖向基准约为 `-175.493 µm`。
历史零预载人格以及 `-86.955117 µm` 竖向补偿均不属于当前车型。

该模型谱系最早来自 WRL 的
`mbs_simpack/irw_4WDB/main_model/Vehicle4WDB_R300mV60kmph.spck`。主模型另从 WRL 当前车型的
R600、R800、R1000 以及 80/120/160/200 km/h 直线模型导入对应
Track 元素；只搬入线路及其必要激励，不搬入各源文件中的车辆、求解器、控制器或输出设置。

主模型以平面—竖向组合线路为活动 Track，其他曲线与直线线路保留为可切换元素；AAR5、AAR6
和 ERRI low 均使用模型内部的功率谱密度与 Type-108 激励，不加载导出的轨道点列。本快照
不需要 `.tre`；被动复现时八路外部转矩持续写零，因此“被动”只表示无主动控制。

模型继续以 `S1002.prw`、`LM.prw` 和 `UIC60.prr` 的标准名称引用轮轨型面。ORVD 不分发这些原生
型面；运行前必须确保 SIMPACK 型面搜索路径能提供这三个文件。不要把 ORVD 的 JSON 型面反向转换成
SIMPACK 型面来替换它们。

被动运行采用普通初始化，使 `Set vehicle velocities` 写入车辆速度与八轮初始转速；不得使用
`--continuation-run` 绕过这一步。

初始化阶段会由八个 `$F_PS_BarFixed_*` Type-43 力元报告既有的大转角警告；本快照保留原模型的
角度计算模式，不借搬运模型之机改变已登记的 SIMPACK Type-43 与冻结 WRL 衬套语义差异。

本目录不保存 `.sbr`、`.sir`、后处理工程、日志、优化表格或其他运行结果，也不随 ORVD 二进制安装包安装。
