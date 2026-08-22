# GZ18 SIMPACK 参考模型

本目录保存 ORVD 迁移与资格工作实际参考的 GZ18 SIMPACK 模型快照。它用于来源审计和在具备
SIMPACK 的环境中复查模型，不是 ORVD 运行时车型参数的第二份权威；ORVD 计算仍只读取
`vehicle_library/gz18/` 下的产品 JSON。

## 模型组成

- `main_model/gz18_straight_aar6.spck`：已保存 60 km/h 移动初态的主模型，包含可切换的
  R300＋AAR5 与直线＋AAR6 线路；
- `database/mbs_db_substructure/bogie_motor.spck`：近期资格运行使用的构架子结构，含实际
  Type-5/Type-86 名义预载；
- `database/mbs_db_substructure/wheelset_motor.spck`：刚性轮对子结构。

P035 生成流程在同一个已解析内存状态上保存了模型及求解器状态集；当前主 `.spck` 已携带所需的
关节位置、关节速度、力元动态状态和轮轨接触描述状态，不再重复分发额外状态文件。P035 至 P057
共用这套机械、预载、接触人格和移动初态。P047 的 Result-102 观察派生模型不属于车型机械权威，
因此没有复制到这里。

该模型谱系最早来自 WRL 的
`mbs_simpack/vehicle_GZ18/main_model/vehicle_GZ18.spck`。轨道不平顺由 SIMPACK 模型内部的
Type-108 功率谱密度设置生成；AAR5 与 AAR6 使用当前公共谱、随机实现和门控合同。导出的
点列是跨实现复现数据，不是本参考模型的输入依赖。本快照不需要 `.tre`：不平顺定义及功率谱已
包含并连接在主 `.spck` 内，缺少外部 `.tre` 并不表示无轨道不平顺。

## 模型边界

主模型指向子结构数据库的搜索路径使用包内相对路径 `../database`。

模型继续以 `LM.prw` 和 `UIC60.prr` 的标准名称引用轮轨型面。ORVD 不分发这些原生型面；运行前
必须确保 SIMPACK 型面搜索路径能提供这两个文件。不要把 ORVD 的 JSON 型面反向转换成 SIMPACK
型面来替换它们。

当前 `vehicle.applystartvel=1` 启动态须使用普通的 `simpack-slv --integration` 求解，让求解器按
`vehicle.startvel` 建立相容的车辆与轮系速度；不得使用 `--continuation-run` 绕过这一步。运行日志
必须出现 `Set vehicle velocities.`，否则本次求解不能作为当前 60 km/h 启动合同的复现结果。

本目录不保存 `.sbr`、`.sir`、后处理工程、日志或其他运行结果，也不随 ORVD 二进制安装包安装。
