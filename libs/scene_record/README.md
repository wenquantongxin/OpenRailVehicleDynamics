# scene_record

`ORVD::scene_record` 读写场景记录与静态资源包，保存公共场景观察（`ORVD::scene_observation`）的
拓扑、逐帧刚体状态、标量与线路采样。
接口见 [`scene_record_writer.h`](include/orvd/scene_record/scene_record_writer.h)、
[`scene_record_reader.h`](include/orvd/scene_record/scene_record_reader.h) 与
[`scene_assets.h`](include/orvd/scene_record/scene_assets.h)。公共描述编解码属于
[`scene_observation`](../scene_observation/README.md)。

## 目录组成

| 文件 | 内容 |
|---|---|
| `scene.json` | `schema`、公共 `description`、静态 `assets`、`frame_table` 与 `scalar_status_table` |
| `frames.f64le` | 帧表：每帧一行小端 binary64 |
| `scalar_statuses.u8` | 每帧每标量一个状态字节：0 未就绪、1 有效、2 占位 |
| `visual_definition.json` | 调用方提供的参数化视觉定义，校验为 JSON 对象后原样复制；可以没有 |

`SceneRecordWriter(directory, description, assets, with_wheel_spin_angles)` 写入记录，
`ReadSceneRecord(directory)` 返回 `SceneRecord{description, assets, frames}`。
记录的 `schema` 为 `orvd.scene_record`。写入目录须不存在；`Close()` 发布 `scene.json`，
读取器以该文件识别完整记录。

## 静态资源包

`SceneAssets` 包含可选 `track`、原样视觉定义文本 `visual_definition_json` 与 `world_frame`。
空视觉文本表示缺省。`WriteSceneBundle(directory, assets)` 写入不含帧表的目录，最后发布的
`scene.json` 只有 `schema: "orvd.scene_bundle"` 和 `assets`；`ReadSceneBundle(directory)` 读取它。
`ReadSceneAssets(scene_json_path)` 可从记录或资源包读取静态资源，无需帧文件。

`assets` 对象包含 `world_frame`、`length_unit: "meter"`、`angle_unit: "radian"`、
可空 `track` 和可空 `visual_definition_file`。轨道包含七列采样数组，并保存
`rail_datums.left/right.{lateral_meters,vertical_meters}`，即采样时实际使用的轨道系轨面基准。
所有列等长非空，里程严格递增，几何数值有限；轨道姿态为单位四元数。
轨道可在构造时提供，或在关闭记录前通过 `WriteTrackSampleTable` 写入一次。

`ComposeSceneAssets(base, selected)` 按资产项覆盖：selected 有轨道便替换轨道，有视觉文本便替换视觉，
缺省项沿用 base；双方世界系必须相同。记录的 description 与 assets 世界系也必须一致。

## 帧表布局

一行的列依次为：`time_seconds`、`time_nanoseconds`、`sample_index`、`phase`，
然后每个刚体 13 个值（位置 3、四元数 `w,x,y,z` 4、原点线速度 3、角速度 3），
然后每个车轮放置一个未折返自转角（记录带自转角列时），然后每个标量一个值。
各块的偏移与个数由 `scene.json` 的 `frame_table.columns` 声明。

- **帧行号与样本身份**：帧行号是帧表的 0 起行序，定位与插值用它；`sample_index` 是运行程序给出的样本身份，
  可与行号不同，缺失时为 `-1`（`absent_integer_identity`），只用于显示。`time_nanoseconds` 同理。
- **整数身份范围**：整数身份经 binary64 存储，绝对值不超过 `integer_identity_exact_range`（2^53）时精确，超出即拒绝写入。
- **坐标与单位**：位置为米，角为弧度，时间为秒；世界系是 ORVD 模型世界系，轨道场景下即轨道惯性系 I（+x 沿线路原点处的增里程方向、+y 向右、+z 向下）。
  四元数把刚体坐标映射到世界坐标；车轮刚体的姿态已含自转。
- **未折返自转角**：`wheel_spin_angles` 是各车轮绕 `spin_axis_in_wheel_body_frame` 的连续转角，只用于选择两样本之间的旋转分支，不叠加到姿态上；没有独立回转副的刚性轮对不提供此列。
- **缺测**：状态不是“有效”的标量必须写 0，读取方按状态显示，不把 0 当测量值。

## 车轮放置

`description.wheel_placements` 每项来自接触计划：`interface_name`、`wheel_body_name`、
`carrier_body_name`、`side`、`datum_in_wheel_body_frame_meters`、`spin_axis_in_wheel_body_frame`、
`nominal_rolling_radius_meters`、可空 `spin_joint_name` 与可空 `spin_angle_scalar`。`carrier_body_name` 是承载该轮非自转型面系的刚体：独立旋转车轮的轴桥，
或刚性轮对自身（此时等于 `wheel_body_name`）。显示端通过它把同一载体的左右轮配对。公共描述编解码拒绝引用拓扑未列出刚体的放置。
`spin_angle_scalar` 用 `{name, sign}` 引用单位为 `rad` 的标量，sign 为 +1 或 -1。

## 标量命名约定

`scene_record` 保存调用方给出的标量定义、数值与状态；公共描述要求标量名非空且唯一。资格运行器导出的车辆标量按下列约定命名，
查看器据此查找：

- 轨道坐标：`<名称>.track_station_meters`、`<名称>.lateral_meters`、`<名称>.yaw_radians`；摇头为 Track-T 平面内右偏为正的方向角，车体／构架取前向轴，轮对／轴桥取轮轴的法向。
- 车轮的接触结果：`<接口名>.contact_patch_count`、`<接口名>.vertical_support_force_on_wheel_newtons`、`<接口名>.normal_force_newtons`。

资格运行器的载体通道以接触载体名为前缀，代表刚体通道以刚体名为前缀。

## 消费方

安装后通过 `find_package(OpenRailVehicleDynamics CONFIG REQUIRED)` 链接 `ORVD::scene_record`。
查看器见 [`apps/scene_viewer_web`](../../apps/scene_viewer_web/README.md)。
