# scene_record

`ORVD::scene_record` 把公共场景观察（`ORVD::scene_observation`）的拓扑、逐帧刚体状态、
标量与线路采样写成一个可携带的目录，并能把它读回来。它不依赖 UDP、React 或 Three.js，
也不依赖 `tools/dynamics_qualification`；运行器只是它的一个调用方。
接口见 [`scene_record_writer.h`](include/orvd/scene_record/scene_record_writer.h) 与
[`scene_record_reader.h`](include/orvd/scene_record/scene_record_reader.h)，本文只说明契约，不另立规格。

## 目录组成

| 文件 | 内容 |
|---|---|
| `scene.json` | 记录标识、世界系与单位、刚体列表、车轮放置、标量定义、帧表布局、状态码、线路采样表、视觉定义文件名 |
| `frames.f64le` | 帧表：每帧一行小端 binary64 |
| `scalar_statuses.u8` | 每帧每标量一个状态字节：0 未就绪、1 有效、2 占位 |
| `visual_definition.json` | 调用方提供的参数化视觉定义，原样复制、不解析；可以没有 |

`Close()` 最后写 `scene.json`。没有 `scene.json` 的目录是未完成记录，读取器拒绝；写入器也拒绝已存在的目录。

## 帧表布局

一行的列依次为：`time_seconds`、`time_nanoseconds`、`sample_index`、`phase`，
然后每个刚体 13 个值（位置 3、四元数 `w,x,y,z` 4、原点线速度 3、角速度 3），
然后每个车轮放置一个未折返自转角（记录带自转角列时），然后每个标量一个值。
各块的偏移与个数写在 `scene.json` 的 `frame_table.columns` 中，读取方按文件所述布局读，不假设。

- **帧行号与样本身份**：帧行号是帧表的 0 起行序，定位与插值用它；`sample_index` 是运行程序给出的样本身份，
  可与行号不同，缺失时为 `-1`（`absent_integer_identity`），只用于显示。`time_nanoseconds` 同理。
- **整数身份范围**：整数身份经 binary64 存储，绝对值不超过 `integer_identity_exact_range`（2^53）时精确，超出即拒绝写入。
- **坐标与单位**：位置为米，角为弧度，时间为秒；世界系是 ORVD 模型世界系，轨道场景下即轨道惯性系 I（+x 沿线路原点处的增里程方向、+y 向右、+z 向下）。
  四元数把刚体坐标映射到世界坐标；车轮刚体的姿态已含自转。
- **未折返自转角**：`wheel_spin_angles` 是各车轮绕 `spin_axis_in_wheel_body_frame` 的连续转角，只用于选择两样本之间的旋转分支，不叠加到姿态上；没有独立回转副的刚性轮对不提供此列。
- **缺测**：状态不是“有效”的标量必须写 0，读取方按状态显示，不把 0 当测量值。

## 车轮放置

`wheel_placements` 每项来自接触计划的定义，不由名字推断：`interface_name`、`wheel_body_name`、
`carrier_body_name`、`side`、`datum_in_wheel_body_frame_meters`、`spin_axis_in_wheel_body_frame`、
`nominal_rolling_radius_meters`。`carrier_body_name` 是承载该轮非自转型面系的刚体：独立旋转车轮的轴桥，
或刚性轮对自身（此时等于 `wheel_body_name`）。显示端通过它把同一载体的左右轮配对。写入器拒绝引用拓扑未列出刚体的放置。

## 标量命名约定

`scene_record` 只保存调用方给出的标量定义、数值与状态，不检查名称。资格运行器导出的车辆标量按下列约定命名，
查看器据此查找：

- 刚体的轨道坐标：`<刚体名>.track_station_meters`、`<刚体名>.lateral_meters`、`<刚体名>.yaw_radians`；
  当前导出接触计划的载体和配方的代表刚体。
- 车轮的接触结果：`<接口名>.contact_patch_count`、`<接口名>.vertical_support_force_on_wheel_newtons`、`<接口名>.normal_force_newtons`。

生产端状态：载体标量的前缀目前取接触计划的 carrier 名，两个闭合场景里它恰好等于载体刚体名；
运行器改为显式取载体定义的 `body_name` 是后续收口项，见
[三维场景显示综合完善计划](../../docs/planning/scene_visualization/SCENE_VISUALIZATION_IMPROVEMENT_PLAN.md)。

## 消费方

安装后通过 `find_package(OpenRailVehicleDynamics CONFIG REQUIRED)` 链接 `ORVD::scene_record`。
自包含查看器 [`apps/scene_viewer_web`](../../apps/scene_viewer_web/README.md) 用自己的解析器读同一目录，不链接 C++。
