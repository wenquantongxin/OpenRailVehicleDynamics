# scene_observation

`ORVD::scene_observation` 从模型与接触计划采样场景值。
公共 [`SceneDescription`](include/orvd/scene_observation/scene_description.h) 包含 `world_frame`、
`topology`、`scalar_definitions` 与可选 `nominal_sample_period_seconds`。
`EncodeSceneDescription` / `DecodeSceneDescription` 在该值与 JSON 对象间往返。

描述 JSON 包含世界系、长度/时间/角度单位、四元数语义、刚体、车轮放置、标量定义、状态码、阶段码与名义采样周期。
`kSceneWorldFrame`、`kSceneLengthUnit`、`kSceneTimeUnit`、`kSceneAngleUnit`、`kSceneOrientation`
固定公共坐标与单位文本；自定义 world_frame 仍须与使用的静态资源一致。
刚体、车轮接口、标量各自名称非空且唯一，车轮必须引用已声明刚体；车轮几何有限、自转轴为单位向量、名义滚动半径为正。

车轮的可选 `spin_joint_name` 完整往返；可选 `spin_angle_scalar: {name, sign}` 明确指定显示使用的自转标量。
它引用的标量必须存在、单位必须为 `rad`，sign 为 +1 或 -1。
名义周期须为有限正数，实际时间由各帧给出。

`TrackSampleTable` 保存中心线、姿态、曲率、超高与左右轨面世界坐标七列采样值，同时保留
`left_rail_datum_in_track`、`right_rail_datum_in_track`。
`SampleTrackGeometry` 原样保存实际输入的两个 `RailDatumPlacement`，其横向向右为正、垂向向下为正。
记录与可独立加载的资源包由 [`scene_record`](../scene_record/README.md) 管理。
