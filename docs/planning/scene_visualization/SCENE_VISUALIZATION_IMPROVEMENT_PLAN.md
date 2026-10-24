# 场景数据与显示

不同车型、线路和工况共用场景描述、离线回放与实时观察接口。

## 职责

- `scene_observation` 提供公共描述、位姿和物理读数，`scene_record` 保存记录与独立资源。
- `scene_stream` 提供可移植消息编码，`state_stream` 负责 Linux UDP 发送。
- 查看器负责渲染与交互；车型外观、角色和名称由视觉定义提供。

显示消费已发布样本，数值推进与控制时钟由运行程序管理。
文件与实时输入共用场景定义和样本语义，静态资源、离线历史与实时缓冲分别管理。

## 数据组织

- 长线路分段绘制，使用区段局部坐标。
- 静态资源包提供轨道与视觉定义。
- 实时服务接收 UDP，通过 WebSocket 发布样本，查看器使用有界缓冲。

使用方式见[查看器说明](../../../apps/scene_viewer_web/README.md)，数据契约见
[场景记录说明](../../../libs/scene_record/README.md)与 [OSTS 规格](../../../libs/scene_stream/README.md)。
