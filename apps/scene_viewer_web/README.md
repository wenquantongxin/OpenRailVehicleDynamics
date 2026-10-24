# ORVD Scene Viewer

基于 React + Three.js 的场景查看器，支持离线回放、实时观察、物理读数与长线路浏览。

## 启动与打开记录

从仓库根目录运行：

```bash
cd apps/scene_viewer_web
npm install --package-lock=false
npm run dev
```

打开终端显示的地址，默认是 `http://localhost:5173/`。Windows PowerShell 使用 `npm.cmd`。

- 点击 **Open record**，选择场景记录文件夹。
- 或将记录放入 `public/records/<name>/`，访问
  `http://localhost:5173/?record=/records/<name>/&t=20&labels=1`。

通过 `?record=/records/<name>/&bundle=/bundles/<name>/` 为记录补载资源包。两个地址均指向包含
`scene.json` 的目录，相对地址以页面地址解析。资源包提供的轨道或视觉定义覆盖记录内对应资源，
未提供的项沿用记录。仅给出 `bundle` 时仍需选择记录；文件夹打开始终使用文件夹内的完整资源。

`t` 指定起始秒；`view` 可选 `overview`、`bogie`、`headon`、`plan`、`side`。
空格播放／暂停，左右方向键逐样本移动，`1–5` 切换视角，`L` 切换标注，`X` 切换车体显示。

## 实时观察

在查看器目录构建并启动服务：

```bash
npm run build
npm run serve -- --bundle demo=/absolute/path/to/record-or-bundle
```

打开 `http://localhost:6641/?live=1`，生产者将 OSTS UDP 消息发送到 `127.0.0.1:10099`，
描述中的 `resource_bundle` 填写 `demo`。服务只发布配置目录中的静态资源。
`--host`、`--port` 设置网页监听地址，`--udp-host`、`--udp-port` 设置 UDP 监听地址；
`--bundle 名称=目录` 可重复提供。开发模式将 `/live` 与 `/bundles` 代理到本机 6641 端口。

实时模式可暂停显示或恢复实时，接收持续进行；新运行自动接续。
断线自动重连，同运行保留视角与暂停画面；实时传输保留最新状态，可合并中间样本。
`live` 与 `record` 互斥，`bundle=` 可显式覆盖资源。
使用 ORVD 记录作为数据源，见[回放发送器](../../tools/state_stream/README.md)。

## 记录与显示

记录格式见 [scene_record](../../libs/scene_record/README.md)。
GZ18 与 IRW 被动运行器使用 `--scene-record` 导出 `scene_record/`，命令参数见
[运行工具说明](../../tools/dynamics_qualification/README.md)。

车辆外观与角色由记录中的视觉定义指定，IRW 与 GZ18 共用查看器。
读数显示已发布的有效值，缺测显示 `—`。轮角优先使用专用列，无专用列时使用描述中绑定的有效 `rad` 标量及其符号。
长线路按显示窗口分段加载几何，采用区段局部坐标。离线样本与线路表完整载入；实时样本采用有界缓冲。

## 开发

```bash
npm test
npm run build
```

构建包含类型检查，产物位于 `dist/`；`npm run preview` 预览产物。
`npm run check-record -- <记录目录>` 检查记录解析。

模块关系见[场景数据与显示](../../docs/planning/scene_visualization/SCENE_VISUALIZATION_IMPROVEMENT_PLAN.md)。
