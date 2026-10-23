# ORVD Scene Viewer

基于 React + Three.js 的离线场景查看器，支持车辆回放、物理读数与长线路浏览。

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

`t` 指定起始秒；`view` 可选 `overview`、`bogie`、`headon`、`plan`、`side`。
空格播放／暂停，左右方向键逐样本移动，`1–5` 切换视角，`L` 切换标注，`X` 切换车体显示。

## 记录与显示

记录格式见 [scene_record](../../libs/scene_record/README.md)。
IRW 被动运行器使用 `--scene-record` 导出 `scene_record/`，命令参数见
[运行工具说明](../../tools/dynamics_qualification/README.md)。

车辆外观与角色由记录中的视觉定义指定，IRW 与 GZ18 共用查看器。
读数显示记录中的有效值，缺测显示 `—`。长线路按显示窗口分段加载几何，采用区段局部坐标；
离线样本与线路采样表完整载入。

## 开发

```bash
npm test
npm run build
```

构建包含类型检查，产物位于 `dist/`；`npm run preview` 预览产物。
`npm run check-record -- <记录目录>` 检查记录解析。

后续路线见[场景显示计划](../../docs/planning/scene_visualization/SCENE_VISUALIZATION_IMPROVEMENT_PLAN.md)。
