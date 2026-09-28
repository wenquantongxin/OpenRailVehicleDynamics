# ORVD scene viewer (web)

自包含的 React + Three.js 回放器，只读取 `ORVD::scene_record` 写出的场景记录目录
（`scene.json`、`frames.f64le`、`scalar_statuses.u8`、`visual_definition.json`）。
浏览器不链接 C++，不重做动力学，也不接收 UDP。界面以英文为主、中文作副标。

## 运行

```bash
npm install
npm run dev
```

两种加载方式：

- 把记录目录放到（或软链到）`public/records/<name>/`，打开
  `http://localhost:5173/?record=/records/<name>/`；
- 在页面上用 “Open record · 打开记录” 选择记录文件夹。

`npm run build` 先做类型检查再打包到 `dist/`，`npm run preview` 提供打包结果。从局域网另一台电脑访问时，
用 `npx vite preview --host <本机局域网地址>` 启动，再在那台电脑上打开 `http://<本机局域网地址>:4173/`；
`localhost` 指的是浏览器所在的电脑。预览服务读的是
`dist/`，新记录放进 `public/records/` 后要重新 `npm run build`。
`npm run check-record -- <记录目录>` 用查看器自己的解析器在 Node 中读一份记录并打印摘要；
`npm run check-interpolation -- <记录目录>` 用相隔十个样本的两帧插值到中间样本时刻并与记录比较，
车轮误差必须小于 0.05 rad；`npm test` 检查播放状态归属、车轮插值、接触未就绪的显示、由位姿计算的俯视转角、曲线分段与轨道基准、长记录标题。

## 生成记录

被动资格运行器带 `--scene-record` 时在其输出目录下写 `scene_record/`，例如从仓库根目录：

```bash
OMP_NUM_THREADS=4 ./build/tools/dynamics_qualification/orvd_irw_passive_scenario \
  irw_r300_no_irregularity_v60_passive \
  vehicle_library/irw/vehicle_definition.json \
  vehicle_library/irw/startup_states/moving_startup_60kmh.json \
  track_library/geometries/r300_centerline_superelevation_1100m.json \
  . none /path/to/output 45000000000 10000000 --scene-record
```

45 s 覆盖直线、缓和曲线、R300 圆曲线和出口缓和曲线。视觉定义随记录原样复制，改了
`vehicle_library/irw/visualization/visual_definition.json` 之后要重新导出记录。

## 画面与操作

- 视角（Views · 视角）：整车、一位端转向架、迎面、俯视、侧视。预设写在车体所在里程处的线路坐标系里，
  曲线上 “迎面” 仍在车前；切换有 0.9 s 补间。“跟随” 让相机随车体和线路方向移动，“环绕” 缓慢绕行。
- 显示（Display · 显示）：车体透视、实体或隐藏；部件标注；线路；坐标轴；全部标量抽屉。
- 部件标注只在部件在画面中足够大时出现；名称来自视觉定义。两台转向架分别标注：一位端是车体坐标 +x 的一端，
  转向架、轴位和车轮都从一位端起编号；轴桥和车轮用轴位号区分。标注放在锚点旁的空位：不压转向架轮廓、
  互不重叠、引线不交叉，放不下时隐藏。
- 卡片：01 轴桥（接触状态、按原比例绘制的俯视图、各轴读数）、02 线路（平面小图、里程、
  区段、半径、超高、车速）、03 车辆（车体和构架相对线路的横移与摇头）、04 轮重、07 回放。
- 时间轴带有车体处的曲率、曲线要素点（TS 直缓、SC 缓圆、CS 圆缓、ST 缓直）和接触事件
  （橙色两点接触，红色轮轨分离）。

键盘：空格播放或暂停，← → 逐样本，Shift + ← → 跳 1 s，1–5 切视角，L 标注，X 车体模式，
F 跟随，O 环绕，T 线路，A 坐标轴，S 全部标量，Esc 关闭抽屉。

深链参数：`record`、`view`（`overview`、`bogie`、`headon`、`plan`、`side`）、`t`（秒）、`hide`
（逗号分隔的部件名）、`labels=1`、`carbody=xray|solid|hidden`、`follow=0`、`orbit=1`、`track=0`、`axes=1`。

## 显示约定

- 记录中的坐标是 ORVD 轨道惯性系（x 沿增里程、y 向右、z 向下）；根节点绕 x 转 +90° 变到
  Three.js 的 Y-up，其余代码不置换分量。
- 刚体四元数为 `w,x,y,z`、体系到世界系，已含车轮自转。相邻帧之间位置线性插值、姿态最短路径
  插值；对带 `wheel_spin_angles` 的车轮，帧间旋转按记录里的未折返自转角之差拆成“绕自转轴的
  自转 + 小的残余旋转”再插值。两端帧处结果就是采样姿态，不叠加任何额外旋转。
- 车轮的位置、轴向与半径来自记录的 `wheel_placements`；轮缘朝车轮刚体原点一侧，即车辆内侧。
  车轮踏面带按该轮的承载接触斑数着色：两点接触为橙色，轮轨分离为红色，未就绪为灰色。未就绪不按单点接触处理：
  部分车轮未就绪时状态显示 “Contact partly not ready 部分接触未就绪” 和有效轮数，时间轴也不把它记为事件。
- 三维画面和轴桥俯视图都按原比例绘制，不放大位移。毫米级横移和毫弧度级摇头在图上几乎看不出，数值见读数表。
  俯视图中轴桥的转角由记录位姿与当地线路坐标计算，向右轨转为正；表中的摇头角是记录的原始标量
  （车体基下的 X-Z-Y 分解，正方向随车体基而定，悬停表头可见定义），两者不混用。
  俯视图在两转向架之间用断开画法省略车体段，并附 1 m 比例尺。
- 线路显示几何（钢轨、轨枕、道床、里程标）全部由记录的线路表构造，只含设计线形，不含不平顺。钢轨、垫板和扣件
  放在记录的左右轨基准点上，轨枕与道床按记录轨距缩放。地面平面位于线路最低点下方，路基带沿线跟随中心线高度，
  线路较高处以 1:1.5 边坡落到地面，所以有坡度或整体抬高的线路既不悬空也不埋入地面。
  区段与曲线要素点由记录的曲率识别：|k| 小于最大曲率的 1e-4 为直线，曲率平台为圆曲线，其余为
  缓和曲线；每个曲率平台保留自己的半径，不同半径的圆曲线不会合并；在随附的 R300 线路上要素点与线形定义相差不超过 0.3 m。
- 读数只显示记录里有的量；状态非“有效”的标量显示 “Not ready 未就绪” 或 “Placeholder 占位”。
  车速是车体线速度的模；条形刻度取整条记录的最大值并写在卡上。

## 提示与缺省

- 已有记录时再打开失败，原记录保留，页面顶部显示错误提示和原因，可关闭。
- 记录没有视觉定义时，每个刚体以坐标轴显示，车轮按记录放置用示意截面绘出，并显示提示。
- 深链 `hide` 隐藏的部件不受车体模式、线路、坐标轴开关影响。

## 视觉定义

`visual_definition.json` 由车辆库提供，渲染器无关：

- 每个零件是 `box`（可带 `corner_radius_meters`）、`cylinder`、`sphere` 或 `axes`，写在所属刚体的体系里；
- `appearance` 是显示角色（`shell`、`glass`、`floor`、`structure`、`accent`、`dark`），不是材质；
- 可选 `label` 给出 `en`、`zh` 名称与锚点；车轮标注写在 `wheel_visual.labels`，按轮轨接口名指定；
- `wheel_visual` 给出示意轮截面（宽度、背面偏移、轮缘高与厚、轮辋深、辐板厚、轮毂）。

## 字体

不加载网页字体，按平台使用系统字体：macOS 用 SF、Menlo、PingFang SC；Windows 10/11 用 Segoe UI、
Consolas、Microsoft YaHei；Ubuntu 24.04 用 Ubuntu Sans、Ubuntu Sans Mono、Noto Sans CJK SC。

## 截图

无头 Chrome 的 `--screenshot` 参数可能在首帧 WebGL 画面出来之前截图，得到只有界面没有三维的图。
用 DevTools 协议等待若干秒后再调用 `Page.captureScreenshot` 更可靠。
