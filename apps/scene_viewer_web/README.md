# ORVD scene viewer (web)

自包含的 React + Three.js 回放器，只读取 `ORVD::scene_record` 写出的场景记录目录
（`scene.json`、`frames.f64le`、`scalar_statuses.u8`、`visual_definition.json`）。
浏览器不链接 C++，不重做动力学，也不接收 UDP。记录契约见
[`libs/scene_record/README.md`](../../libs/scene_record/README.md)。

## 运行

```bash
npm install
npm run dev
```

两种加载方式：

- 把记录目录放到（或软链到）`public/records/<name>/`，打开
  `http://localhost:5173/?record=/records/<name>/`；
- 直接在页面上选择记录文件夹（浏览器目录选择）。

深链参数：`record`、`t`（起始秒）、`view`（`overview`、`bogie`、`headon`、`plan`、`side`）、
`hide`（逗号分隔的部件名）、`carbody`（`xray`、`solid`、`hidden`）、`follow`、`orbit`、`labels`、`track`、`axes`。
请求的视角若需要记录未绑定的角色（例如没有构架绑定时的 `bogie`），退回 `overview`；按钮与快捷键同样受此限制。

`npm run build` 先做类型检查再打包到 `dist/`，`npm run preview` 提供打包结果。
`npm run check-record -- <记录目录>` 用查看器自己的解析器与显示绑定解析读一份记录并打印摘要；
`npm run check-interpolation -- <记录目录>` 用相隔十个样本的两帧插值到中间样本时刻并与记录比较，
车轮误差必须小于 0.05 rad；`npm test` 覆盖播放状态归属、插值端点、显示绑定解析、读数模型、线路分段与副标题。

## 生成记录

被动资格运行器带 `--scene-record` 时在其输出目录下写 `scene_record/`，例如从仓库根目录：

```bash
OMP_NUM_THREADS=4 ./build/tools/dynamics_qualification/orvd_irw_passive_scenario \
  irw_r300_no_irregularity_v60_passive \
  vehicle_library/irw/vehicle_definition.json \
  vehicle_library/irw/startup_states/moving_startup_60kmh.json \
  track_library/geometries/r300_centerline_superelevation_1100m.json \
  . none /path/to/output 1000000000 10000000 --scene-record
```

记录格式变化后旧记录会被明确拒绝（例如缺少 `carrier_body_name`），需要重新导出，不做兼容读取。

## 显示绑定

查看器从不按刚体名字猜角色。装载时只解析一次：

1. 记录的 `wheel_placements` 通过 `carrier_body_name` 把左右轮配到各自的载体（独立轮的轴桥，或刚性轮对自身）；
2. 视觉定义的 `display_bindings` 给出车体、按端序的构架列表（含各自显示为运行部的成员刚体）和载体的显示名与次序；
3. 读数卡、场景分组、相机锚点和标注障碍都消费这一份结果（`src/scene/vehicle_display_bindings.ts`）。

引用记录里不存在的刚体、重复归组、漏列载体，在解析时报错并保留原记录；视觉定义未绑定车体或构架，
则相应读数、俯视示意和转向架视角显示为不可用，载体、轮重与接触状态仍然可用。
没有视觉定义时，刚体以坐标轴显示，车轮按放置绘出，载体按记录出现顺序以原名列出。

## 显示约定

- 记录中的坐标是 ORVD 轨道惯性系（x 沿增里程、y 向右、z 向下）；根节点绕 x 转 +90° 变到
  Three.js 的 Y-up，其余代码不置换分量。
- 帧行号（`frameIndex`）是帧表的行序，定位与插值用它；`sampleIndex` 是记录的样本身份，只显示。
- 刚体四元数为 `w,x,y,z`、体系到世界系，已含车轮自转。相邻帧之间位置线性插值、姿态最短路径
  插值；对带 `wheel_spin_angles` 的车轮，帧间旋转按记录里的未折返自转角之差拆成“绕自转轴的
  自转 + 小的残余旋转”再插值，因此半周以上的帧间自转也能选对方向。两端帧处结果就是采样姿态，
  不叠加任何额外旋转。
- 记录没有自转角时，查看器按记录的角速度估计相邻样本间的轮转角；超过四分之一周就在状态行给出警告。
  这是采样限制，不由查看器猜测。
- 车轮的位置、轴向与半径来自记录的 `wheel_placements`，不在视觉定义里手写。
- 读数只显示记录里有的量；状态非“有效”的标量显示为“未就绪”或“占位”，记录未导出的标量显示为“未记录”，
  未记录横移的载体在俯视示意里按标称位置虚线画出，不当作零测量。
