# GZ18 / IRW 参考模型与 ORVD 动力学模型差异观察表

## 文档职责

本文件专门登记 GZ18、IRW 的 SIMPACK 参考模型与 ORVD 第一方动力学模型之间可能存在的机械、
力元、轮轨、控制或数值时相差异。重点是 WRL 多轮宏观闭合与误差归因仍可能遗漏、但在迁移时
重新阅读源模型才暴露的“漏网项”。

本文件不是实施路书，也不是数值验收清单。条目只有在形成明确裁决后，才进入
[DISCUSSION_AND_DECISION_LOG.md](DISCUSSION_AND_DECISION_LOG.md) 或
[MIGRATION_ROADMAP.md](MIGRATION_ROADMAP.md)。历史结果、提交标识和工件路径只能作为调查入口，
不得成为 ORVD 产品身份、运行时依赖或数值金标。

## 记录边界

只登记下列问题：

- 两端实际消费的拓扑、参数、作用点、表达坐标系或更新时相可能不同；
- WRL 曾以等效合并、占位体、默认值或车型专用逻辑代替参考模型中的另一种表达；
- 宏观响应可能因抵消、工况未激发或观测量不足而掩盖局部差异；
- 迁移时容易把休眠定义误认成活动物理，或把另一车型的实现误套到当前车型。

不登记普通的代码组织差异、已经明确裁决的坐标换基、纯命名差异，以及没有实际消费者的孤立
参数或标记，除非它们足以诱发迁移误建。

引用参考模型时保留其 `$B_DUM`、`$M_DUM_*` 等原始标识。ORVD 不把 `DUM` 解释成物理部件，
而将两具承担子模型连接职责的刚体命名为 `front_carbody_interface_body` 与
`rear_carbody_interface_body`；其零自由度刚性约束仍采用多体动力学通用术语 `weld`。

## 状态口径

| 状态 | 含义 |
|---|---|
| 待核实 | 已有具体疑点，但尚未同时核对两端的实际消费者。 |
| 已确认差异 | 已证明两端活动动力学语义不同，尚未裁决 ORVD 应采用哪一种。 |
| 已裁决 | 差异成立，且迁移处置已经明确。 |
| 已否证 | 表面结构不同，但两端活动动力学同义，或疑点来自休眠定义、错误类比。 |
| 重新打开 | 新证据使先前结论失效，需要重新核验。 |

每个条目应回答：参考模型真正计算了什么、WRL/ORVD 真正计算了什么、差异是否进入力或状态、
可能影响哪些工况，以及最小下一步是什么。仅看到同名参数或标记不足以判定差异。

## 活跃候选

- MD-002：GZ18 活动曲线的超高模式整数与实际滚转基准尚未建立可靠映射。**不影响已完成的直线 G51–G61 资格**：
  这些工况是直线、水平、零超高，`kind` 在那里不可观测。它现在是 G62/G63 首个正式 SIMPACK 曲线资格的前置事实，
  须以不含车辆的线路查询核实，不再继续后移。
- MD-007：三维互穿纵向长度解析失败时，QCH 未公开处置语义；近期 WRL 使用解析弦长基线并计数。
  项目负责人已裁决恢复 WRL 兼容语义，G52b 后续修复已经落实。
- MD-008：现有迁移来源把 Kalker 表外渐近公式归于 Kalker 1972a／1990 Table E3，但本轮未独立持书
  逐式复核；SIMPACK 在 `a/b=0.1/10` 的精确切换语义也尚未资格化。当前迁移继续复现 WRL 的
  硬切换，不以车辆响应边界规避，也不擅自平滑。
- MD-009：不平顺空间场的采样变化率是否应解释为轨侧材料速度，QCH／SIMPACK 消费语义尚未核实。
  当前 G57 主线明确复现近期 WRL：不平顺值与斜率进入型面位姿，轨侧材料点速度保持为零；本条不阻塞
  G56/G57，也不为潜在替代解释增加运行时计算。
- MD-010：不平顺两条纵向斜率构成的完整轨型姿态是否还应整体旋转接触力坐标系。近期 WRL 与 ORVD
  都只用完整姿态定位钢轨材料参考点，力坐标系仍由横断面接触角构造；当前迁移保持该冻结语义，
  待同输入 SIMPACK 对拍后再决定是否重开。
- MD-011：ORVD 蠕滑参考速度使用三维路径速度，近期 WRL 使用平面站位速率。现有资格线路水平，
  两者逐位相同；首个真实非零纵坡资格工况前须裁决，不在 G57 静默改式。
- MD-012：IRW 末期冻结 WRL 的八个纵向拉杆构架侧衬套采用半角系与共同中点，SIMPACK QCH
  Type-43 则采用 From 基本构、真实 From/To 端点及仅 From 侧平动力支承矩。当前迁移先复现冻结
  WRL；完成 IRW 全流程 ORVD—WRL 端到端闭合后，再重开与 SIMPACK 的原子语义对拍。

## 已筛查条目

### MD-001 — GZ18 二系垂向阻尼是否错误地使用了空气弹簧作用点

- 车型：GZ18
- 层级：二系悬挂力元与作用点
- 状态：**已否证；`Csd` 的路由、作用点和力臂不是 SIMPACK—WRL 差异，ORVD 尚待 G50 实现**

#### 疑点

GZ18 的 SIMPACK 子结构定义了 `$M_BF_SD_{L,R}` 与 `$M_DUM_SD_{L,R}`。前、后两个转向架实例
合计 8 个二系垂向减振器标记；WRL 的 GZ18 建树没有创建它们，而是把 `Csd=20 kN·s/m`
作为空气弹簧竖向阻尼。若 SIMPACK 另有独立垂向减振器连接这些 SD 标记，两端就会因纵向、横向
作用点不同而产生不同的支承矩。

#### 核实结果

该推断不成立：GZ18 的 SIMPACK 活动模型也没有用这些 SD 标记连接任何力元。

1. `bogie_motor.spck` 每个转向架确实定义 4 个 SD 标记：BF 左右各一、DUM 左右各一；主模型
   实例化前后两个转向架，因此共有 8 个。
2. GZ18 整个 SPCK 树中没有 `force.from` 或 `force.to` 引用这些 SD 标记；`$_Ksd` 只有定义，
   没有活动消费者。
3. SIMPACK 的 `$F_AirSpring_A/B` 实际连接 `$M_BF_SS_{L,R}` 与 `$M_DUM_SS_{L,R}`，并把
   `$_Csd*1000` 写入 Type-5 力元的竖向并联阻尼参数。
4. WRL GZ18 使用相同的 SS 端点，并以 `d_air=(0,0,Csd)` 构造空气弹簧衬套。因此就所质疑的
   `Csd` 竖向阻尼而言，两端路由、作用点和力臂一致；本条不提前声称尚未迁入的 ORVD 力元已同义。
5. 后转向架虽把 `bsd_x` 覆盖为 `-0.44 m`，该值只改变未被力元消费的 SD 标记，不进入动力学。
   前转向架中 `b2=bsd_y=1.9 m`、`h2=hsd=0.638 m`、`bsd_x=0`，SD 与 SS 位置本来就重合。
6. SH17 确实创建独立 SD 标记并将它们接入竖向 Maxwell 通道；这是另一车型的活动机械定义，
   不能反推 GZ18 也应采用该表达。

#### 迁移约束

- G49 不因这 8 个休眠 SPCK 标记扩充 GZ18 活动机械图。
- G49 中源模型的两具 `$B_DUM` 只按真实职责建为前、后车体接口体，不把它们误称为二系悬挂承座。
- G50 应继续把 `Csd` 作为空气弹簧 SS 端点间的竖向阻尼，不得仅凭 `Ksd/Csd` 名称或 SH17
  实现，额外创建独立 GZ18 二系垂向减振器。
- 只有在参考模型以后新增了实际引用 SD 标记的力元，或取得与当前 SPCK 相矛盾的活动模型证据时，
  才把本条重新打开。

#### 源码锚点

- WRL `mbs_simpack/vehicle_GZ18/main_model/vehicle_GZ18.spck`：前后转向架子结构实例及后架
  `bsd_x=-440 mm` 覆盖。
- WRL `mbs_simpack/vehicle_GZ18/database/mbs_db_substructure/bogie_motor.spck`：SD 标记定义；
  `$F_AirSpring_A/B` 的 SS 端点与 `Dz=$_Csd*1000`。
- WRL `scripts_cpp/rigid_wheelset/src/gz18_marker_frames.cc`：GZ18 SS 标记。
- WRL `scripts_cpp/rigid_wheelset/src/gz18_force_elements.cc`：空气弹簧 `d_air.z=Csd` 及 SS 端点。
- WRL `scripts_cpp/rigid_wheelset/src/sh17_marker_frames.cc`、`sh17_force_elements.cc`：仅作
  “另一车型确有独立 SD 通道”的反例。
- WRL `mbs_simpack/vehicle_GZ18/GZ18列车的SIMPACK设置整理.md`：参数和活动消费者的既有说明。

### MD-002 — GZ18 活动曲线的超高模式整数表示什么

- 车型：GZ18
- 层级：轨道几何与启动身份
- 状态：**待核实；不得把未经验证的整数映射写成已确认差异**

#### 已知事实与未证映射

1. GZ18 活动线路是地图式线路（Cartographic Track）`$Trk_Curve_R300m_60kmph`。其
   `track.cart.superelev.kind=1`、`track.cart.superelev.reflen=1.500 m`，超高全幅值为
   `0.120 m`。
2. SIMPACK QCH 定义三种应用方式：绕中心线、绕内轨、绕外轨。绕内轨时线路中心抬升 `u/2`，
   绕外轨时下降 `u/2`；但已核对的 QCH 文字没有给出这三项与 SPCK 整数的明确对应表。
3. SIMPACK 系统默认的 `track.cart.superelev.kind` 同样是 `1`。这使“零基枚举下 1=绕内轨”和
   “一基枚举下 1=绕中心线”都不能仅靠选项排列排除；目前不得采用任一推断。
4. WRL 的线路中心只由水平曲线与纵坡积分得到，随后在该中心线上施加超高滚转。ORVD G47 的
   `TrackGeometry` 也保持中心线不随超高移动，只把无侧滚切向系绕自身纵轴滚转。两者均为绕中心线
   语义。

同一 SPCK 还含 `track.meas.superelev.reflen=1.506 m`。当前活动线路走地图式定义且
`track.cart.superelev.fromfile=0`，现有证据指向 `1.500 m` 才是活动参考长；G62 须把这一点与
无车辆线路查询一并确认，不能仅因两个字段都存在就任选其一。

#### 条件性动力学影响

若 `kind=1` 经验证为绕中心线，则此疑点被否证。若它表示绕内轨或外轨，则全幅 `u=0.120 m`
会使线路中心相对 WRL／ORVD 产生 `0.060 m` 的竖向平移；该差异改变轨道系原点而不仅是姿态，
可能进入启动高度、轮轨压缩量与接触力。历史启动状态或预载轨道偏移是否吸收了它，必须另行核对。

#### 最小裁决

G62 首个 SIMPACK 曲线资格前，从已求值线路中心或一个不含车辆的受控线路查询中确定
`kind=1` 的实际滚转基准，同时确认活动参考长。只有映射确定后，才裁决 ORVD 的 R300 资产是否与其同义。
核对只需比较同一站位上的线路中心位置与轨型系姿态，不需要车辆运行，也不把输出或哈希纳入 Git。

#### 源码锚点

- SIMPACK QCH `Cartographic Track: User Interface`：`Kind` 与 `Reference baselength`。
- SIMPACK 2021x `run/conf/defaults.sys`：地图式超高模式的系统默认整数同为 `1`。
- WRL `mbs_simpack/vehicle_GZ18/main_model/vehicle_GZ18.spck`：活动线路、
  `track.cart.superelev.kind`、地图式／测量参考长。
- WRL `scripts_python/rwc/track.py`、`scripts_cpp/drake_sim/src/track_scenario_crv300m.cc`：
  线路中心积分与超高角构造。
- ORVD `libs/track_geometry/src/track_geometry.cc`：中心线与无侧滚切向系上的超高滚转。

### MD-003 — 纵坡正号在参考模型与 ORVD 之间相反

- 车型：共性
- 层级：轨道几何坐标口径
- 状态：**已裁决；当前来源线路为零纵坡，转换规则尚未被活动工况触发**

#### 两端实际语义

参考线路采用 `+z` 向下，纵坡参数 `p>0` 时按 `z += p·ds` 积分，即正值表示下坡。ORVD 的字段
`centerline_upward_grade` 则把上坡定义为正，并明确
`d(centerline z)/d(track_station) = -centerline_upward_grade`。因此接口转换唯一为：

```text
centerline_upward_grade_ORVD = -vertical_slope_reference
```

当前已核对的 GZ18／IRW 来源线路纵坡均为零，所以同号误搬不会改变现有工况，也不会被现有宏观
回归自动发现。

#### 迁移约束

以后引入首条非零纵坡线路时，加载或一次性迁移步骤必须显式执行上述换号，并以中心线竖向导数
核对；不得把参考模型字段直接复用为 ORVD 的 `centerline_upward_grade`。

#### 源码锚点

- WRL `scripts_python/rwc/track.py`：参考线路的 `z += p*ds` 积分。
- ORVD `libs/track_geometry/include/orvd/track_geometry/track_geometry.h`、
  `libs/track_geometry/src/track_geometry.cc`：正上坡字段与中心线竖向导数。

### MD-004 — FE86 帮助页的显示式遗漏输入速度项

- 车型：共性
- 层级：串联弹簧—黏性阻尼本构与状态口径
- 状态：**已裁决；ORVD 本构由同库另一推导交叉支持，不按不完整显示式改写**

#### 疑点

SIMPACK QCH FileId 37668 的 FE86 数学页把同一弹簧伸长 `xs` 的导数也写进阻尼力，随后将
内部伸长状态写成只含 `xs·cs/ds` 的正号齐次式，未出现总端点相对速度输入项。若只读这一页，容易
误判 ORVD 的 Maxwell 方程多出了一项或符号相反。

#### 交叉核实

同一 QCH 的 FileId 37652（FE80，Force States Option 2）保留了完整的输入速度项。对其同拓扑
推导令第二刚度为零，可得：

```text
ẋs = v_relative - (K/C) xs
F = K xs
dF/dt = K v_relative - (K/C) F
```

这与 ORVD 的活动实现一致。故当前证据说明 FE86 页的最终显示式不完整，不说明 SIMPACK 求解器
内部实现错误。

两端状态口径也不同：SIMPACK Type-86 以串联弹簧伸长 `xs`（米）表达动态状态，ORVD 直接以
同一支路的力 `F`（牛）表达，二者由 `F=K·xs` 一一换算。GZ18 的一系 Type-86 模板虽残留
`force.st.dyn/st.equi` 字段，但串联刚度为零，不能据字段存在把它们算作活动 Maxwell 状态；ORVD
只为两条串联刚度非零的二系横向减振器声明状态。

#### 迁移约束

- 后续复核必须同时核对状态量纲与完整微分方程，不得把 `xs` 和 `F` 的数值直接逐项比较。
- 仅凭休眠的状态字段不得增加 ORVD 状态维数；活动性由本构参数与真实消费者共同决定。

#### 源码锚点

- SIMPACK QCH FileId 37668：FE86 数学页及不完整显示式。
- SIMPACK QCH FileId 37652：FE80 Force States Option 2 的完整状态推导。
- ORVD `libs/forces/include/orvd/forces/vehicle_force_elements.h`、
  `libs/forces/src/vehicle_force_plan.cc`：以力为状态的串联本构。
- WRL `mbs_simpack/vehicle_GZ18/database/mbs_db_substructure/bogie_motor.spck`：一系与二系
  Type-86 参数及残留状态字段。

### MD-005 — `LM.prw` 自称 ERRI S1002（已否证，非差异）

- 车型：GZ18、IRW、SH17 共性
- 层级：轮型资产身份
- 状态：**已否证；不是开放差异，无需下游处置**

`LM.prw` 的头部数据段有 `comment = 'ERRI S1002 Wheel Profile, according to Nefzger, ...'`，看上去
像文件自称是另一个型面。横向比对否证了这一读法：同一句注释**逐字**出现在 `LM.prw`、`S1002.prw`
与 `DIN5573-28.prw` 三个不同型面里，而 DIN5573-28 显然不是 S1002。这是随机文档模板里未被改写的
样板字段，不构成身份主张。`LM.prw` 与 `S1002.prw` 的点数据不同，且 `LM.prw` 自己的文件头注释块
写的是 `File Name : LM.prw`。

G51 最初按源模型 `rwpair.wheel.prof.file` 的字面值记录 `LM.prw`。G52c 已把产品运行身份迁为
无扩展名逻辑标识 `gz18_reference_wheel_profile` 与 `uic60_rail_profile`，再由安装数据根解析到
随包 JSON；这不是把 LM 改写为 S1002。`LM.prw` 只保留为本地来源与 SIMPACK 互操作锚，不是
产品运行身份，也无需为文件头的样板注释再登记一个 S1002 别名。

### MD-006 — SIMPACK 型面遍历语义在 WRL 数值路径中被显式左右侧口径取代

- 车型：GZ18、IRW、SH17 共性
- 层级：轮轨型面格式、接触几何输入口径
- 状态：**已裁决；ORVD 数值路径复现近期 WRL，SIMPACK 本地互操作另行保留遍历声明**

#### 两端实际语义

SIMPACK QCH FileId 39502 明确把 `inversion` 定义为点列次序反转，并说明点列遍历方向决定型面
多边形的接触外侧与内侧。因此它不是无意义的文件排版字段。当前本地资产的原始点行都按横坐标
升序书写；`LM.prw`、`S1002.prw`、`DIN5573-28.prw` 声明 `inversion=1`，SIMPACK 的有效遍历
为横坐标降序；`UIC60.prr` 声明 `inversion=0`，有效遍历为升序。

WRL 的活动数值路径采用另一套承载方式。冻结脚本只提取坐标行并按横坐标升序排序，不读取
`inversion`；生成头也只保存升序坐标数组。接触几何构造器再按显式 `WheelSide` 对左侧横坐标
取反，并对车轮、钢轨点列重新升序排序。故近期 GZ18 与 IRW 资格化运行消费的是“型面角色、
显式左右侧、坐标符号和升序点列”的组合，运行时不消费 SIMPACK 多边形遍历声明。

这是一处真实的语义承载差异，但尚无证据表明它导致当前四份资格化型面的数值结果错误：近期
WRL 的接触几何已经在上述显式左右侧口径下闭合。不能反过来把 QCH 的遍历声明重新混入 ORVD
数值预处理。尤其 IRW 的源横坐标重离散先以输入数组首尾确定网格相位，再对结果排序；机械反转
输入会把不能整除步长的余量短段移到另一端。当前资格化相位来自 WRL 冻结器的升序数组，不来自
SIMPACK 的有效遍历。GZ18 的弧长重扫则先在显式右侧升序点列上运行，不受该元数据控制。

#### 迁移裁决

- ORVD 随包资格化 JSON 继续只承载规范型面角色、坐标约定与升序点列；通用值类型仍可保留
  严格单调的作者顺序，供明确依赖起点相位的预处理使用。G52 数值求解遵循近期 WRL 的显式
  左右侧与镜像／排序语义，不把 `inversion` 变成运行时物理参数。
- 本地 `.prw/.prr` 兼容读写必须遵循 QCH：读取时在归一化点列前解析有效遍历方向，写回时用
  `inversion` 恢复同一有效遍历。坐标逐位相同不足以证明互操作往返同义。
- 从 ORVD JSON 新写 SIMPACK 文件时，当前受支持的规范口径采用车轮降序、钢轨升序的有效遍历；
  这是本批已核实资产与坐标角色的转换合同，不宣称为所有未知供应商文件的普遍定律。超出该
  规范口径的文件必须保留其读取所得遍历元数据，或响亮拒绝，不得静默猜测。
- ORVD 提交 `3ed5d49` 已把 SIMPACK 遍历方向放在开发期兼容元数据中，并保持产品点列升序；
  该分层与本条裁决一致。后续 G52b/G52c 只需验证真实接触消费者仍不反向读取这项元数据。

#### 源码与文档锚点

- SIMPACK QCH FileId 39502，`Rail and Wheel Profile (.prr and .prw) Files`：`inversion`
  与型面外侧／内侧定义。
- WRL `model_data/profiles/wheel/{LM,S1002,DIN5573-28}.prw`、
  `model_data/profiles/rail/UIC60.prr`：原始点行和 `inversion` 声明。
- WRL `scripts_cpp/tools/freeze_gz18_assets.py`：`load_profile_points()` 只提取坐标并排序；
  `generate_profile_header()` 只写坐标数组。
- WRL `scripts_cpp/rigid_wheelset/src/gz18_contact_model_config.cc`、
  `scripts_cpp/irw/src/irw_contact_model_config.cc`：近期活动人格向核心传入生成数组。
- WRL `scripts_cpp/rwc_core/src/contact_geometry.cc`：`MirrorAndSortProfile()`、弧长重扫与源横坐标
  重离散的实际消费顺序。
- ORVD `tools/profile_conversion/simpack_profile_io.{h,cc}`：开发期兼容元数据与 QCH 同义写回。

本条由 CodeX 于 2026-08-05 对 QCH 与近期 WRL 活动路径分层复核。

### MD-007 — 三维互穿纵向长度解析失败时的处置语义

- 车型：GZ18、后续 IRW 共性
- 层级：轮轨法向力、数值试算时相
- 状态：**已裁决；G52b 后续修复已恢复近期 WRL 兼容语义**

#### 疑点

三维互穿纵向长度无法解析时，是终止整次接触求值，还是继续采用解析弦长。QCH 对 EEC 和等效接触椭圆
作了理论说明，但未公开这一失败边界的实现处置，不能把任一选择写成 QCH 明文要求。

#### 两端实际消费者

近期 WRL 始终先计算 `L_base=2·sqrt(2·R_eff·penetration)`，有限且为正的三维长度成功解析后才覆盖；
失败时保留基线、置回退标志，并累计尝试数与回退数。2026-08-02 的 IRW 平顺 30 s 正式 RHS 工件
记录 302/10,934,468 次回退／尝试，AAR5 30 s 工件记录 4,686/23,936,829 次。计数包含重复 RHS 和
被拒步试算，不能解释成唯一接受态样本，但足以证明近期 WRL 的该分支不是结构性不可达。

**CodeX 同输入边界复核（2026-08-08）**：不能把上述 IRW 计数与 ORVD 的 GZ18 名义姿态扫描直接
比较，更不能由此声称两端的“触发条件不同”。两仓的三维长度括根实现采用相同的 `5e-4 rad` 起步、
`0.25 rad` 搜索上限、两侧括根和 36 次二分，源码结构与常量同形。要判断触发语义是否漂移，必须用
同一车型、同一型面、同一人格、同一位姿与侵入状态逐次差分。跨车型回退数只登记各自路径的活动性，
不作为 ORVD—WRL 完成门或可直接比较的观测量。

ORVD 提交 `8990dd4` 曾短暂改为几何生产端和法向消费端双重强失败；`84ea1c8` 已恢复解析弦长基线、
三维成功才覆盖及失败计数的近期 WRL 兼容语义。自 `443142d` 起接触已进入系统 RHS，三维长度缺失本身
不再终止 RHS；其他真实配置错误、几何歧义与内部不变量破坏仍按致命事务处理。

#### 动力学影响

解析弦长与三维互穿长度不同，会改变 EEC 等效椭圆和法向力；强失败则直接失去该次 RHS 试算乃至推进器
事务。两者都不是纯诊断差异。近期端到端复现优先级高于从 QCH 沉默处推导一个更严格的新合同。

#### 最小核验或裁决

项目负责人裁决（CodeX 记录，2026-08-08）：G52b 后续修复恢复 WRL 兼容语义——解析弦长作基线、三维成功才
覆盖、失败从同一次结果报告。ORVD 以 `geometric_patch_count` 同时表达本次几何斑数和解析尝试分母，
以 `analytic_longitudinal_length_fallback_count` 表达回退分子，不保留与斑数定义恒等的第二个公开计数。
不得为计数重新求值，不在热路径格式化日志或动态分配；通用 CVODE 可恢复错误分类不随本条扩建，其他
真实配置错误和内部不变量破坏仍按致命事务处理。

#### 源码锚点

- WRL `scripts_cpp/rwc_core/src/normal_force_eec.cc:200-211`：基线、成功覆盖与回退标志。
- WRL `scripts_cpp/rwc_core/src/contact_kernel.cc:286-290`：尝试／回退计数。
- WRL `scripts_cpp/tests/test_interpenetration_3d_length.cc:375-385`：成功与失败两条回归。
- 外置卷 `work_order_08/runs/drake_A_passive_smooth_30s_retry_01/` 与
  `work_order_08/runs/drake_B_passive_aar5_30s/` 的 2026-08-02 运行 JSON：近期正式 RHS 计数。
- ORVD `libs/wheel_rail_contact/src/contact_geometry.cc`、`normal_contact_force.cc`、
  `wheel_rail_contact_model.cc`：G52b 后续修复后的测量缺失、解析基线与同次求值计数链。
- SIMPACK QCH FileId 38410、38415：未规定该失败边界。

### MD-008 — Kalker 有限表与表外渐近式的切换语义

- 车型：GZ18、后续 IRW 共性
- 层级：轮轨切向接触、系数表外插
- 状态：**已裁决当前迁移语义；SIMPACK 精确切换仍待核实**

#### 疑点

WRL 在 `a/b<0.1` 或 `a/b>10` 时从有限表切到首项渐近式。渐近式本身是否有理论出处，以及 SIMPACK
是否也在相同边界采用相同硬切换，是两个不同问题。

#### 两端实际消费者

现有迁移源码与历史来源说明把有限表归于 Kalker 1990 Appendix E/Table E3，并把表外渐近值进一步
归于 Kalker 1972a；但两个仓库均未收录可供本轮独立逐式核验的原书或完整参考文献条目。因此这是
**来源归属登记**，不是本轮已经持书确认的公式证据。WRL 与 ORVD 当前都选择 `kAsymptotic`，尚无
证据证明 SIMPACK 的边界和拼接方式相同。有限表与渐近式在下端的 C11/C22/C23 跳跃约为
-0.833%/-2.134%/-5.638%，上端约为
-3.686%/-2.350%/-0.076%；这是算法拼接不连续，不是物理量应有的不连续。

#### 动力学影响

系数跳跃会直接改变 FASTSIM 的切向刚度和蠕滑力。横移、偏航和轮缘接触是车辆真实响应，不能为回避
表外路径而设置许可上限。近期正载荷存档约落在 `a/b=0.394–9.914`，只能说明当前工件未资格化表外正载荷，
不能证明表外路径不应存在。

#### 最小核验或裁决

项目负责人裁决（CodeX 记录，2026-08-08）：端到端迁移期间保持近期 WRL 的 `kAsymptotic` 与现有
`0.1/10` 硬切换；补理论出处，明确 SIMPACK 精确切换尚未资格化。未经独立车辆动力学模型实验不得平滑，
也不得以姿态或 `a/b` 范围拒绝真实响应。未来若核验，使用 SIMPACK／WRL／ORVD 同一接触输入跨两端对拍，
该核验不阻塞 G52c，也不增加运行主线计算。

#### 源码锚点

- Kalker, *Three-Dimensional Elastic Bodies in Rolling Contact* (1990), Appendix E, Table E3。
- WRL `scripts_cpp/generated/gz18_contact_config.gen.h:141`：活动人格允许表外渐近式。
- ORVD `libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_model.h:84`：当前
  GZ18 默认选择 `kAsymptotic`。

### MD-009 — 不平顺空间采样变化率是否属于轨侧材料速度

- 车型：GZ18；后续 IRW 共性
- 层级：轮轨接触位姿、材料点相对速度与蠕滑率
- 状态：**已裁决当前迁移语义；SIMPACK 内部消费方式待核实**

#### 疑点

车辆以站位速率 `s_dot` 扫过固定空间不平顺 `y(s)`、`z(s)` 时，移动观察点会得到
`y'(s)·s_dot`、`z'(s)·s_dot`。这两个量是空间场沿车辆轨迹的采样变化率，但不自动等同于固定钢轨
材料粒子的惯性速度。若把它们作为非零轨侧材料速度减入轮轨相对速度，将改变当前刚性型面蠕滑路径；
QCH 尚未提供足以判定 SIMPACK 内部如何处理这一层的公开实现证据。

#### 两端实际消费者

近期资格化 WRL 的 GZ18 人格选择 `kRigidProfile`。外层在共享轴站位和有效型面站位两阶段求取横、垂
不平顺及其斜率，斜率进入有效偏航、俯仰与轨型刚体位姿。代码同时保留遗留的
`creep_y_irreg_dot`／`creep_z_irreg_dot` 字段，其中前者保持零、后者可写入 `z'(s)·s_dot`；但活动
刚性型面分支不消费这两个字段，而是以轮材料点刚体速度减去零轨侧材料速度构成相对速度。

ORVD G57 按项目负责人裁决复现这一近期 WRL 语义：两条不平顺斜率进入位姿，轨侧材料点速度保持为零。
不增加非零轨速计算、状态、版本、计数或观测，也不把空间采样变化率静默解释成另一种物理量。

#### 动力学影响

若未来把非零采样变化率引入轨侧材料速度，它会改变接触点相对速度，进而可能改变法向相对速度、
蠕滑率、FASTSIM 切向力与车辆响应；这不是纯重命名或无消费者字段清理。另一方面，固定钢轨的材料粒子
在惯性系中静止，不能只凭移动观察者看到型面高度随站位变化，就断言钢轨材料正在运动。

#### 最小核验或裁决

项目负责人裁决（CodeX 记录，2026-08-09）：当前端到端迁移优先复现近两三周已固化的 WRL 路径，
G57 采用零轨侧材料速度。MD-009 作为潜在模型解释差异保留，但不阻塞 G56/G57，不增加运行主线成本。
只有出现 QCH／SIMPACK 一手证据，或完成 SIMPACK—WRL—ORVD 同一不平顺、同一接触输入的受控对拍后，
才可重新打开并另立物理人格；既有长窗闭合结论不能自动迁移到改写后的非零轨速公式。

#### 源码锚点

- WRL `scripts_cpp/drake_sim/src/wheel_rail_contact_system.cc:789-827`：两阶段不平顺求值、斜率和遗留
  `creep_*_irreg_dot` 字段。
- WRL `scripts_cpp/rwc_core/src/contact_kernel.cc:193-223`：刚性型面分支以零轨侧材料速度构成相对速度。
- WRL `scripts_cpp/rwc_core/src/creepage.cc:61-74`：刚性型面蠕滑直接消费材料点相对速度。
- WRL `scripts_cpp/generated/gz18_contact_config.gen.h:51,57,136`：GZ18 活动运输、有效偏航和遗留速度开关。
- ORVD `MIGRATION_ROADMAP.md` 的 G57 与 `DISCUSSION_AND_DECISION_LOG.md` 的 DEC-027、R015。

### MD-010 — 完整不平顺轨型姿态是否应旋转接触力坐标系

- 车型：GZ18；后续 IRW 共性
- 层级：轮轨接触坐标系、力方向与虚功
- 状态：**已裁决当前迁移语义；SIMPACK 完整三维消费方式待核实**

#### 疑点

G57 由横、垂不平顺斜率构造
`Rz(atan2(y',1))·Ry(-atan2(z',1))`，并与基础轨道位姿和轨底坡滚转合成完整钢轨型面姿态。
若把该姿态视为真实三维轨面局部基，它似乎还应整体旋转法向力、切向力、法向接近速度和蠕滑坐标；
若它只是刚性型面材料参考点的放置姿态，则接触力坐标仍可由横断面共同法向／表面坡角构造。

#### 两端实际消费者

近期资格化 WRL 的 `kRigidProfile` 路径用完整轨型姿态把钢轨材料参考点放到惯性空间；低层接触和
扳手坐标系仍只消费横断面的轨侧表面坡角与型面滚转偏移。ORVD G57 逐式保留同一分层：
`RailProfileFrame` 的完整姿态到达材料点定位，而 `WheelRailContactModel` 的接触坐标系继续只绕
横向剖面的 x 轴滚转。故本条不是 G57 相对冻结 WRL 的迁移错误，而是冻结模型与“完整三维局部轨面”
解释之间的潜在差异。

#### 动力学影响

随包 AAR6 资产的最大相邻节点坡度为 `0.002534501012200242`。用约 `68.3 kN` 支承力和
`16.6667 m/s` 路径速度只作量级换算，倾斜坐标的力分量约为 `173 N`、沿坡采样变化率约为
`0.0422 m/s`；这两者不是已观测的 ORVD—SIMPACK 力误差，但足以说明该疑点不能按浮点边角忽略。

#### 最小核验或裁决

项目负责人既定方向是先端到端复现近两三周冻结 WRL。因此 G57 不改当前公式，不增加运行时人格或
分支。G60/G61 已按冻结坐标完成 AAR6 长窗且保留了轮轨力边界；本条不因宏观响应通过而自动关闭。
后续受控 A/B 须在仓外以同一接触输入执行：一列保持当前冻结坐标，
一列让完整三维轨型姿态旋转接触力坐标；比较 `Q/N/Tx/Ty`、空间扳手虚功及 SIMPACK 原生结果。
证据明确后再决定是否另立物理人格或替换公式。

#### 源码锚点

- WRL `scripts_cpp/rwc_core/src/pose_builder.cc:292-316` 与
  `scripts_cpp/drake_sim/src/wheel_rail_contact_system.cc:830-895`：完整轨型姿态的构造与输入。
- WRL `scripts_cpp/rwc_core/src/contact_kernel.cc:193-253,579-595`、
  `scripts_cpp/rwc_core/src/coord_transforms.cc:115-117`：材料点放置与横断面接触坐标。
- ORVD `libs/forces/src/wheel_rail_contact_force_plan.cc` 的 G57 轨型姿态构造。
- ORVD `libs/wheel_rail_contact/src/wheel_rail_contact_model.cc` 与 `contact_creepage.cc` 的接触坐标消费。

### MD-011 — 蠕滑参考速度采用路径速度还是平面站位速率

- 车型：GZ18；后续 IRW 共性
- 层级：轮轨蠕滑率与线路参数化
- 状态：**已确认休眠差异；待首个非零纵坡资格工况裁决**

#### 疑点

`TrackGeometry` 的独立变量是平面投影站位 `s`。在线路纵坡为 `g` 时，三维中心线弧速为
`sqrt(1+g^2)·ds/dt`。蠕滑参考速度究竟应采用平面 `ds/dt`，还是轮对沿三维中心线的路径速度，
会在非零纵坡下产生确定差异。

#### 两端实际消费者

近期 WRL 从载体运动学取得平面站位速率，并把该值送入蠕滑参考速度。ORVD 的力计划同时计算
`station_rate` 与 `path_rate`：不平顺位姿正确消费前者，`WheelProfileRigidMotion::arc_rate` 当前
消费后者。全部已资格场景均为水平线，`sqrt(1+g^2)=1`，因此现有 G55/G57 数值逐位相同，差异休眠。

#### 动力学影响

在 2% 纵坡上，两种速度相差比例约 `1.9998e-4`，会同量级进入归一化蠕滑率；它不是当前 GZ18
平直启动的误差来源，也不足以在没有真实带坡对拍时反向改写冻结路径。

#### 最小核验或裁决

G57 保持现状并用非零纵坡合成夹具明确区分“位姿必须用 `ds/dt`”与“当前蠕滑仍用路径速度”两个
输入，避免日后误合并。首个真实带坡 GZ18/IRW 资格工况前，对同一状态并排计算 WRL 字面式、ORVD
路径式和 SIMPACK 原生结果，再决定统一到哪一口径；本条不增加当前运行时状态、分支或观测。

#### 源码锚点

- WRL `scripts_cpp/drake_sim/src/carrier_kinematics.cc:49-57`：平面站位速率。
- WRL `scripts_cpp/drake_sim/src/wheel_rail_contact_system.cc:901-913` 与
  `scripts_cpp/rwc_core/src/creepage.cc:188-191`：蠕滑参考速度消费。
- ORVD `libs/forces/src/wheel_rail_contact_force_plan.cc:319-341,520-554`：站位速率、路径速度及两处消费者。

### MD-012 — IRW 纵向拉杆构架侧衬套与 SIMPACK Type-43 的加载语义不同

- 车型：IRW
- 层级：六分量衬套本构坐标系、作用点与刚体支承矩
- 状态：**已裁决当前迁移语义；先复现 WRL 末期冻结路径，完成端到端闭合后再重开 SIMPACK 语义**

#### 疑点

IRW 的 SIMPACK 模型把前、后转向架各 A–D 四个纵向拉杆构架侧固定元定义为 Type-43。QCH 的
Component 力元公式与 WRL 末期冻结执行路径在本构表达基、力的作用点和附加支承矩分配上不同。
若迁移时把二者含混地称为同一种“完整六分量衬套”，会在 ORVD 尚未复现 WRL 前同时改动迁移基线
和局部物理公式，无法区分移植错误与模型语义差异。

#### 两端实际消费者

SIMPACK 活动模型的八个 Type-43 均为：From=`$M_LongiBar_*_FrameSide`，
To=`$M_Frame_LongiBar_*`；三向平动刚度为 `1e8 N/m`，三向平动阻尼为 `100 N·s/m`，三向转动
刚度为零，三向转动阻尼为 `100 N·m·s/rad`，名义量为零，参数 31 为 `0`。QCH FileId 37480、
37952、37482–37484 规定：位移、相对速度及六个本构分量通常在 From（或显式 Reference）轴表达；
本构力与自由力矩施于 From，等大反向量施于 To；平动力因 From/To 标记分离产生的附加
`r×F` 支承矩只施于 From。对刚体，这等价于把 From 刚体所受的平动力移到 To 标记的空间位置。
本构自由力矩与 From-only 平动力支承矩是两个量，不得重复计算。

WRL 末期冻结路径则把这八个元无条件路由为 `FullBushingConfig`：在 From 与 To 姿态之间的半角
坐标系中计算平移本构与速度，在两体瞬时重合的共同中点施加等大反向力；转动部分采用 Space-XYZ
RPY 坐标率及其共轭广义力矩到物理力矩的映射。该配置没有 Type-43 语义选择项，正式构图入口也没有
切换到 From 基与真实端点的分支。2026-08-02 的 IRW 被动 30 s 资格运行使用的承重源文件与
2026-07-29 仓外 Type-43 原子候选基线字节相同，故末期冻结运行确实仍消费半角系与共同中点公式。

#### 已有同状态证据与动力学边界

仓外 P46 在同一组 SIMPACK 位姿和速度上，以 From 基相对位移、相对速度及物理相对角速度独立重建
八个元，最大力残差约 `8.87e-6 N`，本构自由力矩最大残差约 `5.59e-15 N·m`。P124 又把 From 基
平动、真实端点、参数 31 对应的角运动学与自由力矩、From-only 支承矩作为不可拆原子候选；固定状态
共同原点矩和虚功残差约为 `1e-12` 量级。这些结果确认 SIMPACK 局部 Type-43 合同，但不代表候选
已经进入 WRL 产品路径。

同一批研究也表明，替换该局部公式在已测早期窗口内只引起很小的四轴横移与摇头变化：P46 的
4.2 s 候选相对冻结路径最大变化低于约 `0.256 µm / 0.168 µrad`；P124 的预登记窗口最大池化
均方根变化约 `0.0214 µm/µrad`。因此，WRL—SIMPACK 的局部扳手差异可以与既有宏观高精度闭合
同时存在；不得由宏观闭合反推每个内部力元逐项同义，也不得由固定状态闭合提前宣称长窗响应更优。

#### 迁移裁决与重新打开条件

项目负责人裁决（CodeX 记录，2026-08-09）：IRW 首轮 ORVD 迁移以 WRL 末期冻结执行路径为基线，
采用半角系、共同中点及冻结 RPY 力矩语义；本阶段不静默改成 QCH Type-43，不增加双语义运行时开关。
实现与验收必须区分两类问题：

- ORVD 与冻结 WRL 在相同状态或整车响应上不闭合，属于迁移实现错误，应在当前主线修复；
- ORVD 与 WRL 已闭合，而二者共同在 Type-43 局部扳手上偏离 SIMPACK，属于本条已登记模型差异，
  不阻塞 IRW 首轮端到端迁移。

只有在 ORVD 完成冻结 WRL 的 IRW 全流程端到端复现后，才重新打开本条：以同一状态和同一整车工况
并排运行冻结半角／中点公式与 QCH Type-43 原子公式，分别比较八元逐实例扳手、共同原点矩、虚功、
四轴横移／摇头和轮轨三向力，再由 SIMPACK 原生结果裁决是否替换。重开前不得把仓外候选升级为产品
身份，也不得因其短窗宏观效应较小而否定已经闭合的 SIMPACK 局部合同。

#### 源码与证据锚点

- SIMPACK QCH FileId 37480、37952、37482–37484、39455、39458：Type-43 本构、Component
  加载规则、输出表达系及角运动学。
- WRL `mbs_simpack/irw_4WDB/ref_files/Bogie_R300_FREE_PROFILE_RESOLUTION.spck`：八个活动
  `$F_PS_BarFixed_*` Type-43 的 From/To 与刚阻参数。
- WRL `scripts_cpp/irw/src/irw_force_elements.cc:238-252,411-428`：八元登记及
  `FullBushingConfig` 路由。
- WRL `scripts_cpp/drake_sim/src/specialized_suspension.cc:121-145,375-420`：半角运动学、共同中点
  与 RPY 力矩加载。
- WRL `scripts_cpp/irw/src/irw_simulation_runner.cc:1012-1017`：末期冻结正式构图入口。
- WRL 外置研究 `p46_longibar_fixed_from_marker_endpoint_contract`、
  `p124_type86_numerical_requalification_type43_early_window/type43`：同状态闭合、短窗影响及明确未晋级记录。

## 新条目模板

```markdown
### MD-NNN — 简短问题名

- 车型：GZ18 / IRW / 共性
- 层级：拓扑 / 力元 / 轮轨 / 控制 / 数值时相 / 其他
- 状态：待核实 / 已确认差异 / 已裁决 / 已否证 / 重新打开

#### 疑点

#### 两端实际消费者

#### 动力学影响

#### 最小核验或裁决

#### 源码锚点
```
