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

- MD-002：GZ18 活动曲线绕轨道中心线滚转、活动超高参考基长 `1.5 m` 已裁决；最终 `3 m`
  叠加平滑多项式及 Track Line 离散／插值仍未由高精度无车辆查询闭合。**不影响已完成的直线
  G51–G61 资格**。G62 已迁入相同 R300 标量剖面与 C² 缝合，同时按负责人裁决保留 ORVD
  更准确的通用中心线积分，并继续保留这条 SIMPACK 上游差异。
- MD-007：三维互穿纵向长度解析失败时，QCH 未公开处置语义；近期 WRL 使用解析弦长基线并计数。
  项目负责人已裁决恢复近期 WRL 冻结执行语义，G52b 后续修复已经落实。
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
- MD-013：冻结 WRL、ORVD 与现有 SIMPACK 工件的接触斑容量和观察输出形状不同。G71 将消费的
  冻结 WRL A/B 参考工件已见至多双斑；ORVD 不按对方槽数截断，比较时区分统一 Track-T 总力、
  逐斑局部力和 WRL 最大法向力主斑列。
- MD-014：冻结 WRL A 层使用最大二阶 BDF 与 SPGMR；ORVD 主线使用稠密数值 Jacobian 与稠密
  线性求解，并按资格实例固定最大阶数。两者的墙钟、内部计数和末位轨迹不能冒充同一后端结果；
  主线只保留轻量只读统计和具名内部配方，不开放公共数值旋钮。
- MD-015：SIMAT 的 100 Hz 控制机械观测比 P179 WRL／SIMPACK Realtime 晚一通信拍。ORVD 的
  控制器／调理器纯递推复现 WRL Git 固化且与 P179 一致的源快照；当前接受态输入和启动双更新按
  P179 外置资格源快照实现。SIMAT 观察龄只作时相差异，不进入产品运行时选项。
- MD-016：QCH 已确认 SIMPACK Type-110／Type-93 主动力矩在 From 构架标记系表达、原始正号作用
  于 From；冻结 WRL 则以轴桥／轮体共同转动轴的 `+Y` 为轴、轮侧为正号。G73 按 WRL 复现并
  保持构架反力端，不增加表达基选择人格。

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
- 状态：**已裁决；滚转基准与活动参考基长已关闭，SIMPACK 最终叠加平滑仍待高精度查询**

#### 已裁决语义

1. GZ18 活动线路是地图式线路（Cartographic Track）`$Trk_Curve_R300m_60kmph`。其
   `track.cart.superelev.kind=1`、`track.cart.superelev.reflen=1.500 m`，超高全幅值为
   `0.120 m`。
2. 项目负责人明确裁定该活动线路绕轨道中心线施加超高。历史同段 Track Plot 也给出全幅区
   中心线 `z=0`、两侧分别为 `+0.06/-0.06 m`，与中心线不升降相符。该结论来自已求值线路和
   项目裁决，不是由整数 `1` 的排列顺序猜测。
3. WRL 的线路中心只由水平曲线与纵坡积分得到，随后在该中心线上施加超高滚转。ORVD G47 的
   `TrackGeometry` 也保持中心线不随超高移动，只把无侧滚切向系绕自身纵轴滚转。两者均为绕中心线
   语义。

同一 SPCK 还含 `track.meas.superelev.reflen=1.506 m`。当前活动线路走地图式定义且
`track.cart.superelev.fromfile=0`；因此 `1.500 m` 是活动地图式超高参考基长，`1.506 m` 是
未活动 measured-track 字段。后者仍可能服务测量线路的轨距测量／超高表达，但不是当前活动
地图式线路的候选输入，二者不得混用。ORVD 的字段相应命名为
`superelevation_reference_baselength_meters`，不再把它误称为轨间跨度、轨距或轨型定位间距。

QCH `.trc` 文件头的 `data.par(1)` 使用 `0/1/2` 编码；该文件格式字段不能未经验证直接套作
SPCK GUI 存储字段的整数映射。G62 不再用这张编码表重开项目负责人的滚转基准裁决。

#### 剩余模型差异

QCH 给出了原始 BLOSS 公式和覆盖段边界的 `3 m` 平滑窗结构，但没有公开最终叠加多项式。
近期 WRL 的 `legacy_sampled_c2_fd_v1` 与 ORVD 当前均使用匹配值、一阶导和二阶导的五次 C² 桥；
历史 SIMPACK Track Plot 在叠加窗内部与该桥存在数微米可辨差异，窗外原始 BLOSS 和平台相符。
这说明滚转基准已经关闭，但最终叠加平滑和 Track Line 离散／插值仍是独立上游边界。

后续以不含车辆的受控线路查询在四个段边界及各自 `±1.5 m` 窗内高精度输出中心线、姿态、
曲率和超高及其导数。该查询只关闭最终平滑与离散边界，不再决定 `kind=1` 或 `1.5 m` 的含义；
不需要车辆运行，也不把查询输出或哈希纳入 Git。当前迁移按项目既定主线先复现近期 WRL，
所以 G62/G63 可以使用明确标为“与近期 WRL 使用相同 R300 标量剖面和 C² 缝合”的资产，
但不得据此关闭本条或宣称与 SIMPACK 最终叠加公式逐式一致。G62 固定站位复核还量化了数值
积分差异：WRL 以 `0.5 m` 中点法累计中心线，ORVD 以解析航向和每面板八点高斯求积；实际
`0–280 m` 运行区间最大平面位置差 `47.15 µm`，全 `[0,1150] m` 最大 `108.37 µm`，最大航向差
`1.042 µrad`。负责人裁定保留 ORVD 实现，不复制该中点累计误差；这组数只说明离散执行差异，
不得未经归因扣到 SIMPACK 的最终 `3 m` 平滑公式。

#### 源码锚点

- SIMPACK QCH `About Tracks`、`About Cartographic Track`、`Cartographic Track: User Interface`
  及 `.trc` 文件格式：超高几何、原始 BLOSS、平滑窗与文件字段编码。
- SIMPACK 2021x `run/conf/defaults.sys`：地图式超高模式的系统默认整数同为 `1`。
- WRL `mbs_simpack/vehicle_GZ18/main_model/vehicle_GZ18.spck`：活动线路、
  `track.cart.superelev.kind`、地图式／测量参考长。
- WRL `model_data/validation_refs/replay_track/TrkZs_IRW-Trk_R300_60kmh.txt`：同一 R300
  段表的历史已求值中心线和左右侧高度旁证。
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

### MD-006 — SIMPACK 型面遍历语义与 ORVD 当前统一网格相位分层承载

- 车型：GZ18、IRW、SH17 共性
- 层级：轮轨型面格式、接触几何输入口径
- 状态：**已裁决；当前 ORVD 使用右侧等弧长重扫后镜像，SIMPACK 互操作另行保留遍历声明**

#### 两端实际语义

SIMPACK QCH FileId 39502 明确把 `inversion` 定义为点列次序反转，并说明点列遍历方向决定型面
多边形的接触外侧与内侧。因此它不是无意义的文件排版字段。当前本地资产的原始点行都按横坐标
升序书写；`LM.prw`、`S1002.prw`、`DIN5573-28.prw` 声明 `inversion=1`，SIMPACK 的有效遍历
为横坐标降序；`UIC60.prr` 声明 `inversion=0`，有效遍历为升序。

WRL 的历史活动数值路径采用另一套承载方式。冻结脚本只提取坐标行并按横坐标升序排序，不读取
`inversion`；生成头也只保存升序坐标数组。接触几何构造器再按显式 `WheelSide` 对左侧横坐标
取反，并对车轮、钢轨点列重新升序排序。故近期 GZ18 与 IRW 资格化运行消费的是“型面角色、
显式左右侧、坐标符号和升序点列”的组合，运行时不消费 SIMPACK 多边形遍历声明。

这是一处真实的语义承载差异，但没有证据表明它导致四份冻结 WRL 资格型面的数值结果错误。历史
IRW 的源横坐标重离散先以输入数组首尾确定网格相位，再对结果排序；机械反转输入会把不能整除
步长的余量短段移到另一端。该相位来自 WRL 冻结器的升序数组，不来自 SIMPACK 有效遍历。

2026-08-13 的产品裁决不再复现这一 IRW 历史分支。当前 ORVD 对 GZ18 与 IRW 都先在物理右侧
升序点列上按 `0.5 mm` 等弧长重扫，再把完整结果精确镜像到左侧；重扫后的左右节点因此具有同一
网格相位。两车均在重扫节点上建立第二自然三次样条。`inversion` 仍不参与这一数值路径，只由
本地 SIMPACK `.prw/.prr` 互操作工具负责保存和恢复。

#### 迁移裁决

- ORVD 随包资格化 JSON 继续只承载规范型面角色、坐标约定与升序点列；`inversion` 不成为运行时
  物理参数。
- 当前产品数值路径按修订后的 ADR-0005 统一：以物理右侧为唯一重扫起点，按 `0.5 mm` 等弧长布点，随后
  对左侧精确镜像并建立第二自然三次样条。作者点列顺序不再作为车型专有的源横坐标网格相位。
- 本地 `.prw/.prr` SIMPACK 互操作读写必须遵循 QCH：读取时在归一化点列前解析有效遍历方向，写回时用
  `inversion` 恢复同一有效遍历。坐标逐位相同不足以证明互操作往返同义。
- 从 ORVD JSON 新写 SIMPACK 文件时，当前受支持的规范口径采用车轮降序、钢轨升序的有效遍历；
  这是本批已核实资产与坐标角色的转换合同，不宣称为所有未知供应商文件的普遍定律。超出该
  规范口径的文件必须保留其读取所得遍历元数据，或响亮拒绝，不得静默猜测。
- ORVD 提交 `3ed5d49` 已把 SIMPACK 遍历方向放在开发期互操作元数据中，并保持产品点列升序；
  该分层与本条裁决一致。当前接触消费者不得反向读取这项元数据。

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
- ORVD `libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_profile_preprocessing.h`、
  `libs/wheel_rail_contact/src/wheel_profile_preprocessing.cc` 与 ADR-0005：当前右侧等弧长重扫、
  左侧精确镜像和唯一产品实现。
- ORVD `tools/profile_conversion/simpack_profile_io.{h,cc}`：开发期互操作元数据与 QCH 同义写回。

本条由 CodeX 于 2026-08-05 对 QCH 与冻结 WRL 活动路径分层复核，并于 2026-08-13 按现行产品
裁决更新；历史证据未被改写。

### MD-007 — 三维互穿纵向长度解析失败时的处置语义

- 车型：GZ18、后续 IRW 共性
- 层级：轮轨法向力、数值试算时相
- 状态：**已裁决；G52b 后续修复已恢复近期 WRL 冻结执行语义**

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
三维成功才覆盖及失败计数的近期 WRL 冻结执行语义。自 `443142d` 起接触已进入系统 RHS，三维长度缺失本身
不再终止 RHS；其他真实配置错误、几何歧义与内部不变量破坏仍按致命事务处理。

#### 动力学影响

解析弦长与三维互穿长度不同，会改变 EEC 等效椭圆和法向力；强失败则直接失去该次 RHS 试算乃至推进器
事务。两者都不是纯诊断差异。近期端到端复现优先级高于从 QCH 沉默处推导一个更严格的新合同。

#### 最小核验或裁决

项目负责人裁决（CodeX 记录，2026-08-08）：G52b 后续修复恢复近期 WRL 冻结执行语义——解析弦长作基线、三维成功才
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

### MD-013 — 多接触斑容量与资格观察语义不同

- 车型：共性，G71 首次成为 IRW 的真实消费者
- 层级：轮轨接触拓扑与观察输出
- 状态：**已裁决当前资格语义；超过共同工件容量的范围尚未资格化**

#### 疑点

冻结 WRL、ORVD 和现有 SIMPACK SBR 对“发现多少候选斑、进入动力学多少斑、对外写出多少斑”使用
不同的固定容量与输出形状。若只看一个宽表中的 `Tx/Ty`，很容易把最大法向力主斑、全部斑合力或
不同接触系下局部切向力误称为同一个量。

#### 两端实际消费者

冻结 WRL 接触核最多保留 16 个候选斑，正式力／结果面最多输出 4 斑；冻结 A/B 运行选择
`all_patches`，故全部正式斑进入车辆动力学，常规车辆 CSV 的局部 `N/Tx/Ty` 则只写最大法向力
主斑。ORVD 几何和正式结果面均定容为 16 斑，
每个返回斑逐一计算法向力、蠕滑力和空间扳手，车辆 RHS 把全部斑运输到轮体原点后确定性累加。
现有 SIMPACK A 层 SBR 提供每接口 5 个斑槽；这只是本次对拍工件的通道形状，不据此推断厂商求解器
的普遍硬上限，也不反向限制 ORVD。

G77 开跑前的同人格 SIMPACK Realtime 直接实验又为每轮五个 Type-78 槽输出原生
`-N/-Tx/-Ty`。该实验以 `raw_minus_n < 0` 识别活动压缩槽，并按最大 `N=-raw_minus_n` 选择主槽；
`raw_minus_tx/raw_minus_ty` 已是由轨侧 `Tx/Ty` 取反后的规范轮侧切向力，离线比较不得再次取反。
活动压缩槽数不是 SIMPACK 的 `result.count/ch_022`，五槽也仍只代表本次输出形状。ORVD 一侧继续
使用真实返回斑数；两边都禁止跨不同局部接触系直接累加 `Tx/Ty`。

冻结 IRW A/B 30 s 工件的峰值斑数为 2，处于三方共同可表达范围。约 `3.659 s` 连续 13 个
`100 µs` 样本只有 7 轮接触是零斑事件；它与峰值双斑是两个不同拓扑观察，不预设发生在同一时刻。

#### 动力学影响与当前裁决

项目负责人裁决（CodeX 记录，2026-08-10）：ORVD 不再因资格观察遇到第二个斑而拒绝。每接口始终
发布真实斑数、总 `Q`、总 `N` 和统一载体投影 Track-T 表达的轮侧合力；每斑另存局部
`N/Tx/Ty`、相对载体投影 Track-T 原点的轮面接触点、Track-T 力和接触系角。不同接触系中的局部
`Tx/Ty` 禁止直接相加。宽表中的便利 `N/Tx/Ty` 明确选择
最大法向力主斑，与 WRL 历史导出规则同名对照；逐斑长表才是完整观察面。若 ORVD 将来返回超过
SIMPACK 5 槽或冻结 WRL 4 槽的结果，须报告“参考工件无法逐斑表示”，不得截断斑、修改 RHS 或把
容量差异直接判成物理错误。

SIMPACK SBR 的只读提取、逐斑端点和符号核对只进入项目 `tmp/`，不入 Git、不成为产品输入或金标。
G70/G71 的宏观位移、摇头和总轮轨力资格不依赖跨时刻持久的斑 ordinal；逐斑匹配必须另有明确的
接触点／侧别依据。

#### 源码锚点

- WRL `scripts_cpp/rwc_core/include/rwc_core/types.h`、
  `scripts_cpp/rwc_core/src/contact_kernel.cc`：16 个候选槽与 4 个正式输出槽。
- WRL `scripts_cpp/drake_sim/src/wheel_rail_contact_system.cc`、
  `scripts_cpp/irw/src/irw_csv_writer.cc`：A/B 的全部正式斑进入动力学，常规 CSV 选择最大法向力主斑。
- ORVD `libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_geometry.h`、
  `libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_model.h`、
  `libs/forces/include/orvd/forces/wheel_rail_contact_force_plan.h` 与
  `libs/forces/src/wheel_rail_contact_force_plan.cc`：16 斑正式结果、逐斑求力、RHS 聚合及总量／逐斑观察。
- WRL 冻结 A/B 运行元数据：`peak_patch_count=2`；
  `irw_layered_passive_30s_alignment.json`：7 轮短暂边界。

### MD-014 — IRW 长窗的线性求解后端不同

- 车型：IRW
- 层级：积分器数值执行身份与性能
- 状态：**已确认执行差异；当前主线不统一后端**

#### 疑点

相同的 CVODE BDF 与误差容差并不意味着相同的线性求解路径。若忽略线性后端差异，ORVD 与冻结
WRL 的墙钟、Jacobian/RHS 计数或末位轨迹差异会被错误归因给车辆模型或轮轨算法。

#### 两端实际消费者

冻结 WRL A 层使用 CVODE BDF 最大二阶、`rhs_state_eval` 与 SPGMR：Krylov 维数 20、线性容差
因子 `0.05`、零重启、线性建立频率 20。ORVD 使用稠密数值 Jacobian 和稠密线性求解，线性建立
建议频率为 30、Jacobian 年龄为 51；最大 BDF 阶数由资格实例冻结，不再是全局身份。IRW A 与
100 Hz 受控 IRW 使用最大二阶及 J0：`rtol/q/v/z=1e-6/1e-6/1e-5/1e-6 N`；IRW B 使用最大
五阶及 t8：`rtol/q/v/z=1e-8/1e-8/1e-7/1e-6 N`。主线不迁入 SPGMR，也不公开任意阶数或容差
组合面。

#### 动力学影响与当前裁决

两种后端以及 ORVD 的不同实例配方可能产生不同的 RHS／线性 RHS 次数、误差测试与非线性收敛历史、
墙钟和末位轨迹；因此性能比较必须留下实际最大阶数、容差和线性后端身份，不能把速度比解释为某一个
接触优化的净加速，也不要求不同后端逐位轨迹相同。主线按需读取累计统计快照，分别记录成功内步、
普通与线性求解器 RHS、误差测试失败、非线性迭代／收敛失败、线性建立和 Jacobian 次数。该接口
没有逐步回调、热路径计时、额外状态求值或车辆专用日志。

#### 源码锚点

- WRL 冻结 A 层运行脚本与运行元数据：CVODE／SPGMR 配置和原生统计。
- ORVD `cvode_continuous_state_advancer.cc`：逐实例 BDF 最大阶数、稠密矩阵／线性求解器和只读统计查询。
- ORVD `continuous_state_advancer.h`、`system_continuous_state_advancer.h`：后端中立统计值与系统层转发。
- ORVD `irw_integration_recipes.h`：IRW 被动与 100 Hz 全状态轮速导向的闭合内部
  默认积分器／容差配方；研究 override 与物理场景身份分离。

### MD-015 — IRW 100 Hz 控制事件的机械观察龄与启动更新时相

- 车型：IRW
- 层级：离散控制事件、机械观测与驱动转矩指令调理
- 状态：**已确认时相差异并已裁决 ORVD 主线语义**

#### 疑点

P179 三个平台都标称 100 Hz、10 ms 零阶保持和同一全状态导向—轮速 PI—转矩约束链，但控制事件
`t_k` 消费的机械状态龄并不相同。若把 SIMAT 通信标签直接等同于冻结 WRL 控制事件，会把协同通信
次序造成的一拍观察龄误写成控制算法自身延迟；若只看车辆宏观量，又会漏掉早期转矩的明确时相差异。

#### 两端实际消费者

P179 资格运行所用 WRL C++ 与 SIMPACK Realtime 在控制边界 `t_k` 读取当前接受机械状态：八轮相对转速、
四轴横移、摇头和站位。全状态控制器先生成轮侧请求，历史 `bridge_proxy` 再消费本事件请求、当前
八轮转速和上一接受事件的八路内部记忆，形成作用于 `[t_k,t_{k+1})` 的八路保持转矩。ORVD 迁移后
把这一调理职责命名为 `WheelDriveTorqueCommandConditioner`。

SIMPACK–SIMAT 的通信路径不同：事件 `k=0` 消费 H3 初态；`k>=1` 的八轮相对转速、四轴横移和
摇头消费上一通信事件的机械输入。站位／参考日程、控制器与调理器记忆、转矩提交时刻并未一起延迟。
P156 受控反事实只给 WRL 研究副本增加这一观察龄后，`k=1` 的八路调理后转矩由最大差
`75.691371 N·m` 变为逐位相同，11 个事件的转矩 RMS 从 `26.686492` 降至
`20.178508 N·m`。这证明差异承重，但不证明应改变冻结 WRL 产品语义。

P179 还具有独立的启动双更新合同：同一 H3 输入先执行一次只更新控制器／调理器记忆的初始化更新，
两条递推仍各走一个完整 `10 ms` 采样周期，但该次输出不进入任何正时长车辆保持区间；随后执行一次
`t=0` 周期更新，其输出作用于 `[0,0.01 s)`。P160 将该合同加入 WRL 研究副本后，首个 10 ms
转矩区间闭合，0–0.1 s 转矩 RMS 降低 `87.5453%`。启动双更新与 `k>=1` 的机械观察龄不是同一个
问题，不得因拒绝 SIMAT 一拍观察龄而一并删除。

P179 冻结 Drake 的累计控制边界还会影响控制记录落入整数观察行的归属，但不改变车辆状态的比较
时钟：从事件 `U2432` 起，累计边界晚于名义整数样本超过 `1e-12 s` 的归属容差，所以标准化行
`2432` 仍保存 `U2431` 的保持量，行 `2433..3000` 才对应 `U2432..U2999`。全部
`U0..U2999` 都存在；冻结 WRL 不存在的是作用时长为零的整数网格终点事件 `U3000`。因此车辆状态
仍按共同时间和共同站位比较，控制记录则须把原始一基计数 `research_periodic_update_count` 转为
零基事件序号 `U=count-1` 后对齐，不得把标准化行号或名义时间标签直接当作事件号，也不得据此移动
机械轨迹。

#### 动力学影响与当前裁决

项目负责人裁决（CodeX 记录，2026-08-10）：ORVD 的控制器／调理器纯递推以 WRL Git 固化且与 P179
源快照一致的 C++ 为参考；每个 100 Hz 周期事件消费当前接受机械状态、启动双更新和 10 ms 零阶保持
则按 P179 外置资格源快照实现。不建立 SIMAT 上一通信拍缓存，也不增加时相选择分支；SIMAT 的一拍
观察龄只作为协同通信差异登记。控制器状态、调理器记忆和八路保持转矩在同一事件事务中提交。

资格比较须分别记录机械观测时相、启动调度以及请求／调理后／实际保持转矩。禁止把 SIMAT、Realtime
与 WRL 三列笼统称为完全相同的事件人格，也不得通过移时或结果拟合掩盖差异。当前不增加非主线公式、
额外状态求值或运行期日志。

#### 源码与证据锚点

- WRL `ae5d77c`；`scripts_cpp/irw/src/irw_full_state_pi_controller.cc`、
  `scripts_cpp/irw/src/irw_cvode_driver.cc` 与 `scripts_cpp/drake_sim/src/motor_bridge_proxy.cc`：
  当前接受态控制、调理器更新和保持转矩提交。
- P179 `P179_PROTOCOL.md`、`H3_AND_CONTROL_IDENTITY_MANIFEST.json`：100 Hz、启动双更新、普通
  `bridge_proxy` 与三平台移交身份。
- P156 外置 `p156_drake_100hz_observation_age_counterfactual/RESEARCH_NOTE.md`：SIMAT 上一通信拍
  机械观察龄的字段边界与早期转矩证据。
- P160 外置 `p160_drake_100hz_startup_double_update_counterfactual/RESEARCH_NOTE.md`：同一 H3
  输入启动双更新的独立证据。
- ORVD DEC-039 与路书 G73–G77：现行迁移处置。

### MD-016 — IRW 主动转矩的 SIMPACK 标记表达基与冻结 WRL 轴桥轴

- 车型：IRW
- 层级：主动转矩作用对、转矩轴与表达坐标系
- 状态：**已确认差异；ORVD 首版处置已裁决**

#### 疑点

SIMPACK 两套已冻结主动力矩定义都把 From 标记放在转向架构架、To 标记放在独立车轮，并选择
转矩 `Y` 分量。QCH 明确 component 力元按 From Marker 坐标系公式化，实际力和矩也在 From
Marker 坐标系表达；正矩施于 From，反矩施于 To。冻结 WRL 则用车轮转动副父体轴桥的局部 `+Y`
经世界姿态旋转后作为转矩轴，并把正标量施于车轮。车轮局部 `+Y` 与轴桥局部 `+Y` 由转动副拓扑
保持共轴；真正可因 Ball-RPY 相对共同转动轴偏转的是构架 From 标记 `+Y`。

#### 两端实际消费者

SIMPACK Realtime 使用 Type-110 `$F_Motor_{A..D}`：From 为 `$M_Frame_Motor_*`，To 为相应
`$M_IRW_Motor_{L/R}`，`force.par(7)=2` 选择转矩轴。SIMAT 使用 Type-93
`$F_Motor_*_Simat`，From／To 相同，只有 `force.par(5)` 的转矩 `Y` 输入承重。现有源文件能够
确认作用对和分量编号；QCH `Formulations` 与 `Outputs` 两页又把 component 力元和 applied
force／torque 的表达基明确为 From Marker。模型的 `glob.compat.afcf=0`，未启用旧式全局参考系
兼容改写。因此 SIMPACK 原生标量正号是沿构架 From 标记 `+Y` 施于构架、反向施于轮体。

冻结 WRL `TorqueApplierSystem` 另行绑定车轮转动副、车轮体和构架体。它从转动副父体轴桥的世界
姿态取得局部 `+Y`，向车轮施加 `+tau` 纯转矩、向构架施加 `-tau` 纯转矩。轴桥只提供轴方向，
不承受反力。该系统还含一个宽幅末端钳位；P179 的活动约束由上游 `bridge_proxy` 承重，ORVD 将其
统一迁入 G74 的 `WheelDriveTorqueCommandConditioner`，G73 不保留第二份幅值权威。

#### 动力学影响与当前裁决

当构架 From 标记 `+Y` 与轴桥／轮体共同转动轴在动态姿态下不同，同一标量转矩会产生不同世界系
力偶方向和虚功；即使两轴暂时共轴，SIMPACK From 正号与 WRL 轮侧正号仍相反。差异可进入轮速、
构架响应与轮轨力。项目负责人已裁决：ORVD 首轮必须先闭合近期冻结 WRL 的计算路径。因此 G73
已由 G73 冻结“轴桥 `+Y` 提供轴、轮体受正转矩、对应构架受反转矩”，并用构架轴与共同转动轴
可分辨的姿态夹具及同状态 WRL 对拍验证；不为 SIMPACK 的另一表达合同增加运行时策略或第二套
产品公式。

若 ORVD 与冻结 WRL 不闭合，按迁移错误立即修复。与 SIMPACK 原生输出核对时，必须先把 From 侧
原始标量转换为规范轮侧标量 `tau_wheel = -tau_from_raw`，再分别观察构架轴与轴桥轴的方向差；
不得以直接同号比较制造假差异，也不得因宏观量接近而宣布两套轴合同等价。P179 同人格车辆长窗后
再决定是否需要把 SIMPACK From 基公式晋级为产品主线。

#### 源码锚点

- WRL `scripts_cpp/drake_sim/src/torque_applier_system.cc`：轴桥父体 `+Y`、轮体正转矩与构架反转矩。
- SIMPACK `mbs_simpack/irw_4WDB/ref_files/Bogie_IRWs_4WDBv3.spck:1969-2101`：
  Type-110／Type-93 的 From／To、轴号与转矩 `Y` 输入。
- SIMPACK `mbs_simpack/irw_4WDB/ref_files/IRW_4WDBv31.spck:478-528`：独立车轮标记与轴桥—车轮
  转动副拓扑。
- 本地 QCH `Force Elements / Formulations`、`Outputs`、Type-93 与 Type-110 参数页：component
  力元的 From Marker 表达基、正反作用方向与 `Y` 分量身份。
- ORVD DEC-040、R029 与路书 G73：已落地迁移处置。

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
