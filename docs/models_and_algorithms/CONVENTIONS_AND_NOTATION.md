[English](CONVENTIONS_AND_NOTATION.en.md)

# 坐标与记号约定

本篇统一 ORVD 理论文档使用的坐标系、正号、站位、位姿、状态和扳手记号。正文给出的代码链接只用于指出这些理论量在本库中的落点，不构成接口或配置参考。

## 1. 用途与适用范围

其他理论文档默认遵守本篇，只声明自己新增的记号。若某一模型采用不同的正号或表达系，必须在公式出现前说明。

本篇覆盖线路几何、轮轨接触、多体动力学、力元、系统状态和时间积分所共用的理论量。软件架构、缓存、工作区、异常、测试和实验术语不在本篇范围内。

坐标系以下标标识。向量最后一个下标表示表达系；旋转矩阵 $R_{AB}$ 把 B 系分量变换为 A 系分量。米、秒、弧度、牛顿和帕斯卡均使用 SI 制。

## 2. 坐标系与正号

### 2.1 轨道惯性系 I

惯性系 I 固定在线路起点：`+x` 指向起点处站位增加方向，`+y` 指向面向站位增加方向时的右侧，`+z` 向下，三轴构成右手系。重力沿 `+z`。代码定义见 [`track_inertial_frame.h`](../../libs/track_geometry/include/orvd/track_geometry/track_inertial_frame.h)。

以站位 $s$ 参数化中心线 $\mathbf C(s)$，航向为 $\psi(s)$，向上纵坡为 $g(s)$：

$$
\mathbf C'(s)=
\begin{bmatrix}
\cos\psi(s)\\
\sin\psi(s)\\
-g(s)
\end{bmatrix},
\qquad
\lVert\mathbf C'(s)\rVert=\sqrt{1+g(s)^2}
$$

因此中心线的水平投影按单位弧长参数化；只有 $g=0$ 时三维导数也是单位向量。

### 2.2 无侧滚切向系与轨型系 T

无侧滚切向系的三个单位轴为

$$
\mathbf x_0=\frac{\mathbf C'(s)}{\sqrt{1+g^2}},\qquad
\mathbf y_0=\begin{bmatrix}-\sin\psi&\cos\psi&0\end{bmatrix}^{\mathsf T},\qquad
\mathbf z_0=\mathbf x_0\times\mathbf y_0
$$

轨型系 T 由无侧滚系绕自身 `+x` 轴转过超高角 $\phi$ 得到：

$$
R_{IT}=R_{I0}R_x(\phi),\qquad
\phi=\arcsin\left(\frac{u}{b}\right)
$$

$u$ 是有符号超高，$b$ 是超高参考基长，理论定义域为 $|u|<b$。正超高使右侧参考点更低；由于 `+z` 向下，它对应正滚转。代码实现见 [`track_geometry.cc`](../../libs/track_geometry/src/track_geometry.cc)。

平面曲率满足

$$
\frac{d\psi}{ds}=\kappa(s)
$$

正曲率表示右转。向上纵坡为正，所以 $dz/ds=-g$。

### 2.3 型面坐标系与左右侧

轮轨型面的横向坐标沿轨道右侧为正，竖向坐标向下为正。资产点列经 `ResolveForSide` 变成物理侧点列：右侧保留横向符号，左侧横向镜像后重新按升序排列。代码落点见 [`profile_points.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/profile_points.h)。

不旋转的轮型面系 W 采用 `[周向 x，横向 y，径向向下 z]`。其原点位于车轴上的型面基准，姿态不包含车轮自旋；轴对称型面的几何不随自旋改变。字母 P 专用于接触力的轮侧作用点；它由接触组装中的换点约定定义，不默认等同于未变形回转面上的精确材料点。

轨距基准的滚转按侧改变符号。左右横向基准和轨距面偏移只有在输入轨型关于中心线镜像对称时才相应镜像；这不是任意轨型的无条件性质。

### 2.4 平面曲率、纵坡与超高

曲率 $\kappa$、纵坡 $g$ 和超高 $u$ 都以平面投影站位 $s$ 为自变量。线路几何的解析段、接缝和定义域外延长均须保持这一个站位定义，不能把 $s$ 换成三维弧长 $\ell$。

ORVD 的 Hermite 三次混合段使用归一化多项式

$$
H(\xi)=3\xi^2-2\xi^3,\qquad 0\le\xi\le1
$$

将它用于曲率过渡时得到 Bloss 型曲率律；它不是曲率随站位线性变化的回旋线。

### 2.5 轮轨位姿标量

`ContactPoseScalars` 用四个标量描述轮型面相对轨型面的内禀几何位姿：滚转 $\varphi$、冲角 $\beta$、横向偏移 $d_y$ 和竖向抬升 $d_z^{\uparrow}$。竖向抬升向上为正，是本项目 `+z` 向下约定中唯一反向的位移正号。代码定义见 [`wheel_rail_pose.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h)。

这四个量在轮、轨型面基准之间度量，不等同于两个刚体原点之间的平移或完整相对姿态。冲角还包含横向运动方向，不是某个旋转矩阵的单一欧拉角分量。

### 2.6 接触几何中的高度与穿透

接触几何把轮面高度与轨面高度写在同一个横断面坐标中，重叠函数为

$$
g_c(y)=z_w(y)-z_r(y)
$$

$g_c>0$ 表示未变形曲面发生几何互穿。`vertical_penetration_meters` 是沿轨型系竖向的最深重叠；`normal_penetration_meters` 是同一深度在局部轨面法向上的投影。

每个斑保留两种角：公法线角用于纵向尺度构造，`rail_slope_angle_radians` 用于接触坐标系和力的表达。两者不能互换。

### 2.7 接触系 C

接触系 C 相对轨型系 T 只绕纵向轴转过接触角 $\alpha$：

$$
R_{TC}=
\begin{bmatrix}
1&0&0\\
0&\cos\alpha&-\sin\alpha\\
0&\sin\alpha&\cos\alpha
\end{bmatrix}
$$

其第三轴是接触法向。纵向、横向蠕滑率和切向力都在 C 中表达；自旋蠕滑率的单位为 $\mathrm{m}^{-1}$。实现见 [`contact_creepage.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h)。

### 2.8 轨道不平顺的表达系

横向和竖向不平顺位移在轨型系中表达，横向向右、竖向向下为正，自变量仍为站位 $s$。空间斜率乘以站位速率形成相应的采样变化率；该变化率描述车辆沿固定空间场取样，不自动等同于钢轨材料点速度。

## 3. 站位、弧长与速率

站位 $s$ 是中心线水平投影的弧长，三维弧长 $\ell$ 满足

$$
\frac{d\ell}{ds}=\sqrt{1+g^2},\qquad
\dot\ell=\sqrt{1+g^2}\,\dot s
$$

因此非零纵坡上，平面站位速率 $\dot s$ 与三维路径速率 $\dot\ell$ 不可互换。位姿归约中的方向角以 $\dot s$ 为基准；当前蠕滑实现使用三维路径速率形成参考速度。具体映射分别见 [`wheel_rail_pose.cc`](../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) 与 [`contact_creepage.cc`](../../libs/wheel_rail_contact/src/contact_creepage.cc)。

轨型系随站位的角变化率记为 $\boldsymbol\omega_{IT}$，在 I 中表达，并满足

$$
\frac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}
$$

空间点到中心线的站位投影使用局部种子跟踪，而不是在整条线路上搜索全局最近点；其数学目标和 Newton 更新见[线路几何与轨道坐标系](track_geometry/TRACK_GEOMETRY_AND_FRAMES.md)。

## 4. 位姿与运动学记号

### 4.1 单字母系记号

$R_{AB}$ 表示 B 系在 A 系中的旋转，满足

$$
\mathbf v_A=R_{AB}\mathbf v_B,\qquad R_{AC}=R_{AB}R_{BC}
$$

$\mathbf p_{AoBo\_A}$ 表示从 A 原点到 B 原点、在 A 中表达的位置向量。$\boldsymbol\omega_{AB\_E}$ 表示 B 相对 A 的角速度、在 E 中表达；$\mathbf v_{ABo\_E}$ 表示 B 原点相对 A 的速度、在 E 中表达。

### 4.2 四元数与自由体

自由体使用七个广义位置和六个广义速度：位置为 `[四元数 w,x,y,z；原点位置]`，速度为 `[角速度；原点平移速度]`。四元数的四个分量承载三个转动自由度，因此一般有 $n_q\ne n_v$。代码布局见 [`multibody_coordinate_ranges.h`](../../libs/multibody_model/include/orvd/multibody_model/multibody_coordinate_ranges.h)。

### 4.3 Ball-RPY 球铰

Ball-RPY 的位置采用 Z-Y-X 合成：

$$
R_{FM}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})
$$

其三个广义速度是物理角速度 $\boldsymbol\omega_{FM\_F}$，不是三个欧拉角的导数；二者通过位置相关映射联系。

### 4.4 轮对姿态的 X-Z-Y 解析

`ResolveRollYawPitch` 采用 X-Z-Y 顺序。对旋转矩阵 $R$：

$$
\begin{aligned}
\mathrm{roll}&=\operatorname{atan2}(R_{21},R_{11}),\\
\mathrm{yaw}&=\operatorname{atan2}\left(-R_{01},\sqrt{R_{00}^2+R_{02}^2}\right),\\
\mathrm{pitch}&=\operatorname{atan2}(R_{02},R_{00})
\end{aligned}
$$

它对应 $R=R_x(\mathrm{roll})R_z(\mathrm{yaw})R_y(\mathrm{pitch})$，与 Ball-RPY 的 Z-Y-X 合成不同。

### 4.5 位置导数映射

广义速度与位置导数的关系写为

$$
\dot q=N(q)v
$$

对四元数自由体，反向映射是到四元数切空间的左伪逆；任意与四元数平行的 $\dot q$ 分量不代表物理角速度。

## 5. 状态、力与单位

### 5.1 连续状态 `[q; v; z]`

ORVD 连续状态写作

$$
x=\begin{bmatrix}q\\v\\z\end{bmatrix},\qquad
\dot x=\begin{bmatrix}N(q)v\\\dot v\\\dot z\end{bmatrix}
$$

$q$、$v$ 是多体广义位置和速度；$z$ 是力元内部状态。当前 $z$ 的来源是串联弹簧黏性阻尼中的内力状态。时间、数值投影种子和控制器保持量不属于这个连续状态。

### 5.2 扳手的取矩点与表达系

空间扳手写作 $\mathcal W_Q^E=(\boldsymbol\tau_Q^E,\mathbf f^E)$：力和力矩在 E 中表达，力矩关于 Q 取矩。把取矩点从 Q 移到 O 时

$$
\boldsymbol\tau_O^E=\boldsymbol\tau_Q^E+\mathbf p_{OQ}^E\times\mathbf f^E
$$

改变表达系与改变取矩点是两个不同运算。

轮轨相互作用以成对扳手表示；在同一取矩点和表达系中，两半的力与力矩互为相反数。轮侧力作用点 P 与计算相对速度所用的轨面材料参考点 R 是不同的点。

### 5.3 逆动力学符号

本库所需广义力采用

$$
\tau_{\mathrm{required}}
=M(q)\dot v+C(q,v)v-\tau_{\mathrm{gravity}}-\tau_{\mathrm{damping}}
$$

$\tau_{\mathrm{gravity}}$ 和 $\tau_{\mathrm{damping}}$ 表示施加在系统上的广义力，因此在“为实现给定加速度所需的力”中带负号。

### 5.4 单位

| 量 | 单位 |
|---|---|
| 长度、站位、半轴、穿透 | m |
| 时间 | s |
| 速度 | m/s |
| 角度 | rad |
| 角速度 | rad/s |
| 曲率、自旋蠕滑率 | 1/m |
| 力、力矩 | N，N·m |
| 应力、弹性模量 | Pa |
| 质量、转动惯量 | kg，kg·m² |

纵坡、超高比、纵横向蠕滑率和泊松比无量纲。

## 6. 核心中英术语

| 中文 | English | 代码中的主要名称 |
|---|---|---|
| 站位 | track station | `track_station_meters` |
| 三维弧长 | three-dimensional arc length | `arc_rate_meters_per_second` |
| 航向 | heading | `heading_radians` |
| 平面曲率 | planar curvature | `curvature_radians_per_meter` |
| 纵坡 | grade | `centerline_upward_grade` |
| 超高 | superelevation | `superelevation_meters` |
| 轨型系 | track frame | `TrackFramePose` |
| 型面 | profile | `ProfilePoints` |
| 轨距基准 | rail gauge datum | `RailGaugeDatum` |
| 接触斑 | contact patch | `ContactPatch` |
| 竖向穿透 | vertical penetration | `vertical_penetration_meters` |
| 等效穿透 | equivalent penetration | `equivalent_penetration_meters` |
| 公法线角 | common-normal angle | `common_normal_angle_radians` |
| 接触系 | contact frame | `ContactFrame` |
| 纵向蠕滑率 | longitudinal creepage | `longitudinal` |
| 横向蠕滑率 | lateral creepage | `lateral` |
| 自旋蠕滑率 | spin creepage | `spin_per_meter` |
| 法向力 | normal force | `NormalContactResult` |
| 切向力 | tangential force | `TangentialContactResult` |
| 扳手 | wrench | `SpatialWrench` |
| 广义位置 | generalized position | `q` |
| 广义速度 | generalized velocity | `v` |
| 内部力状态 | internal force state | `z` |
| 质量矩阵 | mass matrix | `CalcGeneralizedMassMatrix` |
| 逆动力学 | inverse dynamics | `CalcRequiredGeneralizedForces` |
| 前向动力学 | forward dynamics | `CalcGeneralizedVelocityDerivatives` |
| 数值 Jacobian | numerical Jacobian | `DenseFiniteDifferenceJacobianProvider` |
| 时间积分器 | time integrator | `ContinuousStateAdvancer` |
