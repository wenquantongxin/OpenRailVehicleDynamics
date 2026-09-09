[English](CONVENTIONS_AND_NOTATION.en.md)

# 坐标与记号约定

本篇统一 ORVD 理论文档使用的坐标系、正号、站位、位姿、状态和扳手记号。正文给出的代码链接只用于指出这些理论量在本库中的落点，不构成接口或配置参考。

## 1. 用途与适用范围

其他理论文档默认遵守本篇，只声明自己新增的记号。若某一模型采用不同的正号或表达系，应在公式出现前说明。第 6 节汇总各篇共用的中英术语。

本篇覆盖线路几何、轮轨接触、多体动力学、力元、系统状态和时间积分所共用的理论量。软件架构、缓存、工作区、异常、测试和实验术语不在本篇范围内。

坐标系以字母标识。旋转矩阵 $R_{AB}$ 把 B 系分量变换为 A 系分量；向量的表达系由最后一个下标、上标或首次引入时的文字明确给出。米、秒、弧度、牛顿和帕斯卡均使用 SI 制。

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

轮轨型面的横向坐标沿轨道右侧为正，竖向坐标向下为正。侧号记为 $\varsigma$：右侧 $\varsigma=+1$，左侧 $\varsigma=-1$（`WheelSide`）。资产点列经 `ResolveForSide` 变成物理侧点列：两侧都按横向坐标升序重排，右侧保留作者书写的横向符号，左侧先做横向镜像。作者点列允许递减书写，因此右侧的顺序同样可能改变。代码落点见 [`profile_points.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/profile_points.h) 的 `ProfilePoints::ResolveForSide`。

不旋转的轮型面系 W 采用 `[周向 x，横向 y，径向向下 z]`。其原点位于车轴上的型面基准，姿态不包含车轮自旋；轴对称型面的几何不随自旋改变。字母 `P` 专用于接触力的轮侧作用点；它由接触组装中的换点约定定义，不默认等同于未变形回转面上的精确材料点。

位姿归约与型面放置链中的轨底坡幅值记为 $\phi_c$，是一个正数；某一侧钢轨按侧带符号的轨底坡滚转为

$$
\phi_r=-\varsigma\,\phi_c
$$

即右侧钢轨向轨道中心倾斜，在横向向右、竖向向下的约定下是绕纵轴的负滚转（`RailGaugeDatum::roll_radians`）。左右横向基准和轨距面偏移只有在作者轨型关于其自身横向零点 $y=0$ 镜像对称时才互为镜像；这不是任意轨型的无条件性质。代码落点见 [`rail_gauge_datum.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/rail_gauge_datum.h) 的 `ComputeRailGaugeDatum`。

### 2.4 钢轨侧参考系与型面放置

轨底坡系 $T_c$ 是位姿标量编码与二维接触几何使用的横截面参考系。它相对当前轨型系 T 只含按侧带符号的轨底坡滚转：

$$
R_{TT_c}=R_x(\phi_r)
$$

轮、轨型面基准之间的横竖分离、二维接触几何中的高度比较，以及竖向穿透与其法向投影，都按这个横截面参考表达。$T_c$ 不是钢轨型面在三维空间中的完整放置系：钢轨型面的放置原点另记为 $\mathbf o_c$，放置姿态记为 $R_{T\mathrm{rail}}$。后者除轨底坡外还含方向与高低不平顺的两个斜率角，并含承载站位与有效型面站位两处轨型系之间的姿态差，因此不能写成 $R_{TT_c}$。

局部切向钢轨系 $T_\ell$ 是位姿滚转提取所用的另一个参考系；它在纯轨底坡姿态之上再叠加两个由不平顺空间斜率形成的角：方向不平顺角 $\psi_\epsilon$ 与高低不平顺角 $\theta_\epsilon$，

$$
R_{TT_\ell}=R_z(\psi_\epsilon)R_y(\theta_\epsilon)R_x(\phi_r)
$$

其中 $\theta_\epsilon$ 的符号取法使钢轨沿站位下降对应绕 $+y$ 的负转角。在位姿归约内，$T_\ell$ 只用于度量位姿滚转 $\varphi$，横竖分离不在其中度量；两个不平顺斜率为零时，同一站位上的 $T_\ell$ 与 $T_c$ 重合。两角的构造见[轮轨位姿归约与不平顺输入](wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.md)。

### 2.5 平面曲率、纵坡与超高

曲率 $\kappa$、纵坡 $g$ 和超高 $u$ 都以平面投影站位 $s$ 为自变量。线路几何的解析段、接缝和定义域外延长均须保持这一个站位定义，不能把 $s$ 换成三维弧长 $\ell$。

ORVD 的 Hermite 三次混合段使用归一化多项式

$$
H(\xi)=3\xi^2-2\xi^3,\qquad 0\le\xi\le1
$$

将它用于曲率过渡时得到 Bloss 型曲率律；它不是曲率随站位线性变化的回旋线。

### 2.6 轮轨位姿标量

`ContactPoseScalars` 用四个标量描述轮型面相对轨型面的内禀几何位姿：滚转 $\varphi$、冲角 $\beta$、横向偏移 $d_y$ 和竖向抬升 $d_z^{\uparrow}$。竖向抬升向上为正，是本项目 `+z` 向下约定中唯一反向的位移正号。代码定义见 [`wheel_rail_pose.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h) 的 `ContactPoseScalars`。

这四个量在轮、轨型面基准之间度量，不等同于两个刚体原点之间的平移或完整相对姿态。轮、轨型面基准之间的横竖分离在轨底坡系 $T_c$ 中度量；两个偏移 $d_y$、$d_z^{\uparrow}$ 由该分离再经一次含滚转 $\varphi$ 的自逆反射编码得到，只有 $\varphi=0$ 时 $d_y$ 才等于 $T_c$ 中的横向分离、$d_z^{\uparrow}$ 才等于 $T_c$ 中竖向分离的反号，编码式见[轮轨位姿归约与不平顺输入](wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.md)。滚转 $\varphi$ 相对局部切向钢轨系 $T_\ell$ 度量。冲角 $\beta$ 是轮对摇头与方向不平顺所给局部钢轨方向之差；实现以 $\operatorname{atan2}(\dot\eta_y,\dot s)$ 形成该方向角。它不是某个旋转矩阵的单一欧拉角分量。

### 2.7 接触几何中的高度与穿透

接触几何把轮面高度与轨面高度写在轨底坡系 $T_c$ 的同一个横截面坐标中。设 $z_r(Y)$ 为钢轨横截面高度，$H(Y)$ 为轮面投影到该横截面所得的单值上包络，竖向互穿函数为

$$
g(Y)=H(Y)-z_r(Y)
$$

$g>0$ 表示未变形曲面发生几何互穿；接触判定另取一个间隙阈值 $\epsilon$，判据是严格不等式 $g>\epsilon$，见[接触几何](wheel_rail_contact/CONTACT_GEOMETRY.md)。此处的 $H(Y)$ 是投影上包络，与第 2.5 节的 Hermite 混合多项式 $H(\xi)$ 不是同一函数；此处的 $g(Y)$ 以横截面横坐标为自变量，与第 2.1 节以站位为自变量的纵坡 $g(s)$ 也不是同一个量。

竖向穿透 $\delta_v$（`vertical_penetration_meters`）是沿 $T_c$ 竖向的最深重叠；法向穿透 $\delta_n$（`normal_penetration_meters`）是同一深度在局部轨面法向上的投影；等效穿透 $\delta_{\mathrm{eq}}$（`equivalent_penetration_meters`）是法向力律实际使用的穿透。

每个斑保留两种角：公法线角 $\gamma$（`common_normal_angle_radians`）用于纵向尺度构造，接触坐标系角 $\alpha$（`rail_slope_angle_radians`）用于接触坐标系和力的表达。两者不能互换。轨面在斑形心处的自身坡角记为 $\alpha_r$，是接触几何的内部量。

滚动半径使用小写字母：标称滚动半径记为 $r_0$（`nominal_rolling_radius_meters`），接触处的局部滚动半径记为 $r$（`rolling_radius_meters`）。旋转矩阵使用大写 $R$；材料参考点 `R` 只作为点标号或位置向量下标出现。

### 2.8 接触坐标系 C

接触坐标系 C 相对轨型系 T 只绕纵向轴转过接触坐标系角 $\alpha$：

$$
R_{TC}=
\begin{bmatrix}
1&0&0\\
0&\cos\alpha&-\sin\alpha\\
0&\sin\alpha&\cos\alpha
\end{bmatrix}
$$

其第三轴是接触法向。接触几何链另以 $c_r$ 记自身采用的轨底坡幅值（`ContactGeometryConfiguration::rail_cant_radians`），于是

$$
\alpha=\alpha_r-\varsigma c_r.
$$

当前实现有意分别保存 $c_r$ 与位姿链的 $\phi_c$；二者代表相同类型的几何角，但不能在推导中直接视为同一个数。公法线角 $\gamma$ 则由接触几何在消去配对滚转与冲角后形成，用于纵向尺度构造；它与 $\alpha$ 不是同一个角。

纵向与横向蠕滑率记为 $\xi_x$、$\xi_y$（`longitudinal`、`lateral`），自旋蠕滑率记为 $\xi_{sp}$（`spin_per_meter`），单位为 $\mathrm{m}^{-1}$；三者与切向力都在 C 中表达。`ContactFrame` 与 `Creepages` 的代码定义见 [`contact_creepage.h`](../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h)。$R_{TC}$ 的构造实现见 [`contact_creepage.cc`](../../libs/wheel_rail_contact/src/contact_creepage.cc) 的 `MakeContactFrame`。

### 2.9 轨道不平顺的表达系

横向和竖向不平顺位移在轨型系中表达，横向向右、竖向向下为正，自变量仍为站位 $s$。空间斜率乘以站位速率形成相应的采样变化率；该变化率描述车辆沿固定空间场取样，不自动等同于钢轨材料点速度。

## 3. 站位、弧长与速率

站位 $s$ 是中心线水平投影的弧长，三维弧长 $\ell$ 满足

$$
\frac{d\ell}{ds}=\sqrt{1+g^2},\qquad
\dot\ell=\sqrt{1+g^2}\,\dot s
$$

因此非零纵坡上，平面站位速率 $\dot s$ 与三维路径速率 $\dot\ell$ 不可互换。位姿归约中的方向角以 $\dot s$ 为基准；当前蠕滑实现使用三维路径速率形成参考速度。具体映射分别见 [`wheel_rail_pose.cc`](../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) 与 [`contact_creepage.cc`](../../libs/wheel_rail_contact/src/contact_creepage.cc)。

轨型系随站位的角变化率记为 $\boldsymbol\omega_{IT}$，即在 I 中表达，并满足

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

$\mathbf p_{AoBo\_A}$ 表示从 A 原点到 B 原点、在 A 中表达的位置向量。$\boldsymbol\omega_{AB\_E}$ 表示 B 相对 A 的角速度、在 E 中表达；$\mathbf v_{ABo\_E}$ 表示 B 原点相对 A 的速度、在 E 中表达。第 5.2 节的扳手以明确的上标表示表达系。像第 3 节的 $\boldsymbol\omega_{IT}$ 这样沿用既有简写的量，则在首次引入时用文字说明表达系。

旋转矩阵的字母下标是坐标系标识，两个数字下标则是矩阵元：$R_{ij}$ 指第 $i$ 行第 $j$ 列，$i,j\in\{1,2,3\}$，即行列编号从 1 起。要指出某一列时写 $R\mathbf e_j$，其中 $\mathbf e_j$ 是第 $j$ 个标准基向量。

### 4.2 四元数与自由体

自由体使用七个广义位置和六个广义速度：位置为 `[四元数 w,x,y,z；原点位置]`，速度为 `[角速度；原点平移速度]`，均在世界系表达。四元数的四个分量承载三个转动自由度，因此一般有 $n_q\ne n_v$。区间类型见 [`multibody_coordinate_ranges.h`](../../libs/multibody_model/include/orvd/multibody_model/multibody_coordinate_ranges.h)。上述两条分量顺序约定见 [`multibody_model.h`](../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) 的 `MultibodyModel::GetFreeBodyPositionRange` 与 `MultibodyModel::GetFreeBodyVelocityRange`。

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
\mathrm{roll}&=\operatorname{atan2}(R_{32},R_{22}),\\
\mathrm{yaw}&=\operatorname{atan2}\left(-R_{12},\sqrt{R_{11}^2+R_{13}^2}\right),\\
\mathrm{pitch}&=\operatorname{atan2}(R_{13},R_{11})
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

轮轨相互作用以成对扳手表示；在同一取矩点和表达系中，两半的力与力矩互为相反数。轮侧力作用点 `P` 与计算相对速度所用的轨面材料参考点 `R` 是不同的点，其位置向量分别写作 $\mathbf x_P$、$\mathbf x_R$，以免与旋转矩阵 $R$ 混淆。

### 5.3 逆动力学符号

本库所需广义力采用

$$
\tau_{\mathrm{required}}
=M(q)\dot v+C(q,v)v-\tau_{\mathrm{gravity}}-\tau_{\mathrm{damping}}
$$

$\tau_{\mathrm{gravity}}$ 和 $\tau_{\mathrm{damping}}$ 表示施加在系统上的广义力，因此在“为实现给定加速度所需的力”中带负号。本节的 $\tau$ 是广义力，与第 5.2 节中作为空间力矩的 $\boldsymbol\tau$ 不是同一个量。$M(q)$ 是系统质量矩阵，$C(q,v)$ 是系统级的科氏与离心项矩阵；力元级的标量刚度与阻尼一律用小写 $k$、$c$。

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

字母 $N$ 有两个既定用途：在多体与数值链中表示第 4.5 节的位置导数映射 $N(q)$，在轮轨接触链中表示法向总力 $N$；后者的弹性与阻尼两部分记为 $F_e$、$F_d$（`NormalContactResult`）。状态维数记为 $n_x$。

## 6. 核心中英术语

| 中文 | English | 代码中的主要名称 |
|---|---|---|
| 站位 | track station | `track_station_meters` |
| 三维弧长 | three-dimensional arc length | —（源码中只以其速率形式出现） |
| 三维路径速率 | three-dimensional path rate | `path_rate_meters_per_second`（接触模型入口字段名为 `arc_rate_meters_per_second`） |
| 航向 | heading | `heading_radians` |
| 平面曲率 | planar curvature | `curvature_radians_per_meter` |
| 纵坡 | grade | `centerline_upward_grade` |
| 超高 | superelevation | `superelevation_meters` |
| 无侧滚切向系 | roll-free tangent frame | — |
| 轨型系 | track frame | `TrackFramePose` |
| 型面 | profile | `ProfilePoints` |
| 轨距基准 | rail gauge datum | `RailGaugeDatum` |
| 轨底坡 | rail cant | `rail_cant_radians` |
| 轨底坡系 | rail-cant frame | — |
| 局部切向钢轨系 | local tangential rail frame | — |
| 方向不平顺角 | alignment angle | — |
| 高低不平顺角 | vertical irregularity angle | — |
| 冲角 | angle of attack | `ContactPoseScalars::yaw_radians` |
| 接触斑 | contact patch | `ContactPatch` |
| 竖向穿透 | vertical penetration | `vertical_penetration_meters` |
| 法向穿透 | normal penetration | `normal_penetration_meters` |
| 等效穿透 | equivalent penetration | `equivalent_penetration_meters` |
| 公法线角 | common-normal angle | `common_normal_angle_radians` |
| 接触坐标系角 | contact-frame angle | `rail_slope_angle_radians` |
| 接触坐标系 | contact frame | `ContactFrame` |
| 纵向蠕滑率 | longitudinal creepage | `longitudinal` |
| 横向蠕滑率 | lateral creepage | `lateral` |
| 自旋蠕滑率 | spin creepage | `spin_per_meter` |
| 蠕滑系数 | creepage coefficients | `KalkerCoefficients` |
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
