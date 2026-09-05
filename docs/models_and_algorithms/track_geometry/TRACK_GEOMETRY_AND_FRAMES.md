[English](TRACK_GEOMETRY_AND_FRAMES.en.md)

# 线路几何与轨道坐标系

本篇说明 ORVD 线路层 `orvd::track_geometry` 的几何模型与算法：沿站位铺开的标量剖面、由曲率与竖向剖面积分得到的三维中心线、轨道惯性系与两个随线路移动的坐标系、有限定义区间之外的切线延长，以及空间点到中心线的局部分支投影。公式要么直接对应源码中的计算，要么给出该计算所依据的数学定义；最后以一张紧凑映射表连接理论对象与实现位置。

## 1. 范围

本篇建模的对象是 [`libs/track_geometry`](../../../libs/track_geometry/README.md) 中的标量剖面 `TrackScalarProfile`（平面曲率与超高共用同一表示）、线路 `TrackGeometry`、位姿与运动学量 `TrackFramePose` 与 `TrackFrameKinematics`，以及局部分支投影 `TrackStationProjection`。重点是这些对象表达的物理量、解析关系和数值离散，不展开配置文件格式或调用接口清单。

本篇明确不做的事：竖向剖面 `TrackVerticalProfile` 的恒坡段、抛物线竖曲线与圆弧竖曲线公式见 [`TRACK_VERTICAL_PROFILE_MODELLING.md`](TRACK_VERTICAL_PROFILE_MODELLING.md)，本篇只写它进入中心线与轨道系的方式；轨道不平顺不属于线路几何，见 [`TRACK_IRREGULARITY_SPECTRA.md`](../track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md)；轮轨位姿归约如何消费 `TrackFrameKinematics` 属于 [`WHEEL_RAIL_POSE_REDUCTION.md`](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.md)。投影种子的时间推进与回滚属于动力学和数值方法，本篇只讨论单次几何投影。

坐标系、正号、站位定义与单位后缀全部沿用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)，本篇不重复其第 2 节与第 3 节的结论，只在需要时引用。

## 2. 记号

以下只列本篇新增的记号；$s$、$\psi$、$\kappa$、$g$、$u$、$b$、$\phi$、$\mathbf C(s)$、$R_{IT}$、$R_{I0}$、$\mathbf x_0,\mathbf y_0,\mathbf z_0$、$\boldsymbol\omega_{IT}$ 与定义区间 $[s_{\min},s_{\max}]$ 的含义见[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)第 2、3 节。

| 记号 | 含义 | 对应标识符 |
|---|---|---|
| $v(s)$ | 一条标量剖面的值，代入曲率剖面时为 $\kappa$，代入超高剖面时为 $u$ | `TrackScalarProfile::Value` |
| $s_i$、$L_i$ | 第 $i$ 段的起点站位与长度，起点由 `start_track_station_meters` 累加各段 `length_meters` 得到 | `TrackScalarSegment::length_meters` |
| $o$、$\hat s=s-o$ | 一个片的多项式原点与局部坐标 | `polynomial_origin_track_station_meters` |
| $x=\hat s/L_i$ | Hermite 混合段的归一化坐标 | `kHermiteCubicBlend` |
| $\Delta v$ | Hermite 混合段的终值减起值 | `end_value`、`start_value` |
| $c_k$ | 片在局部坐标上的升幂多项式系数 | `coefficients` |
| $F(\ell)$、$I_i$ | 片多项式在局部坐标零点取零值的原函数；剖面从 $s_{\min}$ 到第 $i$ 片起点的积分 | `integral_at_piece_start_` |
| $s_b$、$w$ | 接缝边界站位与接缝窗口全宽 | `TrackSeamTransition::window_length_meters` |
| $\xi$、$d_r$、$H_r(\xi)$ | 窗口归一化坐标、六个边界数据、五次 Hermite 基函数 | `kQuinticBasis` |
| $\Delta s$ | 声明的站位节点间距 | `station_node_spacing_meters` |
| $B_j$、$n_j$ | 曲率断点与竖向断点有序并集中的第 $j$ 个断点；断点区间被分成的面板数 | `nodes_` |
| $\xi_q$、$w_q$ | 八点 Gauss–Legendre 求积的横坐标与权 | `kQuadratureAbscissae`、`kQuadratureWeights` |
| $h$ | 常曲率面板的半转角 | `half_turn` |
| $\mathbf t$、$n$ | 未归一化的三维切向 $\mathbf C'(s)$ 及其模 $\sqrt{1+g^2}$ | `tangent`、`slope_norm` |
| $\boldsymbol\omega_0$ | 无侧滚切向系相对惯性系的站位旋转率 | `roll_free_rate` |
| $\mathbf p$、$f(s)$ | 待投影的空间点；投影目标函数，README 中记作 objective | `EvaluateObjectiveDerivatives` |
| $\tau$、$\ell_0$、$\beta$ | 一阶残差的相对容差、绝对尺度地板与由二者构成的残差门 | `kObjectiveGradientRelativeTolerance`、常数 `1.0`、`gradient_bound` |
| $s_{\text{seed}}$ | 调用方提供的分支种子站位 | `branch_seed_track_station_meters` |

## 3. 模型

### 3.1 标量剖面：常值段与 Hermite 三次混合段

一条标量剖面是从 `start_track_station_meters` 起首尾相接的若干段 `TrackScalarSegment`，每段长度 `length_meters` 必须有限且严格为正，段终点必须是严格晚于起点的有限站位。段的形状只有两种。`TrackScalarSegmentShape::kConstant` 在整段上保持 `start_value`；`TrackScalarSegmentShape::kHermiteCubicBlend` 以归一化坐标 $x$ 把值从 `start_value` 混合到 `end_value`：

$$
v(s)=v_{\text{start}}+\Delta v\,\left(3x^{2}-2x^{3}\right),\qquad x=\frac{s-s_i}{L_i},\qquad \Delta v=v_{\text{end}}-v_{\text{start}}
$$

实现不存储 $x$ 的多项式，而是在以段起点为原点的物理局部坐标 $\hat s$ 上存储升幂系数：构造函数写入 `piece.coefficients[0] = segment.start_value`、`piece.coefficients[2] = 3.0 * change / (length * length)`、`piece.coefficients[3] = -2.0 * change / (length * length * length)`，一次项系数为零：

$$
v(s)=c_0+c_2\,\hat s^{2}+c_3\,\hat s^{3},\qquad c_0=v_{\text{start}},\quad c_2=\frac{3\,\Delta v}{L_i^{2}},\quad c_3=-\frac{2\,\Delta v}{L_i^{3}}
$$

对 $x$ 求导可见这条混合的两端性质：

$$
v'(s)=\frac{6\,\Delta v}{L_i}\,x\,(1-x),\qquad v''(s)=\frac{6\,\Delta v}{L_i^{2}}\,(1-2x)
$$

一阶导在 $x=0$ 与 $x=1$ 处为零，二阶导在两端分别取 $6\Delta v/L_i^{2}$ 与 $-6\Delta v/L_i^{2}$；当 $\Delta v\ne0$ 时，它与相邻常值段的零二阶导之间存在跳变，$\Delta v=0$ 时则退化为常值段。这条曲线因此不是非平凡的回旋线：回旋线的曲率随站位线性变化，其非零曲率变化率不会在两端降为零。头文件 [`track_geometry_segments.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_geometry_segments.h) 写明这是本库有意的选择（"It is deliberately not a clothoid"），模块 [`README.md`](../../../libs/track_geometry/README.md) 进一步规定下游不得按回旋线标准解释这里的缓和段；把这种段用于曲率剖面时，它就是本库的缓和曲线。

没有接缝窗口的相邻段界采用连续剖面假设：前一段的声明终值等于后一段的声明起值。实现比较声明值而不是重新求值多项式，因为 Hermite 混合在浮点算术中到达终值时未必逐位复现。若允许值跳变，由剖面积分得到的航向虽仍连续，但轨道系及其站位导数会在跳变处失去通常的微分意义，因此这种情形不属于本模型。

### 3.2 五次接缝过渡

相邻两段可以另行声明一个 `TrackSeamTransition`：由前一段的序号 `preceding_segment_index` 指认边界 $s_b$（该段的终点站位），窗口全宽为 `window_length_meters`，记作 $w$，窗口 $[s_b-w/2,\ s_b+w/2]$ 以边界为中心。接缝按拓扑而不是按写出的站位指认，因此不存在必须与段长累加和在浮点上精确相等的问题。窗口内剖面改走一条五次多项式，它在窗口两端匹配两侧原段自身多项式的值、一阶导与二阶导。两端数据由原段多项式在窗口端点处求得：左端取前一段在 $s_b-w/2$ 处的 $(f_0,f_0',f_0'')$，右端取后一段在 $s_b+w/2$ 处的 $(f_1,f_1',f_1'')$。以窗口归一化坐标 $\xi=(s-s_b+w/2)/w\in[0,1]$ 表示，五次多项式是

$$
p(\xi)=\sum_{r=0}^{5} d_r\,H_r(\xi),\qquad
\mathbf d=\begin{bmatrix} f_0 & w f_0' & w^{2} f_0'' & f_1 & w f_1' & w^{2} f_1''\end{bmatrix}
$$

其中六个基函数 $H_r$ 的升幂系数就是 [`track_profile_quintic.cc`](../../../libs/track_geometry/src/track_profile_quintic.cc) 中的常量表 `kQuinticBasis`，行依次对应起点的值、一阶导、二阶导，再对应终点的值、一阶导、二阶导：

```text
kQuinticBasis[6][6] = {
    {1.0, 0.0, 0.0, -10.0, 15.0, -6.0},
    {0.0, 1.0, 0.0, -6.0, 8.0, -3.0},
    {0.0, 0.0, 0.5, -1.5, 1.5, -0.5},
    {0.0, 0.0, 0.0, 10.0, -15.0, 6.0},
    {0.0, 0.0, 0.0, -4.0, 7.0, -3.0},
    {0.0, 0.0, 0.0, 0.5, -1.0, 0.5},
};
```

归一化系数 $a_k=\sum_r d_r\,K_{rk}$（$K$ 即上表）再按幂次换回以窗口起点为原点的物理局部坐标：

$$
c_k=\frac{a_k}{w^{k}},\qquad k=0,\dots,5,\qquad p(s)=\sum_{k=0}^{5}c_k\,\left(s-s_b+\tfrac{w}{2}\right)^{k}
$$

这一换算就是 `internal::BuildQuinticHermiteCoefficients` 的全部内容。它是标量剖面与竖向剖面共用的唯一五次边界内核：`TrackScalarProfile` 在 [`track_geometry_segments.cc`](../../../libs/track_geometry/src/track_geometry_segments.cc) 中用它，`TrackVerticalProfile` 在 [`track_vertical_profile.cc`](../../../libs/track_geometry/src/track_vertical_profile.cc) 中以原段的解析 $(g,g',g'')$ 调用同一函数。接缝是声明的几何区间，不是构造后静默施加的平滑；关于某段"自身值"的任何陈述都只对该段落在窗口之外的部分成立。

五次桥的定义要求窗口宽度 $w$ 有限且严格为正，窗口中心对应一个确有左右相邻段的内部边界，一个边界至多配置一个窗口，且半窗 $w/2$ 不超过任一相邻段的长度。各窗口必须互不重叠，端点相接允许。这些条件保证每个窗口的六项边界数据有唯一来源，也保证最终片划分在整个站位域上单值。

### 3.3 三维中心线

平面曲率剖面 $\kappa(s)$ 与竖向剖面 $g(s)$ 共享同一站位轴。航向是曲率的精确积分，中心线的水平分量沿航向积分，高程取竖向剖面积分的相反数：

$$
\psi(s)=\int_{s_{\min}}^{s}\kappa(\sigma)\,d\sigma,\qquad
\begin{bmatrix}x(s)\\ y(s)\end{bmatrix}=\int_{s_{\min}}^{s}\begin{bmatrix}\cos\psi(\sigma)\\ \sin\psi(\sigma)\end{bmatrix}d\sigma,\qquad
z(s)=-\int_{s_{\min}}^{s}g(\sigma)\,d\sigma
$$

实现中 $\psi(s)$ 是 `curvature_.IntegralFromStart(s)`，$z(s)$ 是 `-grade_.IntegralFromStart(s)`，二者都是解析积分；只有水平分量需要求积，且只在曲率不为常数的地方需要（第 4.2 节）。中心线从惯性系原点出发、起始航向为零：构造函数把首节点的 `heading_radians` 置为 `0.0`、位置置为零向量。一阶与二阶站位导数为

$$
\mathbf C'(s)=\begin{bmatrix}\cos\psi\\ \sin\psi\\ -g\end{bmatrix},\qquad
\mathbf C''(s)=\begin{bmatrix}-\kappa\sin\psi\\ \kappa\cos\psi\\ -g'\end{bmatrix},\qquad
\left\lVert\mathbf C'(s)\right\rVert=\sqrt{1+g^{2}}
$$

分别由 `CenterlineDerivativeUnchecked` 与 `CenterlineSecondDerivativeUnchecked` 形成；其中 $g'$ 是 `grade_.FirstDerivativePerMeter`。站位是平面投影里程，所以 $\mathbf C'$ 的水平投影是单位向量，而完整三维导数的模是 $\sqrt{1+g^2}$，纵坡非零处不等于一；$\mathbf C''$ 只在投影的二阶判据中使用，因此保持私有。竖向剖面各段的 $g$、$g'$、$g''$ 与 $\int g$ 的闭式见 [`TRACK_VERTICAL_PROFILE_MODELLING.md`](TRACK_VERTICAL_PROFILE_MODELLING.md) 第 4 节；本篇只用到它们是解析的这一事实。

### 3.4 轨道系：惯性系 I、无侧滚切向系与轨型系 T

惯性系 `I` 由线路定义：原点在线路起点，`+x` 沿起点处站位增加方向，`+y` 指向右侧，`+z` 向下，`DownwardUnitVectorInInertial()` 返回 `(0.0, 0.0, 1.0)`。两个坐标系沿线路移动。无侧滚切向系的三轴为

$$
\mathbf x_0=\frac{\mathbf t}{n},\qquad \mathbf t=\mathbf C'(s),\qquad n=\sqrt{1+g^{2}},\qquad
\mathbf y_0=\begin{bmatrix}-\sin\psi\\ \cos\psi\\ 0\end{bmatrix},\qquad
\mathbf z_0=\mathbf x_0\times\mathbf y_0
$$

对应 `EvaluateTrackFrame` 中的 `roll_free_x`、`roll_free_y`、`roll_free_z`。$\mathbf y_0$ 始终水平并指向轨道右侧；坡度非零时 $\mathbf z_0$ 不是惯性系竖向。轨型系 `T` 是无侧滚系绕自身 $\mathbf x_0$ 轴滚转超高角 $\phi$ 后的系：

$$
R_{IT}=R_{I0}\,R_x(\phi),\qquad
R_x(\phi)=\begin{bmatrix}1&0&0\\ 0&\cos\phi&-\sin\phi\\ 0&\sin\phi&\cos\phi\end{bmatrix},\qquad
\phi=\arcsin\frac{u}{b}
$$

其中 $R_{I0}$ 的三列依次是 $\mathbf x_0,\mathbf y_0,\mathbf z_0$，实现为 `rotation = rotation_roll_free * roll_about_x`；`TrackRollRadians` 返回 `std::asin(superelevation_.Value(definition_station) / superelevation_reference_baselength_meters_)`。$u$ 是超高剖面的值 `SuperelevationMeters`，$b$ 是构造参数 `superelevation_reference_baselength_meters`。

有符号超高的定义：在轨型系横轴 $\mathbf y_T$（$R_{IT}$ 的第二列）上距中心线各 $b/2$ 处取两个抽象参考点 $\mathbf p_{\text{right}}=+\tfrac{b}{2}\mathbf y_T$ 与 $\mathbf p_{\text{left}}=-\tfrac{b}{2}\mathbf y_T$，$u$ 是它们沿无侧滚系竖轴 $\mathbf z_0$ 的有符号分离量：

$$
u=(\mathbf p_{\text{right}}-\mathbf p_{\text{left}})\cdot\mathbf z_0=b\sin\phi,\qquad
(\mathbf p_{\text{right}}-\mathbf p_{\text{left}})\cdot\mathbf e_z=\frac{u}{\sqrt{1+g^{2}}}
$$

第一式由 $\mathbf y_T=\cos\phi\,\mathbf y_0+\sin\phi\,\mathbf z_0$ 直接得到，第二式（$\mathbf e_z$ 是惯性系竖向单位向量）说明只有纵坡为零时 $u$ 才等于惯性系竖向高差。正超高表示右侧参考点更低，对应绕 $\mathbf x_0$ 的正滚转：它把 `+y` 转向 `+z`，而 `+z` 向下。这两个参考点不声明实际钢轨或轨型参考点恰好位于其处；$b$ 不是名义轨距，也不是轮轨型面定位所用的参考点间距。ORVD 只实现中心线基准：中心线位置不随超高改变，内轨、外轨基准下的中心线位移不属于当前模型。

滚转定义的适用域是剖面上每一点满足 $\lvert u\rvert<b$，且不采用裁剪：达到四分之一圈不再符合线路几何的含义，$\phi'$ 也在该处奇异，因此可容许区间是开的而不是闭的。实现由 `TrackScalarProfile::MaximumAbsoluteValue` 在每个片的两端与其导数多项式的实根处求极值（第 4.1 节），不是以离散抽样近似这一条件，因而五次接缝的内部极值也计入。

`TrackFrameKinematics` 把位姿与它的一阶站位导数放在一次求值中返回：位姿 `TrackFramePose` 携带 $R_{IT}$ 与中心线位置；`centerline_derivative_in_inertial_meters_per_meter()` 返回 $\mathbf t=\mathbf C'(s)$，模长 $\sqrt{1+g^2}$ 而不是一；`track_frame_rotation_rate_in_inertial_radians_per_meter()` 返回以惯性系表达的 $\boldsymbol\omega_{IT}$，满足

$$
\frac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})\,R_{IT}
$$

该恒等式定义了返回旋转率与姿态导数之间的关系；$\boldsymbol\omega_{IT}$ 的构造见第 4.3 节。位姿与导数由同一次求值形成，使二者采用一致的几何状态。

### 3.5 定义区间与三维切线延长

线路几何定义在有限站位区间 $[s_{\min},s_{\max}]$ 上，端点是曲率剖面的起终站位（构造期三条剖面已被规范到共同终点，第 4.2 节）。`ClassifyTrackStation` 对 $s<s_{\min}$ 返回 `TrackStationRegion::kBeforeDefinedInterval`，对 $s>s_{\max}$ 返回 `kAfterDefinedInterval`，其余返回 `kWithinDefinedInterval`；两个端点属于区间内。对区间外的任意有限站位，`TrackGeometry` 从最近的定义边界 $s_b$ 沿三维切线作直线延长：

$$
\mathbf C(s)=\mathbf C(s_b)+(s-s_b)\,\mathbf C'(s_b),\qquad
\kappa(s)=0,\qquad \mathbf C''(s)=\mathbf 0,\qquad
\psi(s)=\psi(s_b),\quad g(s)=g(s_b),\quad u(s)=u(s_b),\quad g'(s)=u'(s)=0
$$

实现中 `CenterlinePositionUnchecked` 对域外站位组合边界位置与边界导数；航向、超高、纵坡与滚转角取边界值，曲率、中心线二阶导数以及纵坡和超高的变化率取零。这里的切线延长属于 `TrackGeometry` 的几何定义，而底层标量剖面仍只定义在自身有限区间内。它是一种明确的延拓选择，不等价于把原剖面的最后一个解析片继续外推。

`support_start_track_station_meters()` 表示剖面支集的实现近似：它取第一个多项式系数不全为零的片的起点，整条剖面恒为零时为空。因为它检查解析片而不是样本，当第一个非零片是接缝窗口时，返回窗口起点而不是原段边界。竖向剖面也使用同一思想：恒坡原段按起始坡度判零，接缝片按全部系数判零。

### 3.6 站位投影的目标函数

空间点 $\mathbf p$ 到中心线的投影以点到中心线距离平方之半为目标函数，其一阶、二阶站位导数为

$$
f(s)=\tfrac12\left\lVert\mathbf p-\mathbf C(s)\right\rVert^{2},\qquad
f'(s)=-(\mathbf p-\mathbf C)\cdot\mathbf C',\qquad
f''(s)=\left\lVert\mathbf C'\right\rVert^{2}-(\mathbf p-\mathbf C)\cdot\mathbf C''
$$

对应 `EvaluateObjectiveDerivatives` 中的 `gradient` 与 `hessian`（README 中记作 objective、objective′、objective″）。合格站位须使一阶导落入尺度化的残差门并且二阶导严格为正：

$$
\lvert f'(s)\rvert\le\beta(s),\qquad \beta(s)=\tau\left(\lVert\mathbf p-\mathbf C\rVert\,\lVert\mathbf C'\rVert+\ell_0\right),\qquad \ell_0=1\,\mathrm m,\qquad f''(s)>0
$$

$\tau$ 是 `kObjectiveGradientRelativeTolerance`，值为 `1.0e-10`。残差门随距离缩放，因为 $f'$ 的量纲是长度；源码中的数值常数 `1.0` 在 SI 坐标中对应 $\ell_0=1\,\mathrm m$，给零距离处留下同量纲的绝对地板。$f''>0$ 排除距离的驻点是极大值的情形，例如点位于圆曲线曲率中心之外时。该模型有意只描述局部分支：调用方给出能标识当前分支的站位种子 $s_{\text{seed}}$，本层不做全线最近点搜索、不扫描节点、不改选远处的根；原因与算法见第 4.4 节。

## 4. 算法

### 4.1 标量剖面的构造与求值

剖面构造可分为五步。第一步逐段累加站位，并按第 3.1 节形成各原始段的局部多项式。第二步由内部段界与窗口宽度形成全部接缝区间，并共同检查第 3.2 节的几何前提。第三步要求所有未被窗口覆盖的段界满足声明值连续。第四步把窗口按起点排序，再沿站位游标切出片序列：窗口之前的原段部分、窗口本身、窗口之后的原段部分依次成片。原段被窗口切开后仍保留原段起点作为多项式原点，避免把原系数围绕切点重新展开而引入额外舍入；接缝片的原点是窗口起点。第五步计算每个片起点的累计积分 $I_i$、记录断点表（各片起点加整条剖面终点），并求支持起点。

每个片保存至多六个升幂系数与系数个数（常值一项、Hermite 混合四项、接缝六项），求值用 Horner 格式在局部坐标 $\hat s=s-o$ 上进行，`Value`、`FirstDerivativePerMeter`、`SecondDerivativePerMeterSquared` 分别对应 `EvaluatePolynomial`、`EvaluateFirstDerivative`、`EvaluateSecondDerivative`。积分是解析的：每个片的原函数取局部坐标零点处为零，

$$
F(\ell)=\sum_{k}\frac{c_k}{k+1}\,\ell^{\,k+1},\qquad
\int_{s_{\min}}^{s}v(\sigma)\,d\sigma=I_i+F(s-o_i)-F\!\left(s_{i,\text{start}}-o_i\right)
$$

其中 $i$ 是 $s$ 所在的片。`IntegralFromStart` 实现该式；片查找 `PieceIndexAt` 对片起点二分，取起点不晚于 $s$ 的最后一个片，因此内部片界取右侧片、剖面终点取最后一片。由此得到的导数在无接缝段界上是右侧单侧值：例如常值段与混合段交界处，`SecondDerivativePerMeterSquared` 返回右片的二阶导。底层剖面的数学域严格限于 $[s_{\min},s_{\max}]$，裁剪与切线延长只发生在更高层的 `TrackGeometry` 中。

`LocalPolynomialDegree` 去掉尾部为零的系数后报告局部多项式次数，次数为零时 `TrackGeometry` 走闭式常曲率路径；起终值相等的 Hermite 混合与两侧相同常值之间的接缝因此都被识别为常值。`MaximumAbsoluteValue` 对每个片在区间两端以及导数多项式的全部实根处求值，从而得到解析片上的绝对值极值。导数多项式次数至多四；`FindPolynomialRootsInClosedInterval` 递归求低一阶导数的根，把区间切成单调段，并以二分收口变号根。端点与分割点的尺度化判零保留偶重根，邻近根再按尺度化距离合并。这一步把 $|u|<b$ 等全区间条件化成有限个候选点上的判定。

三条剖面的终点只有末位舍入差时，`ShortenDomainEndTo` 把较长剖面的末片和断点表规范到共同的最短终点；它不改变任何更早的片。

### 4.2 中心线节点表与横向位移

节点间距 $\Delta s$ 与超高参考基长 $b$ 取有限正值，曲率、超高和竖向剖面共享同一站位域。起点要求相同；终点允许因不同分段累加顺序产生末位舍入差。实现把终点除以尺度 $\max\{1,\lvert s_{\min}\rvert,\lvert e_1\rvert,\lvert e_2\rvert\}$ 后比较，容差随两条剖面的断点数和机器精度增长；随后以三者中最短的终点作为共同终点。这个规范化只消除浮点累加路径的差别，不代表允许物理长度不同的剖面拼接。

节点表建立在曲率断点与竖向断点的有序并集 $\{B_j\}$ 上（排序去重）。断点区间 $[B_j,B_{j+1}]$ 长度 $\ell_j$ 被分成 $n_j=\max\{1,\lceil \ell_j/\Delta s\rceil\}$ 个面板，节点位于 $B_j+\ell_j\,q/n_j$，$q=0,\dots,n_j-1$，最后再追加终点节点。一个节点区间因此既不跨越水平积分公式的切换，也不跨越中心线导数公式的切换；每个区间在其中点处查询 `LocalPolynomialDegree` 决定是否常曲率，并记录该处曲率值。

每个节点存航向 $\psi_n$（`curvature_.IntegralFromStart` 的精确积分）与中心线位置。从节点到其区间内任一站位的水平位移由 `HorizontalDisplacementFromNode` 给出。曲率常数时用圆弧弦的半角形式：

$$
h=\tfrac12\,\kappa\,L,\qquad
\Delta\mathbf C_{xy}=L\,\frac{\sin h}{h}\begin{bmatrix}\cos(\psi_n+h)\\ \sin(\psi_n+h)\end{bmatrix},\qquad
\left.\frac{\sin h}{h}\right|_{h=0}:=1
$$

写成正弦之差除以曲率会先相减两个几乎相等的数再乘以半径，在缓曲线短面板上损失大部分有效位；半角形式没有相消，且在 $h=0$ 时自然退化为直线，无需单独分支（`chord_ratio` 在 `half_turn == 0.0` 时取 `1.0`）。曲率不为常数时，航向是多项式，其正弦与余弦没有初等原函数，改用固定的八点 Gauss–Legendre 规则：

$$
\Delta\mathbf C_{xy}=\frac{L}{2}\sum_{q=1}^{8}w_q\begin{bmatrix}\cos\psi\!\left(m+\tfrac{L}{2}\xi_q\right)\\ \sin\psi\!\left(m+\tfrac{L}{2}\xi_q\right)\end{bmatrix},\qquad m=s_n+\tfrac{L}{2}
$$

横坐标与权是源码中的常量表：

```text
kQuadraturePointCount = 8
kQuadratureAbscissae = {-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                         0.1834346424956498,  0.5255324099163290,  0.7966664774136267,  0.9602898564975363}
kQuadratureWeights   = { 0.1012285362903763,  0.2223810344533745,  0.3137066458778873,  0.3626837833783620,
                         0.3626837833783620,  0.3137066458778873,  0.2223810344533745,  0.1012285362903763}
```

固定阶与受 $\Delta s$ 限制的面板长度使水平积分误差由曲率变化率、求积阶数和面板尺度共同决定，而不是由自适应停止条件决定。代码没有为任意正 $\Delta s$ 给出统一的绝对误差上界，因此 $\Delta s$ 是数值近似的一部分，不是纯粹的存储参数。

节点位置是沿节点的连加，普通浮点求和的累计误差会随节点数增长，使更细的间距在求积变好的同时可能让绝对位置变差。构造函数用 Neumaier 补偿显著抑制而非数学上消除这种累计误差：对每个水平分量维持和 $S$ 与补偿 $c$，

$$
t=S+a,\qquad
c\leftarrow c+\begin{cases}(S-t)+a,&\lvert S\rvert\ge\lvert a\rvert\\ (a-t)+S,&\text{otherwise}\end{cases},\qquad
S\leftarrow t,\qquad x_n=S+c
$$

节点的 $z$ 分量直接取 `-grade_.IntegralFromStart`，不参与连加。非节点站位的位置是 `NodeIndexAtOrBefore` 二分找到的节点位置加上该节点到目标站位的一次 `HorizontalDisplacementFromNode`，即最多补积一个面板的剩余部分；落在最后一个节点上的站位归入以它结束的区间。

### 4.3 轨道系运动学

`EvaluateTrackFrame` 一次求出位姿与站位导数。先取 $\psi$、$\kappa$、$g$、$g'$、$u$、$u'$（区间外按第 3.5 节取零或边界值），再按第 3.4 节形成三轴与旋转。三轴的站位导数是

$$
\mathbf t'=\begin{bmatrix}-\kappa\sin\psi\\ \kappa\cos\psi\\ -g'\end{bmatrix},\qquad
n'=\frac{g\,g'}{n},\qquad
\mathbf x_0'=\frac{\mathbf t'}{n}-\mathbf x_0\,\frac{n'}{n},\qquad
\mathbf y_0'=\begin{bmatrix}-\kappa\cos\psi\\ -\kappa\sin\psi\\ 0\end{bmatrix},\qquad
\mathbf z_0'=\mathbf x_0'\times\mathbf y_0+\mathbf x_0\times\mathbf y_0'
$$

对应 `tangent_rate`、`slope_norm_rate`、`roll_free_x_rate`、`roll_free_y_rate`、`roll_free_z_rate`。正交三元组的角速度等于列与列导数叉积之和的一半，这样写避免手写三个特例；再叠加绕 $\mathbf x_0$ 的滚转率：

$$
\boldsymbol\omega_0=\tfrac12\left(\mathbf x_0\times\mathbf x_0'+\mathbf y_0\times\mathbf y_0'+\mathbf z_0\times\mathbf z_0'\right),\qquad
\phi'=\frac{u'/b}{\sqrt{1-(u/b)^{2}}},\qquad
\boldsymbol\omega_{IT}=\boldsymbol\omega_0+\phi'\,\mathbf x_0
$$

$\phi'$ 的分母正是开区间 $\lvert u\rvert<b$ 保证不为零的量。`TrackFrameKinematics` 由 $R_{IT}$、`CenterlinePositionUnchecked(s)`、$\mathbf t$ 与 $\boldsymbol\omega_{IT}$ 组成，从而在同一站位上同时给出姿态、平移切向与旋转率。

### 4.4 局部分支投影

`ProjectPointOntoSeededBranch(point, seed)` 从给定种子开始，最多作两次 Newton 校正。记第 $k$ 次站位为 $s_k$，则

$$
s_{k+1}=s_k-\frac{f'(s_k)}{f''(s_k)},\qquad k=0,1,
$$

并在 $s_0,s_1,s_2$ 每一点先检查第 3.6 节的尺度化驻点条件。$f''>0$ 既是局部极小值的二阶条件，也是执行下一次 Newton 校正的前提；在非凸点用同一公式会朝距离极大值推进，因而不属于本算法。投影只使用 $\mathbf C$、$\mathbf C'$、$\mathbf C''$，三者在定义区间外按切线延长求值，所以种子与根可以位于区间外，也可以跨越定义边界。

不做全线搜索的原因是分支身份而不只是代价：同一空间点在一条线路上可以有多个距离驻点，例如圆曲线附近或先升后降的竖曲线下方。全局最近点会在不同根之间改选，而局部分支投影把 $s_{\text{seed}}$ 视为当前几何分支的组成部分。线路几何本身不保存时间历史；种子由更高层随动力学状态推进。

该算法不设围绕种子的有限搜索窗，也不依赖节点网格括住根；否则根是否落在窗壁或是否被节点栅格跨过会人为改变分支连续性。其代价是局部性前提更强：调用方必须让当前状态处于某个满足 $f''>0$ 的正则根附近，并使种子落在至多两次校正可达的 Newton 吸引域内。该吸引域的大小依赖曲率、纵坡及点到中心线的距离，代码没有给出统一半径。

### 4.5 数值近似与非光滑点

实现混合了解析计算与受控数值近似。标量剖面的值、导数、积分与航向为片内解析计算；常曲率水平位移采用半角弦形式以避免相消；变曲率面板使用固定八点 Gauss–Legendre 求积；节点位置采用 Neumaier 补偿求和。三剖面的共同终点只吸收可由浮点累加解释的末位差，$\lvert u\rvert<b$ 则通过解析片极值判定。局部投影用带绝对地板的尺度化残差和严格的 $f''>0$ 条件，并把迭代次数固定为至多两次。因而中心线水平位置与投影根是数值近似，剖面多项式及其解析积分不是。

非光滑点如下。Hermite 混合段两端：值与一阶导连续，二阶导通常跳变；退化的 $\Delta v=0$ 段或端部二阶导恰好匹配的相邻段是例外。无接缝的段界：值连续，一阶导在常值与混合的交界处连续为零，二阶导可跳变；两常值段等值交界处完全光滑。接缝窗口两端：值、一阶导、二阶导连续，三阶导可跳变。定义边界：$\mathbf C$ 与 $\mathbf C'$ 连续、$R_{IT}$ 连续，但 $\kappa$、$g'$、$u'$ 在边界外侧取零，因此 $\mathbf C''$ 与 $\boldsymbol\omega_{IT}$ 在边界处可跳变。片查找在片界处取右侧片、在终点取最后一片，所以导数查询在这些站位返回单侧值。$\phi=\arcsin(u/b)$ 的导数在 $\lvert u\rvert\to b$ 时无界，因而模型限定在 $\lvert u\rvert<b$。

复杂度：剖面的一次求值或积分是对片起点的二分加 Horner，即 $O(\log P)$，$P$ 为片数；一次中心线位置是对节点的二分 $O(\log N)$ 加一次闭式或八次航向求值；一次 `EvaluateTrackFrame` 是常数个剖面查询加一次位置；一次投影至多三次目标导数求值。构造期为 $O(N)$ 个面板位移加 $O(P)$ 次极值搜索。

## 5. 实现映射

下表只定位承载核心数学关系的实现，不作为接口或配置参考。

| 理论对象或算法 | 主要实现 | 源文件 |
|---|---|---|
| 常值与 Hermite 三次标量剖面、解析导数与积分 | `TrackScalarProfile` | [`track_geometry_segments.cc`](../../../libs/track_geometry/src/track_geometry_segments.cc) |
| 五次 $C^2$ 接缝 | `internal::BuildQuinticHermiteCoefficients` | [`track_profile_quintic.cc`](../../../libs/track_geometry/src/track_profile_quintic.cc) |
| 片内极值与导数多项式根 | `MaximumAbsoluteValue`、`FindPolynomialRootsInClosedInterval` | [`track_geometry_segments.cc`](../../../libs/track_geometry/src/track_geometry_segments.cc) |
| 航向、中心线节点与水平求积 | `TrackGeometry`、`HorizontalDisplacementFromNode` | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| 轨道姿态与站位旋转率 | `EvaluateTrackFrame` | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| 有限域外的三维切线延长 | `CenterlinePositionUnchecked` 及各剖面查询 | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| 距离目标的一、二阶导与 Newton 投影 | `EvaluateObjectiveDerivatives`、`ProjectPointOntoSeededBranch` | [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| 竖向剖面的解析量 | `TrackVerticalProfile` | [`track_vertical_profile.cc`](../../../libs/track_geometry/src/track_vertical_profile.cc) |

## 6. 理论假设与适用范围

- 站位 $s$ 是中心线在水平面的投影里程，不是三维弧长。因此 $\|\mathbf C'(s)\|=\sqrt{1+g^2}$；只有 $g=0$ 时其值为一。
- 平面曲率和超高只由常值片、Hermite 三次混合片与可选的五次 $C^2$ 接缝组成。对曲率从 $\kappa_0$ 过渡到 $\kappa_1$，Hermite 律
  $$
  \kappa(s)=\kappa_0+(\kappa_1-\kappa_0)(3x^2-2x^3)
  $$
  正是 Bloss 缓和曲线的曲率律，而不是回旋线。当前模型不包含线性曲率的回旋线、正弦型缓和曲线或由采样点定义的任意线形；这些线形不能由现有段类型无损表达。
- 超高采用中心线滚转模型，$u=b\sin\phi$ 是沿无侧滚系竖轴的有符号分离量。模型不定义以内轨或外轨为固定基准时中心线应如何平移，也不把 $b$ 等同于名义轨距。
- 水平中心线对常曲率面板解析积分，对变曲率面板作固定八点 Gauss–Legendre 求积。$\Delta s$ 决定面板尺度；当前推导没有给出适用于任意曲率片和任意 $\Delta s$ 的统一绝对误差界。
- 有限定义区间之外采用边界三维切线延长。这是人为选定的几何延拓，并非由区间内曲率、纵坡或超高的解析式唯一推出。
- 空间点投影是由种子标识的局部分支问题，不是全线最近点问题。至多两次 Newton 校正假设种子已处于一个满足 $f''>0$ 的正则极小根的局部吸引域；该吸引域尚无统一解析半径。
- 光滑阶由片型决定：Hermite 混合端点通常为 $C^1$、五次接缝端点为 $C^2$；定义边界的切线延长保持 $\mathbf C$ 与 $\mathbf C'$ 连续，但 $\mathbf C''$ 和轨道系旋转率可以跳变。
