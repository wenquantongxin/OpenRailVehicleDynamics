[English](TRACK_VERTICAL_PROFILE_MODELLING.en.md)

# 轨道竖向剖面及其三维耦合

本篇说明 ORVD 如何用水平投影站位上的恒坡段、抛物线竖曲线和圆弧竖曲线构造竖向剖面，如何用显式五次接缝连接相邻段，以及竖向剖面如何与平面曲率和超高共同形成三维中心线与轨道坐标系。公式给出数学模型及其代码实现所采用的解析关系。

## 1. 范围与记号

轨道惯性系记为 `I`，其 `+x` 沿线路起点处的站位增加方向，`+y` 指向面向站位增加方向时的右侧，`+z` 向下。站位 $s$ 是中心线在水平面内的投影弧长，而不是三维中心线弧长。完整坐标约定见[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)，平面曲率、中心线积分和轨道系的一般算法见[线路几何与轨道坐标系](TRACK_GEOMETRY_AND_FRAMES.md)。

| 记号 | 含义 |
|---|---|
| $[s_{\min},s_{\max}]$ | 线路几何的有限定义区间 |
| $\hat s$ | 一段内部从零开始的局部水平投影站位 |
| $g(s)$ | 中心线向上坡度；沿站位增加方向上坡为正 |
| $I_g(s)$ | 从剖面起点累计的坡度积分 $\int_{s_{\min}}^s g(\sigma)\,d\sigma$ |
| $\theta(s)=\arctan g(s)$ | 向上坡度倾角，取 $(-\pi/2,\pi/2)$ 内的主值 |
| $\psi(s)$、$\kappa(s)$ | 水平航向角和平面曲率，$d\psi/ds=\kappa$ |
| $\mathbf C(s)$ | 在 `I` 中表达的三维中心线位置 |
| $\ell$ | 三维中心线弧长 |
| $\kappa_v$ | “水平投影站位—向上高程”平面内竖剖面的有符号曲率 |
| $u(s)$、$b$、$\phi(s)$ | 有符号超高、超高参考基长和轨道滚转角 |

轨道竖向不平顺是叠加在理想线路上的局部激励，不属于本文所称的竖向剖面；竖曲线也只指坡度连续变化的过渡段，不是所有非零坡度区段的统称。

## 2. 水平投影站位与高程

由于 `+z` 向下而 $g$ 以上坡为正，中心线高程分量满足

$$
\frac{dz}{ds}=-g(s),\qquad z(s)-z(s_{\min})=-I_g(s).
$$

三维中心线导数的水平投影是单位向量：

$$
\mathbf C'(s)=
\begin{bmatrix}
\cos\psi(s)\\
\sin\psi(s)\\
-g(s)
\end{bmatrix},
\qquad
\left\lVert\mathbf C'(s)\right\rVert=\sqrt{1+g(s)^2}.
$$

因此水平投影站位与三维弧长之间满足

$$
\frac{d\ell}{ds}=\sqrt{1+g^2},\qquad \dot\ell=\sqrt{1+g^2}\,\dot s.
$$

坡度非零时，$s$、$\ell$ 及其时间变化率不能互换。竖向线形的所有段长和接缝宽度均沿 $s$ 度量。

## 3. 竖向剖面的数学对象

一条竖向剖面由首尾相接的有限段序列组成。每个原始段解析给出 $g$、$g'$、$g''$ 以及从该段起点累计的 $\int g\,ds$；整条剖面的 $I_g$ 由前段积分和当前段局部积分相加得到。ORVD 当前实现三种原始段：

| 物理线形 | 参数 | 段内坡度性质 |
|---|---|---|
| 恒坡段 | $L>0$、$g_0$ | $g=g_0$ |
| 抛物线竖曲线（PL2） | $L>0$、$g_1\ne g_2$ | $g$ 对 $\hat s$ 线性 |
| 圆弧竖曲线（CIR） | $L>0$、$g_1\ne g_2$ | $\sin\theta$ 对 $\hat s$ 线性，竖剖面曲率恒定 |

端点坡度相等的 PL2 或 CIR 与恒坡段描述同一退化极限，因此模型以恒坡段作为唯一表达。没有接缝窗口覆盖的相邻段界要求坡度值连续；高阶导数是否连续由两侧段型决定。

剖面本身只定义在 $[s_{\min},s_{\max}]$ 内。第 8 节所述域外行为属于组合后的线路几何，而不是把最后一个竖向解析段继续外推。

## 4. 竖向线形单元

### 4.1 恒坡段

长度为 $L$、坡度为 $g_0$ 的恒坡段在 $0\le\hat s\le L$ 上满足

$$
g(\hat s)=g_0,\qquad g'(\hat s)=g''(\hat s)=0,
$$

$$
\int_0^{\hat s}g(u)\,du=g_0\hat s,
\qquad
z(\hat s)-z(0)=-g_0\hat s.
$$

零坡度是恒坡段的特例；恒坡段本身不是竖曲线。

### 4.2 抛物线竖曲线（PL2）

PL2 连接坡度 $g_1$ 与 $g_2$，坡度沿水平投影站位线性变化：

$$
g(\hat s)=g_1+\frac{g_2-g_1}{L}\hat s,
\qquad 0\le\hat s\le L.
$$

令 $q=(g_2-g_1)/L$，则

$$
g'=q,\qquad g''=0,
$$

$$
\int_0^{\hat s}g(u)\,du=g_1\hat s+\frac{q}{2}\hat s^2,
\qquad
z(\hat s)-z(0)=-g_1\hat s-\frac{q}{2}\hat s^2.
$$

“抛物线”指高程关于水平投影站位为二次函数。它不表示坡度倾角 $\theta$ 对站位线性，因为

$$
\frac{d\theta}{ds}=\frac{q}{1+g^2},
\qquad
\frac{d^2\theta}{ds^2}=-\frac{2gq^2}{(1+g^2)^2}.
$$

PL2 保证段内高程和坡度平滑，但其 $g'$ 在与恒坡段直接相接时通常发生跳变；需要更高接缝光滑性时使用第 5 节的显式接缝。

### 4.3 圆弧竖曲线（CIR）

CIR 是“水平投影站位—向上高程”平面内的严格圆弧。以 $L$、$g_1$、$g_2$ 参数化，并定义

$$
\theta_i=\arctan g_i,
\qquad
\sin\theta_i=\frac{g_i}{\operatorname{hypot}(1,g_i)},
\qquad
\cos\theta_i=\frac{1}{\operatorname{hypot}(1,g_i)}.
$$

有符号逆半径和半径为

$$
\rho=\frac{1}{R}=\frac{\sin\theta_2-\sin\theta_1}{L},
\qquad
R=\frac{L}{\sin\theta_2-\sin\theta_1}.
$$

在段内令

$$
q(\hat s)=\sin\theta_1+\rho\hat s.
$$

由于有限坡度对应 $\cos\theta>0$，坡度由

$$
\sin\theta(\hat s)=q(\hat s),
\qquad
\cos\theta(\hat s)=\sqrt{1-q(\hat s)^2},
\qquad
g(\hat s)=\frac{q(\hat s)}{\sqrt{1-q(\hat s)^2}}
$$

得到。高程可以写为

$$
z(\hat s)-z(0)=R\left[\cos\theta(\hat s)-\cos\theta_1\right],
$$

而代码使用与其数学等价、避免大半径乘以接近余弦之差的坡度积分形式：

$$
\int_0^{\hat s}g(u)\,du
=\hat s\,
\frac{q(\hat s)+\sin\theta_1}
{\sqrt{1-q(\hat s)^2}+\cos\theta_1},
\qquad
z(\hat s)-z(0)=-\int_0^{\hat s}g(u)\,du.
$$

段内导数为

$$
\frac{d\theta}{ds}=\frac{\rho}{\cos\theta},
\qquad
\frac{d^2\theta}{ds^2}=\frac{\rho^2\sin\theta}{\cos^3\theta},
$$

$$
g'=\frac{\rho}{\cos^3\theta},
\qquad
g''=\frac{3\rho^2\sin\theta}{\cos^5\theta}.
$$

CIR 的坡度不是线性函数，倾角也不是线性函数；线性的是 $\sin\theta$。$\rho$ 的符号区分两种竖向弯曲方向，不需要另设上凹或下凹标志。

### 4.4 倾角率与竖剖面曲率

对任意可微坡度，

$$
\frac{d\theta}{ds}=\frac{g'}{1+g^2}.
$$

令 $\ell_v$ 为参数曲线 $(s,h(s))$ 的弧长，其中 $dh/ds=g$，则 $d\ell_v/ds=\sqrt{1+g^2}$。它与三维中心线的 $d\ell/ds$ 数值相同，但两者的曲率具有不同的几何对象。竖剖面的有符号曲率为

$$
\kappa_v=\frac{d\theta}{d\ell_v}=\frac{g'}{(1+g^2)^{3/2}}.
$$

对 CIR，代入 $g'=\rho/\cos^3\theta$ 和 $1+g^2=1/\cos^2\theta$ 得

$$
\kappa_v=\rho=\frac{1}{R},
$$

所以 CIR 在竖剖面平面内曲率恒定，尽管 $d\theta/ds$ 随坡度改变。

## 5. 分段边界与五次接缝

### 5.1 未设接缝的边界

未设接缝时，相邻原始段在共同边界 $s_b$ 具有相同坡度值。于是 $z$ 和 $z'=-g$ 连续，中心线切向连续；$g'$ 或 $g''$ 仍可跳变，因此中心线二阶导数或更高导数可以具有单侧极限而没有共同的双侧值。

这类边界不附加隐式平滑。剖面在内部片界采用右侧片的导数，在完整剖面终点采用左侧片的导数；这些返回值是明确的单侧量。

### 5.2 五次坡度桥

显式接缝窗口以原始段边界 $s_b$ 为中心，全宽为 $w>0$，端点为 $s_L=s_b-w/2$ 与 $s_R=s_b+w/2$。窗口左端的数据取自左侧原始段，右端的数据取自右侧原始段：

$$
\mathbf d=
\begin{bmatrix}
g_L & w g'_L & w^2 g''_L & g_R & w g'_R & w^2 g''_R
\end{bmatrix},
\qquad
\xi=\frac{s-s_L}{w}\in[0,1].
$$

窗口内的坡度由唯一的五次 Hermite 多项式给出：

$$
g_{\mathrm{seam}}(s)=\sum_{r=0}^{5}d_r H_r(\xi),
$$

$$
\begin{aligned}
H_0(\xi)&=1-10\xi^3+15\xi^4-6\xi^5,\\
H_1(\xi)&=\xi-6\xi^3+8\xi^4-3\xi^5,\\
H_2(\xi)&=\tfrac12\xi^2-\tfrac32\xi^3+\tfrac32\xi^4-\tfrac12\xi^5,\\
H_3(\xi)&=10\xi^3-15\xi^4+6\xi^5,\\
H_4(\xi)&=-4\xi^3+7\xi^4-3\xi^5,\\
H_5(\xi)&=\tfrac12\xi^3-\xi^4+\tfrac12\xi^5.
\end{aligned}
$$

这些基函数使接缝在 $s_L$ 与 $s_R$ 分别匹配两侧原始公式的 $(g,g',g'')$。窗口内部完全替代原始坡度；原始边界 $s_b$ 只是拓扑中心，不是接缝多项式的公式切换点。

### 5.3 接缝积分与光滑阶

接缝对累计坡度积分的贡献同样解析：

$$
\int_{s_L}^{s}g_{\mathrm{seam}}(\sigma)\,d\sigma
=w\sum_{r=0}^{5}d_r\int_0^{\xi}H_r(\eta)\,d\eta.
$$

因此高程使用接缝替换后的面积，而不是继续累计被覆盖的原始段。接缝坡度在两个窗口端点达到 $C^2$；相应的高程由 $z'=-g$ 得到，在这些端点具有连续到三阶的导数，四阶导数通常可以跳变。

一个接缝窗口位于确有左右相邻段的内部边界，半窗不越过任一相邻原始段，且不同接缝窗口的内部互不重叠。这些几何条件保证每个站位只由一个原始公式或一个接缝公式定义。

## 6. 与平面线形的三维耦合

### 6.1 中心线积分

平面曲率 $\kappa(s)$ 与竖向坡度 $g(s)$ 共享同一个水平投影站位。航向、水平位置和竖向位置分别为

$$
\psi(s)=\int_{s_{\min}}^s\kappa(\sigma)\,d\sigma,
$$

$$
\begin{bmatrix}x(s)\\y(s)\end{bmatrix}
=\int_{s_{\min}}^s
\begin{bmatrix}\cos\psi(\sigma)\\\sin\psi(\sigma)\end{bmatrix}d\sigma,
\qquad
z(s)=-I_g(s),
$$

其中实现采用 $\mathbf C(s_{\min})=\mathbf 0$ 和 $\psi(s_{\min})=0$。竖向剖面不会被附加在固定世界 `x` 方向的一条二维曲线上；它与不断改变航向的水平中心线通过共同站位组成同一条空间曲线。

二阶站位导数为

$$
\mathbf C''(s)=
\begin{bmatrix}
-\kappa\sin\psi\\
\kappa\cos\psi\\
-g'
\end{bmatrix}.
$$

因此竖向原始段界和接缝边界构成二阶中心线几何的分段位置；接缝窗口覆盖的原始边界仍保留在线路拓扑中，但真正的公式切换发生在接缝窗口两端。

### 6.2 平面曲率、竖剖面曲率与空间曲率

令三维单位切向为 $\mathbf T$，并使用第 7 节的无侧滚轴 $\mathbf y_0$、$\mathbf z_0$，则

$$
\mathbf T=\frac{\mathbf C'}{\sqrt{1+g^2}},
$$

$$
\frac{d\mathbf T}{d\ell}
=\kappa\cos^2\theta\,\mathbf y_0-\kappa_v\,\mathbf z_0,
$$

$$
\left\lVert\frac{d\mathbf T}{d\ell}\right\rVert
=\sqrt{\kappa^2\cos^4\theta+\kappa_v^2}.
$$

平面曲率、竖剖面曲率和三维中心线曲率因而是三个不同的量。超高绕中心线滚转坐标轴，不改变中心线本身或上述空间曲率。

### 6.3 解析竖向积分与水平求积

竖向位置始终由 `TrackVerticalProfile` 的解析 $I_g$ 形成。水平位置只依赖 $\kappa$：常曲率区间使用圆弧弦的闭式，变曲率区间使用分面板的固定阶 Gauss–Legendre 求积。中心线节点位于平面曲率断点与竖向原始段界、接缝边界的有序并集上，使每个节点区间不跨越 $\mathbf C''$ 分片位置。超高不改变中心线，所以超高段界不单独产生中心线积分节点。

这一区分意味着竖向剖面的解析精度不由水平积分面板替代；反过来，PL2 或 CIR 的闭式高程也不会消除变曲率水平积分中的数值近似。

## 7. 与超高和轨道系的耦合

### 7.1 无侧滚轨道系

令 $n=\sqrt{1+g^2}$、$\mathbf t=\mathbf C'$，无侧滚轨道系的三轴为

$$
\mathbf x_0=\frac{\mathbf t}{n},
\qquad
\mathbf y_0=
\begin{bmatrix}
-\sin\psi\\
\cos\psi\\
0
\end{bmatrix},
\qquad
\mathbf z_0=\mathbf x_0\times\mathbf y_0.
$$

$\mathbf y_0$ 始终水平并指向线路右侧；坡度非零时，$\mathbf z_0$ 不等于惯性系向下竖轴。

### 7.2 超高滚转

ORVD 把有符号超高 $u$ 定义为超高参考基线两端参考点沿 $\mathbf z_0$ 的分离量，并使用

$$
\phi=\arcsin\left(\frac{u}{b}\right),
\qquad |u|<b.
$$

最终轨型系 `T` 由无侧滚系绕自身纵轴滚转得到：

$$
R_{IT}=R_{I0}R_x(\phi),
\qquad
R_x(\phi)=
\begin{bmatrix}
1&0&0\\
0&\cos\phi&-\sin\phi\\
0&\sin\phi&\cos\phi
\end{bmatrix}.
$$

正超高使右侧参考点更低。若基线两端在轨型系横轴上相距 $b$，它们沿惯性系向下竖轴的分离量为

$$
\Delta z_I=b\sin\phi\,(\mathbf z_0\cdot\mathbf e_z)
=\frac{u}{\sqrt{1+g^2}}.
$$

所以坡度非零时，输入的 $u$ 不等于两点在惯性系 `z` 方向上的高差。

### 7.3 轨道系站位旋转率

轨道系的站位旋转率由与位姿相同的 $\kappa$、$g$、$g'$、$u$ 和 $u'$ 构造。无侧滚轴的导数为

$$
\mathbf t'=\mathbf C'',
\qquad
n'=\frac{gg'}{n},
\qquad
\mathbf x_0'=\frac{\mathbf t'}{n}-\mathbf x_0\frac{n'}{n},
$$

$$
\mathbf y_0'=
\begin{bmatrix}
-\kappa\cos\psi\\
-\kappa\sin\psi\\
0
\end{bmatrix},
\qquad
\mathbf z_0'=\mathbf x_0'\times\mathbf y_0+\mathbf x_0\times\mathbf y_0'.
$$

令

$$
\boldsymbol\omega_0=\frac12\left(
\mathbf x_0\times\mathbf x_0'
+\mathbf y_0\times\mathbf y_0'
+\mathbf z_0\times\mathbf z_0'
\right),
$$

$$
\phi'=\frac{u'/b}{\sqrt{1-(u/b)^2}},
\qquad
\boldsymbol\omega_{IT}=\boldsymbol\omega_0+\phi'\mathbf x_0.
$$

这里的撇号均表示对 $s$ 求导，$\boldsymbol\omega_{IT}$ 因而是每单位站位的旋转率，并满足

$$
\frac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}.
$$

### 7.4 超高基准的理论边界

当前实现采用中心线基准：超高只旋转轨道系，$\mathbf C(s)$ 不随 $u$ 平移。超高参考基长 $b$ 是抽象参考基线的长度，不等同于名义轨距或轮轨型面定位距离。

**仅理论：** 若以内轨或外轨为固定基准，超高变化必须同时引起中心线的空间平移，使指定钢轨参考点保持不动。这两种基准需要额外定义固定侧、参考点和位移方向，当前 `TrackGeometry` 不表示这种中心线平移。

## 8. 定义区间与不平顺分层

### 8.1 三维切线延长

平面曲率、竖向剖面和超高在组合时共享有限定义区间 $[s_{\min},s_{\max}]$。对区间外的有限站位，令 $s_b$ 为最近边界，组合后的线路几何采用三维切线延长：

$$
\mathbf C(s)=\mathbf C(s_b)+(s-s_b)\mathbf C'(s_b),
\qquad
\mathbf C'(s)=\mathbf C'(s_b),
\qquad
\mathbf C''(s)=\mathbf 0.
$$

域外保持边界的 $\psi$、$g$、$u$ 和 $\phi$，并令 $\kappa=g'=u'=0$。因此中心线和轨道姿态在定义边界连续，但中心线二阶导数与轨道系旋转率可以发生跳变。这是组合线路的明确延拓，不是由区间内最后一个 PL2 或 CIR 唯一推出的外推。

### 8.2 理想竖向线形与不平顺

理想竖向剖面决定中心线高程、坡度和轨道系。竖向不平顺是在轨型参考处叠加的局部位移场，不改写 $g(s)$、$I_g(s)$ 或理想中心线。车辆沿线路采样不平顺时产生的位移变化率属于轮轨相对运动，不是新的恒坡段、PL2 或 CIR。

## 9. 近似、非光滑性与适用条件

- 本模型始终以水平投影站位为自变量；恒坡段、PL2 与 CIR 分别服从第 4 节的不同解析关系，名称相似不能使这些公式互换。
- 恒坡段、PL2、CIR 及五次接缝的坡度、导数和积分均在各片内解析求值。数值近似位于变平面曲率的水平中心线积分，而不位于竖向段公式。
- 无接缝边界只保证坡度值连续，$g'$ 与更高导数可以跳变；五次接缝在窗口端点匹配到 $g''$，但 $g'''$ 通常仍可跳变。
- CIR 采用有限坡度分支 $\theta\in(-\pi/2,\pi/2)$，其严格圆弧定义要求 $L>0$ 且 $g_1\ne g_2$。PL2 同样以 $g_1\ne g_2$ 保持非退化，等坡度情形由恒坡段表达。
- 超高采用中心线滚转且满足 $|u|<b$；内轨或外轨固定基准只在第 7.4 节作为理论边界说明。
- 域外三维切线延长、显式接缝窗口及理想线形与不平顺的分层都是模型组成部分，不是从局部曲线公式自动推导出的唯一选择。

## 10. 核心代码映射

下表只定位承载本文数学关系的核心实现。

| 理论对象 | 主要实现 | 源文件 |
|---|---|---|
| 恒坡段、PL2、CIR 的坡度、导数与解析积分 | `TrackVerticalProfile` 及三个具名竖向段类型 | [`track_vertical_profile.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_vertical_profile.h)、[`track_vertical_profile.cc`](../../../libs/track_geometry/src/track_vertical_profile.cc) |
| 五次 $(g,g',g'')$ 接缝 | `internal::BuildQuinticHermiteCoefficients` | [`track_profile_quintic.cc`](../../../libs/track_geometry/src/track_profile_quintic.cc) |
| 三维中心线、切线延长与轨道系运动学 | `TrackGeometry`、`EvaluateTrackFrame` | [`track_geometry.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_geometry.h)、[`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| 位姿、中心线导数与站位旋转率的值类型 | `TrackFramePose`、`TrackFrameKinematics` | [`track_frame_pose.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_frame_pose.h) |
