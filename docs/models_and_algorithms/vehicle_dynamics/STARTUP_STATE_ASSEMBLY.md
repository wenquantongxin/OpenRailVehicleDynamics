[English](STARTUP_STATE_ASSEMBLY.en.md)

# 起动状态装配

本篇说明一辆车的起动状态如何由一组有物理意义的初始量装配成系统连续状态的初值 $y_0=[q_0;v_0;z_0]$，以及哪些随记录一起给定的量进入运动方程却不属于状态。已发布的线路几何、不平顺谱、轮轨接触与力元各篇是组件；[整车多体动力学方程](MULTIBODY_EQUATIONS_OF_MOTION.md)（下称多体篇）把组件产生的体扳手与多体层自身的重力、关节阻尼合成完整右端 $[N(q)v;\dot v;\dot z]$，本篇构造该右端从 $t_0$ 起推进的初值，[轮轨接触力计划的运动学装配](../wheel_rail_contact/CONTACT_FORCE_PLAN_KINEMATICS.md)（下称接触力计划篇）说明多体状态如何变成每个轮轨接口的接触输入，[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)（下称时间积分篇）说明求解器如何推进并对完整右端作差分。本篇不是求解器：它不求静平衡、不求预载、不求接触，只把已经解析好的量按固定的几何与运动学规则写成状态。全篇围绕三个问题组织：给定了哪些有物理意义的初始量（第 2 节）；它们如何构成 $q_0$、$v_0$、$z_0$（第 3 至 5 节）；哪些量影响方程但不属于状态（第 6 节）。起动记录的类型见 [`resolved_startup_state.h`](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h)；装配由 [`assemble_resolved_initial_context.cc`](../../../libs/configuration/src/assemble_resolved_initial_context.cc) 的 `AssembleResolvedInitialContext` 完成。

## 1. 范围与记号

### 1.1 对象

装配的输入有四样：一辆已装配的车，即多体篇第 2.1 节的刚体树（自由体、回转副、Ball-RPY 球铰、焊接副）连同它的力元与可选的轮轨接触力计划；一条线路；车辆布局参考刚体所在的站位 $s_{\mathrm{ref}}$；一份起动记录，即别处已经解析好的一组初始量。输出是起动时刻 $t_0=0$ 的连续状态 $y_0=[q_0;v_0;z_0]$，以及与之配套、写进运行上下文而不属于状态的参数。连续状态的三块沿用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)（下称基座）第 5.1 节：$q$ 由每个自由体的七个广义位置与每个带坐标关节的广义位置组成，$v$ 由每个自由体的六个广义速度与关节速率组成，$z$ 由每个串联弹簧—黏性阻尼元件的内力组成。每个自由体与每个带坐标的关节各自拥有 $q$ 与 $v$ 中的恰好一段，每个串联元件拥有 $z$ 中的恰好一个分量；这些段互不重叠并且合起来铺满三个向量，装配按名称逐段填写，不依赖任何声明顺序。

起动记录只分两类：带公共自转生成的记录，除逐体逐关节的量外还给出一个公共的有效滚动半径，用它与整车起动速度派生一部分角速度与关节速率；显式给出的记录，每个自由体的角速度与每个回转副的速率都直接陈述，没有公共自转量。与之对应，轮轨载体也只分两类：刚性轮对，以及带独立旋转车轮的车轴体。前者通常配带公共自转生成的记录，后者配显式记录，但记录类型是记录自己的属性，不由载体类型推断。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| I、$T(s)$、B、F、M | 轨道惯性系；站位 $s$ 处的轨型系；自由体的体系；球铰的父系与子系 | 基座 §2.1–2.2、§4.3 |
| $R_{IT}(s)$、$\mathbf C(s)$ | 轨型系的姿态与原点，原点即中心线上的点 | 基座 §2.1–2.2；线路几何篇 §3.4 |
| $\boldsymbol\omega_{IT}$ | 轨型系每米转动率，在 I 中表达 | 基座 §3 |
| $s_{\mathrm{ref}}$ | 车辆布局参考刚体的站位，本次运行的参数 | 本篇新增 |
| $\Delta s_{\mathrm{mech},B}$、$\Delta s_{\mathrm{res},B}$ | 自由体 $B$ 的机械站位偏置（车辆定义）与解析站位偏移（起动记录） | 本篇新增 |
| $s_B$ | 自由体 $B$ 的起动站位 | 本篇新增 |
| $\mathsf q_{AB}$ | 与 $R_{AB}$ 对应的单位四元数，分量顺序 $(w,x,y,z)$ | 基座 §4.2 |
| $y_B$、$z_B$、$\boldsymbol\rho_B$ | 体原点在 $T(s_B)$ 中的横向与竖向偏置，以及 $\boldsymbol\rho_B=(0,y_B,z_B)^{\mathsf T}$ | 本篇新增 |
| $\boldsymbol\omega_{\mathrm{expl},B}$ | 显式给出的 B 相对 I 的角速度，在 B 中表达 | 本篇新增 |
| $\mathbf c_B$ | 公共自转系数向量，无量纲，在 B 中表达 | 本篇新增 |
| $V_0$ | 整车起动速度，每个体原点的惯性速度在 $T(s_B)$ 中的纵向分量 | 本篇新增 |
| $v_{y,B}$、$v_{z,B}$ | 体原点的惯性速度在 $T(s_B)$ 中的横向与竖向分量 | 本篇新增 |
| $r_{\mathrm{eff}}$、$\Omega_0$ | 公共有效滚动半径；公共自转量 $\Omega_0=V_0/r_{\mathrm{eff}}$ | 本篇新增 |
| $\theta_j$、$\dot\theta_j$、$\lambda_j$ | 回转副 $j$ 的角、速率与速率对 $\Omega_0$ 的无量纲倍数 | 本篇新增 |
| $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$、$\boldsymbol\omega_{FM\_F}$ | 球铰的三个角与子系相对父系的角速度，在父系中表达 | 基座 §4.3 |
| $F_i$ | 第 $i$ 个串联元件的内力，即参考端所受的轴向力 | 串联篇 §1.1 |
| $\mathbf f_0$ | 平动元件的名义力，在其参考端坐标系中表达 | 平动篇 §2.3 |
| $z_r$ | 钢轨型面原点在 $T$ 中的竖向基准，沿 $+z_T$ | 位姿归约篇 §2 |
| $g$、$\mathbf g_I$ | 重力加速度大小与重力向量 $g\,\mathbf e_z^I$ | 基座 §2.1；多体篇 §2.4 |
| $[q;v;z]$、$y_0$、$N(q)$ | 连续状态、其初值与位置导数映射 | 基座 §5.1；时间积分篇 §1.1 |

### 1.3 与基座及他篇记号的关系

坐标系、旋转矩阵与角速度的下标沿用基座第 4.1 节：$R_{TB}$ 把体系分量变到轨型系分量，$\boldsymbol\omega_{IB\_B}$ 是 B 相对 I 的角速度在 B 中表达，$\mathbf v_{IBo\_T}$ 是 B 的原点相对 I 的速度在 T 中表达。本篇以线路几何篇、串联篇、平动篇、位姿归约篇分别简称[线路几何与轨道坐标系](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.md)、[串联弹簧—黏性阻尼力元](../force_elements/SERIES_SPRING_VISCOUS_DAMPER.md)、[三向平动弹簧—阻尼力元](../force_elements/TRANSLATIONAL_SPRING_DAMPER.md)与[轮轨位姿归约与不平顺输入](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.md)。以下符号在本篇局部使用，均在此声明。无衬线的 $\mathsf q$ 是单位四元数，与广义位置 $q$ 不是同一个量。$V_0$ 是整车起动速度，取大写以区别于状态的速度块 $v$ 及其初值 $v_0$。$g$ 只表示重力加速度大小，与多体篇第 2.4 节相同；基座第 2.1 节的纵坡 $g(s)$ 在本篇不以符号出现，只通过 $\lVert\mathbf C'(s)\rVert$ 提及。$\Omega_0$ 是标量的公共自转量，与角速度向量 $\boldsymbol\omega$ 无关，也不是接触力计划篇与蠕滑篇中表示车轮俯仰率的 $\Omega$。$\mathbf c_B$ 是无量纲系数向量，与多体篇的质心偏置 $\mathbf c$ 及基座第 5.3 节的力元阻尼 $c$ 无关。$\theta_j$ 是回转副的角，与基座第 2.4 节的高低不平顺角 $\theta_\epsilon$ 无关。$F_i$ 沿用串联篇的力状态 $F$，与接触链法向力的弹性部分 $F_e$ 无关。$\dot s_B$ 只在第 4.4 节的推导中出现，是由 $V_0$ 派生的量，不是状态。

## 2. 给定的初始量

本节按物理量陈述起动记录给出的东西，不按记录的字段。记录里没有绝对里程：车辆放在线路的哪里是本次运行的参数 $s_{\mathrm{ref}}$，记录只陈述车辆自身的姿态、偏置与运动，因此同一份记录可以装配在线路的任何有限站位上。

**每个自由体 $B$。** 六组量，全部在该体自身站位 $s_B$ 处的局部轨型系 $T(s_B)$ 或体系 B 中陈述：姿态 $R_{TB}$，以单位四元数 $\mathsf q_{TB}$ 给出；解析站位偏移 $\Delta s_{\mathrm{res},B}$，即解析过程把该体沿线路移动了多少，机械布局本身属于车辆定义，不在记录中重复；体原点在 $T(s_B)$ 中的横向偏置 $y_B$ 与竖向偏置 $z_B$，按基座第 2.2 节向右、向下为正；显式惯性角速度 $\boldsymbol\omega_{\mathrm{expl},B}$，即 B 相对 I 的角速度在 B 中表达；公共自转系数向量 $\mathbf c_B$，无量纲、在 B 中表达，显式记录中为零；体原点的惯性速度在 $T(s_B)$ 中的横向分量 $v_{y,B}$ 与竖向分量 $v_{z,B}$。纵向分量不逐体给出，见下文的整车量。

**每个回转副 $j$。** 角 $\theta_j$，以及速率的一条生成规则：或者直接给出速率 $\dot\theta_{\mathrm{expl},j}$，或者给出无量纲倍数 $\lambda_j$，令速率等于 $\lambda_j$ 乘以公共自转量。两种规则产生的是同一个物理量，见第 5.1 节。

**每个 Ball-RPY 球铰。** 三个角 $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$，按基座第 4.3 节合成 $R_{FM}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})$；以及子系 M 相对父系 F 的角速度 $\boldsymbol\omega_{FM\_F}$，在 F 中表达。焊接副没有坐标，记录中也没有它的条目。

**每个串联弹簧—黏性阻尼元件 $i$。** 内力 $F_i$：沿该元件的作用轴、作用于其参考端的轴向力，正向与[串联弹簧—黏性阻尼力元](../force_elements/SERIES_SPRING_VISCOUS_DAMPER.md)第 1.1 节相同。

**每个三向平动弹簧—阻尼元件。** 名义力 $\mathbf f_0$：作用于参考端、在参考端坐标系中表达的常向量，含义见[三向平动弹簧—阻尼力元](../force_elements/TRANSLATIONAL_SPRING_DAMPER.md)第 2.3 节。

**整车量。** 起动速度 $V_0>0$，是每个自由体原点的惯性速度在其局部轨型系中的纵向分量；运行方向，当前只有一种定义，即朝站位递增方向起动；带公共自转生成的记录另给出有效滚动半径 $r_{\mathrm{eff}}>0$；钢轨型面竖向基准偏移，一个沿 $+z_T$ 的有符号长度；以及记录被解析时所用的重力加速度大小 $g$。

下表按去向汇总这些量，第三个问题的答案就在最后一列。

| 初始量 | 陈述所在的坐标系 | 去向 |
|---|---|---|
| $\mathsf q_{TB}$、$\Delta s_{\mathrm{res},B}$、$y_B$、$z_B$ | $T(s_B)$ | $q_0$ 的自由体段（第 3 节） |
| $\boldsymbol\omega_{\mathrm{expl},B}$、$\mathbf c_B$ | B | $v_0$ 的自由体段（第 4 节） |
| $V_0$、$v_{y,B}$、$v_{z,B}$ | $T(s_B)$ | $v_0$ 的自由体段（第 4 节） |
| $\theta_j$；$\dot\theta_{\mathrm{expl},j}$ 或 $\lambda_j$ | 关节轴 | $q_0$ 与 $v_0$ 的回转副段（第 5 节） |
| $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$、$\boldsymbol\omega_{FM\_F}$ | F | $q_0$ 与 $v_0$ 的球铰段（第 5 节） |
| $F_i$ | 元件作用轴 | $z_0$（第 5 节） |
| $r_{\mathrm{eff}}$ | — | 只参与 $\Omega_0=V_0/r_{\mathrm{eff}}$ 的生成，不进状态（第 4 节） |
| $\mathbf f_0$ | 参考端坐标系 | 上下文参数（第 6 节） |
| 钢轨型面竖向基准偏移 | $T$ 的 $+z$ 轴 | 接触几何常量 $z_r$（第 6 节） |
| $g$、运行方向 | I | 前提与定义，不进状态（第 6 节） |

## 3. 站位与姿态

### 3.1 站位的三项之和

自由体 $B$ 的起动站位是三项之和：

$$
s_B=s_{\mathrm{ref}}+\Delta s_{\mathrm{mech},B}+\Delta s_{\mathrm{res},B}.
$$

$s_{\mathrm{ref}}$ 是车辆布局参考刚体被放置的站位，是本次运行的参数；$\Delta s_{\mathrm{mech},B}$ 是该体相对布局参考刚体的机械站位偏置，属于车辆定义，车辆放在线路的哪里它都不变；$\Delta s_{\mathrm{res},B}$ 是解析过程沿线路移动该体的量，属于起动记录。三项各有一个来源：机械布局只有一个权威，记录只陈述解析改变了什么。

### 3.2 每个站位上的完整轨型系

线路在站位 $s$ 处给出轨型系 $T(s)$：姿态 $R_{IT}(s)$ 与原点 $\mathbf C(s)$，即中心线上的点，定义见基座第 2.1 至 2.2 节与[线路几何与轨道坐标系](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.md)第 3.4 节。$R_{IT}(s)$ 含航向、纵坡与超高三者的全部效应；站位落在线路定义区间之外时，取线路几何篇第 3.5 节的三维切线延长。装配对每个自由体都在其自身站位 $s_B$ 处取完整的 $T(s_B)$，不对任何线路作直线化或水平化的简化：曲线、坡道与超高上的起动使用同一组公式。

### 3.3 姿态与位置

体系 B 在 I 中的姿态由两个旋转合成，实现为两个单位四元数的乘积：

$$
R_{IB}=R_{IT}(s_B)\,R_{TB},
\qquad
\mathsf q_{IB}=\mathsf q_{IT}(s_B)\,\mathsf q_{TB},
$$

其中 $\mathsf q_{IT}(s_B)$ 是与 $R_{IT}(s_B)$ 对应的单位四元数，乘积是四元数乘法，其合成次序与矩阵乘积相同。四元数的模长对乘法可乘，$\mathsf q_{TB}$ 与 $\mathsf q_{IT}$ 都是单位四元数，所以 $\mathsf q_{IB}$ 也是；它的四个分量按 $(w,x,y,z)$ 的顺序进入 $q_0$ 中该体的段，多体篇第 3.1 节的位置导数映射按存储值使用它。体原点在 I 中的位置是轨型系原点加上在 $T(s_B)$ 中表达的横竖偏置：

$$
\mathbf p_{IoBo\_I}=\mathbf C(s_B)+R_{IT}(s_B)\,\boldsymbol\rho_B,
\qquad
\boldsymbol\rho_B=\begin{bmatrix}0\\ y_B\\ z_B\end{bmatrix}.
$$

偏置向量没有纵向分量，因此体原点位于其自身站位的横截面内：$\mathbf p_{IoBo\_I}-\mathbf C(s_B)$ 与 $\mathbf C'(s_B)$ 正交，$s_B$ 是线路几何篇第 3.6 节投影目标函数的驻点，在该节的二阶条件成立时就是体原点的投影站位。第 6.5 节再次用到这一事实。

在平直、无超高的线路上（其航向按基座第 2.1 节恒为零，中心线按线路几何篇第 3.3 节从惯性系原点出发），$R_{IT}=\mathbf 1$、$\mathbf C(s)=(s-s_{\min},0,0)^{\mathsf T}$，$s_{\min}$ 为线路起点站位，公式退化为 $R_{IB}=R_{TB}$、$\mathbf p_{IoBo\_I}=(s_B-s_{\min},y_B,z_B)^{\mathsf T}$。在一般线路上，两个自由体各自的 $T(s_{B_1})$ 与 $T(s_{B_2})$ 不同，即使 $R_{TB_1}=R_{TB_2}$，二者在 I 中的姿态也不同，其相对姿态 $R_{B_1B_2}=R_{TB_1}^{\mathsf T}R_{IT}(s_{B_1})^{\mathsf T}R_{IT}(s_{B_2})R_{TB_2}$ 由两站位之间的线路决定：车辆在 $t_0$ 沿线路弯过来。

## 4. 速度的形成

### 4.1 角速度

自由体的角速度先在体系中合成，再换到 I：

$$
\boldsymbol\omega_{IB\_B}=\boldsymbol\omega_{\mathrm{expl},B}+\mathbf c_B\,\Omega_0,
\qquad
\Omega_0=\frac{V_0}{r_{\mathrm{eff}}},
\qquad
\boldsymbol\omega_{IB\_I}=R_{IB}\,\boldsymbol\omega_{IB\_B}.
$$

$\Omega_0$ 是公共自转量，只在带公共自转生成的记录中定义；采用显式记录时没有 $r_{\mathrm{eff}}$，每个体的 $\mathbf c_B$ 为零，角速度就是 $\boldsymbol\omega_{\mathrm{expl},B}$。$\mathbf c_B$ 无量纲：采用公共自转生成时，绕体系某根轴自转的刚体通常取沿该轴的单位向量乘以由自转正向决定的符号，生成的自转分量大小由 $\Omega_0$ 统一给出；不参与生成的刚体取 $\mathbf c_B=\mathbf 0$，其角速度全部由 $\boldsymbol\omega_{\mathrm{expl},B}$ 给出。哪些刚体参与生成由记录逐体给出，不由载体类别决定。$\boldsymbol\omega_{IB\_I}$ 的三个分量进入 $v_0$ 中该体段的前三个位置，是多体篇第 2.1 节自由体广义速度的角速度部分。

### 4.2 原点速度

体原点的惯性速度先在 $T(s_B)$ 中写出，纵向分量取整车的 $V_0$，横竖分量取该体的值，再换到 I：

$$
\mathbf v_{IBo\_T}=\begin{bmatrix}V_0\\ v_{y,B}\\ v_{z,B}\end{bmatrix},
\qquad
\mathbf v_{IBo\_I}=R_{IT}(s_B)\,\mathbf v_{IBo\_T}.
$$

$V_0$ 不逐体复制：它是整车的一个量，每个自由体的纵向分量都由它生成。$\mathbf v_{IBo\_I}$ 进入 $v_0$ 中该体段的后三个位置。

### 4.3 只换基，不加运输项

第 4.1 与 4.2 节的两个量都是惯性速度，即相对 I 的角速度与相对 I 的原点速度，记录只是把它们写在人能核对的基（体系或局部轨型系）里。把它们换到 I 因此是纯粹的换基，不出现任何运输项。作为对照：若记录给出的是相对一个随体沿线路移动的轨型系的速度，惯性速度还要加上 $\dot s\big(\mathbf C'(s)+\boldsymbol\omega_{IT}\times R_{IT}\boldsymbol\rho_B\big)$，角速度还要加上 $\dot s\,\boldsymbol\omega_{IT}$；这些项在本篇的装配中一个也没有，因为记录的数已经是惯性量。一个后果是：在曲线上起动时，若希望车体带有跟随曲线的摇头角速度，它必须写在 $\boldsymbol\omega_{\mathrm{expl},B}$ 里，装配不会替记录补上。

### 4.4 起动速度与站位速率

$V_0$ 是惯性速度的局部纵向分量，一般不等于站位速率 $\dot s_B$。设体原点保持在其投影站位的横截面内运动，$\mathbf p=\mathbf C(s)+R_{IT}(s)\boldsymbol\rho$，$\boldsymbol\rho$ 的纵向分量恒为零。对时间求导并用基座第 3 节的 $\dfrac{dR_{IT}}{ds}=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}$，

$$
\dot{\mathbf p}=\big(\mathbf C'(s)+\boldsymbol\omega_{IT}\times R_{IT}\boldsymbol\rho\big)\dot s+R_{IT}\dot{\boldsymbol\rho}.
$$

取沿 $\mathbf x_T=\mathbf C'/\lVert\mathbf C'\rVert$ 的分量，$\dot{\boldsymbol\rho}$ 没有纵向分量而不作贡献，得

$$
V_0=\Big(\lVert\mathbf C'(s_B)\rVert+\big(\boldsymbol\omega_{IT\_T}\times\boldsymbol\rho_B\big)_x\Big)\dot s_B,
$$

其中 $\boldsymbol\omega_{IT\_T}=R_{IT}^{\mathsf T}\boldsymbol\omega_{IT}$，下标 $x$ 取 $T$ 中的纵向分量。$\lVert\mathbf C'\rVert$ 只在纵坡为零时等于一（基座第 2.1 节）；第二项在线路有转动率且体原点偏离中心线时可能非零。例如在水平、无超高、曲率为 $\kappa$ 的曲线上，$\boldsymbol\omega_{IT\_T}=(0,0,\kappa)^{\mathsf T}$，于是 $V_0=(1-\kappa y_B)\dot s_B$：同一个 $V_0$ 下，位于曲线内侧的体站位速率更大。$\dot s_B$ 不是状态，也不在装配中出现；接触力计划在每次求值时按同一关系从当前状态重新导出它，见接触力计划篇第 4 节。

### 4.5 公共自转是可选的生成规则，不构成滚动约束

公共自转生成给出满足 $\Omega_0\,r_{\mathrm{eff}}=V_0$ 的名义自转分量：$\mathbf c_B$ 为单位向量时，它是该刚体轴向角速率中由生成规则贡献的部分，轴向总角速率还含 $\boldsymbol\omega_{\mathrm{expl},B}$ 在该轴上的投影。这只是一次代数展开：它不构成运动约束，也不保证完整的起动状态在接触处满足纯滚动，因为 $r_{\mathrm{eff}}$ 不是接触处的滚动半径；运动方程（多体篇第 5 节）中没有滚动约束，自转与平动从 $t_0$ 起是独立的状态，此后由完整动力学耦合。$r_{\mathrm{eff}}$ 因此只参与速度的生成，它既不是接触链的标称滚动半径 $r_0$，也不是接触处的局部滚动半径 $r$（基座第 2.7 节），不进入任何接触几何。显式记录不使用这一规则；若其作者希望起动瞬间满足某种滚动关系，那是记录中各显式值之间的算术，装配对此既不要求也不检验。

## 5. 关节坐标与力元内力

### 5.1 回转副：一个角，两种速率来源

回转副 $j$ 的广义位置是绕关节轴的角 $\theta_j$，广义速度是其速率 $\dot\theta_j$（多体篇第 2.1 节）。角直接取记录值；速率由两种规则之一生成：

$$
\dot\theta_j=\dot\theta_{\mathrm{expl},j}
\qquad\text{或}\qquad
\dot\theta_j=\lambda_j\,\Omega_0 .
$$

两条规则给出同一个物理量，即子坐标系相对父坐标系绕关节轴的相对角速率，只是来源不同。倍数形式服务于按定义与 $\Omega_0$ 成比例的速率：例如一个部件经绕自转轴的回转副挂在自转的刚体上并且应当在 I 中不自转，它的相对速率必须抵消该刚体的完整轴向转动率；当该轴向转动率只来自生成分量、且生成系数 $\mathbf c_B$ 为沿该轴的单位向量时，这就是 $-\Omega_0$ 乘以轴向符号（系数长度不为一时按 $-\mathbf c_B\cdot\mathbf e\,\Omega_0$ 取，$\mathbf e$ 为轴向单位向量），写成倍数可以避免在记录里出现 $V_0/r_{\mathrm{eff}}$ 的第二份副本。采用显式记录时只有第一种形式；采用公共自转生成时两种形式都可以出现，按每个关节的记录决定，不由载体类别决定。

### 5.2 球铰与焊接副

Ball-RPY 球铰的三个广义位置就是记录中的三个角 $(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})$，其合成 $R_{FM}=R_z(\mathrm{yaw})R_y(\mathrm{pitch})R_x(\mathrm{roll})$ 按基座第 4.3 节；三个广义速度就是记录中的 $\boldsymbol\omega_{FM\_F}$，即子系相对父系的物理角速度，在父系中表达。它不是三个角的时间导数：二者由多体篇第 3.2 节的位形相关映射联系，该映射在 $\cos(\mathrm{pitch})=0$ 处奇异；装配不求这个映射，两组量都按记录所述直接写入。焊接副没有广义坐标，子系相对父系的位姿是模型的常量，记录中没有它的条目。

### 5.3 串联内力与 $z_0$

$z$ 块的每个分量属于一个串联元件，其初值是记录给出的内力：

$$
z_0=\begin{bmatrix}F_1\\ \vdots\\ F_{n_z}\end{bmatrix},
$$

$F_i$ 沿元件作用轴、作用于参考端，正向按串联篇第 1.1 节。它必须作为初值给出，因为串联篇第 2.1 节已说明这个力不能由 $(q,v)$ 代数确定。$F_i=0$ 表示元件松弛起动；$F_i\neq0$ 是起动瞬间的一个内力，但不是静预载：串联元件没有静刚度，若两端此后相对静止，该力按松弛时间 $c/k$ 衰减到零（串联篇第 2.3 节）。

### 5.4 初值的合成

综合第 3 至 5 节，

$$
q_0=\Big[\ \{\mathsf q_{IB};\ \mathbf p_{IoBo\_I}\}_{B}\ ;\ \{\theta_j\}_{j}\ ;\ \{(\mathrm{roll},\mathrm{pitch},\mathrm{yaw})\}\ \Big],
\qquad
v_0=\Big[\ \{\boldsymbol\omega_{IB\_I};\ \mathbf v_{IBo\_I}\}_{B}\ ;\ \{\dot\theta_j\}_{j}\ ;\ \{\boldsymbol\omega_{FM\_F}\}\ \Big],
$$

其中花括号表示按名称填入各自的段，段在向量中的位置属于模型，不由本篇规定。$y_0=[q_0;v_0;z_0]$ 与时刻 $t_0=0$ 一起写入一个新的运行上下文，成为时间积分篇第 1.1 节初值问题的初始条件 $y(t_0)=y_0$。

## 6. 配套的非状态量

### 6.1 名义力

每个三向平动元件的名义力 $\mathbf f_0$ 写入运行上下文的参数，而不是状态：它在每次右端求值时被读取，从不被积分，积分器的试算上下文复制同一组值。其数学角色见平动篇第 2.3 节：它是弹性部分在两端原点重合处的值，也是本构中唯一能把无力位形移离原点重合处的项。记录给出的 $\mathbf f_0$ 因此改变每个平动元件的力—位移关系，而不改变 $y_0$。

### 6.2 钢轨型面竖向基准

钢轨型面竖向基准偏移是一个沿 $+z_T$ 的有符号长度，按原值成为左右两侧轮轨位姿常量中的钢轨侧竖向基准 $z_r$，即[轮轨位姿归约与不平顺输入](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.md)第 3.5 节中钢轨型面实际基准 $z_{rd}=z_r+z_\epsilon$ 的固定部分；它同样是该篇第 3.8 节放置到有效站位的钢轨型面原点 $\mathbf o_c$ 在该站位轨型系中竖向坐标的固定部分。$+z_T$ 向下，所以正值把钢轨型面下移，负值上移。它是接触几何在整个运行期间的常量：改变它就改变 $t_0$ 及其后每一时刻的轮轨相对位姿，因而改变记录所描述的那个起动状态的含义。

### 6.3 同一重力前提

多体模型的重力是常向量 $\mathbf g_I=g\,\mathbf e_z^I$，$g>0$，方向由轨道惯性系固定（基座第 2.1 节，多体篇第 2.4 节）。起动记录中的串联内力、名义力与钢轨型面竖向基准偏移是在某一个 $g$ 之下解析出来的一组量，本篇假定装配所用模型的 $g$ 与记录解析时的 $g$ 是同一个数：这是记录有意义的前提，不是记录的内容。$g$ 是模型常量，不属于 $y_0$。

### 6.4 运行方向

当前只有一种运行方向定义：车辆朝站位递增方向起动。$V_0$ 是大小，方向由这一定义承担，因此每个自由体的 $\mathbf v_{IBo\_T}$ 纵向分量为正，第 4.4 节的 $\dot s_B$ 在分母为正时也为正。

### 6.5 投影种子

与起动状态一同装配的轮轨接触力计划为每个载体记一个初始投影站位，它由其站位参考刚体的 $s_B$ 给出，即第 3.1 节的同一个三项之和。它是接触力计划篇第 3 节局部分支投影的种子初值，不属于 $[q;v;z]$；按第 3.3 节，$s_B$ 是该体原点在 $t_0$ 的投影目标函数驻点，在该节所引的二阶条件成立时就是投影站位，此时种子与投影在起动瞬间一致。

## 7. 数学性质与适用条件

- **运动学一致性。** $\mathsf q_{IB}$ 是两个单位四元数之积，因而是单位四元数，$R_{IB}$ 是真旋转；四元数的整体符号不携带物理内容，$\pm\mathsf q_{IB}$ 给出同一个 $R_{IB}$。$v_0$ 中每个自由体段是在 I 中表达的角速度与原点速度，与多体篇第 2.1 节自由体广义速度的定义相同，$\dot q_0=N(q_0)v_0$ 因此直接有定义，起动时不需要任何投影或归一化。
- **只在 $t_0$ 贴合线路。** 位置与姿态按各体自身站位的完整轨型系构造，速度则是记录给出的惯性值，没有任何项把它们与线路的曲率或转动率绑定。$t_0$ 之后各体只服从运动方程以及力元与接触力计划写出的载荷，线路不再作为约束出现。
- **起动速度与站位速率。** $V_0$ 对全部自由体相同，而由第 4.4 节的关系，曲线、坡道上各体的 $\dot s_B$ 一般互不相同，也不等于 $V_0$；$\dot s_B$ 由接触力计划在每次求值中重新导出。
- **无平衡、无预载求解。** $t_0$ 的载荷就是各本构律在 $(q_0,v_0,z_0)$、名义力与 $z_r$ 之下给出的值，$\dot v(t_0)=\mathbf 0$ 既不被施加也不被检验；记录若与平衡不一致，运动从一段暂态开始。非零的串联内力若无相对运动维持，按各元件的松弛时间衰减。
- **同一记录在不同放置下不是同一个初值问题。** 记录以局部轨型系陈述各体的位姿。在平直、无超高的线路上，任意 $s_{\mathrm{ref}}$ 都给出相同的体间相对位姿；在一般线路上，体间相对位姿可能随放置改变，力元在 $t_0$ 看到的变形随之改变，而 $z_0$、名义力与 $z_r$ 都不随放置调整。
- **定义域。** $V_0>0$；带公共自转生成时 $r_{\mathrm{eff}}>0$，否则 $\mathbf c_B=\mathbf 0$ 且回转副只有显式速率；$\lVert\mathsf q_{TB}\rVert=1$；每个 $s_B$ 有限，可以落在定义区间之外的切线延长上；超高的定义域 $\lvert u\rvert<b$ 由线路自身保证（基座第 2.2 节）。球铰的三个角本身没有限制，但 $\cos(\mathrm{pitch})=0$ 使 $N(q)$ 奇异，积分不能从那里起步（多体篇第 3.2 节）。
- **不在 $y_0$ 中的量。** 时刻 $t_0$、投影种子与名义力都在上下文中而不在连续状态中，与基座第 5.1 节一致；$\Omega_0$、$r_{\mathrm{eff}}$ 与 $\dot s_B$ 是生成或派生的量，装配完成后不再保留。

## 8. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 起动记录的物理量：自由体、回转副、球铰、串联内力、名义力、公共自转生成、整车量 | `ResolvedStartupState`、`FreeBodyStartupState`、`RevoluteJointStartupState`、`BallRpyJointStartupState`、`SeriesSpringViscousDamperForceState`、`TranslationalSpringDamperNominalForce`、`CommonWheelSpinGeneration`，见 [`resolved_startup_state.h`](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h) |
| 运行方向的定义 | `StartupRunningDirection`，见 [`resolved_startup_state.h`](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h) |
| 机械站位偏置 $\Delta s_{\mathrm{mech},B}$ | `VehicleFreeBodyStationOffsetDefinition`，见 [`vehicle_definition.h`](../../../libs/configuration/include/orvd/configuration/vehicle_definition.h) |
| 站位之和、$\mathsf q_{IB}$ 与 $\mathbf p_{IoBo\_I}$、$\boldsymbol\omega_{IB\_I}$ 与 $\mathbf v_{IBo\_I}$、关节坐标与 $z_0$ 的形成 | `AssembleResolvedInitialContext`，见 [`assemble_resolved_initial_context.cc`](../../../libs/configuration/src/assemble_resolved_initial_context.cc) |
| 站位 $s$ 处的 $R_{IT}(s)$ 与 $\mathbf C(s)$ | `TrackGeometry::EvaluateTrackFrame`、`TrackFramePose`，见 [`track_geometry.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_geometry.h) 与 [`track_frame_pose.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_frame_pose.h) |
| 自由体段内的分量顺序 | `MultibodyModel::GetFreeBodyPositionRange`、`MultibodyModel::GetFreeBodyVelocityRange`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| $z$ 块中属于各串联元件的分量 | `SystemInstance::series_spring_damper_force_state_range`，见 [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| $y_0$ 与 $t_0$ 写入运行上下文 | `SystemInstance::SetTimeAndContinuousState`，见 [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| 名义力 $\mathbf f_0$ 作为上下文参数 | `SystemInstance::SetNominalForce`，见 [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| 钢轨型面竖向基准 $z_r$ 进入位姿常量 | `MakePoseConstants`，见 [`wheel_rail_contact_personality_assembly.cc`](../../../libs/configuration/src/wheel_rail_contact_personality_assembly.cc)；`WheelRailPoseConstants`，见 [`wheel_rail_pose.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h) |
| 重力方向与模型重力常量 | `GravitationalAccelerationInInertial`，见 [`track_inertial_frame.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_inertial_frame.h)；`MultibodyModel::gravity_vector`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 载体初始投影站位取 $s_B$ | `MakeCarrier`，见 [`assembled_vehicle_contact_scenario.cc`](../../../libs/configuration/src/assembled_vehicle_contact_scenario.cc)；`WheelRailContactCarrierDefinition::initial_projection_station_meters`，见 [`wheel_rail_contact_force_plan.h`](../../../libs/forces/include/orvd/forces/wheel_rail_contact_force_plan.h) |
