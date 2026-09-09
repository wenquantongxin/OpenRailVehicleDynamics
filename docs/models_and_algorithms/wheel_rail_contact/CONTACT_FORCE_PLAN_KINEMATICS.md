[English](CONTACT_FORCE_PLAN_KINEMATICS.en.md)

# 轮轨接触力计划的运动学装配

本篇是轮轨接触链的收尾一篇，处在组件与整车右端之间。已发布的线路几何、不平顺谱、轮轨接触与力元各篇是组件；[整车多体动力学方程](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.md)把组件产生的体扳手与多体层自身的重力、关节阻尼合成完整右端 $[N(q)v;\dot v;\dot z]$，[起动状态装配](../vehicle_dynamics/STARTUP_STATE_ASSEMBLY.md)构造初值 $y_0$，本篇说明多体状态如何变成每个轮轨接口的接触输入、接触结果如何变回作用于车轮刚体的一条等效扳手，[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 2.3 节说明求解器如何对完整右端作差分。本篇不含接触力学：位姿归约、接触几何、法向力、蠕滑率与切向力分别由[轮轨位姿归约与不平顺输入](WHEEL_RAIL_POSE_REDUCTION.md)、[单轮接触模型组装与成对扳手](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.md)及其上游各篇负责，本篇只形成它们的输入并消费它们的输出。载体站位的投影、轨型系中的位姿与速率、去自旋的几何姿态、型面刚体运动、不平顺取样、钢轨型面放置与逐斑扳手的合成都在这里定义。实现是 [`wheel_rail_contact_force_plan.h`](../../../libs/forces/include/orvd/forces/wheel_rail_contact_force_plan.h) 与 [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) 中的 `WheelRailContactForcePlan`。

## 1. 范围与记号

### 1.1 对象

本篇的对象是接触力计划：一个与已终结的多体模型绑定、构造后不再改变的力源。每次右端求值时，它把多体状态 $(q,v)$ 变成每个轮轨接口的接触输入，把接触结果写成作用于该接口车轮刚体的一条等效扳手。它自己不含任何接触力学，也不持有随时间演化的状态；它持有的是线路几何、可选的不平顺场、每一侧的接触模型与固定几何量，以及载体与接口的定义。

载体分两类。刚性轮对：车轮刚体就是载体，自转包含在这个刚体的姿态与角速度里。带独立旋转车轮的车轴体：载体是不自转的车轴体，每个车轮是经一个回转副与之相连的独立刚体；车轮的原点速度与绝对角速度取自车轮刚体本身，不自转的型面姿态与站位历史仍取自载体。两类载体走同一条数学路径，只在第 5 节的接口运动学处分叉。

本篇回答四个问题：载体在线路上的站位如何确定并随时间延续；载体的位姿与速度如何换到轨型系并去掉自转；每个接口的型面刚体运动、不平顺输入与钢轨型面放置如何形成；各接触斑的扳手如何合成一条体扳手。第 2 节到第 7 节依次回答，第 8 节给出计算顺序，第 9 节列出适用条件。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| $I$、$T$、$T(s)$ | 轨道惯性系；载体站位 $s_c$ 处的轨型系；任意站位 $s$ 处的轨型系 | 基座 §2.1–2.2 |
| $B$、$W$ | 载体的体系；不自转的轮型面系 | 基座 §2.3；本篇 §2.1 |
| $R_{BW}$ | 轮型面轴基在体系中的固定姿态，把 $W$ 系分量换成 $B$ 系分量 | 本篇新增 |
| $\mathbf p_{Bo}$、$R_{IB}$、$\mathbf v_{IBo\_I}$、$\boldsymbol\omega_{IB\_I}$ | 载体体原点在 $I$ 中的位置、载体姿态、体原点速度与角速度 | 基座 §4.1 |
| $s_c$、$s_e$ | 载体的共享站位；某一侧的有效型面站位 | 位姿归约篇 §2 |
| $\mathbf C(s)$、$\mathbf C'(s)$、$R_{IT}(s)$、$\boldsymbol\omega_{IT}(s)$ | 中心线位置、其站位导数、轨型系姿态、轨型系每米转动率，后两者在 $I$ 中表达 | 基座 §2.1、§3 |
| $\mathbf c'$、$\boldsymbol\omega_T$ | $\mathbf C'(s_c)$ 与 $\boldsymbol\omega_{IT}(s_c)$ 在 $T$ 中的表达 | 本篇新增 |
| $\mathbf r_B$、$\mathbf v_B$、$\boldsymbol\omega_B$ | 载体体原点在 $T$ 中的位置、惯性速度与惯性角速度，均在 $T$ 中表达 | 本篇新增 |
| $y_B$、$z_B$ | $\mathbf r_B$ 的横、竖分量，即位姿归约篇的 $y_w$、$z_w$ | 位姿归约篇 §2 |
| $\phi_w$、$\psi_w$、$\chi$ | $R_{TW}$ 的 X-Z-Y 滚转、摇头与俯仰 | 位姿归约篇 §2、§3.10 |
| $\bar R_{TW}$ | 去自转的几何姿态 $R_x(\phi_w)R_z(\psi_w)$ | 本篇新增 |
| $\mathbf a$ | 车轴方向 $\bar R_{TW}\mathbf e_2$，在 $T$ 中表达 | 位姿归约篇 §3.3 |
| $\dot s_c$、$\dot\ell$ | 载体的带符号站位速率与路径速率 | 基座 §3 |
| $\boldsymbol\omega_{TB\_T}$ | 载体相对轨型系的角速度，在 $T$ 中表达 | 基座 §4.1 |
| $\dot\chi$ | 载体的 X-Z-Y 俯仰率 | 本篇新增 |
| $\kappa$ | 站位 $s_c$ 处的平面曲率 | 基座 §2.2 |
| $\mathbf o_{\mathrm{wheel}}$、$\mathbf v_{\mathrm{wheel}}$、$\boldsymbol\omega_{\mathrm{wheel}}$ | 车轮刚体原点在 $T$ 中的位置、惯性速度与惯性角速度 | 位姿归约篇 §3.8（位置） |
| $\dot q_j$、$\mathbf u$ | 独立车轮回转副的广义速度与其单位正轴 | 本篇新增 |
| $\Omega$ | 交给接触模型的车轮俯仰率标量 | 蠕滑篇 §1 |
| $\sigma$ | 沿车轴带符号的轮型面横向基准 | 位姿归约篇 §2 |
| $\mathbf o_W$、$\mathbf v_o$ | 轮型面基准点的位置与速度 | 单轮篇 §3.1 |
| $y_\epsilon$、$z_\epsilon$、$y_\epsilon'$、$z_\epsilon'$、$\dot y_\epsilon$、$\dot z_\epsilon$ | 不平顺位移、空间斜率与时间变化率 | 位姿归约篇 §3.9 |
| $I_T$ | 线路的定义区间 | 位姿归约篇 §3.9 |
| $\psi_e$ | 以 $s_c$ 处方向斜率修正后的摇头 | 位姿归约篇 §3.9 |
| $y_r$、$z_r$、$\phi_r$ | 钢轨型面的横、竖基准与带符号轨底坡 | 位姿归约篇 §2 |
| $\mathbf o_c$、$R_{T\mathrm{rail}}$ | 钢轨型面的放置原点与放置姿态 | 位姿归约篇 §3.8 |
| $\widehat\psi_\epsilon$、$\widehat\theta_\epsilon$ | 有效站位上的方向与高低空间斜率角 | 位姿归约篇 §3.8 |
| $\varphi$、$\beta$、$d_y$、$d_z^{\uparrow}$ | 四个位姿标量 | 基座 §2.6 |
| $\mathbf x_{P,k}$、$\mathbf f_{T,k}$、$\mathbf m_{P,k}$ | 第 $k$ 个接触斑的轮侧作用点，以及轨对轮的力与力矩，在 $T$ 中表达 | 单轮篇 §3.2、§4 |
| $\mathcal W_Q^E=(\boldsymbol\tau_Q^E,\mathbf f^E)$ | 关于 $Q$ 取矩、在 $E$ 中表达的扳手 | 基座 §5.2 |

### 1.3 与既有各篇记号的关系

坐标系、正号与旋转记号沿用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)。$T$ 指 $s_c$ 处的轨型系，与位姿归约篇相同；同一载体的两侧接口共用这一个 $T$。$\phi_w$、$\psi_w$ 的下标 $w$ 沿用位姿归约篇，指不自转轮型面的载体，即本篇的载体；车轮刚体的量一律用下标 $\mathrm{wheel}$，沿用该篇第 3.8 节的 $\mathbf o_{\mathrm{wheel}}$。俯仰角记 $\chi$，即位姿归约篇第 3.10 节中一般记为 $\theta$ 的第三个 X-Z-Y 角，改用 $\chi$ 是为了不与高低不平顺角 $\theta_\epsilon$ 混淆。$R_{TW}$ 与 $\bar R_{TW}$ 有意区分：前者由多体状态直接算出，刚性轮对时含自转；后者只保留滚转与摇头，就是单轮篇第 3.2 节所称不含轮自转的 $R_{TW}$。源码把不自转的轮型面系称作 profile 系并以字母 P 标记，本篇按基座第 2.3 节把 $P$ 留给接触力的轮侧作用点，型面系记 $W$。$\Omega$ 是蠕滑篇的车轮自转率，前进滚动时为负；$\dot s_c$ 就是位姿归约篇的 $\dot s$。加粗的 $\mathbf v_B$、$\mathbf v_{\mathrm{wheel}}$ 是速度向量，与位姿归约篇第 3.6 节的标量分离 $v_c$ 无关。本篇的 $\boldsymbol\omega_T$ 是轨型系每米转动率在 $T$ 中的表达，单位是弧度每米；蠕滑篇用同一字母表示接触处的相对角速度，二者不在同一篇出现。$\dot q_j$ 是一个回转副的广义速度，属于基座第 5.1 节连续状态的 $v$ 块。[整车多体动力学方程](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.md)第 2.1 节把回转副的轴记作 $\mathbf a$，本篇把 $\mathbf a$ 留给位姿归约篇第 3.3 节的车轴方向，回转副的正轴改记 $\mathbf u$。

## 2. 载体与车轮

### 2.1 载体

载体是站位参考刚体。一条载体定义由三样东西组成：一个刚体，其体系记 $B$；一个初始投影站位；一个常旋转 $R_{BW}$。$R_{BW}$ 是不自转的轮型面系 $W$ 的轴基在体系 $B$ 中的固定姿态，按基座第 4.1 节的约定把 $W$ 系分量换成 $B$ 系分量；$W$ 按基座第 2.3 节以 $y$ 轴沿车轴、$z$ 轴沿径向向下取轴。它是一个常量正交矩阵，只做坐标轴的重新对准；体系本身已按 $W$ 的轴向约定取轴时它就是单位阵。它不含、也不能含车轮的自转相位：自转由多体状态随时间携带，一个常矩阵去不掉它，去自转在第 4.2 节完成。

轮型面系在轨型系中的姿态由载体姿态与常旋转合成：

$$
R_{TW}=R_{TB}R_{BW},\qquad R_{TB}=R_{IT}^{\mathsf T}R_{IB}.
$$

刚性轮对时 $R_{TB}$ 随车轮一起转动，$R_{TW}$ 因此含自转；车轴体时 $R_{TB}$ 是不自转体的姿态，$R_{TW}$ 只含它的滚转、摇头与小俯仰。

### 2.2 接口

接口由四项组成：一个载体、一个车轮刚体、左右侧之一、一个可选的回转副。刚性轮对的接口不带回转副，其车轮刚体就是载体刚体；带独立旋转车轮的接口必须命名把车轮连到载体的回转副，该回转副恰有一个广义位置与一个广义速度，见[整车多体动力学方程](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.md)第 2.1 节。一个载体的左右两侧各可带一个接口，两侧共用第 3 节与第 4 节的全部载体量。

每一侧各有一个接触模型与一组固定几何量：钢轨型面基准 $y_r$、$z_r$，带符号轨底坡 $\phi_r$，轮型面横向基准 $\sigma$，标称滚动半径 $r_0$ 与位姿归约篇第 3.4 节的俯仰修正选择；钢轨型面纵向原点约定对两侧共用。它们随计划构造一次确定，都不是状态。每个接口在每次求值中恰写出一条体扳手，因此计划写出的体扳手条数等于接口数。

## 3. 载体站位投影

### 3.1 投影问题

每次求值先把每个载体的体原点投影到线路。以多体层世界系查询得到的体原点位置 $\mathbf p_{Bo}$ 为空间点，本篇把多体层的世界系与轨道惯性系 $I$ 视为同一坐标系。投影是[线路几何与轨道坐标系](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.md)第 3.6 节的局部分支问题：以

$$
f(s)=\tfrac12\left\lVert\mathbf p_{Bo}-\mathbf C(s)\right\rVert^2
$$

为目标，从一个标识当前分支的种子出发，按该篇第 4.4 节至多两次 Newton 校正，接受满足尺度化驻点条件且 $f''(s)>0$ 的站位。所得站位记 $s_c$，称为该载体的共享站位：同一载体的两侧接口都以它为 $T$ 的站位。被投影的是体原点，不是任一侧的轮型面基准；两侧各自的有效站位 $s_e$ 在第 6.1 节由 $s_c$ 派生。

驻点条件 $f'(s_c)=-(\mathbf p_{Bo}-\mathbf C(s_c))\cdot\mathbf C'(s_c)=0$ 与基座第 2.2 节的 $\mathbf C'=\sqrt{1+g^2}\,\mathbf x_0$（$g$ 为该处的向上纵坡；超高滚转绕 $\mathbf x_0$ 进行，$T$ 的 $x$ 轴 $\mathbf x_T$ 就是 $\mathbf x_0$）合起来说明：在精确投影处，体原点在 $T$ 中的纵向坐标为零，即 $\mathbf e_1^{\mathsf T}\mathbf r_B=0$。第 4.3 节的站位速率用到这一点。

### 3.2 投影种子及其推进

种子取自被接受的步历史，不是连续状态的一部分。每个载体的种子初值是载体定义中的初始投影站位；系统的运行时上下文为每个载体各持一个种子。右端求值只读取种子数组：它用种子做投影、得到 $s_c$ 并向下游传递，但不把 $s_c$ 写回种子。每个被接受的内部步在其端点安装状态后，由一趟只做投影的求值以当前种子为出发点重新投影全部载体，并用所得站位整体替换种子；所有试算上下文都复制这一份已接受的种子。基座第 5.1 节把数值投影种子明确排除在 $[q;v;z]$ 之外，正是指这一组量。

这一安排的数学后果是：在一个内部步内种子固定，右端是 $(t,y)$ 的函数，与本步内的试算次序无关；跨步时种子随被接受的端点前移，局部分支得以延续，而线路几何层本身不保存任何时间历史。

## 4. 载体姿态与速率

### 4.1 换到轨型系

在 $s_c$ 处一次求出轨型系的位姿及其站位导数：中心线位置 $\mathbf C(s_c)$、姿态 $R_{IT}$、中心线导数 $\mathbf C'(s_c)$ 与每米转动率 $\boldsymbol\omega_{IT}(s_c)$，见线路几何篇第 3.4 节与第 4.3 节。以下 $R_{IT}$ 不带宗量时均指 $R_{IT}(s_c)$。载体的位置、速度与角速度换到 $T$：

$$
\mathbf r_B=R_{IT}^{\mathsf T}\left(\mathbf p_{Bo}-\mathbf C(s_c)\right),\qquad
\mathbf v_B=R_{IT}^{\mathsf T}\mathbf v_{IBo\_I},\qquad
\boldsymbol\omega_B=R_{IT}^{\mathsf T}\boldsymbol\omega_{IB\_I}.
$$

$\mathbf v_{IBo\_I}$ 与 $\boldsymbol\omega_{IB\_I}$ 是多体层给出的体系相对世界系的空间速度，平动分量取在体原点，见[整车多体动力学方程](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.md)第 4.2 节。这三式只换基：$\mathbf v_B$、$\boldsymbol\omega_B$ 仍是惯性速度与惯性角速度，不是相对轨型系的速度，没有减去轨型系自身运动的运输项。这与单轮篇第 2 节的静止钢轨假设一致：钢轨固定在 $I$ 中，接触处的相对速度就是车轮材料的惯性速度，所以送入接触模型的必须是惯性速度。线路量同样换到 $T$：

$$
\mathbf c'=R_{IT}^{\mathsf T}\mathbf C'(s_c),\qquad
\boldsymbol\omega_T=R_{IT}^{\mathsf T}\boldsymbol\omega_{IT}(s_c).
$$

因为 $\mathbf C'$ 沿 $T$ 的 $x$ 轴，精确算术下 $\mathbf c'=(\sqrt{1+g^2},0,0)^{\mathsf T}$；实现按上式由旋转形成，不代入这一闭式。

### 4.2 去自转的几何姿态

对 $R_{TW}=R_{IT}^{\mathsf T}R_{IB}R_{BW}$ 作基座第 4.4 节的 X-Z-Y 解析，得

$$
R_{TW}=R_x(\phi_w)\,R_z(\psi_w)\,R_y(\chi),\qquad \psi_w\in\left[-\tfrac{\pi}{2},\tfrac{\pi}{2}\right],
$$

三个角的显式表达见位姿归约篇第 3.10 节。滚转与摇头保留，俯仰 $\chi$ 舍去，几何姿态按前两个角重建：

$$
\bar R_{TW}=R_x(\phi_w)\,R_z(\psi_w),\qquad R_{TW}=\bar R_{TW}\,R_y(\chi).
$$

去掉自转的是这一步，不是常旋转 $R_{BW}$。刚性轮对的 $R_{BW}$ 是常量而车轮在转，$R_{TW}$ 里的自转全部落在 $\chi$ 中，只有舍去 $\chi$ 才能把它去掉；车轴体的 $\chi$ 是不自转体的小俯仰，同样被舍去。两类载体交给接触几何的型面姿态因此都不含绕车轴的转角，这是型面轴对称所允许的：轴对称型面绕车轴转动不改变其几何放置，见单轮篇第 3.2 节。

$\bar R_{TW}$ 有一个不依赖解析步骤的刻画。$R_y(\chi)\mathbf e_2=\mathbf e_2$，故车轴方向

$$
\mathbf a=R_{TW}\mathbf e_2=\bar R_{TW}\mathbf e_2=
\begin{bmatrix}-\sin\psi_w\\ \cos\phi_w\cos\psi_w\\ \sin\phi_w\cos\psi_w\end{bmatrix}
$$

与 $\chi$ 无关，正是位姿归约篇第 3.3 节的 $\mathbf a$。在 $\cos\psi_w>0$ 时，$\bar R_{TW}$ 是形如 $R_x(\cdot)R_z(\cdot)$、摇头限于闭四分之一圈、把 $\mathbf e_2$ 转到 $\mathbf a$ 的唯一旋转：去自转的几何姿态由车轴方向唯一决定，与体系绕车轴转到了哪个相位无关。

### 4.3 站位速率与路径速率

体原点位置可分解为 $\mathbf p_{Bo}=\mathbf C(s_c)+R_{IT}(s_c)\,\mathbf r_B$，投影使 $\mathbf e_1^{\mathsf T}\mathbf r_B$ 恒为零。对时间求导，并用基座第 3 节的 $dR_{IT}/ds=\operatorname{skew}(\boldsymbol\omega_{IT})R_{IT}$：

$$
\mathbf v_{IBo\_I}=\dot s_c\left(\mathbf C'(s_c)+\boldsymbol\omega_{IT}(s_c)\times R_{IT}\mathbf r_B\right)+R_{IT}\dot{\mathbf r}_B.
$$

换到 $T$ 并取纵向分量，$\mathbf e_1^{\mathsf T}\dot{\mathbf r}_B=0$，得站位速率

$$
\dot s_c=\frac{\mathbf e_1^{\mathsf T}\mathbf v_B}{\mathbf e_1^{\mathsf T}\left(\mathbf c'+\boldsymbol\omega_T\times\mathbf r_B\right)}.
$$

分子是体原点惯性速度沿轨型系纵轴的分量；分母第一项是 $\sqrt{1+g^2}$，第二项是轨型系每米转动带来的运输项，在无纵坡的平面曲线上等于 $-\kappa\,(y_B\cos\phi-z_B\sin\phi)$（$\phi$ 为基座第 2.2 节的超高角），即负曲率乘以体原点相对中心线的水平横向偏距，超高为零时分母退化为熟悉的 $1-\kappa y_B$。分母还有一个精确的表达：由 $\mathbf C''=(\sqrt{1+g^2})'\,\mathbf x_T+\sqrt{1+g^2}\,\boldsymbol\omega_{IT}\times\mathbf x_T$ 与 $\mathbf e_1^{\mathsf T}\mathbf r_B=0$ 可得线路几何篇第 3.6 节的 $f''(s_c)=\sqrt{1+g^2}\;\mathbf e_1^{\mathsf T}\left(\mathbf c'+\boldsymbol\omega_T\times\mathbf r_B\right)$，所以站位速率的分母就是 $f''(s_c)/\lVert\mathbf C'(s_c)\rVert$，在精确投影处与投影的正则性条件 $f''>0$ 同号。$\dot s_c$ 带符号，符号即行进方向。

路径速率按基座第 3 节由站位速率换算：

$$
\dot\ell=\left\lVert\mathbf C'(s_c)\right\rVert\,\dot s_c=\sqrt{1+g^2}\,\dot s_c,
$$

同样带符号。它是蠕滑篇第 3.1 节参考速度中的 $\dot\ell$；位姿归约篇的斜率角则只用 $\dot s_c$，两者在非零纵坡上不可互换。

### 4.4 相对角速度与俯仰率

轨型系 $T$ 随 $s_c(t)$ 移动，它在 $I$ 中的角速度是 $\boldsymbol\omega_{IT}\dot s_c$。载体相对轨型系的角速度在 $T$ 中为

$$
\boldsymbol\omega_{TB\_T}=\boldsymbol\omega_B-\boldsymbol\omega_T\,\dot s_c.
$$

$R_{BW}$ 是常量，这也是 $W$ 相对 $T$ 的角速度。把它与已解析的 $(\phi_w,\psi_w)$ 一起送入位姿归约篇第 3.10 节的 X-Z-Y 速率逆映射，三个角速率中只保留俯仰率：

$$
\dot\chi=\frac{\omega_y\cos\phi_w+\omega_z\sin\phi_w}{\cos\psi_w},
$$

其中 $\omega_y$、$\omega_z$ 是 $\boldsymbol\omega_{TB\_T}$ 的分量。滚转率与摇头率不进入下游。$\chi$ 在几何上被舍去，它的变化率却被保留：刚性轮对时 $\dot\chi$ 是车轮自转角（第三个 X-Z-Y 角）相对轨型系的变化率，车轴体时是该体相对轨型系的俯仰率。最后取该站位的平面曲率 $\kappa=\kappa(s_c)$，供第 6.1 节使用。

## 5. 接口运动学

### 5.1 刚性轮对

车轮刚体就是载体，车轮量就是载体量：

$$
\mathbf o_{\mathrm{wheel}}=\mathbf r_B,\qquad
\mathbf v_{\mathrm{wheel}}=\mathbf v_B,\qquad
\boldsymbol\omega_{\mathrm{wheel}}=\boldsymbol\omega_B,\qquad
\Omega=\dot\chi.
$$

同一轮对的两侧接口共用这四个量，型面姿态也共用 $\bar R_{TW}$。

### 5.2 带独立旋转车轮的车轴体

车轮刚体不是载体。对车轮刚体重复第 4.1 节的三个换基，用的是载体的同一个 $R_{IT}$ 与同一个 $\mathbf C(s_c)$，得到车轮原点在 $T$ 中的位置 $\mathbf o_{\mathrm{wheel}}$、惯性速度 $\mathbf v_{\mathrm{wheel}}$ 与惯性角速度 $\boldsymbol\omega_{\mathrm{wheel}}$。车轮刚体自身的姿态不被读取：它的型面姿态是载体的 $\bar R_{TW}$，它的站位是载体的 $s_c$，它不单独投影。俯仰率标量取

$$
\Omega=\dot\chi-\dot q_j,
$$

其中 $\dot q_j$ 是接口所命名回转副的广义速度，即 $v$ 中属于该回转副的那一个分量。

### 5.3 俯仰率减号的来源

上式的减号来自连接约定，不来自“独立车轮”这一名称，推导如下。令 $\mathbf u$ 为回转副的单位正轴，其正向按“车轮相对载体的角速度等于 $\dot q_j\mathbf u$”约定，这样父子次序的影响都吸收在 $\mathbf u$ 的符号里；$\mathbf u_W=R_{WB}\mathbf u_B$ 是它在轮型面系中的表达。车轮相对轨型系的角速度比载体多出关节项：

$$
\boldsymbol\omega_{T\,\mathrm{wheel}\_T}=\boldsymbol\omega_{TB\_T}+\dot q_j\,R_{TW}\mathbf u_W.
$$

实现采用的公式以轴向条件 $\mathbf u_W=-\mathbf e_2$ 为前提：回转副的正轴在轮型面系中沿车轴的反向。于是 $R_{TW}\mathbf u_W=-R_{TW}\mathbf e_2=-\bar R_{TW}\mathbf e_2=-\mathbf a$，恰是 X-Z-Y 第三转轴 $R_x(\phi_w)R_z(\psi_w)\mathbf e_y$ 的反向。位姿归约篇第 3.10 节的正向关系 $\boldsymbol\omega=\dot\phi\,\mathbf e_x+\dot\psi\,R_x(\phi)\mathbf e_z+\dot\theta\,R_x(\phi)R_z(\psi)\mathbf e_y$ 说明，一个恰沿第三转轴、大小为 $\lambda$ 的角速度解析为 $(\dot\phi,\dot\psi,\dot\theta)=(0,0,\lambda)$，且在 $\cos\psi_w\ne0$ 时这一解析唯一。关节项因此只改变俯仰率，滚转率与摇头率与载体相同：

$$
\Omega=\dot\chi-\dot q_j.
$$

这与车轮和载体共用同一个 $\bar R_{TW}$ 相容：关节只在车轴周围加一个转角，不改变车轴方向。一般地，若回转副正轴平行于车轴，$\mathbf u_W=\varepsilon\,\mathbf e_2$，$\varepsilon=\pm1$，则 $\Omega=\dot\chi+\varepsilon\,\dot q_j$，$\varepsilon=\mathbf u^{\mathsf T}R_x(\phi_w)R_z(\psi_w)\mathbf e_2$ 在 $T$ 中计算；实现对应 $\varepsilon=-1$。正轴沿 $+\mathbf e_2$ 的连接会给出加号。正轴不平行于车轴的连接一般会让 $\dot q_j$ 影响滚转率或摇头率，不再只改变俯仰率，车轮也不再与载体共用型面姿态，不在本模型之内。由此得到的 $\Omega$ 是车轮刚体相对轨型系的角速度按载体的 $(\phi_w,\psi_w)$ 解析所得的俯仰率，就是蠕滑篇的 $\Omega$：以 $y_T$ 向右、$z_T$ 向下的右手系计，前进滚动时为负。

## 6. 型面运动与不平顺输入

以下各量按接口逐个形成，所用固定几何量取自该接口所在侧。

### 6.1 有效摇头与有效站位

型面横截面按位姿归约篇第 3.9 节分两阶段选取。先在共享站位 $s_c$ 取横向不平顺的空间斜率，修正载体摇头：

$$
\psi_e=\psi_w-\operatorname{SafeAtan2Ratio}\!\left(y_\epsilon'(s_c)\,\dot s_c,\ \dot s_c\right),
$$

再由修正后的方向选出该侧的有效站位：

$$
s_e=s_c+\frac{-\sigma\sin\psi_e}{1-\kappa\left(y_B+\sigma\cos\psi_e\right)}.
$$

$\operatorname{SafeAtan2Ratio}$ 是位姿归约篇第 3.2 节带分母地板的比值角。这是修正方向 $\psi_e$ 下的有效截面站位映射：分子是按 $\psi_e$ 而非 $\psi_w$ 算出的纵向伸出，一般不等于第 6.4 节真实型面基准相对体原点的纵向偏置 $-\sigma\sin\psi_w$；分母是平面曲率修正因子，只在平面条件下与第 4.3 节站位速率的分母同形。两阶段选择的推导与性质在位姿归约篇第 3.9 节，此处不重推。$s_c$ 落在 $I_T$ 之外或没有不平顺场时，$s_c$ 处的斜率取零，前进且 $\dot s_c$ 高于地板时 $\psi_e=\psi_w$。

### 6.2 不平顺取样

进入本装配的场是位姿归约篇第 3.9 节的 $y_\epsilon$、$z_\epsilon$：在各通道定义域与 $I_T$ 的交集内取自然三次样条，其外取零；没有不平顺场时四个量恒为零。除第 6.1 节在 $s_c$ 取的一条横向斜率外，该侧的位移与两条斜率都在有效站位取样：$y_\epsilon(s_e)$、$z_\epsilon(s_e)$、$y_\epsilon'(s_e)$、$z_\epsilon'(s_e)$。$s_e$ 是否在 $I_T$ 内与 $s_c$ 分别判定。时间变化率以载体站位速率形成：

$$
\dot y_\epsilon=y_\epsilon'(s_e)\,\dot s_c,\qquad
\dot z_\epsilon=z_\epsilon'(s_e)\,\dot s_c.
$$

严格的链式法则会给出 $y_\epsilon'(s_e)\,\dot s_e$；实现不形成 $\dot s_e$，而用载体的 $\dot s_c$。这两条变化率是为下游方向角构造而形成的辅助输入，与载体站位速率配对，不是有效站位取样值的完整时间导数。在位姿归约之内这一配对没有额外后果：这两条变化率只以位姿归约篇第 3.2 节的比值 $\operatorname{SafeAtan2Ratio}(\dot y_\epsilon,\dot s_c)$、$\operatorname{SafeAtan2Ratio}(\dot z_\epsilon,\dot s_c)$ 进入位姿归约，其分母正是同一个 $\dot s_c$，所以所得方向角只依赖 $s_e$ 处的空间斜率以及 $\dot s_c$ 的符号与其是否高于地板。它们也不是钢轨材料点的速度；接触模型仍按单轮篇第 2 节取钢轨材料速度为零。

### 6.3 位姿归约的输入

位姿归约篇第 3 节的输入在此装配为：载体放置 $(y_B,z_B,\phi_w,\psi_w)$，即体原点在 $T$ 中的横竖坐标与去自转姿态的两个角；站位速率 $\dot s_c$；第 6.2 节的不平顺位移与变化率；该侧的固定几何量。带独立旋转车轮时放置仍取载体的量，不取车轮刚体的，位姿归约篇第 1 节已说明这一点。$\sigma$ 与滚动半径在归约内部按该篇第 3.5 节施加，这里不预先加到放置上。归约输出四个位姿标量 $\varphi$、$\beta$、$d_y$、$d_z^{\uparrow}$，是接触输入的第一部分。

### 6.4 轮型面刚体运动

接触输入的第二部分是轮型面基准的刚体运动，它由车轮刚体的量与去自转姿态合成：

$$
\mathbf o_W=\mathbf o_{\mathrm{wheel}}+\sigma\,\bar R_{TW}\mathbf e_2=\mathbf o_{\mathrm{wheel}}+\sigma\,\mathbf a,\qquad
\mathbf v_o=\mathbf v_{\mathrm{wheel}}+\boldsymbol\omega_{\mathrm{wheel}}\times\left(\mathbf o_W-\mathbf o_{\mathrm{wheel}}\right).
$$

姿态取 $\bar R_{TW}$，角速度取 $\boldsymbol\omega_{\mathrm{wheel}}$，弧速率取路径速率 $\dot\ell$，俯仰率标量取第 5 节的 $\Omega$。第二式是刚体上两点速度的搬移，与[力元连接运动学与空间扳手](../force_elements/FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)第 2.2 节同一公式。

这里要分清几何去自转与材料去自转。姿态 $\bar R_{TW}$ 不含自转，因为轴对称型面的放置不需要它；角速度 $\boldsymbol\omega_{\mathrm{wheel}}$ 含自转，因为单轮篇第 3.1 节在钢轨材料参考点处的材料速度 $\mathbf v_o+\boldsymbol\omega_{\mathrm{wheel}}\times(\mathbf x_R-\mathbf o_W)$ 需要它，力臂 $\mathbf x_R-\mathbf o_W$ 有径向分量，自转由此进入蠕滑。基准点速度中的搬移项 $\boldsymbol\omega_{\mathrm{wheel}}\times\sigma\mathbf a$ 却不含自转率：把 $\boldsymbol\omega_{\mathrm{wheel}}$ 分解为沿车轴的分量与垂直于车轴的分量，$\mathbf o_W-\mathbf o_{\mathrm{wheel}}=\sigma\mathbf a$ 与前者平行，叉积只剩后者的贡献。刚性轮对的自转沿 $\mathbf a$，第 5.3 节轴向条件下独立车轮的关节转动也沿 $\mathbf a$，因此两类载体的搬移项都只由摇头、滚转与随线路转动的角速度分量决定；$\mathbf v_o$ 本身是车轮原点的惯性速度 $\mathbf v_{\mathrm{wheel}}$ 加上这一搬移项。自转率以 $\Omega$ 单独进入蠕滑篇第 3.1 节的轮周速度。

### 6.5 钢轨型面放置

接触输入的第三部分是钢轨型面在 $T$ 中的刚体放置。型面原点先放在有效站位 $s_e$ 的轨型系中、横竖坐标为基准加不平顺位移之处，再换到 $T$：

$$
\mathbf o_c=R_{IT}^{\mathsf T}\left(\mathbf C(s_e)+R_{IT}(s_e)\begin{bmatrix}0\\ y_r+y_\epsilon(s_e)\\ z_r+z_\epsilon(s_e)\end{bmatrix}-\mathbf C(s_c)\right).
$$

放置姿态是位姿归约篇第 3.8 节的五因子乘积 $R_{T\mathrm{rail}}=R_{IT}^{\mathsf T}R_{IT}(s_e)\,R_z(\widehat\psi_\epsilon)\,R_y(\widehat\theta_\epsilon)\,R_x(\phi_r)$，其中两个带帽角由 $s_e$ 处的空间斜率 $y_\epsilon'(s_e)$、$z_\epsilon'(s_e)$ 直接形成。纵向原点约定也按该节施加：型面坐标约定以车轮刚体原点 $\mathbf o_{\mathrm{wheel}}$ 为参照、以 $\Delta s=s_e-s_c$ 为目标纵向坐标平移 $\mathbf o_c$，轨道站位约定保持 $\mathbf o_c$ 不变；参照点必须是车轮刚体原点而不是已含 $\sigma$ 的基准 $\mathbf o_W$，理由见该节。有效站位与钢轨型面放置的完整推导在位姿归约篇第 3.8 节到第 3.10 节，本篇只链接。

四个位姿标量、钢轨型面放置与轮型面刚体运动合起来就是单轮篇第 2 节列出的接触输入，接触模型按该篇第 6 节的算法给出零个、一个或多个接触斑的成对扳手。

## 7. 等效体扳手

### 7.1 逐斑换点与累加

设某接口得到 $K$ 个承载的接触斑，$K=0$ 也允许。第 $k$ 个斑的轨对轮扳手 $(\mathbf m_{P,k},\mathbf f_{T,k})$ 关于轮侧作用点 $\mathbf x_{P,k}$ 取矩、在 $T$ 中表达，见单轮篇第 5.3 节；当前模型不输出斑内直接自旋力矩，$\mathbf m_{P,k}=\mathbf 0$，见该篇第 4 节，下式按一般形式写出。先按基座第 5.2 节把每个斑的扳手搬到车轮刚体原点：

$$
\boldsymbol\tau_k=\mathbf m_{P,k}+\left(\mathbf x_{P,k}-\mathbf o_{\mathrm{wheel}}\right)\times\mathbf f_{T,k},
$$

力臂从新取矩点指向旧取矩点，与单轮篇第 5.1 节一致。再累加：

$$
\mathbf f_T=\sum_{k=1}^{K}\mathbf f_{T,k},\qquad
\boldsymbol\tau_T=\sum_{k=1}^{K}\boldsymbol\tau_k.
$$

先搬再加的次序不可颠倒：关于不同点取矩的扳手不能相加。取矩点是车轮刚体的原点，不是型面基准 $\mathbf o_W$；带独立旋转车轮时按车轮刚体而非载体取，两者原点在空间中是否重合由连接决定，不改变取矩点的定义。$K=0$ 时两个和为零向量。

### 7.2 换基与写出

累加后的扳手仍在 $T$ 中，按基座第 5.2 节只换基不换点，旋到惯性系：

$$
\mathbf f^I=R_{IT}\,\mathbf f_T,\qquad
\boldsymbol\tau^I=R_{IT}\,\boldsymbol\tau_T.
$$

用的是载体站位 $s_c$ 处的 $R_{IT}$，与第 4.1 节换入 $T$ 时同一个矩阵。结果写成一条作用于车轮刚体的体扳手条目：作用点是体系中的原点 $\mathbf 0$，表达系是世界系，力矩关于该点取，即

$$
\mathcal W^{I}_{\mathrm{wheel}\,o}=\left(\boldsymbol\tau^I,\ \mathbf f^I\right).
$$

多体层如何接纳这条条目见[整车多体动力学方程](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.md)第 4.3 节：条目已在体原点，多体层只需累加到该刚体的空间力，广义力投影发生在前向动力学的递推中，不在本篇。每个接口恰写一条；无接触的接口写零扳手而不是不写，因此条目列表的长度与次序在构造时就固定，不随接触状态变化。

### 7.3 钢轨侧

接触模型对每个斑同时给出轨对轮与轮对轨两半，见单轮篇第 5.3 节。本篇只消费轨对轮一半：钢轨是线路携带的几何，属于固定的世界，没有状态与惯量，多体树中没有可以承受轮对轨载荷的刚体，与单轮篇第 2 节的静止钢轨假设一致。因此接触力计划的全部输出就是每接口一条车轮体扳手，钢轨侧载荷不进入多体系统。

## 8. 计算实现

一次求值分两段。第一段串行准备，含全部多体查询：

1. 对每个载体：读体原点位姿，以种子作局部分支投影得到 $s_c$（第 3 节）。
2. 对每个载体：读体系空间速度，在 $s_c$ 处求轨型系位姿及其站位导数，形成 $\mathbf r_B$、$\mathbf v_B$、$\boldsymbol\omega_B$、$(\phi_w,\psi_w)$、$\dot s_c$、$\dot\ell$、$\dot\chi$ 与 $\kappa$（第 4 节）。
3. 对每个接口：刚性轮对直接复用载体量；独立车轮读车轮刚体的位姿与空间速度并换到载体的 $T$，读回转副速度，形成 $\Omega$（第 5 节）。

第二段逐接口求值，只读第一段准备好的量：形成 $\psi_e$、$s_e$ 与不平顺取样，装配位姿归约输入，重建 $\bar R_{TW}$ 并形成轮型面刚体运动与钢轨型面放置，求四个位姿标量（第 6 节），调用该侧接触模型，把各斑扳手搬到车轮原点、累加、旋到惯性系并写出（第 7 节）。各接口的求值彼此不交换数据，其结果只依赖第一段的输出与本接口的固定几何量。

被接受的内部步端点另有一趟只做第 1 步的求值：以当前种子为出发点重新投影全部载体并整体替换种子（第 3.2 节），不求速度、不求轨型系姿态、不求接触。

## 9. 数学性质与适用条件

- **局部分支的有效性。** 站位由种子标识的局部分支给出，不是全线最近点。前提是当前体原点位于某个满足 $f''>0$ 的正则根附近，且种子落在至多两次 Newton 校正可达的吸引域内；该域的大小依赖曲率、纵坡与体原点到中心线的距离，线路几何篇第 4.4 节未给出统一半径。种子随被接受的步前移，分支因此连续；线路几何层不保存历史。
- **右端的函数性质。** 种子固定时，从 $(q,v)$ 到体扳手的映射是确定的函数，投影结果不改变任何被积分的量；种子只在被接受的端点更新，右端在一个内部步内因此是 $(t,y)$ 的单值函数。
- **两个分母。** 站位速率的分母 $\mathbf e_1^{\mathsf T}(\mathbf c'+\boldsymbol\omega_T\times\mathbf r_B)$ 在精确投影处等于 $f''(s_c)/\lVert\mathbf C'(s_c)\rVert$，投影正则时为正；有效站位的分母 $1-\kappa(y_B+\sigma\cos\psi_e)$ 须非零，它趋近零时横向偏置到中心线站位的局部映射发生几何奇异，位姿归约篇第 5 节已列出。两者在无纵坡、无超高的平面曲线上同为 $1-\kappa y$ 形，分别采用体原点横坐标 $y_B$ 与有效映射中的横向坐标 $y_B+\sigma\cos\psi_e$；后者按 $\psi_e$ 而非 $\psi_w$ 形成，不是第 6.4 节真实型面基准的横坐标。
- **X-Z-Y 的奇点。** 姿态解析在 $\cos\psi_w=0$ 处奇异，俯仰率的分母同为 $\cos\psi_w$；解析把 $\psi_w$ 限制在闭的四分之一圈内。$\lvert\psi_w\rvert=\pi/2$ 意味着车轴沿线路纵向，是公式的边界而不是轨道车辆的运行姿态。第 5.3 节的唯一性论证同样要求 $\cos\psi_w\ne0$。
- **场的定义区。** 不平顺在各通道定义域之外与线路定义区间之外取零，$s_c$ 与 $s_e$ 的区间判定彼此独立；边界处若位移非零，场在那里不连续，见位姿归约篇第 5 节。线路自身在定义区间外按切线延长，投影与轨型系在区间外仍有定义，只是不平顺为零。
- **几何去自转与材料去自转的区别。** 型面姿态 $\bar R_{TW}$ 不含自转，依据是型面轴对称；角速度 $\boldsymbol\omega_{\mathrm{wheel}}$ 含自转，依据是材料速度需要它。基准点速度 $\mathbf v_o$ 中的搬移项不含自转率，自转以标量 $\Omega$ 进入蠕滑参考速度。对不轴对称的型面，舍去 $\chi$ 不再成立，本模型不覆盖。
- **速度的性质。** 送入接触模型的 $\mathbf v_o$ 与 $\boldsymbol\omega_{\mathrm{wheel}}$ 是惯性量在 $T$ 中的表达，不减运输项；唯一相对轨型系的量是形成俯仰率所用的 $\boldsymbol\omega_{TB\_T}$。这与钢轨固定在 $I$ 中相符：钢轨材料的惯性速度为零，轮材料的惯性速度就是相对速度。
- **单一轨型系。** 一个载体及其两侧接口的全部量都在 $T(s_c)$ 中表达，有效站位 $s_e$ 处的轨型系只进入钢轨型面放置，其原点经 $\mathbf C(s_e)$ 与 $R_{IT}(s_e)$、其姿态经前两个因子；等效扳手也以 $R_{IT}(s_c)$ 旋回惯性系。$s_e$ 处的轨型系不用于表达任何速度。
- **轴向条件。** 独立车轮的俯仰率公式 $\Omega=\dot\chi-\dot q_j$ 以回转副正轴在轮型面系中沿 $-\mathbf e_2$ 为前提；正轴沿 $+\mathbf e_2$ 时符号相反，正轴不平行于车轴的连接不在本模型之内。这是连接约定的后果，不是独立车轮的一般规律。
- **常旋转的作用域。** $R_{BW}$ 只重新对准坐标轴，它不去自转、不含时间，也不改变体原点；型面基准的偏置 $\sigma$ 不是 $R_{BW}$ 的一部分，它在第 6.4 节沿去自转后的车轴方向 $\mathbf a$ 施加。

## 10. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 载体定义、回转副定义与接口定义 | `WheelRailContactCarrierDefinition`、`IndependentWheelRevoluteJointDefinition`、`WheelRailContactInterfaceDefinition`，见 [`wheel_rail_contact_force_plan.h`](../../../libs/forces/include/orvd/forces/wheel_rail_contact_force_plan.h) |
| 每侧接触模型、固定几何量与钢轨纵向原点约定 | `WheelRailContactRuntimePersonality`，见 [`wheel_rail_contact_runtime_personality.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_runtime_personality.h)；`WheelRailPoseConstants`、`RailProfileOriginMode`，见 [`wheel_rail_pose.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_pose.h) |
| 载体与车轮刚体在世界系中的位姿与空间速度 | `MultibodyModel::CalcPoseInWorld`、`MultibodyModel::CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 载体站位 $s_c$ 的局部分支投影 | `WheelRailContactForcePlan::EvaluateCarrierProjections`，见 [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc)；`TrackGeometry::ProjectPointOntoSeededBranch`，见 [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) |
| 种子的初值、右端只读与被接受端点的更新 | `SystemRuntimeContext` 的构造、`SystemInstance::UpdateWheelRailProjectionStationHints`，见 [`system_instance.cc`](../../../libs/system_assembly/src/system_instance.cc)；`CompiledSystemPlan::CalcStateTimeDerivatives`，见 [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc)；`WheelRailContactForcePlan::CalcProjectionStationHints`，见 [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc)；`AdvanceToImpl`，见 [`system_continuous_state_advancer.cc`](../../../libs/integrators/src/system_continuous_state_advancer.cc) |
| 轨型系位姿及其站位导数 | `TrackGeometry::EvaluateTrackFrame`、`TrackFrameKinematics`，见 [`track_geometry.cc`](../../../libs/track_geometry/src/track_geometry.cc) 与 [`track_frame_pose.h`](../../../libs/track_geometry/include/orvd/track_geometry/track_frame_pose.h) |
| 换到轨型系、X-Z-Y 解析、站位速率、路径速率与俯仰率 | `WheelRailContactForcePlan::CompleteCarrierKinematics`，见 [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc)；`ResolveRollYawPitch`、`ResolveRollYawPitchRates`，见 [`roll_yaw_pitch.cc`](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc) |
| 接口运动学与俯仰率标量 $\Omega$ | `WheelRailContactForcePlan::CompleteInterfaceKinematics`，见 [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) |
| 有效摇头、有效站位、不平顺取样、$\bar R_{TW}$ 的重建、轮型面刚体运动与钢轨型面放置 | `CalcAppliedForcesImpl` 内的 `evaluate_interface`，见 [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc)；`SafeAtan2Ratio`、`PlaceRailProfileLongitudinalOrigin`，见 [`wheel_rail_pose.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc)；`TrackIrregularityField`，见 [`track_irregularity_field.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/track_irregularity_field.h) |
| 接触输入的三部分与逐斑结果 | `WheelRailContactInput`、`RailProfileFrame`、`WheelProfileRigidMotion`、`WheelRailContactResult`，见 [`wheel_rail_contact_model.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_model.h) |
| 逐斑换点、累加与换基 | `TransportWrench`、`RotateWrench`，见 [`contact_wrench.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_wrench.h) 与 [`contact_wrench.cc`](../../../libs/wheel_rail_contact/src/contact_wrench.cc) |
| 作用于车轮刚体的体扳手条目 | `AppliedBodyWrench`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
