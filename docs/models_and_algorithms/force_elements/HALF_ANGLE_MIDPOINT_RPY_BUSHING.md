[English](HALF_ANGLE_MIDPOINT_RPY_BUSHING.en.md)

# 半角中点 RPY 衬套

本篇陈述并推导六分量线性衬套族的本构律。平动三分量在两端姿态之间的半角中间系 B 中陈述，速度输入取两刚体在瞬时公共中点处的材料相对速度；转动三分量以对端在参考端中的 space-XYZ 滚转、俯仰、偏航角为坐标陈述，速率输入取这三个角的时间导数，物理力矩由功率共轭换回。所得的力与力矩以一对互为相反数的扳手同施于两端原点的瞬时世界中点，不附加支承矩。两端相对运动、扳手换点与三种施加方式的功率恒等式见[力元连接运动学与空间扳手](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)（下称共用篇），本篇只推导本族特有的量——半角系、中点速度、角坐标及其速率映射——并证明本族满足共用篇第 4.5 节的组织原则。类型与契约见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) 的 `HalfAngleMidpointRollPitchYawBushing`，求值在 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) 的 `VehicleForcePlan::CalcAppliedForces` 中。

## 1. 对象与记号

### 1.1 元件

一个衬套连接两个坐标系：A 端与 C 端，分别刚性固定在刚体 $\mathcal A$ 与 $\mathcal C$ 上。本族按共用篇第 1.1 节以 A 为参考端、C 为对端取相对运动，但两组本构常数都不直接在 A 中陈述：平动刚度 $\mathbf k_t$ 与阻尼 $\mathbf c_t$ 是半角中间系 B 三个轴上的对角量，转动刚度 $\mathbf k_r$ 与阻尼 $\mathbf c_r$ 是三个角坐标 $\boldsymbol\eta$ 上的对角量。本族的定义含三条契约：平动速度输入是两刚体在瞬时公共中点处的材料相对速度；转动速率输入是角坐标的时间导数，物理力矩由功率共轭得到；两个扳手同施于瞬时世界中点，不带参考端支承矩。本族没有内部状态，全部载荷是当前 $(q,v)$ 的代数函数，不占用基座第 5.1 节的 $z$ 块。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| A、C、$\mathcal A$、$\mathcal C$、I | 两端坐标系、承载它们的刚体、惯性系 | 共用篇 §1.1 |
| B | 半角中间系，满足 $R_{AB}=R_{BC}$ | 本篇 §2.2 |
| $\mathbf p_A$、$\mathbf p_C$、$\mathbf d_I$、$\mathbf d_A$ | 两端原点位置与相对位置 $\mathbf p_C-\mathbf p_A$ | 共用篇 §1.2、§2.1 |
| $\mathbf d_B$、$\mathbf d_C$ | 相对位置在 B、C 中的表达 | 本篇 §2.3 |
| $R_{IA}$、$R_{IC}$、$R_{AC}$ | 两端姿态与相对姿态 | 共用篇 §2.1 |
| $R_{AB}$、$R_{BA}$、$R_{IB}$ | 半角系的姿态 | 本篇 §2.2 |
| $q_{AC}=(q_0,\mathbf q)$ | $R_{AC}$ 的标量部非负单位四元数，$\mathbf q=(q_1,q_2,q_3)^{\mathsf T}$ | 本篇 §2.2 |
| $\theta$、$\mathbf n$ | $R_{AC}$ 的转角与单位转轴 | 本篇 §2.2 |
| $\mathbf v_{Ao}$、$\mathbf v_{Co}$、$\boldsymbol\omega_A$、$\boldsymbol\omega_C$ | 两端原点速度与两刚体角速度，在 I 中表达 | 共用篇 §1.2 |
| $\mathbf u_I$、$\mathbf u_A$ | 对端原点相对参考端的速度，$\mathbf u_A=\dot{\mathbf d}_A$ | 共用篇 §2.3 |
| $\boldsymbol\omega_{rel,I}$、$\boldsymbol\omega_{rel,A}$ | C 相对 A 的角速度 | 共用篇 §2.4 |
| $\boldsymbol\omega_{AB,A}$ | B 相对 A 的角速度，在 A 中表达 | 本篇 §2.4 |
| $\mathbf x_m$、$\mathbf v^{(\mathcal A)}(\mathbf x)$ | 瞬时世界中点、刚体上与 $\mathbf x$ 重合的材料点速度 | 共用篇 §2.5、§2.2 |
| $\mathbf u_{m,I}$、$\mathbf u_{m,A}$、$\mathbf u_{m,B}$ | 中点材料相对速度在 I、A、B 中的表达 | 共用篇 §2.5；B 中表达为本篇 §2.4 |
| $\boldsymbol\eta=(\eta_1,\eta_2,\eta_3)^{\mathsf T}$ | C 在 A 中的 space-XYZ 滚转、俯仰、偏航角 | 本篇 §2.6 |
| $\mathrm c_i$、$\mathrm s_i$ | $\cos\eta_i$、$\sin\eta_i$ | 本篇 §2.6 |
| $\sigma_+$、$\sigma_-$ | 角提取中的两个和差半角 | 本篇 §2.6 |
| $E(\boldsymbol\eta)$、$H(\boldsymbol\eta)$ | $\boldsymbol\omega_{rel,A}=E\dot{\boldsymbol\eta}$ 及其逆 $\dot{\boldsymbol\eta}=H\boldsymbol\omega_{rel,A}$ | 本篇 §2.7 |
| $\mathbf k_t$、$\mathbf c_t$、$\mathbf k_r$、$\mathbf c_r$ | 平动与转动的对角刚度、阻尼向量 | 本篇 §1.1 |
| $\mathbf f_{C,B}$、$\mathbf f_{C,I}$、$\boldsymbol\tau_{C,A}$、$\boldsymbol\tau_{C,I}$ | 作用于 C 端的力与力矩，第二下标为表达系 | 共用篇 §1.3 的双下标写法 |
| $\boldsymbol\tau_\eta$ | 与 $\dot{\boldsymbol\eta}$ 共轭的广义力矩 | 本篇 §2.8 |
| $\mathcal W_m^I[\mathcal A]$、$\mathbf r_m^{\mathcal A}$、$\mathbf x_{\mathcal A}$、$R_{I\mathcal A}$ | 中点扳手、中点在刚体上的体坐标、刚体位姿 | 共用篇 §3.1、§3.4 |
| $\mathcal P$、$\mathcal V_t$、$\mathcal V_r$ | 输送给两刚体的总功率、平动与转动储能 | 共用篇 §1.2；储能的下标为本篇 §4 |
| $\mathbf a\circ\mathbf b$、$\operatorname{skew}(\mathbf w)$ | 逐分量乘积、反对称矩阵 | 共用篇 §1.2 |

### 1.3 记号声明

本篇沿用共用篇第 1.3 节的全部约定：$R_{AB}$ 把 B 中分量变到 A 中；$\mathbf u$、$\boldsymbol\omega_{rel}$ 与 $\mathbf u_m$ 都是相对 A 度量的量，末下标只标表达系；作用于 C 端的载荷写双下标，第一下标为作用端、第二下标为表达系。本族的本构律天然以 C 端所受载荷陈述，所以正文通篇写 $\mathbf f_{C,B}$、$\boldsymbol\tau_{C,A}$，A 端所受载荷是其相反数。刚度与阻尼按基座第 5.3 节小写，下标 $t$、$r$ 区分平动与转动。

以下符号只在本篇有此含义：$\boldsymbol\eta$ 是姿态角坐标，与轮轨链的轮型面横向站位 $\eta$ 及不平顺 $\eta_y$、$\eta_z$ 无关；$\theta$ 与 $\mathbf n$ 是 $R_{AC}$ 的转角与单位转轴；四元数分量 $(q_0,q_1,q_2,q_3)$ 依次对应基座第 4.2 节的 $(w,x,y,z)$；$\sigma_\pm$ 是角提取的中间量。余弦、正弦缩写 $\mathrm c_i$、$\mathrm s_i$ 用直立体，以区别于按基座第 5.3 节保留给阻尼的斜体 $c$；$E(\boldsymbol\eta)$ 是第 2.7 节的速率映射矩阵，不是共用篇扳手记号 $\mathcal W_Q^E$ 中标记表达系的占位 E。未加坐标系下标的矩阵元 $R_{ij}$ 一律指 $R_{AC}$ 的元，按基座第 4.1 节一基编号，指向 Eigen 表达式时行列各减一。

## 2. 模型与推导

### 2.1 相对运动的来源

本族消费共用篇第 2 节的全部相对运动：$\mathbf d_A=R_{IA}^{\mathsf T}(\mathbf p_C-\mathbf p_A)$ 与 $R_{AC}=R_{IA}^{\mathsf T}R_{IC}$（共用篇 §2.1），含运输项的相对速度 $\mathbf u_A=\dot{\mathbf d}_A$（共用篇 §2.3），以及相对角速度 $\boldsymbol\omega_{rel,A}$ 及其与 $\dot R_{AC}$ 的关系 $\dot R_{AC}=\operatorname{skew}(\boldsymbol\omega_{rel,A})R_{AC}$（共用篇 §2.4）。本篇不重推这些量，只在它们之上构造半角系、中点速度和角坐标。

### 2.2 半角中间系 B

$R_{AC}$ 是一个旋转，设其转角为 $\theta$、单位转轴为 $\mathbf n$。每个旋转恰有两个单位四元数 $\pm q$ 与之对应；本族取标量部非负的那一个，并以它定义 $\theta$ 的取值范围：

$$
q_{AC}=\left(q_0,\ \mathbf q\right)=\left(\cos\tfrac{\theta}{2},\ \sin\tfrac{\theta}{2}\,\mathbf n\right),
\qquad
q_0=\tfrac12\sqrt{1+\operatorname{tr}R_{AC}}\ \ge0,
\qquad
\theta\in[0,\pi].
$$

半角中间系 B 定义为绕同一转轴转过半个转角的坐标系。其四元数由 $q_{AC}$ 的半角公式给出：

$$
q_{AB}=\left(\sqrt{\tfrac{1+q_0}{2}},\ \frac{\mathbf q}{2\sqrt{(1+q_0)/2}}\right)
=\left(\cos\tfrac{\theta}{4},\ \sin\tfrac{\theta}{4}\,\mathbf n\right).
$$

第二个等号用了 $\cos\tfrac\theta4=\sqrt{(1+\cos\tfrac\theta2)/2}$ 与 $\sin\tfrac\theta4=\sin\tfrac\theta2\big/\left(2\cos\tfrac\theta4\right)$，二者在 $\theta/4\in[0,\pi/4]$ 上都取正根，所以 $q_{AB}$ 的标量部同样非负；由 $\lVert\mathbf q\rVert^2=1-q_0^2$ 可直接验证 $q_{AB}$ 是单位四元数。把 $q_{AB}$ 转成旋转矩阵得到 $R_{AB}$。由于 $R_{AB}$ 与 $R_{AC}$ 同轴、转角减半，

$$
R_{AB}=R_{BC},
\qquad
R_{AC}=R_{AB}R_{BC}=R_{AB}^{2},
\qquad
R_{BA}=R_{AB}^{\mathsf T},
\qquad
R_{IB}=R_{IA}R_{AB}.
$$

B 在姿态上到 A 与到 C 的距离相同：从 A 转到 B 与从 B 转到 C 是同一个旋转。

标量部的符号选择决定的是 $R_{AC}$ 的哪一个平方根被当作 $R_{AB}$。$-q_{AC}$ 表示同一个旋转，但把它写成绕 $-\mathbf n$ 转 $2\pi-\theta$，对 $0<\theta<\pi$，其半角四元数 $\left(\sqrt{(1-q_0)/2},\ -\mathbf q/(2\sqrt{(1-q_0)/2})\right)$ 对应绕 $-\mathbf n$ 转 $\pi-\theta/2$，是 $R_{AC}$ 的另一个平方根；$\theta=0$ 时该式分母为零，而单位旋转的平方根不止两个（绕任意轴转 $\pi$ 都是），所取的主半角根是单位旋转本身。取 $q_0\ge0$ 即取转角 $\theta/2\le\pi/2$ 的那个平方根——离单位旋转更近的一支；它在 $\theta<\pi$ 上是 $R_{AC}$ 的连续函数。$\theta=\pi$ 时 $q_0=0$，两个代表的标量部都不为负，两个平方根同样远离单位旋转，构造不再有优先分支；半角主分支的连续定义域因此是 $\theta<\pi$ 的相对姿态。它与第 2.7 节角坐标的可逆域 $\lvert\eta_2\rvert<\tfrac\pi2$ 是两个不同的集合：$R_x(\pi)$ 的俯仰为零却落在半角分支的切面上。

### 2.3 半角系中的相对位置

平动变形度量是两端原点的相对位置在 B 中的表达：

$$
\mathbf d_B=R_{BA}\mathbf d_A=R_{BC}\mathbf d_C,
\qquad
\mathbf d_C=R_{AC}^{\mathsf T}\mathbf d_A.
$$

第二个等号由 $R_{BC}R_{AC}^{\mathsf T}=R_{AB}R_{AB}^{\mathsf T}R_{AB}^{\mathsf T}=R_{BA}$ 得到。它表明 $\mathbf d_B$ 不偏向任何一端：从 A 端看到的相对位置转过半个相对转角，与从 C 端看到的相对位置反向转过半个相对转角，得到同一个向量。共用篇第 2.1 节关于 $\mathbf d_A$ 的说明照样适用：$\mathbf d_B$ 是原点之差，不含自然长度，两原点重合时为零。

### 2.4 瞬时中点处的材料相对速度

两端原点的瞬时世界中点为 $\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I$。它不是任何一个刚体上的固定材料点；每一时刻，刚体 $\mathcal A$ 与 $\mathcal C$ 各有一个材料点与它重合。按共用篇第 2.2 节的搬移公式，这两个材料点的速度分别为

$$
\mathbf v^{(\mathcal A)}(\mathbf x_m)=\mathbf v_{Ao}+\boldsymbol\omega_A\times\left(\mathbf x_m-\mathbf p_A\right)=\mathbf v_{Ao}+\tfrac12\,\boldsymbol\omega_A\times\mathbf d_I,
$$

$$
\mathbf v^{(\mathcal C)}(\mathbf x_m)=\mathbf v_{Co}+\boldsymbol\omega_C\times\left(\mathbf x_m-\mathbf p_C\right)=\mathbf v_{Co}-\tfrac12\,\boldsymbol\omega_C\times\mathbf d_I,
$$

其中用了 $\mathbf x_m-\mathbf p_A=\tfrac12\mathbf d_I$ 与 $\mathbf x_m-\mathbf p_C=-\tfrac12\mathbf d_I$。两刚体在公共中点处的材料相对速度是二者之差：

$$
\mathbf u_{m,I}=\mathbf v^{(\mathcal C)}(\mathbf x_m)-\mathbf v^{(\mathcal A)}(\mathbf x_m)
=\left(\mathbf v_{Co}-\mathbf v_{Ao}\right)-\tfrac12\left(\boldsymbol\omega_A+\boldsymbol\omega_C\right)\times\mathbf d_I.
$$

这个式子对两个刚体对称：交换 A、C 只改变 $\mathbf d_I$ 与整体的符号。它与共用篇第 2.3 节含运输项的相对速度 $\mathbf u_I=\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I$ 的关系，由在括号内加减 $\boldsymbol\omega_A\times\mathbf d_I$ 得到：

$$
\mathbf u_{m,I}
=\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)
-\tfrac12\left(\boldsymbol\omega_C-\boldsymbol\omega_A\right)\times\mathbf d_I
=\mathbf u_I-\tfrac12\,\boldsymbol\omega_{rel,I}\times\mathbf d_I.
$$

叉积与旋转可交换，转入 A 再转入 B：

$$
\mathbf u_{m,A}=\mathbf u_A-\tfrac12\,\boldsymbol\omega_{rel,A}\times\mathbf d_A,
\qquad
\mathbf u_{m,B}=R_{BA}\,\mathbf u_{m,A}.
$$

这就是共用篇第 2.5 节登记的中点速度；本族的平动阻尼以它为输入。同一个量有两种等价写法：从两原点的世界速度之差出发，修正项是 $-\tfrac12(\boldsymbol\omega_A+\boldsymbol\omega_C)\times\mathbf d$；从已含运输项的 $\mathbf u$ 出发，修正项是 $-\tfrac12\boldsymbol\omega_{rel}\times\mathbf d$。二者不能混用。

$\mathbf u_{m,B}$ 一般不是 $\mathbf d_B$ 的时间导数。设 B 相对 A 的角速度为 $\boldsymbol\omega_{AB,A}$，即 $\dot R_{AB}=\operatorname{skew}(\boldsymbol\omega_{AB,A})R_{AB}$，对 $\mathbf d_B=R_{AB}^{\mathsf T}\mathbf d_A$ 求导（与共用篇第 2.3 节推导 $\dot{\mathbf d}_A$ 同法）得

$$
\dot{\mathbf d}_B=R_{BA}\left(\mathbf u_A-\boldsymbol\omega_{AB,A}\times\mathbf d_A\right),
\qquad
\mathbf u_{m,B}-\dot{\mathbf d}_B=R_{BA}\left[\left(\boldsymbol\omega_{AB,A}-\tfrac12\,\boldsymbol\omega_{rel,A}\right)\times\mathbf d_A\right].
$$

半角系的角速度一般不是相对角速度的一半：$R_{AB}$ 与 $R_{AC}$ 同轴，但当转轴 $\mathbf n$ 随时间变化时，转角减半并不使角速度减半（转角向量 $\theta\,\mathbf n$ 的导数与角速度之间隔着一个依赖该向量的映射，该映射在 $\theta\,\mathbf n$ 与 $\tfrac12\theta\,\mathbf n$ 处不同）。只有相对转轴保持不变时——包括所有平面相对运动——才有 $\boldsymbol\omega_{AB,A}=\tfrac12\boldsymbol\omega_{rel,A}$，此时 $\mathbf u_{m,B}=\dot{\mathbf d}_B$。一般三维转动下，$\mathbf u_{m,B}$ 是中点材料相对速度而非位移导数，这一区别决定第 4 节储能关系的条件。

### 2.5 平动本构

平动本构是 B 中的对角弹簧—阻尼律，给出的是作用于 C 端的力：

$$
\mathbf f_{C,B}=-\left(\mathbf k_t\circ\mathbf d_B+\mathbf c_t\circ\mathbf u_{m,B}\right),
\qquad
\mathbf f_{C,I}=R_{IB}\,\mathbf f_{C,B}=R_{IA}R_{AB}\,\mathbf f_{C,B}.
$$

$\mathbf d_B$ 从 A 端指向 C 端，所以 $\mathbf k_t$ 为正时 $-\mathbf k_t\circ\mathbf d_B$ 把 C 端拉回 A 端；A 端所受的力 $-\mathbf f_{C,B}$ 则把 A 端拉向 C 端，与[三向平动弹簧—阻尼力元](TRANSLATIONAL_SPRING_DAMPER.md)中参考端所受力的方向一致。三个刚度与三个阻尼分别沿 B 的三个轴作用；因为 B 对两端对称，这组常数不属于任何一端的坐标系。

### 2.6 space-XYZ 滚转、俯仰、偏航角的提取

转动变形度量是 C 在 A 中的 space-XYZ 滚俯偏角 $\boldsymbol\eta=(\eta_1,\eta_2,\eta_3)^{\mathsf T}$：先绕 A 的第一轴转 $\eta_1$，再绕 A 的第二轴转 $\eta_2$，最后绕 A 的第三轴转 $\eta_3$。三次都绕空间固定轴，等价于绕体轴依次作 Z、Y、X 转动，与基座第 4.3 节的 Z-Y-X 合成相同：

$$
R_{AC}=R_z(\eta_3)\,R_y(\eta_2)\,R_x(\eta_1)
=\begin{bmatrix}
\mathrm c_3\mathrm c_2 & \mathrm c_3\mathrm s_2\mathrm s_1-\mathrm s_3\mathrm c_1 & \mathrm c_3\mathrm s_2\mathrm c_1+\mathrm s_3\mathrm s_1\\
\mathrm s_3\mathrm c_2 & \mathrm s_3\mathrm s_2\mathrm s_1+\mathrm c_3\mathrm c_1 & \mathrm s_3\mathrm s_2\mathrm c_1-\mathrm c_3\mathrm s_1\\
-\mathrm s_2 & \mathrm c_2\mathrm s_1 & \mathrm c_2\mathrm c_1
\end{bmatrix},
\qquad
\mathrm c_i=\cos\eta_i,\ \mathrm s_i=\sin\eta_i.
$$

同一个 $R_{AC}$ 对应两族角：若 $(\eta_1,\eta_2,\eta_3)$ 生成它，则 $(\eta_1+\pi,\ \pi-\eta_2,\ \eta_3+\pi)$ 也生成它，且每个角还可加减 $2\pi$。提取必须选定分支。

俯仰角由矩阵元 $R_{31}$ 以及与它同列、同行的其余四个矩阵元确定：

$$
\eta_2=\operatorname{atan2}\!\left(-R_{31},\ \sqrt{\tfrac12\left(R_{11}^2+R_{21}^2+R_{32}^2+R_{33}^2\right)}\right),
\qquad
-R_{31}=\mathrm s_2,
\qquad
\sqrt{\tfrac12\left(R_{11}^2+R_{21}^2+R_{32}^2+R_{33}^2\right)}=\lvert \mathrm c_2\rvert.
$$

第二个参数非负，所以 $\eta_2\in[-\tfrac\pi2,\tfrac\pi2]$：这是选取 $\cos\eta_2\ge0$ 的那一族角。

滚转角与偏航角不直接由矩阵元取 $\operatorname{atan2}$，而是经四元数的和差半角。以 $q_{AC}=(q_0,\mathbf q)$ 为 $R_{AC}$ 的标量部非负四元数，由 $q_{AC}=q_z(\eta_3)\,q_y(\eta_2)\,q_x(\eta_1)$ 展开，四个分量的和差满足

$$
q_1+q_3=\left(\cos\tfrac{\eta_2}{2}-\sin\tfrac{\eta_2}{2}\right)\sin\sigma_+,
\qquad
q_0-q_2=\left(\cos\tfrac{\eta_2}{2}-\sin\tfrac{\eta_2}{2}\right)\cos\sigma_+,
\qquad
\sigma_+=\tfrac{\eta_3+\eta_1}{2},
$$

$$
q_3-q_1=\left(\cos\tfrac{\eta_2}{2}+\sin\tfrac{\eta_2}{2}\right)\sin\sigma_-,
\qquad
q_0+q_2=\left(\cos\tfrac{\eta_2}{2}+\sin\tfrac{\eta_2}{2}\right)\cos\sigma_-,
\qquad
\sigma_-=\tfrac{\eta_3-\eta_1}{2},
$$

其中若由角合成的四元数与标量部非负的代表相差一个符号，则每一行两个等式的右端同乘 $-1$。在 $\lvert\eta_2\rvert<\tfrac\pi2$ 上两个前置因子都为正，因此

$$
\sigma_+=\operatorname{atan2}\left(q_1+q_3,\ q_0-q_2\right),
\qquad
\sigma_-=\operatorname{atan2}\left(q_3-q_1,\ q_0+q_2\right),
$$

$$
\eta_1=\operatorname{wrap}\left(\sigma_+-\sigma_-\right),
\qquad
\eta_3=\operatorname{wrap}\left(\sigma_++\sigma_-\right),
$$

$\operatorname{wrap}$ 对严格大于 $\pi$ 或严格小于 $-\pi$ 的自变量加减一次 $2\pi$，把它折回 $[-\pi,\pi]$。两个 $\operatorname{atan2}$ 各落在 $[-\pi,\pi]$，其和与差落在 $[-2\pi,2\pi]$，折回后 $\eta_1$、$\eta_3$ 各取 $[-\pi,\pi]$ 内的代表。分支由三条数学选择决定：标量部非负的四元数使 $\sigma_\pm$ 落在各自的主值上；四元数换号只使 $\sigma_+$ 与 $\sigma_-$ 同时移动 $\pi$，$\sigma_\pm$ 的和与差因此只移动 $0$ 或 $\pm2\pi$，折回后的 $\eta_1$、$\eta_3$ 与四元数的符号无关，只由 $R_{AC}$ 决定；俯仰取 $\cos\eta_2\ge0$ 的那一族。三者合起来，$\boldsymbol\eta$ 是满足 $\eta_2\in[-\tfrac\pi2,\tfrac\pi2]$、$\eta_1,\eta_3\in[-\pi,\pi]$ 并生成 $R_{AC}$ 的角组；在 $\lvert\eta_1\rvert,\lvert\eta_3\rvert<\pi$ 且 $\cos\eta_2\ne0$ 时它是唯一的，端点 $\pm\pi$ 处的两个代表生成同一个 $R_{AC}$（$\cos\eta_2=0$ 见第 2.7 节）。经和差半角提取与直接取 $\operatorname{atan2}(R_{32},R_{33})$、$\operatorname{atan2}(R_{21},R_{11})$ 在该定义域内给出相同的角。

### 2.7 角速率映射

$\boldsymbol\eta$ 的时间导数不是 $\boldsymbol\omega_{rel,A}$ 的分量。对 $R_{AC}=R_z(\eta_3)R_y(\eta_2)R_x(\eta_1)$ 求导，

$$
\dot R_{AC}=\dot R_z R_y R_x+R_z\dot R_y R_x+R_z R_y\dot R_x,
$$

右乘 $R_{AC}^{\mathsf T}=R_x^{\mathsf T}R_y^{\mathsf T}R_z^{\mathsf T}$，并用单轴旋转的 $\dot R_z R_z^{\mathsf T}=\dot\eta_3\operatorname{skew}(\mathbf e_3)$、$\dot R_yR_y^{\mathsf T}=\dot\eta_2\operatorname{skew}(\mathbf e_2)$、$\dot R_xR_x^{\mathsf T}=\dot\eta_1\operatorname{skew}(\mathbf e_1)$ 及恒等式 $R\operatorname{skew}(\mathbf w)R^{\mathsf T}=\operatorname{skew}(R\mathbf w)$，由共用篇第 2.4 节 $\dot R_{AC}R_{AC}^{\mathsf T}=\operatorname{skew}(\boldsymbol\omega_{rel,A})$ 得

$$
\boldsymbol\omega_{rel,A}
=\dot\eta_1\,R_z(\eta_3)R_y(\eta_2)\mathbf e_1+\dot\eta_2\,R_z(\eta_3)\mathbf e_2+\dot\eta_3\,\mathbf e_3
=E(\boldsymbol\eta)\,\dot{\boldsymbol\eta},
\qquad
E=\begin{bmatrix}
\mathrm c_3\mathrm c_2 & -\mathrm s_3 & 0\\
\mathrm s_3\mathrm c_2 & \mathrm c_3 & 0\\
-\mathrm s_2 & 0 & 1
\end{bmatrix},
\qquad
\det E=\cos\eta_2.
$$

$E$ 的三列是三个瞬时转轴在 A 中的方向：滚转轴是 A 的第一轴经后两次转动搬到的位置，俯仰轴是 A 的第二轴经最后一次转动搬到的位置，偏航轴就是 A 的第三轴。$\det E=\cos\eta_2$，$E$ 在 $\cos\eta_2\ne0$ 时可逆，逆矩阵即角速率映射：

$$
\dot{\boldsymbol\eta}=H(\boldsymbol\eta)\,\boldsymbol\omega_{rel,A},
\qquad
H=E^{-1}=\begin{bmatrix}
\frac{\mathrm c_3}{\mathrm c_2} & \frac{\mathrm s_3}{\mathrm c_2} & 0\\
-\mathrm s_3 & \mathrm c_3 & 0\\
\frac{\mathrm c_3\mathrm s_2}{\mathrm c_2} & \frac{\mathrm s_3\mathrm s_2}{\mathrm c_2} & 1
\end{bmatrix}.
$$

直接相乘可验证 $HE=I$。$H$ 与 $E$ 都不含 $\eta_1$：角速度在 A 中表达，而 A 正是第一次转动的空间固定系，第一个角不影响后两个转轴在 A 中的位置。$\cos\eta_2=0$ 是这组坐标的奇点：此时 $R_{31}=\mp1$，C 的第一轴与 A 的第三轴平行，滚转轴与偏航轴重合，$E$ 降秩，$H$ 的第一、三行无界。这个奇点属于 space-XYZ 坐标本身，与 $\boldsymbol\omega_{rel,A}$ 无关；角坐标的可逆域是 $\lvert\eta_2\rvert<\tfrac\pi2$，它与第 2.2 节半角主分支的定义域 $\theta<\pi$ 是两个独立的集合。

### 2.8 转动本构与物理力矩

转动本构在角坐标上陈述：

$$
\boldsymbol\tau_\eta=-\left(\mathbf k_r\circ\boldsymbol\eta+\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
\qquad
\boldsymbol\tau_{C,A}=H^{\mathsf T}\boldsymbol\tau_\eta,
\qquad
\boldsymbol\tau_{C,I}=R_{IA}\,\boldsymbol\tau_{C,A}.
$$

$\boldsymbol\tau_\eta$ 是与 $\dot{\boldsymbol\eta}$ 共轭的广义力矩，不是空间中的力矩向量；能施加到刚体上的是与 $\boldsymbol\omega_{rel,A}$ 共轭的物理力矩 $\boldsymbol\tau_{C,A}$。二者由功率相等联系：要求对任意 $\boldsymbol\omega_{rel,A}$ 都有

$$
\boldsymbol\tau_{C,A}\cdot\boldsymbol\omega_{rel,A}
=\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}
=\boldsymbol\tau_\eta\cdot H\boldsymbol\omega_{rel,A}
=\left(H^{\mathsf T}\boldsymbol\tau_\eta\right)\cdot\boldsymbol\omega_{rel,A},
$$

即得 $\boldsymbol\tau_{C,A}=H^{\mathsf T}\boldsymbol\tau_\eta$。等价地 $\boldsymbol\tau_\eta=E^{\mathsf T}\boldsymbol\tau_{C,A}$，广义力矩的每个分量是物理力矩在对应瞬时转轴上的投影。写成列向量的组合，

$$
\boldsymbol\tau_{C,A}
=\frac{\tau_{\eta,1}}{\mathrm c_2}\begin{bmatrix}\mathrm c_3\\ \mathrm s_3\\ 0\end{bmatrix}
+\tau_{\eta,2}\begin{bmatrix}-\mathrm s_3\\ \mathrm c_3\\ 0\end{bmatrix}
+\frac{\tau_{\eta,3}}{\mathrm c_2}\begin{bmatrix}\mathrm c_3\mathrm s_2\\ \mathrm s_3\mathrm s_2\\ \mathrm c_2\end{bmatrix},
\qquad
\tau_{\eta,i}=\boldsymbol\tau_{C,A}\cdot\left(E\mathbf e_i\right).
$$

$H^{\mathsf T}$ 的三列是 $E$ 三列的对偶基：物理力矩沿这组对偶方向分解，而不是沿三个转轴分解。滚转与偏航两个广义力矩到物理力矩的放大因子都是 $1/\cos\eta_2$，这是奇点在载荷侧的表现。转动部分的力矩在 A 中表达并直接转入 I，不经过 B。

### 2.9 中点扳手对与功率原则

本族采用共用篇第 3.4 节的第三种施加方式：C 端所受的 $(\boldsymbol\tau_{C,I},\mathbf f_{C,I})$ 与 A 端所受的相反数都关于瞬时世界中点 $\mathbf x_m$ 取矩、施加在 $\mathbf x_m$，

$$
\mathcal W_{m}^{I}[\mathcal C]=\left(\boldsymbol\tau_{C,I},\ \mathbf f_{C,I}\right),
\qquad
\mathcal W_{m}^{I}[\mathcal A]=\left(-\boldsymbol\tau_{C,I},\ -\mathbf f_{C,I}\right),
$$

$$
\mathbf r_m^{\mathcal A}=R_{I\mathcal A}^{\mathsf T}\left(\mathbf x_m-\mathbf x_{\mathcal A}\right),
\qquad
\mathbf r_m^{\mathcal C}=R_{I\mathcal C}^{\mathsf T}\left(\mathbf x_m-\mathbf x_{\mathcal C}\right).
$$

两个扳手关于同一点取矩、互为相反数，合力与合力矩为零；本族不附加任何支承矩。这一对扳手向两刚体输送的总功率按共用篇第 4.1 节逐体计算，作用点速度取第 2.4 节中点处的材料点速度：

$$
\mathcal P
=\mathbf f_{C,I}\cdot\mathbf v^{(\mathcal C)}(\mathbf x_m)+\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_C
-\mathbf f_{C,I}\cdot\mathbf v^{(\mathcal A)}(\mathbf x_m)-\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_A
=\mathbf f_{C,I}\cdot\mathbf u_{m,I}+\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_{rel,I}.
$$

这正是共用篇第 4.5 节表中第三行的端点功率（以对端所受载荷陈述）。要证明本族满足组织原则，只需证明本构律实际使用的速度输入恰是这一行的两个量。平动部分：点积在旋转下不变，$\mathbf f_{C,I}\cdot\mathbf u_{m,I}=\mathbf f_{C,B}\cdot\mathbf u_{m,B}$，而第 2.5 节的阻尼输入正是 $\mathbf u_{m,B}$，弹性项 $-\mathbf k_t\circ\mathbf d_B$ 与它配对；转动部分：由第 2.8 节的共轭关系，$\boldsymbol\tau_{C,I}\cdot\boldsymbol\omega_{rel,I}=\boldsymbol\tau_{C,A}\cdot\boldsymbol\omega_{rel,A}=\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$，而转动律的速率输入正是 $\dot{\boldsymbol\eta}$。于是

$$
\mathcal P
=\mathbf f_{C,B}\cdot\mathbf u_{m,B}+\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}
=-\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf u_{m,B}\right)-\mathbf u_{m,B}\cdot\left(\mathbf c_t\circ\mathbf u_{m,B}\right)
-\boldsymbol\eta\cdot\left(\mathbf k_r\circ\dot{\boldsymbol\eta}\right)-\dot{\boldsymbol\eta}\cdot\left(\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
$$

两刚体收到的功率与六个本构分量各自的功率之和逐项相等。这个等式对任何位形和任何相对运动成立，不要求储能存在。

三条契约在此处互相锁定。把同一个力改施于两端原点并加支承矩，端点功率变为 $\mathbf f_{C,A}\cdot\mathbf u_A$，与以 $\mathbf u_{m,B}$ 为输入的阻尼不共轭；给中点扳手对再附加一个支承矩，则本已平衡的一对扳手多出一个净力偶，其功率 $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$ 不属于任何本构分量；把广义力矩 $\boldsymbol\tau_\eta$ 直接当作物理力矩施加，则转动功率变为 $\boldsymbol\tau_\eta\cdot\boldsymbol\omega_{rel,A}\ne\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$。中点施力、无支承矩、经 $H^{\mathsf T}$ 换回物理力矩，三者共同使上式成立。

## 3. 计算实现

### 3.1 相对运动的取法

每次求值，每个衬套以 A 端为参考端、C 端为对端向共用篇第 5.1 节的相对运动查询取得 $\mathbf d_A$、$R_{AC}$、$\mathbf u_A$、$\boldsymbol\omega_{rel,A}$，以及写载荷所需的 $\mathbf d_I$、$R_{IA}$、$\mathbf p_A$ 与两端原点所在的刚体。A、C 的命名只在这一步决定谁是相对运动的参考系；此后平动部分转入对两端对称的 B，转动部分保留在 A 中。相对空间速度的平动分量按多体层契约已含运输项，因此中点修正只减去 $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$，而不是第 2.4 节从世界速度之差出发的那一种修正。

### 3.2 平动部分的求值

按第 2.2 节由 $R_{AC}$ 取标量部非负的单位四元数，代入半角公式并转成 $R_{AB}$；取 $R_{BA}=R_{AB}^{\mathsf T}$。依次形成

$$
\mathbf d_B=R_{BA}\,\mathbf d_A,
\qquad
\mathbf u_{m,B}=R_{BA}\left(\mathbf u_A-\tfrac12\,\boldsymbol\omega_{rel,A}\times\mathbf d_A\right),
\qquad
\mathbf f_{C,B}=-\left(\mathbf k_t\circ\mathbf d_B+\mathbf c_t\circ\mathbf u_{m,B}\right),
$$

再以 $\mathbf f_{C,I}=R_{IA}R_{AB}\,\mathbf f_{C,B}$ 转入 I。逐分量乘积对应对角刚度与对角阻尼，六个常数各只作用于 B 的一个轴。

### 3.3 转动部分的求值

由同一个 $R_{AC}$ 按第 2.6 节提取 $\boldsymbol\eta$，按第 2.7 节形成 $H(\boldsymbol\eta)$；两者由一次调用同时给出，因为 $H$ 只依赖 $\eta_2$、$\eta_3$ 的正余弦。随后

$$
\dot{\boldsymbol\eta}=H\,\boldsymbol\omega_{rel,A},
\qquad
\boldsymbol\tau_\eta=-\left(\mathbf k_r\circ\boldsymbol\eta+\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
\qquad
\boldsymbol\tau_{C,A}=H^{\mathsf T}\boldsymbol\tau_\eta,
\qquad
\boldsymbol\tau_{C,I}=R_{IA}\,\boldsymbol\tau_{C,A}.
$$

转动部分不使用 B。

### 3.4 扳手的写出

由 $\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I$ 与两个刚体的世界位姿求中点在各自刚体上的体坐标 $\mathbf r_m^{\mathcal A}$、$\mathbf r_m^{\mathcal C}$，然后写出两条在 I 中表达的载荷：给 $\mathcal A$ 的 $(\mathbf r_m^{\mathcal A},\ -\boldsymbol\tau_{C,I},\ -\mathbf f_{C,I})$ 与给 $\mathcal C$ 的 $(\mathbf r_m^{\mathcal C},\ \boldsymbol\tau_{C,I},\ \mathbf f_{C,I})$。两个作用点都不是连接坐标系原点，且随相对运动逐次重新换算；多体层再按共用篇第 3.5 节把力矩移到各刚体原点。本族不写任何状态导数。

### 3.5 改变数学结果的分支

以下选择改变所得载荷，属于本族的定义而非数值细节：

- 四元数取标量部非负的代表，从而 $R_{AB}$ 是 $R_{AC}$ 转角减半的那个平方根（第 2.2 节）。
- 俯仰角以非负的第二参数取 $\operatorname{atan2}$，从而 $\cos\eta_2\ge0$（第 2.6 节）。
- 滚转角与偏航角由和差半角相减、相加后折回 $[-\pi,\pi]$（第 2.6 节）。
- $\cos\eta_2=0$ 处 $H$ 无定义，是角坐标可逆域 $\lvert\eta_2\rvert<\tfrac\pi2$ 的边界（第 2.7 节）。本族所取的连续理论分支同时满足三个开条件：$\theta<\pi$（半角主分支，第 2.2 节）、$\lvert\eta_2\rvert<\tfrac\pi2$（角坐标可逆，第 2.7 节）、$\lvert\eta_1\rvert,\lvert\eta_3\rvert<\pi$（不穿越折回边界，第 4.4 节）；三者相互独立，定义域是三个开域的交。

## 4. 数学性质与适用条件

### 4.1 小位移极限

在静止重合位形附近作联合线性化——位移 $\mathbf d_A$、相对转角 $\theta$、相对平动速度 $\mathbf u_A$ 与相对角速度 $\boldsymbol\omega_{rel,A}$ 同为小扰动——则 $R_{AB}\to I$、$H\to I$、$\boldsymbol\eta\to\theta\,\mathbf n$，而 $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$ 是两个小量之积、属二阶，本族退化为 A 中的线性六分量衬套：

$$
\mathbf f_{C,A}\approx-\left(\mathbf k_t\circ\mathbf d_A+\mathbf c_t\circ\mathbf u_A\right),
\qquad
\boldsymbol\tau_{C,A}\approx-\left(\mathbf k_r\circ\boldsymbol\eta+\mathbf c_r\circ\boldsymbol\omega_{rel,A}\right),
\qquad
\boldsymbol\eta\approx\theta\,\mathbf n.
$$

半角系与角速率映射是有限转动下才出现的修正；中点速度的修正项不是——它在 $R_{AC}=I$ 处就存在，有限相对角速度下是位移的一阶量，只在上述联合线性化中才属高阶。

### 4.2 半角系的定义域与对称性

$R_{AB}$ 在 $\theta<\pi$ 上是 $R_{AC}$ 的连续函数，$\theta=\pi$ 时 $R_{AC}$ 的两个平方根没有优先者。交换两端的名称后，相对姿态变为 $R_{AC}^{\mathsf T}$，其标量部非负四元数为 $(q_0,-\mathbf q)$，半角旋转为 $R_{AB}^{\mathsf T}=R_{CB}$，定义出同一个坐标系 B；$\mathbf d_B$ 与 $\mathbf u_{m,B}$ 反号，$\mathbf f_{C,B}$ 随之反号并改作用于原 A 端。平动部分因此与两端的命名无关。转动部分不然：$R_{AC}^{\mathsf T}$ 的 space-XYZ 角一般不是 $-\boldsymbol\eta$，只在一阶近似下如此，且 $\boldsymbol\tau_{C,A}$ 在 A 中表达；在第 4.1 节的联合小扰动下，交换两端得到的转动载荷从二阶起不同，有限相对角速度下，阻尼部分的差一般可含姿态的一阶项。

### 4.3 中点速度不是位移的导数

由第 2.4 节，$\mathbf u_{m,B}-\dot{\mathbf d}_B=R_{BA}[(\boldsymbol\omega_{AB,A}-\tfrac12\boldsymbol\omega_{rel,A})\times\mathbf d_A]$，相对转轴固定时为零，一般三维转动下不为零。$\mathbf u_{m,A}$ 与 $\mathbf u_A=\dot{\mathbf d}_A$ 相差 $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$，两原点重合或相对角速度平行于连线时相同。

### 4.4 角坐标的奇点与折回边界

$\cos\eta_2=0$ 时 $E$ 降秩、$H$ 的算子范数无界：接近奇点时某些有限的 $\boldsymbol\omega_{rel,A}$ 给出任意大的 $\dot\eta_1$、$\dot\eta_3$，某些有限的 $\boldsymbol\tau_\eta$ 给出任意大的 $\boldsymbol\tau_{C,A}$，但并非每个输入都被放大——$H\mathbf e_3=\mathbf e_3$ 与 $H^{\mathsf T}\mathbf e_2=(-\sin\eta_3,\cos\eta_3,0)^{\mathsf T}$ 始终有限；奇点处逆映射无定义。转动律在 $\lvert\eta_2\rvert<\tfrac\pi2$ 内有定义，放大倍数随相对俯仰逼近直角而无界增长。另外，$\eta_1$、$\eta_3$ 在 $\pm\pi$ 处折回，穿越该边界时角坐标跳变 $2\pi$，弹性力矩 $-\mathbf k_r\circ\boldsymbol\eta$ 随之跳变；这是第 3.5 节所述连续分支的另一条边界。

### 4.5 转动刚度只在角坐标中对角

$\mathbf k_r$、$\mathbf c_r$ 对 $\boldsymbol\eta$、$\dot{\boldsymbol\eta}$ 逐分量作用，物理力矩 $\boldsymbol\tau_{C,A}=-H^{\mathsf T}(\mathbf k_r\circ\boldsymbol\eta)-H^{\mathsf T}\operatorname{diag}(\mathbf c_r)H\,\boldsymbol\omega_{rel,A}$ 在 A 中一般不是对角律：有效的物理阻尼矩阵 $H^{\mathsf T}\operatorname{diag}(\mathbf c_r)H$ 对称半正定但依赖位形，有效的弹性力矩不是任何物理坐标系中的对角弹簧。本族与[侧滚弹簧—阻尼力偶](ROLL_SPRING_DAMPER_COUPLE.md)的区别正在于此：后者直接以矩阵元和 $\boldsymbol\omega_{rel,A}$ 分量为输入，不经姿态映射，没有本节的奇点。

### 4.6 一般六分量虚功率关系

第 2.9 节的 $\mathcal P=\mathbf f_{C,B}\cdot\mathbf u_{m,B}+\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$ 在定义域内无条件成立：$(\mathbf f_{C,B},\boldsymbol\tau_\eta)$ 是与六个速率 $(\mathbf u_{m,B},\dot{\boldsymbol\eta})$ 共轭的广义力。当 $\mathbf c_t$、$\mathbf c_r$ 各分量非负时，两个阻尼项 $\mathbf u_{m,B}\cdot(\mathbf c_t\circ\mathbf u_{m,B})$ 与 $\dot{\boldsymbol\eta}\cdot(\mathbf c_r\circ\dot{\boldsymbol\eta})$ 非负，阻尼部分只从两刚体吸收功率。

### 4.7 储能关系

在角坐标的连续分支内（不穿越 $\pm\pi$ 折回边界、$\cos\eta_2\ne0$），转动弹性功率是全导数，因为 $\dot{\boldsymbol\eta}$ 正是 $\boldsymbol\eta$ 的导数：

$$
\mathcal V_r=\tfrac12\,\boldsymbol\eta\cdot\left(\mathbf k_r\circ\boldsymbol\eta\right),
\qquad
\boldsymbol\eta\cdot\left(\mathbf k_r\circ\dot{\boldsymbol\eta}\right)=\dot{\mathcal V}_r.
$$

平动弹性功率一般不是，其与候选储能的差由 $\mathbf u_{m,B}$ 与 $\dot{\mathbf d}_B$ 之差决定：

$$
\mathcal V_t=\tfrac12\,\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf d_B\right),
\qquad
\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf u_{m,B}\right)-\dot{\mathcal V}_t=\mathbf d_B\cdot\left[\mathbf k_t\circ\left(\mathbf u_{m,B}-\dot{\mathbf d}_B\right)\right].
$$

该差在两种条件下为零。其一，相对转轴固定（含全部平面相对运动），此时 $\mathbf u_{m,B}=\dot{\mathbf d}_B$。其二，平动刚度各向同性，此时弹性力沿两原点连线，与 $\tfrac12\boldsymbol\omega_{rel}\times\mathbf d$ 正交：

$$
\mathbf k_t=k_t\begin{bmatrix}1\\1\\1\end{bmatrix}
\quad\Rightarrow\quad
\mathbf d_B\cdot\left(\mathbf k_t\circ\mathbf u_{m,B}\right)=k_t\,\mathbf d_A\cdot\mathbf u_A=\frac{d}{dt}\left(\tfrac12 k_t\,\lVert\mathbf d_A\rVert^2\right),
$$

此时 $\mathcal V_t=\tfrac12k_t\lVert\mathbf d_A\rVert^2$ 与 B 的选择无关。满足任一条件时总功率写成

$$
\mathcal P=-\frac{d}{dt}\left(\mathcal V_t+\mathcal V_r\right)-\mathbf u_{m,B}\cdot\left(\mathbf c_t\circ\mathbf u_{m,B}\right)-\dot{\boldsymbol\eta}\cdot\left(\mathbf c_r\circ\dot{\boldsymbol\eta}\right),
$$

元件输送给两刚体的功率不超过储能的减少率。不满足条件时，功率恒等式仍成立，但平动弹性部分沿闭合相对运动路径所做的功不必为零。

### 4.8 无力位形与状态

平动刚度、转动刚度各分量为正时，$\mathbf f_{C,B}=\mathbf 0$、$\boldsymbol\tau_\eta=\mathbf 0$ 的静止位形唯一，即 $\mathbf d_A=\mathbf 0$、$R_{AC}=I$；本族没有名义力项，无力位形不能由常数平移。全部载荷由当前 $(q,v)$ 决定，没有内部状态，也不向 $z$ 块写导数。

## 5. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 本族类型、两端与四个刚度阻尼向量的契约 | `HalfAngleMidpointRollPitchYawBushing`，见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 以 A 为参考端、C 为对端取得 $\mathbf d_A$、$R_{AC}$、$\mathbf u_A$、$\boldsymbol\omega_{rel,A}$ | `VehicleForcePlan::CalcAppliedForces` 内的 `CalcRelativeMotion`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 标量部非负的单位四元数 $q_{AC}$ | `VehicleForcePlan::CalcAppliedForces` 调用链中的 `CanonicalQuaternion`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 半角旋转 $R_{AB}$ | `VehicleForcePlan::CalcAppliedForces` 调用的 `CalcHalfAngleRotation`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| $\boldsymbol\eta$ 的提取与角速率映射 $H(\boldsymbol\eta)$ | `VehicleForcePlan::CalcAppliedForces` 调用的 `CalcSpaceXyzRollPitchYawKinematics`，其结果 `SpaceXyzRollPitchYawKinematics::rates_from_parent_angular_velocity` 即 $H$，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| $\mathbf d_B$、$\mathbf u_{m,B}$、$\mathbf f_{C,B}$、$\dot{\boldsymbol\eta}$、$\boldsymbol\tau_\eta$、$\boldsymbol\tau_{C,A}$ 的求值 | `VehicleForcePlan::CalcAppliedForces` 的衬套分支，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 中点体坐标 $\mathbf r_m^{\mathcal A}$、$\mathbf r_m^{\mathcal C}$ 与两条中点扳手 | `VehicleForcePlan::CalcAppliedForces` 调用的 `EmitMidpointBushingWrenchPair`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 换算中点所需的刚体世界位姿 | `MultibodyModel::CalcPoseInWorld`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 载荷条目 $\mathcal W_m^I[\mathcal A]$、$\mathcal W_m^I[\mathcal C]$ | `AppliedBodyWrench`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
