[English](ROLL_SPRING_DAMPER_COUPLE.en.md)

# 侧滚弹簧—阻尼力偶

本篇定义并推导侧滚弹簧—阻尼力偶。这个力元只对两端的相对侧滚作出反应：它从相对姿态 $R_{AC}$ 读取一个无量纲的滚转度量，从相对角速度 $\boldsymbol\omega_{rel,A}$ 读取一个速率分量，把二者线性组合成绕参考端第一轴的力矩，再以一对等大反向的纯力偶施加于两个刚体。两端坐标系、相对运动、三种扳手施加方式与功率恒等式的推导见[力元连接运动学与空间扳手](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)（下称共用篇），本篇沿用其记号与结论，只写本族自己的定义、推导、实现与性质。本族的类型与本构常数是 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) 中的 `RollSpringDamperCouple`。本构求值是 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) 中 `VehicleForcePlan::CalcAppliedForces` 的侧滚族分支。纯力偶对由 [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h) 中的 `internal::EmitCoupleWrenchPair` 写出。

## 1. 对象与记号

### 1.1 元件与两端

本族连接参考端坐标系 A 与对端坐标系 C，二者分别固定在刚体 $\mathcal A$ 与 $\mathcal C$ 上，含义同共用篇第 1.1 节。元件由两个标量常数刻画：侧滚刚度 $k$ 与侧滚阻尼 $c$，都在 A 中陈述，作用轴是 A 的第一轴。它通常用来表示抗侧滚扭杆一类只对相对侧滚产生反力矩的装置。

共用篇第 5.1 节的相对运动含四个量，本族只消费其中两个：相对姿态 $R_{AC}$ 与相对角速度 $\boldsymbol\omega_{rel,A}$。相对位置 $\mathbf d_A$ 与相对平动速度 $\mathbf u_A$ 既不进入本构，也不进入载荷；元件因此对两端原点在各自刚体上的位置不敏感，只对两端的姿态与角速度敏感。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| A、C、$\mathcal A$、$\mathcal C$、I | 参考端与对端坐标系、承载它们的两个刚体、惯性系 | 共用篇 §1.1 |
| $R_{IA}$、$R_{IC}$、$R_{AC}=R_{IA}^{\mathsf T}R_{IC}$ | 两端姿态与相对姿态；矩阵元 $R_{ij}$ 一基编号 | 基座 §4.1、共用篇 §2.1 |
| $\boldsymbol\omega_A$、$\boldsymbol\omega_C$ | 两刚体相对 I 的角速度，在 I 中表达 | 共用篇 §1.2 |
| $\boldsymbol\omega_{rel,A}$、$\boldsymbol\omega_{rel,I}$ | C 相对 A 的角速度，在 A 与 I 中表达 | 共用篇 §2.4 |
| $\omega_{rel,A,1}$、$\omega_{rel,A,2}$ | $\boldsymbol\omega_{rel,A}$ 的第一、第二分量 | 共用篇 §4.3 的分量写法 |
| $\mathbf e_1$、$\mathbf e_2$、$\mathbf e_3$ | 标准基向量 | 基座 §4.1 |
| $\mathbf a_i=R_{IA}\mathbf e_i$ | A 的第 $i$ 条单位轴在 I 中的方向；C 的单位轴直接写 $R_{IC}\mathbf e_j$ | 本篇新增 |
| $\sigma$ | 滚转度量 $(R_{AC})_{32}$，无量纲 | 本篇新增 |
| $\phi$ | 纯滚转下 C 相对 A 绕 A 第一轴的转角 | 本篇新增 |
| $\psi$、$\theta$ | 角坐标对照所用 Z-Y-X 分解中的偏航角与俯仰角 | 本篇新增 |
| $k$、$c$ | 侧滚刚度与侧滚阻尼 | 基座 §5.3 |
| $m$、$\mathbf m_A=m\,\mathbf e_1$、$\mathbf m_I$ | 参考端所受纯力偶的标量，以及它在 A 与 I 中的向量 | 共用篇 §1.3 |
| $\mathcal W_Q^E[\mathcal A]$ | 作用于刚体 $\mathcal A$、关于 Q 取矩、在 E 中表达的扳手 | 共用篇 §1.3 |
| $\mathbf r_{A_o}^{\mathcal A}$、$\mathbf r_{C_o}^{\mathcal C}$ | 两端原点在各自刚体体坐标系中的坐标 | 共用篇 §3.1 |
| $\mathcal P$ | 力元向两刚体输送的总功率 | 共用篇 §1.2 |
| $\mathcal V$、$\mathcal D$ | 纯滚转下的储能与耗散功率 | 共用篇 §1.2 |
| $\operatorname{skew}(\mathbf w)$ | 反对称矩阵 | 共用篇 §1.2 |

### 1.3 符号声明

本篇的 $\phi$ 是纯滚转下 C 相对 A 绕 A 第一轴的转角，只在纯滚转与角坐标对照中出现；它与基座第 2.2 节的超高角 $\phi$、第 2.6 节的位姿滚转 $\varphi$ 是局部重名，三者不出现在同一篇中。$\psi$、$\theta$ 只在第 2.4 节与第 4 节的角坐标对照中出现，与基座的航向 $\psi$、高低不平顺角 $\theta_\epsilon$ 无关。$\sigma$ 是正弦型的无量纲量，不是角度。$\mathcal V$、$\mathcal D$ 沿用共用篇登记的储能与耗散记号，本篇只在纯滚转下定义它们。刚度 $k$ 与阻尼 $c$ 按基座第 5.3 节小写；$k$ 以 N·m/rad 计量，这是它在小角极限下等价扭转刚度的单位，因为它所乘的 $\sigma$ 无量纲，而 $c$ 所乘的 $\omega_{rel,A,1}$ 是真实的角速率。

## 2. 模型与推导

### 2.1 滚转度量与滚转速率

滚转度量取相对姿态矩阵的一个元：

$$
\sigma=(R_{AC})_{32}=\mathbf e_3^{\mathsf T}R_{AC}\,\mathbf e_2=\mathbf a_3\cdot R_{IC}\mathbf e_2 .
$$

下标按基座第 4.1 节一基编号，即第三行第二列，对应 Eigen 零基索引的 `(2, 1)`。第二个等号按基座第 4.1 节：$R_{AC}\mathbf e_2$ 是 C 的第二轴在 A 中的分量列，$\sigma$ 是它的第三分量。第三个等号用 $R_{AC}=R_{IA}^{\mathsf T}R_{IC}$，把同一个数写成两条单位轴在 I 中的点积：$\sigma$ 是 C 的第二轴沿 A 的第三轴的投影，等价地，是 $R_{IC}\mathbf e_2$ 偏离 A 第一、二轴所张平面的仰角的正弦。两条都是单位向量，所以 $|\sigma|\le1$ 对任何相对姿态成立。

滚转速率取相对角速度沿 A 第一轴的分量：

$$
\omega_{rel,A,1}=\mathbf e_1\cdot\boldsymbol\omega_{rel,A}=\mathbf a_1\cdot\left(\boldsymbol\omega_C-\boldsymbol\omega_A\right).
$$

第二个等号用共用篇第 2.4 节的 $\boldsymbol\omega_{rel,A}=R_{IA}^{\mathsf T}(\boldsymbol\omega_C-\boldsymbol\omega_A)$。它是 C 相对 A 此刻绕 A 第一轴转动的快慢，是一个不依赖任何角坐标的几何量。

### 2.2 恢复力矩与阻尼力矩

参考端所受的力矩是恢复项与阻尼项之和，方向沿 A 的第一轴：

$$
m=k\,\sigma+c\,\omega_{rel,A,1},
\qquad
\mathbf m_A=m\,\mathbf e_1 .
$$

对端所受力矩为 $-\mathbf m_A$，这是共用篇第 1.3 节的符号约定。符号的含义：C 相对 A 正向滚转时 $\sigma>0$，弹性项给参考端绕 $\mathbf a_1$ 的正力矩、使其转向 C 的姿态，给对端负力矩、使其转回 A 的姿态，是恢复项；阻尼项反抗相对角速度 $\omega_{rel,A,1}$ 而非相对转角——$\sigma>0$ 而 $\omega_{rel,A,1}<0$ 时它阻碍回零的运动——是耗散项。总力矩的符号由两项共同决定。$k\ge0$ 与 $c\ge0$ 是本构的定义域，见第 4 节。

### 2.3 纯滚转与小角极限

设 C 相对 A 只绕 A 的第一轴转过角 $\phi$：

$$
R_{AC}=R_x(\phi)=
\begin{bmatrix}
1&0&0\\
0&\cos\phi&-\sin\phi\\
0&\sin\phi&\cos\phi
\end{bmatrix},
\qquad
\sigma=(R_{AC})_{32}=\sin\phi .
$$

对时间求导，$\dot R_x(\phi)=\dot\phi\,\operatorname{skew}(\mathbf e_1)\,R_x(\phi)$，与共用篇第 2.4 节的 $\dot R_{AC}=\operatorname{skew}(\boldsymbol\omega_{rel,A})R_{AC}$ 比较得

$$
\boldsymbol\omega_{rel,A}=\dot\phi\,\mathbf e_1,
\qquad
\omega_{rel,A,1}=\dot\phi .
$$

于是纯滚转下的力矩为

$$
m=k\sin\phi+c\,\dot\phi .
$$

小角极限：$\sin\phi=\phi-\tfrac16\phi^3+O(\phi^5)$，故

$$
m=k\,\phi+c\,\dot\phi+O(\phi^3),
$$

元件在一阶上是绕 A 第一轴的线性扭转弹簧与线性扭转阻尼的并联。恢复项对 $\phi$ 的切线刚度是 $k\cos\phi$，在 $\phi=0$ 处等于 $k$，随 $|\phi|$ 增大而减小。

### 2.4 有限耦合转动下两个输入的几何含义

纯滚转下两个输入是 $(\sin\phi,\dot\phi)$，是同一个角的正弦与导数；一般的 $R_{AC}$ 含滚转以外的转动成分，此时二者连这一层关系也没有，各自的含义须分别陈述。

$\sigma=\mathbf a_3\cdot R_{IC}\mathbf e_2$ 只通过 C 的第二轴与 A 的第三轴依赖相对姿态：C 绕自身第二轴的任何转动、A 绕自身第三轴的任何转动都不改变 $\sigma$。它的时间导数由共用篇第 2.4 节的 $\dot R_{AC}$ 得到，

$$
\dot\sigma=\mathbf e_3^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_{rel,A})\,R_{AC}\,\mathbf e_2
=\boldsymbol\omega_{rel,A}\cdot\left(R_{AC}\mathbf e_2\times\mathbf e_3\right)
=(R_{AC})_{22}\,\omega_{rel,A,1}-(R_{AC})_{12}\,\omega_{rel,A,2}.
$$

它一般不等于 $\omega_{rel,A,1}$：纯滚转时二者相差因子 $\cos\phi$，相对角速度有第二分量时还多出一项。反过来，$\omega_{rel,A,1}$ 是角速度沿一条随 $\mathcal A$ 运动的轴的分量，它不是任何位形坐标的时间导数。因此 $(\sigma,\omega_{rel,A,1})$ 是本族的两个模型坐标——一个位形层的标量与一个速度层的标量——而不是某个坐标 $q$ 及其 $\dot q$；只有在纯滚转子流形上它们才退化为 $(\sin\phi,\dot\phi)$。

为看清这一点，可任取一种角坐标分解作对照。按基座第 4.3 节的 Z-Y-X 合成记 $R_{AC}=R_z(\psi)R_y(\theta)R_x(\phi)$，则

$$
\sigma=\sin\phi\cos\theta,
\qquad
\omega_{rel,A,1}=\dot\phi\cos\theta\cos\psi-\dot\theta\sin\psi .
$$

两个量都不是这一分解的滚转角 $\phi$ 及其导数 $\dot\phi$：$\theta=\psi=0$ 时它们退化为 $(\sin\phi,\dot\phi)$，第一个量仍是正弦而非角本身，只在小角一阶近似下才可换成 $\phi$；换用别的分解顺序会得到别的表达式。模型不使用任何一组这样的角，它们只出现在这一对照中。

由此也可以说明力矩为何不经姿态映射。若把 $m$ 理解为与某个角坐标共轭的广义力，则物理力矩须经该角坐标速率映射的转置求得，一般含 A 第二、三轴的分量；[半角中点 RPY 衬套](HALF_ANGLE_MIDPOINT_RPY_BUSHING.md)的转动部分就是这种做法。本族不作这一映射：$m\,\mathbf e_1$ 本身就是物理空间中绕 $\mathbf a_1$ 的力偶，$\sigma$ 与 $\omega_{rel,A,1}$ 是它的两个输入，不是某个角的值与导数。

### 2.5 转入惯性系与纯力偶对

载荷在 I 中写出。把 A 中的力偶转入 I，

$$
\mathbf m_I=R_{IA}\,\mathbf m_A=m\,\mathbf a_1 .
$$

力偶向量沿 A 的第一轴，这条轴随刚体 $\mathcal A$ 运动，不随 $\mathcal C$ 运动。两端按共用篇第 3.3 节的纯力偶对施加：

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf m_I,\ \mathbf 0\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(-\mathbf m_I,\ \mathbf 0\right).
$$

两条载荷不含力，纯力偶与取矩点无关，合力与合力矩为零；平动族的支承矩在这里没有对应物，因为没有力，也就没有力臂。对端所受力偶在 C 中的分量为 $-m\,R_{AC}^{\mathsf T}\mathbf e_1$，即 $R_{AC}$ 第一行的相反数乘 $m$，一般在 C 的三条轴上都有分量；只有纯滚转时 $R_{IC}\mathbf e_1=\mathbf a_1$，它才落在 C 的第一轴上。

### 2.6 功率原则

本族采用共用篇第 4.5 节表中的第二行：纯力偶对配相对角速度。按共用篇第 4.1 节，无力的扳手向刚体输送的功率是力偶与该刚体角速度的点积，两刚体合计

$$
\mathcal P=\mathbf m_I\cdot\boldsymbol\omega_A-\mathbf m_I\cdot\boldsymbol\omega_C
=-m\,\mathbf a_1\cdot\boldsymbol\omega_{rel,I}
=-m\,\mathbf e_1\cdot\boldsymbol\omega_{rel,A}
=-m\,\omega_{rel,A,1}.
$$

第二个等号用 $\mathbf m_I=m\,\mathbf a_1$ 与 $\boldsymbol\omega_{rel,I}=\boldsymbol\omega_C-\boldsymbol\omega_A$；第三个等号用 $\mathbf a_1\cdot\boldsymbol\omega_{rel,I}=\mathbf e_1^{\mathsf T}R_{IA}^{\mathsf T}\boldsymbol\omega_{rel,I}=\mathbf e_1\cdot\boldsymbol\omega_{rel,A}$。本构律以 $\omega_{rel,A,1}$ 为速率输入、以 $m$ 为输出，它所记的功率正是 $-m\,\omega_{rel,A,1}$：端点功率与本构功率相等，本族满足组织原则。这一相等不依赖两端原点的位置，因为纯力偶的功率不含作用点。

代入本构，

$$
\mathcal P=-k\,\sigma\,\omega_{rel,A,1}-c\,\omega_{rel,A,1}^{2}.
$$

第二项对任何运动都不为正，是阻尼耗散。第一项在纯滚转下是储能的全导数：

$$
-k\sin\phi\,\dot\phi=-\frac{d}{dt}\mathcal V,
\qquad
\mathcal V=k\left(1-\cos\phi\right),
\qquad
\mathcal D=c\,\dot\phi^{2},
$$

于是 $\mathcal P=-\dot{\mathcal V}-\mathcal D$。阻尼输入必须取 $\omega_{rel,A,1}$ 而不是 $\dot\sigma$，正是功率原则的要求：力偶对的端点功率恒为 $-m\,\omega_{rel,A,1}$，若把阻尼项改写成 $c\,\dot\sigma$，其端点功率 $-c\,\dot\sigma\,\omega_{rel,A,1}$ 按第 2.4 节的 $\dot\sigma$ 一般不定号，元件在耦合转动下可以向系统做正功。

## 3. 计算实现

### 3.1 相对运动的读取

每次求值，元件先按共用篇第 5.1 节得到一份相对运动。本族读取其中的 $R_{AC}$、$\boldsymbol\omega_{rel,A}$，以及写入载荷所需的 $R_{IA}$ 与两端原点的刚体固定点；$\mathbf d_A$、$\mathbf u_A$、$\mathbf d_I$ 不被读取。$\sigma$ 取 $R_{AC}$ 零基索引 `(2, 1)` 的元，$\omega_{rel,A,1}$ 取 $\boldsymbol\omega_{rel,A}$ 的第一分量。二者都由当前 $(q,v)$ 代数确定，元件没有内部状态，不占用 $z$ 块。

### 3.2 本构求值与转入惯性系

力矩按 $m=k\,\sigma+c\,\omega_{rel,A,1}$ 求值，再按 $\mathbf m_I=R_{IA}\,(m,0,0)^{\mathsf T}$ 转入 I。这一步没有分支：$m$ 是 $R_{AC}$ 的元与 $\boldsymbol\omega_{rel,A}$ 的分量的同一个多项式，在所有相对姿态上以同一表达式求值，包括 $\sigma$ 接近 $\pm1$ 的姿态。第 4 节所述的度量饱和是 $\sigma$ 作为矩阵元的自身性质，实现中不存在任何截断或换支。

### 3.3 纯力偶对的写出

以参考端原点为正端、对端原点为负端，写入两条在 I 中表达的载荷：

$$
\left(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf m_I,\ \mathbf 0\right),
\qquad
\left(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ -\mathbf m_I,\ \mathbf 0\right).
$$

多体层按共用篇第 3.5 节把每条载荷移到所在刚体的原点；力为零，力臂项为零，力矩原样并入两刚体的合力矩。载荷条目中记录的两端原点因此只是形式上的作用点，换成各自刚体上的任何其他固定点，结果不变。

## 4. 数学性质与适用条件

**储能与耗散。** 纯滚转下 $\mathcal V=k(1-\cos\phi)\ge0$、$\mathcal D=c\,\dot\phi^2\ge0$，$k\ge0$、$c\ge0$ 时元件是无源的。$k=0$ 时它是纯侧滚阻尼器，$c=0$ 时是纯侧滚弹簧，二者同时为零时元件对运动没有作用。

**度量饱和与恢复力矩的上界。** $|\sigma|\le1$ 对任何相对姿态成立，故弹性力矩的大小不超过 $k$，与相对转角的大小无关。纯滚转下切线刚度 $k\cos\phi$ 在 $|\phi|$ 从 0 增至 $\pi/2$ 时单调减到零，$|\phi|>\pi/2$ 时为负：元件是软化弹簧，越过 $\pm\pi/2$ 后恢复力矩随转角增大而减小。$\mathcal V$ 在 $\phi=\pi$ 处取极大值，那里弹性力矩为零、切线刚度为 $-k$，是弹性部分的不稳定平衡。$\phi\mapsto\sin\phi$ 在 $(-\pi,\pi]$ 上不是单射，$\phi$ 与 $\pi-\phi$ 给出同一个 $\sigma$，弹性部分无法区分这两种位形。阻尼项以 $\omega_{rel,A,1}$ 为输入，不受饱和影响。

**适用范围。** 作为恢复元件，本族只在 $|\phi|<\pi/2$ 内有正的切线刚度；作为线性扭转弹簧的替身，只在 $|\phi|\ll1$ 内成立，恢复力矩相对线性律的偏差为 $1-\sin\phi/\phi=\tfrac16\phi^2+O(\phi^4)$。需要在大相对转角下保持线性或硬化恢复特性的场合不属于本族。

**弹性部分在整个旋转群上没有势函数。** 若存在 $\mathcal V(R_{AC})$ 使 $-k\,\sigma\,\omega_{rel,A,1}=-\dot{\mathcal V}$ 对一切 $\boldsymbol\omega_{rel,A}$ 成立，则 $\mathcal V$ 沿绕 A 第二、三轴的无穷小转动的方向导数为零；绕第二、三轴的无穷小转动的交换子是绕第一轴的无穷小转动，故 $\mathcal V$ 沿第一轴的方向导数也为零，$\mathcal V$ 为常数，与 $k\ne0$ 矛盾。这一论证是局部的：在 $\sigma\ne0$ 的任何开邻域内弹性力矩场都不可积，沿一般的相对姿态闭合路径弹性部分可以做非零净功。第 2.6 节的 $\mathcal V=k(1-\cos\phi)$ 是限制到纯滚转曲线上的势能，不延伸为整个旋转群上的势函数。用第 2.4 节的 Z-Y-X 对照写出偏差，

$$
-k\,\sigma\,\omega_{rel,A,1}+\frac{d}{dt}\mathcal V
=-k\sin\phi\left[\dot\phi\left(\cos^{2}\theta\cos\psi-1\right)-\dot\theta\cos\theta\sin\psi\right],
$$

方括号中两项在 $\theta=\psi=0$ 时恒为零；$\theta$、$\psi$ 小时，第一项是 $(\theta^2+\psi^2)\,\dot\phi$ 量级，第二项是 $\psi\,\dot\theta$ 量级。相对转动以绕 $\mathbf a_1$ 的滚转为主时，弹性部分与 $\mathcal V$ 的偏差随其余两个角一起消失。

**无角坐标奇点。** $\sigma$ 是 $R_{AC}$ 的一个矩阵元，$\omega_{rel,A,1}$ 是角速度的一个分量，二者对任何相对姿态都有定义且光滑；本族没有角坐标提取带来的奇点或分支，是共用篇第 6 节所述以 $\boldsymbol\omega_{rel,A}$ 分量为速率输入的族没有奇点这一性质的实例。

**对相对平动不敏感。** $\mathbf d_A$ 与 $\mathbf u_A$ 不进入本构；两端原点在各自刚体上的位置改变而姿态不变时，元件及其载荷都不变。

**参考端的选择是本构的一部分。** 互换两端后，度量变为 $(R_{CA})_{32}=(R_{AC})_{23}=\mathbf a_2\cdot R_{IC}\mathbf e_3$，力偶轴变为 $R_{IC}\mathbf e_1$。纯滚转时 $(R_{AC})_{23}=-\sin\phi$ 且 $R_{IC}\mathbf e_1=\mathbf a_1$，互换后的元件给出同一对载荷；含滚转以外成分时二者不同，两个元件不等价。

## 5. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 本族类型、两端与常数 $k$、$c$ | `RollSpringDamperCouple`，见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 相对运动 $R_{AC}$、$\boldsymbol\omega_{rel,A}$、$R_{IA}$ 的一次求取 | `VehicleForcePlan::CalcAppliedForces` 内的 `CalcRelativeMotion`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 相对角速度 $\boldsymbol\omega_{rel,A}$ 的契约 | `MultibodyModel::CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| $\sigma$、$\omega_{rel,A,1}$、$m$ 的求值与 $\mathbf m_I$ 的形成 | `VehicleForcePlan::CalcAppliedForces` 的侧滚族分支，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 纯力偶对的写出 | `internal::EmitCoupleWrenchPair`，见 [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h) |
| 载荷条目与刚体固定点 | `AppliedBodyWrench`、`BodyFixedPoint`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| 载荷换点到刚体原点并进入前向动力学 | `MultibodyModel::CalcStateTimeDerivatives`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
