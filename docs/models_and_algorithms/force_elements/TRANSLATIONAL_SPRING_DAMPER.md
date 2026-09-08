[English](TRANSLATIONAL_SPRING_DAMPER.en.md)

# 三向平动弹簧—阻尼力元

本篇给出三向平动弹簧—阻尼力元的本构律、成对扳手、功率恒等式、储能与耗散，以及对角本构在空间表达中的姿态依赖。该族在参考端坐标系的三个轴上各放一对并联的线性弹簧与黏性阻尼，再加一个逐实例给定的名义力，把参考端所受的力写成相对位置与相对速度的仿射函数，并按端点力对加参考端支承矩的方式施加到两个刚体上。两端的相对运动、三种扳手施加方式与功率恒等式已在[力元连接运动学与空间扳手](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)（下称共用篇）中推导，本篇直接引用，只推导本族自己的部分。类型定义见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) 的 `TranslationalSpringDamper`；本构求值见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) 的 `VehicleForcePlan::CalcAppliedForces`，成对扳手由同一文件中的 `EmitTranslationalWrenchPair` 写出。

## 1. 对象与记号

### 1.1 力元、两端与参考端

力元连接参考端坐标系 A 与对端坐标系 C，二者分别固定在两个不同的刚体 $\mathcal A$ 与 $\mathcal C$ 上，见共用篇第 1.1 节。本族的全部本构常数——三轴刚度 $\mathbf k$、三轴阻尼 $\mathbf c$ 与名义力 $\mathbf f_0$——都在 A 中陈述，得到的力也是作用于参考端的力；对端所受的力是它的相反数。变形度量是共用篇第 2.1 节的相对位置 $\mathbf d_A$，速度输入是共用篇第 2.3 节含运输项的相对速度 $\mathbf u_A=\dot{\mathbf d}_A$。本族不读取相对姿态 $R_{AC}$ 与相对角速度 $\boldsymbol\omega_{rel,A}$，也不携带内部状态。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| A、C、$\mathcal A$、$\mathcal C$、I | 参考端与对端坐标系、承载它们的两个刚体、惯性系 | 共用篇 §1.1 |
| $\mathbf p_A$、$\mathbf p_C$ | 两端原点 $A_o$、$C_o$ 在 I 中的位置 | 共用篇 §1.2 |
| $\mathbf d_A$、$\mathbf d_I$ | 相对位置 $\mathbf p_C-\mathbf p_A$，在 A、I 中表达 | 共用篇 §2.1 |
| $\mathbf u_A$、$\mathbf u_I$ | 对端原点相对参考端（相对刚体 $\mathcal A$）的速度，$\mathbf u_A=\dot{\mathbf d}_A$ | 共用篇 §2.3 |
| $R_{IA}$、$R_{AC}$ | 参考端在 I 中的姿态，对端在参考端中的相对姿态 | 基座 §4.1 |
| $\mathbf v_{Ao}$、$\mathbf v_{Co}$、$\boldsymbol\omega_A$、$\boldsymbol\omega_C$ | 两端原点的速度与两刚体的角速度，在 I 中表达 | 共用篇 §1.3 |
| $\mathbf f_A$、$\mathbf f_I$ | 参考端所受的力，在 A、I 中表达 | 共用篇 §1.3 |
| $\mathcal W_Q^E[\mathcal A]$ | 作用于刚体 $\mathcal A$、关于 Q 取矩、在 E 中表达的扳手 | 共用篇 §1.3 |
| $\mathbf r_{A_o}^{\mathcal A}$、$\mathbf r_{C_o}^{\mathcal C}$ | 两端原点在各自刚体体坐标系中的坐标 | 共用篇 §3.1 |
| $\mathcal P$ | 力元向两刚体输送的总功率 | 共用篇 §1.2 |
| $\mathbf a\circ\mathbf b$ | 逐分量乘积 | 共用篇 §1.2 |
| $\operatorname{skew}(\mathbf w)$ | 反对称矩阵 | 共用篇 §1.2 |
| $\mathbf e_j$ | 第 $j$ 个标准基向量 | 基座 §4.1 |
| $\mathbf k=(k_1,k_2,k_3)$、$\mathbf c=(c_1,c_2,c_3)$ | A 三个轴上的刚度与阻尼 | 基座 §5.3 的向量形式 |
| $\mathbf f_0$ | 名义力：逐实例给定、在 A 中表达、作用于参考端的常向量 | 本篇新增 |
| $\mathbf f_A^{\mathrm{el}}$、$\mathbf f_A^{\mathrm{d}}$ | $\mathbf f_A$ 的弹性部分 $\mathbf k\circ\mathbf d_A+\mathbf f_0$ 与阻尼部分 $\mathbf c\circ\mathbf u_A$ | 本篇新增 |
| $\mathbf d_{0,A}$ | 各轴刚度为正时弹性部分的零力位移，在 A 中表达 | 本篇新增 |
| $\mathcal V$、$\mathcal D$ | 储能函数与耗散率 | 共用篇 §1.2 |
| $\delta\mathbf p_A$、$\delta\boldsymbol\theta_A$、$\delta\mathbf p_C$ | 刚体 $\mathcal A$ 在 $A_o$ 处的虚位移与虚转动、刚体 $\mathcal C$ 在 $C_o$ 处的虚位移，在 I 中表达 | 本篇新增 |
| $\operatorname{diag}(\mathbf k)$ | 以 $\mathbf k$ 为对角线的矩阵，$\operatorname{diag}(\mathbf k)\mathbf x=\mathbf k\circ\mathbf x$ | 本篇新增 |

### 1.3 与共用篇及基座的关系

本篇沿用共用篇第 1.3 节的全部约定与符号避让，并沿用它对[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)（下称基座）的引用方式：$R_{AB}$ 把 B 中分量变到 A 中，粗体 $\mathbf d$ 是相对位置向量，$\mathbf u$ 是相对速度，功率写 $\mathcal P$，未标作用端的 $\mathbf f$ 指参考端所受载荷，末下标为表达系。本篇新增的符号已在上表标出，其中三处需要说明：名义力 $\mathbf f_0$ 的下标 0 不是表达系，它始终在 A 中表达；上标 $\mathrm{el}$、$\mathrm d$ 只区分弹性与阻尼部分，不改变末下标的含义；零力位移 $\mathbf d_{0,A}$ 采用与共用篇 $\mathbf u_{m,A}$ 相同的"限定词在前、表达系在后"双下标写法。本族的刚度与阻尼按基座第 5.3 节写成小写向量 $\mathbf k$、$\mathbf c$，需要矩阵形式时写 $\operatorname{diag}(\mathbf k)$、$\operatorname{diag}(\mathbf c)$，不用大写 $K$、$C$，那两个字母留给系统级矩阵。

## 2. 模型与推导

### 2.1 变形度量与速度输入

按共用篇第 2.1 节与第 2.3 节，

$$
\mathbf d_A=R_{IA}^{\mathsf T}\left(\mathbf p_C-\mathbf p_A\right),
\qquad
\mathbf u_A=R_{IA}^{\mathsf T}\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)=\dot{\mathbf d}_A.
$$

$\mathbf d_A$ 是两端原点之差在 A 中的分量，不含自然长度：两原点重合时 $\mathbf d_A=\mathbf 0$。$\mathbf u_A$ 含运输项 $-\boldsymbol\omega_A\times\mathbf d_I$，是 $C_o$ 相对刚体 $\mathcal A$ 的速度而非相对点 $A_o$ 的速度，它恰是 $\mathbf d_A$ 的时间导数。本族的三个弹簧—阻尼对分别只看 $\mathbf d_A$ 与 $\mathbf u_A$ 的一个分量；变形度量与速度输入互为导数这一点，是第 2.6 节储能全导数的前提。

### 2.2 并联本构律

弹簧与阻尼并联，意味着二者承受同一变形 $\mathbf d_A$ 与同一速度 $\mathbf u_A$，各自的力相加。参考端所受的力为

$$
\mathbf f_A=\mathbf k\circ\mathbf d_A+\mathbf c\circ\mathbf u_A+\mathbf f_0,
\qquad
f_{A,i}=k_i\,d_{A,i}+c_i\,u_{A,i}+f_{0,i},
\quad i=1,2,3.
$$

三个轴互不耦合：第 $i$ 轴的力只依赖第 $i$ 轴的位移与速度。本篇把它拆成弹性部分与阻尼部分，

$$
\mathbf f_A=\mathbf f_A^{\mathrm{el}}+\mathbf f_A^{\mathrm{d}},
\qquad
\mathbf f_A^{\mathrm{el}}=\mathbf k\circ\mathbf d_A+\mathbf f_0,
\qquad
\mathbf f_A^{\mathrm{d}}=\mathbf c\circ\mathbf u_A.
$$

这条律是 $(\mathbf d_A,\mathbf u_A)$ 的仿射函数，处处光滑，没有分支，也没有需要积分的内部量，力由当前相对运动代数地决定。与之相对，[串联弹簧—黏性阻尼力元](SERIES_SPRING_VISCOUS_DAMPER.md)中弹簧与阻尼承受同一个力、变形相加，力因此成为状态。

**正号含义。** $\mathbf f_A$ 是参考端所受的力，$f_{A,i}>0$ 的定义就是参考端沿 A 的第 $i$ 轴正向受力。三项分开看：弹性项 $k_i\,d_{A,i}$（$k_i>0$）在对端原点沿第 $i$ 轴正向离开参考端时为正，参考端被沿 $+\mathbf e_i$ 拉向对端，对端同时受到 $-k_i d_{A,i}\,\mathbf e_i$ 被拉回，元件像一根被拉长的弹簧把两端拉拢；阻尼项 $c_i\,u_{A,i}$ 在对端沿第 $i$ 轴正向相对刚体 $\mathcal A$ 远离时为正，阻止分离；名义力 $f_{0,i}$ 是常量，与运动无关。三项之和的符号由三者共同决定，不能由 $d_{A,i}$ 的符号单独判定。对端所受的力恒为 $-\mathbf f_A$，见第 2.4 节。

### 2.3 名义力的数学角色

变形度量 $\mathbf d_A$ 是原点之差，运动学层不提供自然长度或安装位形（共用篇第 6 节）。若本构只含 $\mathbf k\circ\mathbf d_A$ 且三轴刚度都为正，元件的无力位形就只能是两原点重合的位形；但一个元件在两原点重合的位形下往往承载着力，其弹簧部分的无力位形在别处。名义力 $\mathbf f_0$ 正是为此设置的常向量：它是弹性部分 $\mathbf f_A^{\mathrm{el}}$ 在 $\mathbf d_A=\mathbf 0$ 处的值，即两原点重合且相对静止时参考端所受的力，也是本构律里唯一能把无力位形移离原点重合处的项。

在刚度为正的轴上，它等价于一个零力位移：

$$
d_{0,A,i}=-\frac{f_{0,i}}{k_i},
\qquad
k_i\,d_{A,i}+f_{0,i}=k_i\left(d_{A,i}-d_{0,A,i}\right),
\qquad k_i>0.
$$

三轴刚度都为正时，$\mathbf d_{0,A}=-\operatorname{diag}(\mathbf k)^{-1}\mathbf f_0$ 是弹性部分唯一的无力位形，$\mathbf f_0$ 吸收了无自然长度带来的零位移偏置。在刚度为零的轴上，$f_{0,i}$ 是沿 A 的第 $i$ 轴的常值力，没有对应的零力位移。两种情形下 $\mathbf f_0$ 都是逐实例给定的常量：它不是状态的函数，不被积分，在 A 中表达并作用于参考端，因此像 $\mathbf k$、$\mathbf c$ 一样随参考端的姿态转动（第 2.7 节）。类型中没有另存自然长度，本构律中除 $\mathbf f_0$ 之外没有其他零位移量。

### 2.4 成对扳手

本族按共用篇第 3.2 节的第一种方式施加载荷：两端原点各受一个等大反向的力，参考端另受支承矩。把 $\mathbf f_A$ 转入 I 后，

$$
\mathbf f_I=R_{IA}\mathbf f_A,
\qquad
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

支承矩关于 $A_o$ 取矩、施加在刚体 $\mathcal A$ 上；旋转保持叉积，$\mathbf d_I\times\mathbf f_I=R_{IA}\left(\mathbf d_A\times\mathbf f_A\right)$，所以它在 A 中就是 $\mathbf d_A\times\mathbf f_A$。这一对扳手的合力与关于任意点的合力矩为零（共用篇第 3.2 节）。支承矩为零当且仅当 $\mathbf f_A$ 平行于 $\mathbf d_A$；三轴刚度不相等、名义力不为零或 $\mathbf u_A$ 有横向分量的一般情形下，$\mathbf f_A$ 不沿两原点连线，支承矩不为零。它不是配平补丁：其必要性由第 2.5 节的功率恒等式给出，其来源由第 2.6 节的梯度结构给出。

### 2.5 功率原则的验证

共用篇第 4.5 节要求每一族把速度输入与施力位置成对选定，使本构功率恰等于端点功率。本族取该节表格的第一行：速度输入 $\mathbf u_A$，施力方式为两端原点力对加参考端支承矩。按共用篇第 4.1 节单个刚体上扳手的功率公式，上面两条扳手向两刚体输送的总功率为

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v_{Ao}+\left(\mathbf d_I\times\mathbf f_I\right)\cdot\boldsymbol\omega_A-\mathbf f_I\cdot\mathbf v_{Co}
=-\mathbf f_I\cdot\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)
=-\mathbf f_A\cdot\mathbf u_A.
$$

第二个等号用了共用篇第 4.1 节的混合积恒等式 $\boldsymbol\omega\cdot(\mathbf d\times\mathbf f)=\mathbf f\cdot(\boldsymbol\omega\times\mathbf d)$，第三个等号用了第 2.1 节 $\mathbf u_A$ 的定义与点积的旋转不变性。右端是本构律给出的力与本构律消费的速度的负内积，正是本族的本构功率：扳手对所共轭的速度，恰是本构律读取的速度。本族因此满足组织原则。若本构改用不含运输项的 $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ 而扳手不变，或本构不变而去掉支承矩，两种功率将相差支承矩的功率 $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$，见共用篇第 4.2 节。

### 2.6 储能、耗散与被动性

把本构律代入 $\mathcal P=-\mathbf f_A\cdot\mathbf u_A$，

$$
\mathcal P=-\mathbf u_A\cdot\left(\mathbf k\circ\mathbf d_A\right)-\mathbf f_0\cdot\mathbf u_A-\mathbf u_A\cdot\left(\mathbf c\circ\mathbf u_A\right).
$$

定义储能函数与耗散率

$$
\mathcal V(\mathbf d_A)=\tfrac12\,\mathbf d_A\cdot\left(\mathbf k\circ\mathbf d_A\right)+\mathbf f_0\cdot\mathbf d_A
=\sum_{i=1}^{3}\left(\tfrac12\,k_i\,d_{A,i}^2+f_{0,i}\,d_{A,i}\right),
\qquad
\mathcal D(\mathbf u_A)=\mathbf u_A\cdot\left(\mathbf c\circ\mathbf u_A\right)=\sum_{i=1}^{3}c_i\,u_{A,i}^2.
$$

因为 $\mathbf u_A=\dot{\mathbf d}_A$ 且 $\mathbf k$、$\mathbf f_0$ 为常量，$\dot{\mathcal V}=\mathbf u_A\cdot(\mathbf k\circ\mathbf d_A)+\mathbf f_0\cdot\mathbf u_A$，于是

$$
\mathcal P=-\dot{\mathcal V}-\mathcal D,
\qquad
\int_{t_0}^{t_1}\mathcal P\,dt
=\mathcal V\!\left(\mathbf d_A(t_0)\right)-\mathcal V\!\left(\mathbf d_A(t_1)\right)-\int_{t_0}^{t_1}\mathcal D\,dt.
$$

元件输送给两刚体的功等于储能的减少量减去耗散；两刚体的机械能与 $\mathcal V$ 之和以 $-\mathcal D$ 的速率变化。弹性部分的功率是 $\mathcal V$ 的全导数，这一结论依赖 $\mathbf u_A$ 恰为 $\mathbf d_A$ 的导数，也就是依赖第 2.5 节的成对选定。

**耗散不等式与被动性。** 上式是能量恒等式，对任何参数成立。当 $k_i\ge0$、$c_i\ge0$（$i=1,2,3$）时，$\mathcal D\ge0$ 对一切 $\mathbf u_A$ 成立，从而 $\mathcal P\le-\dot{\mathcal V}$：元件向两刚体输送的功率不超过其储能的释放速率，这是以 $\mathcal V$ 为储存函数的耗散不等式。本构对角，两个二次型也对角，所以分量非负与二次型半正定是同一个条件。被动性比耗散不等式多要求一件事——储存函数有下界，否则可从元件提取的功没有上界；在无界的位移域上这要求 $k_i=0$ 的轴上 $f_{0,i}=0$。某个 $k_i<0$ 使 $\mathcal V$ 沿该轴无下界，元件可以沿该轴输出无界的功；某个 $c_i<0$ 使 $\mathcal D$ 可取负值，元件从相对运动中产生能量。三轴刚度都为正时配方得

$$
\mathcal V(\mathbf d_A)=\tfrac12\sum_{i=1}^{3}k_i\left(d_{A,i}-d_{0,A,i}\right)^2-\tfrac12\sum_{i=1}^{3}\frac{f_{0,i}^2}{k_i},
$$

$\mathcal V$ 在零力位移 $\mathbf d_{0,A}$ 处取最小值，常数项不影响任何力，储存函数有下界。在 $k_i=0$ 而 $f_{0,i}\neq0$ 的轴上，$\mathcal V$ 沿该轴是线性函数、无下界：常值力仍是有势力，能量恒等式与耗散不等式照常成立，但可提取的功没有上界，元件在该轴上不是被动的。满足 $k_i=0\Rightarrow f_{0,i}=0$ 时，对 $\mathcal V$ 加常数可取非负储存函数 $\tfrac12\sum_{k_i>0}k_i(d_{A,i}-d_{0,A,i})^2$，元件被动。

**储能的梯度结构。** 弹性部分的扳手对恰是 $\mathcal V$ 的负梯度，可以直接验证。把 $\mathcal V$ 看成两刚体位形的函数，给刚体 $\mathcal A$ 在 $A_o$ 处虚位移 $\delta\mathbf p_A$、虚转动 $\delta\boldsymbol\theta_A$，给刚体 $\mathcal C$ 在 $C_o$ 处虚位移 $\delta\mathbf p_C$，均在 I 中表达。由 $\delta R_{IA}=\operatorname{skew}(\delta\boldsymbol\theta_A)R_{IA}$ 得

$$
\delta\mathbf d_A=R_{IA}^{\mathsf T}\left(\delta\mathbf p_C-\delta\mathbf p_A-\delta\boldsymbol\theta_A\times\mathbf d_I\right),
$$

记 $\mathbf f_I^{\mathrm{el}}=R_{IA}\mathbf f_A^{\mathrm{el}}$，再用一次混合积恒等式，

$$
-\delta\mathcal V=-\mathbf f_A^{\mathrm{el}}\cdot\delta\mathbf d_A
=\mathbf f_I^{\mathrm{el}}\cdot\delta\mathbf p_A+\left(\mathbf d_I\times\mathbf f_I^{\mathrm{el}}\right)\cdot\delta\boldsymbol\theta_A-\mathbf f_I^{\mathrm{el}}\cdot\delta\mathbf p_C.
$$

三个虚位移的系数正是第 2.4 节两条扳手的弹性部分：$\mathcal A$ 在 $A_o$ 处受力 $\mathbf f_I^{\mathrm{el}}$ 与支承矩 $\mathbf d_I\times\mathbf f_I^{\mathrm{el}}$，$\mathcal C$ 在 $C_o$ 处受力 $-\mathbf f_I^{\mathrm{el}}$ 而无力矩。支承矩是与刚体 $\mathcal A$ 的姿态共轭的广义力，它出现是因为 $\mathcal V$ 通过 $\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I$ 依赖 $\mathcal A$ 的姿态；$\mathcal V$ 不依赖 $\mathcal C$ 的姿态，所以 $\mathcal C$ 端没有力矩。阻尼部分不是梯度，但以同样的力臂结构施加，功率为 $-\mathcal D$。

### 2.7 对角本构在空间表达中的姿态依赖

把 $\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I$、$\mathbf u_A=R_{IA}^{\mathsf T}\mathbf u_I$ 代入本构并转入 I，

$$
\mathbf f_I=R_{IA}\operatorname{diag}(\mathbf k)R_{IA}^{\mathsf T}\,\mathbf d_I+R_{IA}\operatorname{diag}(\mathbf c)R_{IA}^{\mathsf T}\,\mathbf u_I+R_{IA}\mathbf f_0.
$$

两个相似变换后的矩阵是对称矩阵，特征值为 $k_i$、$c_i$，特征向量为 $R_{IA}\mathbf e_i$，即 A 的三个轴在 I 中的方向。本构在 A 中是常系数对角的，在 I 中却随 $R_{IA}$ 变化：元件的刚度主轴与阻尼主轴固定在刚体 $\mathcal A$ 上并随之转动。这带来三个后果。第一，$\mathbf d_I$ 不变而 $\mathcal A$ 转动时力会改变，储能 $\mathcal V$ 因此依赖 $\mathcal A$ 的姿态，这正是第 2.6 节支承矩的来源。第二，名义力也随 A 转动：$R_{IA}\mathbf f_0$ 是固定在刚体 $\mathcal A$ 上的随动力，不是空间固定方向的力。第三，参考端的选择是本构的一部分：把两端互换后相对位置反向并改在 C 中表达，刚度矩阵变成 $R_{CA}\operatorname{diag}(\mathbf k)R_{CA}^{\mathsf T}$，$R_{AC}$ 不把坐标轴映到坐标轴时它一般不再对角（三个 $k_i$ 有重值时可能例外）；速度输入中的运输项由 $\boldsymbol\omega_A$ 改为由 $\boldsymbol\omega_C$ 形成，支承矩也移到另一刚体。只有各向同性的律不受矩阵这一层的影响，见第 2.8 节；互换两端的一般讨论见共用篇第 6 节。

### 2.8 特例

**纯弹簧**（$\mathbf c=\mathbf 0$）。$\mathbf f_A=\mathbf k\circ\mathbf d_A+\mathbf f_0$，$\mathcal P=-\dot{\mathcal V}$，元件保守，其扳手对就是 $-\delta\mathcal V$ 的系数。

**纯阻尼**（$\mathbf k=\mathbf 0$、$\mathbf f_0=\mathbf 0$）。$\mathbf f_A=\mathbf c\circ\mathbf u_A$，$\mathcal P=-\mathcal D\le0$，没有储能。力仍在 A 中按轴陈述，仍随 $\mathcal A$ 的姿态转动，仍带支承矩 $\mathbf d_A\times(\mathbf c\circ\mathbf u_A)$；它不是沿两原点连线、只对连线长度变化率作出反应的一维阻尼器。

**单轴**（$\mathbf k=k\,\mathbf e_j$、$\mathbf c=c\,\mathbf e_j$、$\mathbf f_0=f_0\,\mathbf e_j$）。此时

$$
\mathbf f_A=\left(k\,d_{A,j}+c\,u_{A,j}+f_0\right)\mathbf e_j,
\qquad
\mathbf d_I\times\mathbf f_I=\left(k\,d_{A,j}+c\,u_{A,j}+f_0\right)R_{IA}\left(\mathbf d_A\times\mathbf e_j\right).
$$

力沿 A 的第 $j$ 轴而不沿两原点连线，支承矩在 $\mathbf d_A$ 平行于 $\mathbf e_j$ 或标量力为零时为零，一般非零。[串联弹簧—黏性阻尼力元](SERIES_SPRING_VISCOUS_DAMPER.md)与[奇对称饱和分段线性阻尼力元](SATURATED_PIECEWISE_LINEAR_DAMPER.md)以各自的标量力沿用这一载荷形式。

**各向同性**（$\mathbf k=k\begin{bmatrix}1&1&1\end{bmatrix}^{\mathsf T}$、$\mathbf c=c\begin{bmatrix}1&1&1\end{bmatrix}^{\mathsf T}$）。此时 $R_{IA}\operatorname{diag}(\mathbf k)R_{IA}^{\mathsf T}$ 是 $k$ 倍单位矩阵，

$$
\mathbf f_I=k\,\mathbf d_I+c\,\mathbf u_I+R_{IA}\mathbf f_0.
$$

弹簧与阻尼部分不再依赖 $\mathcal A$ 的姿态，名义力仍随 A 转动。再取 $c=0$、$\mathbf f_0=\mathbf 0$，$\mathbf f_I=k\,\mathbf d_I$ 平行于 $\mathbf d_I$，支承矩恒为零：这是本族唯一退化为中心力的情形，元件成为自然长度为零的线性两点弹簧，$\mathcal V=\tfrac12k\,\lVert\mathbf d_I\rVert^2$。$c\neq0$ 时 $\mathbf u_I$ 一般有垂直于 $\mathbf d_I$ 的分量，力不是中心力；并且 $\mathbf u_I$ 是相对刚体 $\mathcal A$ 的速度而非 $\dot{\mathbf d}_I$，各向同性消除了矩阵的姿态依赖，却没有消除运输项对参考端选择的依赖。

## 3. 计算实现

### 3.1 相对运动的读取

每次求值，先按共用篇第 5.1 节由两端坐标系得到一份相对运动。本族从中读取 $\mathbf d_A$ 与 $\mathbf u_A$ 作为本构输入：$\mathbf u_A$ 直接来自多体层"C 相对 A、在 A 中表达"的空间速度查询，其平动分量按契约是 $C_o$ 在 A 中度量的速度，运输项已含在内，本族不再自行构造。另读取写入载荷所需的 $R_{IA}$、$\mathbf d_I$ 与两端原点的刚体固定点 $(\mathcal A,\mathbf r_{A_o}^{\mathcal A})$、$(\mathcal C,\mathbf r_{C_o}^{\mathcal C})$。相对姿态与相对角速度不被本族使用。

### 3.2 本构求值

$\mathbf f_A$ 由两个逐分量乘积与一次加法得到：$\mathbf k\circ\mathbf d_A$ 加 $\mathbf c\circ\mathbf u_A$，再加本实例的 $\mathbf f_0$。$\mathbf f_0$ 不在力元类型中，而是与状态一同作为求值输入给出：每个实例占三个分量，在 A 中表达、作用于参考端，求值时只读，不被积分。求值没有任何分支，也不改写任何历史量。本族不向连续状态的 $z$ 块贡献分量，$z$ 块只由[串联弹簧—黏性阻尼力元](SERIES_SPRING_VISCOUS_DAMPER.md)构成，见[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 1.1 节。

### 3.3 扳手写出

先算 $\mathbf f_I=R_{IA}\mathbf f_A$，再按共用篇第 5.2 节的端点力对路径写两条载荷：给 $\mathcal A$ 的 $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ 与给 $\mathcal C$ 的 $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$，表达系都是 I。支承矩在 I 中由 $\mathbf d_I$ 与 $\mathbf f_I$ 叉乘得到。多体层随后把两条载荷各自换点到所在刚体的原点，再并入前向动力学，见共用篇第 3.5 节与第 5.3 节。

## 4. 数学性质与适用条件

- **代数、无状态、光滑。** 力是 $(\mathbf d_A,\mathbf u_A)$ 的仿射函数，由当前 $(q,v)$ 唯一决定，不含历史量，处处可微；本族不改变连续状态的维数。
- **耗散不等式与被动性。** $\mathbf k$ 与 $\mathbf c$ 的每个分量非负且有限、$\mathbf f_0$ 有限，是本族的适用条件。在此条件下 $\mathcal P=-\dot{\mathcal V}-\mathcal D$ 且 $\mathcal D\ge0$，耗散不等式成立；被动性还要求储存函数有下界，即 $k_i=0$ 的轴上 $f_{0,i}=0$。三轴刚度为正时 $\mathcal V$ 有下界，存在唯一的无力位形 $\mathbf d_{0,A}$。
- **零位移是正则位形。** 本构写在 $\mathbf d_A$ 的分量上而不是长度上，不含任何单位方向向量，两原点重合处的力 $\mathbf c\circ\mathbf u_A+\mathbf f_0$ 有定义，支承矩在该处为零。两原点可以安放在重合位置，由 $\mathbf f_0$ 承载该位形下的力。
- **一般不是中心力。** 力沿 A 的坐标轴分解，不沿两原点连线，支承矩一般不为零；只有各向同性、零阻尼、零名义力的特例才是沿连线的中心力。把本族当作沿连线的一维元件使用，会得到与预期不同的横向力与力矩。
- **主轴随参考端转动。** 刚度与阻尼主轴、名义力方向都固定在刚体 $\mathcal A$ 上。参考端的选择与参考端坐标系的姿态都是本构的一部分，互换两端或改变 A 的姿态一般得到不同的元件。各向同性只消除系数矩阵对姿态的依赖：名义力为零时，各向同性弹性部分是中心力、支承矩为零、对互换两端不变；各向同性阻尼部分仍以含运输项的 $\mathbf u_A$ 为输入，互换两端后运输项由 $\boldsymbol\omega_C$ 形成，载荷一般不同（第 2.8 节）。
- **本构线性不等于系统贡献线性。** 在 A 中，$\partial\mathbf f_A/\partial\mathbf d_A=\operatorname{diag}(\mathbf k)$ 与 $\partial\mathbf f_A/\partial\mathbf u_A=\operatorname{diag}(\mathbf c)$ 是常矩阵；但 $\mathbf d_A$、$\mathbf u_A$ 通过 $R_{IA}$ 依赖 $\mathcal A$ 的姿态，支承矩 $\mathbf d\times\mathbf f$ 又随位形变化，所以本族对系统切线刚度与切线阻尼的贡献依赖位形，含几何刚度项。
- **名义力的作用域。** $\mathbf f_0$ 是常量，只在刚度为正的轴上等价于零力位移；在刚度为零且 $f_{0,i}\neq0$ 的轴上它是随 A 转动的常值力，其势能沿该轴无下界。
- **两端必须在不同刚体上。** 见共用篇第 6 节；两端同在一个刚体时 $\mathbf u_A=\mathbf 0$、$\mathbf d_A$ 为常量，成对扳手落在同一刚体上而无净效应。
- **适用对象。** 本族描述三个主方向固定在参考端刚体上、三个分量彼此独立作用的平动弹簧—阻尼，适合刚度主轴随一个刚体转动的连接；需要沿连线作用的一维元件时，只有各向同性纯弹簧特例与之相符。

## 5. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 力元类型：两端与三轴刚度、阻尼 | `TranslationalSpringDamper`，见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 求值入口与 $\mathbf f_0$ 的输入契约（每实例三个分量，在 A 中表达、作用于参考端） | `VehicleForcePlan::CalcAppliedForces`，见 [`vehicle_force_plan.h`](../../../libs/forces/include/orvd/forces/vehicle_force_plan.h) |
| 相对运动 $\mathbf d_A$、$\mathbf u_A$、$\mathbf d_I$、$R_{IA}$ 的一次求取 | `VehicleForcePlan::CalcAppliedForces` 内的 `CalcRelativeMotion`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 本构 $\mathbf f_A=\mathbf k\circ\mathbf d_A+\mathbf c\circ\mathbf u_A+\mathbf f_0$ 的求值 | `VehicleForcePlan::CalcAppliedForces` 的平动族分支，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 端点力对与参考端支承矩的写出 | `VehicleForcePlan::CalcAppliedForces` 内的 `EmitTranslationalWrenchPair`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 名义力 $\mathbf f_0$ 的逐实例陈述 | `SystemInstance::SetNominalForce`，见 [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| 相对空间速度的契约，平动分量含运输项 | `MultibodyModel::CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 载荷条目与刚体固定点 | `AppliedBodyWrench`、`BodyFixedPoint`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| 载荷换点到刚体原点并进入前向动力学 | `MultibodyModel::CalcStateTimeDerivatives`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
