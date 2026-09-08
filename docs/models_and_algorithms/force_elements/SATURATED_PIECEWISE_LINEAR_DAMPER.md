[English](SATURATED_PIECEWISE_LINEAR_DAMPER.en.md)

# 奇对称饱和分段线性阻尼力元

本篇是[力元连接运动学与空间扳手](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)（下称共用篇）之下的一个本构族。该族是沿参考端坐标系一根固定轴作用的无记忆阻尼器：以有限个节点在非负速度半轴上定义一条从原点出发、节点间线性插值、末节点外取常值的力曲线，再把它奇延拓到整个速度轴。本篇给出曲线与延拓的定义、定义域上三条要求的数学后果、轴向相对速度输入与成对扳手，并证明该族满足共用篇第 4.5 节的功率原则。力元类型与节点类型见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) 的 `SaturatedPiecewiseLinearDamper`。曲线求值与载荷写出见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) 的 `EvaluateValidatedSaturatedDamperCurve` 与 `VehicleForcePlan::CalcAppliedForces`。

## 1. 对象与记号

### 1.1 力元、作用轴与两端

力元连接参考端坐标系 A 与对端坐标系 C，二者分别固定在刚体 $\mathcal A$ 与 $\mathcal C$ 上，含义与共用篇第 1.1 节相同。本族只沿 A 的一根坐标轴作用，记该轴的编号为 $j\in\{1,2,3\}$，依次为纵向、横向、竖向；作用轴是刚体 $\mathcal A$ 上的固定方向，一般不沿两端原点的连线。本构律是一维的：输入是一个标量速度，输出是一个标量力，二者都取在这根轴上。本族没有内部状态，也不读取任何位移或姿态量。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| A、C、$\mathcal A$、$\mathcal C$、I | 参考端与对端坐标系、承载它们的两个刚体、惯性系 | 共用篇第 1.1 节 |
| $\mathbf u_A$ | 对端原点相对参考端（相对刚体 $\mathcal A$）的速度，在 A 中表达，含运输项，$\mathbf u_A=\dot{\mathbf d}_A$ | 共用篇第 2.3 节 |
| $\mathbf d_I$、$R_{IA}$ | 两端原点的相对位置（I 中）与参考端在 I 中的姿态 | 共用篇第 2.1 节 |
| $\mathbf f_A$、$\mathbf f_I$ | 参考端所受的力，分别在 A 与 I 中表达 | 共用篇第 1.3 节 |
| $\mathcal W_Q^E[\mathcal A]$ | 作用于刚体 $\mathcal A$、关于 Q 取矩、在 E 中表达的扳手 | 共用篇第 1.3 节 |
| $\mathbf r_{A_o}^{\mathcal A}$、$\mathbf r_{C_o}^{\mathcal C}$ | 两端原点在各自刚体体坐标系中的坐标 | 共用篇第 3.1 节 |
| $\mathcal P$ | 力元向两刚体输送的总功率 | 共用篇第 1.2 节 |
| $\mathbf e_j$ | 第 $j$ 个标准基向量 | 基座第 4.1 节 |
| $j$ | 作用轴在 A 中的一基编号，$j\in\{1,2,3\}$ | 本篇 |
| $u$ | 轴向相对速度，$u=u_{A,j}=\mathbf e_j\cdot\mathbf u_A$ | 本篇 |
| $w$ | 轴向相对速率，$w=\lvert u\rvert\ge0$ | 本篇 |
| $n$ | 线性段数；节点共 $n+1$ 个 | 本篇 |
| $(u_i,g_i)$ | 第 $i$ 个节点的速度与力值，$i=0,\dots,n$ | 本篇 |
| $c_i$ | 第 $i$ 段的增量阻尼，即该段斜率，$i=1,\dots,n$ | 本篇 |
| $g$ | 非负速度半轴上的力曲线，$g:[0,\infty)\to[0,\infty)$ | 本篇 |
| $F$ | 奇延拓后的标量本构力 $F(u)$，参考端所受 | 本篇 |
| $\operatorname{sgn}$ | 符号函数 | 本篇 |
| $\lambda$ | 段内插值参数 | 本篇 |
| $\Phi$ | 耗散势 | 本篇 |

### 1.3 与共用篇及基座记号的关系

相对运动、扳手与功率的全部记号沿用共用篇第 1 节，包括扳手记号后以方括号标受力刚体的 $\mathcal W_Q^E[\mathcal A]$，以及写出扳手分量时以末下标标表达系的写法；共用篇已推过的相对速度、换点与功率恒等式本篇只引用不重推。旋转矩阵与标准基向量 $\mathbf e_j$ 依[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)第 4.1 节，向量分量按一基编号。

以下符号在本篇内另有含义，在此声明避让：$g$ 是非负速度半轴上的力曲线，不是基座第 2.1 节以站位为自变量的纵坡 $g(s)$，也不是第 2.7 节的竖向互穿函数 $g(Y)$；标量 $u$ 始终是 $\mathbf u_A$ 的第 $j$ 分量，不是基座第 2.2 节的超高 $u$，也不是[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 1.2 节的欧氏位移 $u$；$F$ 是本族的标量本构力，与轮轨接触链的法向力分量 $F_e$、$F_d$ 无关；不带下标的 $n$ 是线性段数，与状态维数 $n_x$ 等无关；$c_i$ 是第 $i$ 段的增量阻尼，按基座第 5.3 节用小写 $c$，下标标段号；标量 $w$ 是速率，与共用篇 $\operatorname{skew}(\mathbf w)$ 中的哑元向量无关。

## 2. 模型与推导

### 2.1 非负速度半轴上的曲线 $g$

曲线由 $n+1$ 个节点 $(u_i,g_i)$ 给出，$i=0,\dots,n$，其中 $u_i$ 是节点速度，$g_i$ 是节点力值。定义域上的三条要求是：节点速度严格递增；力值有限且非负；首节点为原点。写成

$$
0=u_0<u_1<\cdots<u_n,
\qquad
g_i\ge0\ (i=0,\dots,n),
\qquad
g_0=0,
\qquad n\ge1.
$$

$n\ge1$ 只是说至少有一段。每一段的增量阻尼，即该段的斜率，为

$$
c_i=\frac{g_i-g_{i-1}}{u_i-u_{i-1}},\qquad i=1,\dots,n.
$$

严格递增保证分母为正、每段斜率有限，且各段区间 $[u_{i-1},u_i]$ 只在端点相接。半轴曲线定义为

$$
g(w)=
\begin{cases}
g_{i-1}+c_i\left(w-u_{i-1}\right),& u_{i-1}\le w\le u_i,\quad i=1,\dots,n,\\
g_n,& w>u_n.
\end{cases}
$$

末节点外取常值 $g_n$ 是本族的定义：曲线在 $u_n$ 处饱和，不把最后一段的斜率延伸到定义节点之外。相邻两段在公共节点 $u_i$ 处的取值都等于 $g_i$，因此 $g$ 在 $[0,\infty)$ 上连续；由 $g_0=0$ 得 $g(0)=0$。每段内部 $g$ 是仿射函数，所以 $g$ 是连续的分段仿射函数，在节点 $u_i$（$1\le i\le n$）处的左、右导数分别为 $c_i$ 与 $c_{i+1}$，$i=n$ 时右导数为零。

### 2.2 奇延拓与原点处的取值

整个速度轴上的本构力是 $g$ 的奇延拓：

$$
F(u)=\operatorname{sgn}(u)\,g(|u|),
\qquad
F(0)=0,
\qquad
F(-u)=-F(u).
$$

其中 $\operatorname{sgn}(u)$ 在 $u>0$ 时取 $+1$、$u<0$ 时取 $-1$。原点处的取值不依赖于对 $\operatorname{sgn}(0)$ 的任何约定：无论把它取成 $+1$、$-1$ 还是 $0$，由 $g(0)=0$ 都得到 $F(0)=0$。奇性 $F(-u)=-F(u)$ 对 $u\ne0$ 由定义直接得到，对 $u=0$ 由 $F(0)=0$ 得到。负半轴上的曲线从不单独给出，因此本族的力律按构造对称。

$F$ 在原点连续，因为 $g$ 在 0 处连续且 $g(0)=0$；这正是"首节点为原点"这条要求的作用。若允许 $g_0>0$，奇延拓将在原点产生大小为 $2g_0$ 的跳跃，元件变成带静摩擦型跳变的元件，不属于本族。事实上在首段内

$$
F(u)=c_1\,u,\qquad |u|\le u_1,
$$

所以 $F$ 在原点不仅连续而且可导，$F'(0)=c_1$，原点不是折点。折点只出现在 $\pm u_i$（$1\le i\le n$）处，且仅当该节点两侧斜率不同：内部节点处为 $c_i\ne c_{i+1}$，饱和端 $u_n$ 处为 $c_n\ne0$。

### 2.3 定义域三条要求的数学后果

- **节点速度严格递增。** $g$ 是单值函数，每段斜率有限；曲线没有竖直段，也没有重复节点。段搜索（第 3.2 节）因此对每个 $w$ 有唯一结果。
- **力值有限且非负。** 段内的 $g(w)$ 是 $g_{i-1}$ 与 $g_i$ 的凸组合，饱和区取 $g_n$，故对所有 $w\ge0$ 有 $g(w)\ge0$。从而 $F(u)\,u=|u|\,g(|u|)\ge0$：本构力与轴向相对速度同号或为零，元件只耗散、不做正功；等号成立当且仅当 $u=0$ 或 $g(|u|)=0$。
- **首节点为原点。** $F(0)=0$，元件没有静力，也没有原点处的跳变（第 2.2 节）。
- **未被要求的性质。** 节点力值不要求单调。因此 $c_i$ 可以为负，$F$ 不必是 $u$ 的单调函数，$|F|$ 可以随 $|u|$ 增大而减小；饱和值 $g_n$ 也不必是最大的力值。三条要求保证的是符号意义上的耗散 $F(u)u\ge0$，而不是增量意义上的正阻尼。
- **有界。** $|F(u)|\le\max_i g_i$ 对所有 $u$ 成立，且 $|u|\ge u_n$ 时 $|F(u)|=g_n$。

### 2.4 轴向速度输入与参考端力

本族沿 A 的第 $j$ 轴作用。速度输入取共用篇第 2.3 节的 $\mathbf u_A$ 在该轴上的分量，参考端所受的力沿同一轴：

$$
u=\mathbf e_j\cdot\mathbf u_A=u_{A,j},
\qquad
\mathbf f_A=F(u)\,\mathbf e_j .
$$

$\mathbf u_A=\dot{\mathbf d}_A$ 含运输项：它是对端原点相对刚体 $\mathcal A$ 的速度，不是两原点惯性速度之差；第 $j$ 轴沿两原点连线时这一分量必与 $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ 的对应分量相同，一般方向下二者相差运输项的轴向投影 $\mathbf e_j^{\mathsf T}R_{IA}^{\mathsf T}(\boldsymbol\omega_A\times\mathbf d_I)$，该投影为零时才相同（共用篇第 6 节）。符号约定与共用篇一致，$\mathbf f_A$ 是参考端所受的力。$u>0$ 表示对端原点沿 A 的第 $j$ 轴正向离开刚体 $\mathcal A$，此时 $F(u)\ge0$，参考端被沿第 $j$ 轴正向、即朝对端离去的方向拉动，对端受到反向的力：阻尼器阻碍相对运动。位移 $\mathbf d_A$、相对姿态 $R_{AC}$ 与相对角速度 $\boldsymbol\omega_{rel,A}$ 都不进入本构。在首段 $|u|\le u_1$ 内，本族与刚度为零、阻尼为 $c_1\mathbf e_j$、无名义力的[三向平动弹簧—阻尼力元](TRANSLATIONAL_SPRING_DAMPER.md)给出相同的力。

### 2.5 成对扳手

载荷按共用篇第 3.2 节的端点力对加参考端支承矩施加：

$$
\mathbf f_I=R_{IA}\mathbf f_A=F(u)\,R_{IA}\mathbf e_j,
\qquad
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

$R_{IA}\mathbf e_j$ 是 $R_{IA}$ 的第 $j$ 列，即 A 的第 $j$ 轴在 I 中的方向。支承矩 $\mathbf d_I\times\mathbf f_I$ 施加于刚体 $\mathcal A$、关于 $A_o$ 取矩；$F(u)\ne0$ 且 $\mathbf d_I$ 不平行于作用轴时它不为零。两条载荷合力为零、关于任意点的合力矩为零，见共用篇第 3.2 节。

### 2.6 功率原则的证明

共用篇第 4.5 节的表中，端点力对加支承矩这一行的速度输入是 $\mathbf u_A$，端点功率是 $-\mathbf f_A\cdot\mathbf u_A$。本族的速度输入 $u=\mathbf e_j\cdot\mathbf u_A$ 正是这一行的量在作用轴上的分量，施加方式正是这一行的方式，于是由共用篇第 4.2 节的恒等式，

$$
\mathcal P=-\mathbf f_A\cdot\mathbf u_A=-F(u)\,\mathbf e_j\cdot\mathbf u_A=-F(u)\,u=-|u|\,g(|u|)\le0 .
$$

第二个等号用了 $\mathbf f_A$ 只有第 $j$ 分量，第三个等号用了 $u$ 的定义，第四个等号用了 $F(u)u=|u|g(|u|)$，不等号用了第 2.3 节的非负性。左端是两刚体从力元收到的总功率，$-F(u)u$ 是一维阻尼律自身的功率，二者相等，这就是功率原则。它对任意 $\mathbf d_I$ 与任意 $\boldsymbol\omega_A$ 成立，因为支承矩的功率 $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$ 恰好补上了运输项的功率。若保留 $\mathbf u_A$ 为输入而去掉支承矩，两刚体收到的功率变为 $-\mathbf f_A\cdot R_{IA}^{\mathsf T}\dot{\mathbf d}_I=-F(u)\,u-F(u)\,\mathbf e_j\cdot R_{IA}^{\mathsf T}(\boldsymbol\omega_A\times\mathbf d_I)$，多出的一项等于 $-F(u)$ 乘运输项的轴向投影 $\mathbf e_j^{\mathsf T}R_{IA}^{\mathsf T}(\boldsymbol\omega_A\times\mathbf d_I)$，作用轴沿两原点连线时必为零，一般方向下在 $F(u)=0$ 或该投影为零时为零；反之若换用 $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ 的分量作输入而保留支承矩，本构所记的功率同样不再等于端点功率。功率恒为非正说明本族没有储能：它对两刚体做的功单调不增，等号只在 $u=0$ 或 $g(|u|)=0$ 时成立。

## 3. 计算实现

### 3.1 相对运动的取用

每次求值先按共用篇第 5.1 节取得该力元的一份相对运动。本族只消费其中的 $\mathbf u_A$，以及写载荷所需的 $\mathbf d_I$、$R_{IA}$ 与两端原点的刚体固定点；$\mathbf d_A$、$R_{AC}$、$\boldsymbol\omega_{rel,A}$ 不参与。作用轴由三值枚举 `ForceElementAxis` 指明，纵向、横向、竖向对应一基编号 $j=1,2,3$；取 $\mathbf u_A$ 的第 $j$ 分量，即 Eigen 向量的第 $j-1$ 个元素，得到 $u$。本族没有内部状态，不向 $z$ 块写任何量，$u$ 完全由当前 $(q,v)$ 决定。

### 3.2 曲线求值

给定 $u$，求值分三步，与第 2.1、2.2 节的定义逐条对应。

1. **符号与速率。** $u<0$ 时符号取 $-1$，否则取 $+1$；$w=|u|$。$u=0$ 落入 $+1$ 分支，但因 $g(0)=0$，结果仍是 $F(0)=0$，这一分支不改变数学结果。
2. **饱和分支。** 若 $w\ge u_n$，返回符号乘 $g_n$。这是末节点外常值延拓的实现；$w=u_n$ 本身也走这一分支，所得 $g_n$ 与段 $n$ 在其右端点的取值相同。
3. **段内插值。** 否则自 $i=1$ 起顺序找到第一个满足 $w\le u_i$ 的段，计算 $\lambda=\frac{w-u_{i-1}}{u_i-u_{i-1}}\in[0,1]$ 与 $g(w)=(1-\lambda)\,g_{i-1}+\lambda\,g_i$，返回符号乘 $g(w)$。凸组合形式与第 2.1 节的斜率形式 $g_{i-1}+c_i(w-u_{i-1})$ 代数相同。由于节点速度严格递增，命中的段唯一；$w$ 恰为内部节点 $u_i$ 时命中段 $i$ 且 $\lambda=1$，返回 $g_i$，与段 $i+1$ 在 $\lambda=0$ 处的取值一致。

所有分支边界——原点、每个内部节点、饱和端——两侧取值相同，分支只决定套用哪一段的公式，不引入跳变；这是 $F$ 连续性的实现形式。求值以第 2.1 节的三条要求为前提。标量律 $F(u)$ 另有一个不依赖装配系统的公开入口 `VehicleForcePlan::SaturatedPiecewiseLinearDamperForce`，它对给定的力元与速度直接返回 $F(u)$。

### 3.3 扳手的写出

由 $F(u)$ 组成 $\mathbf f_A=F(u)\,\mathbf e_j$，即只有第 $j$ 分量非零的三维向量；左乘 $R_{IA}$ 得 $\mathbf f_I$，连同 $\mathbf d_I$ 与两端原点的刚体固定点交给端点力对的写入例程，按共用篇第 5.2 节第一条写出两条载荷 $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ 与 $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$。这一路径与平动族、串联族相同，本族不带任何额外的载荷项。

## 4. 数学性质与适用条件

- **奇对称性。** 负半轴从不单独给出，$F(-u)=-F(u)$ 按构造成立，拉伸与压缩方向的特性完全相同。本族适合两个方向特性对称的速度型阻尼器；方向不对称的特性不属于本族。
- **纯耗散、无储能。** $\mathcal P=-|u|\,g(|u|)\le0$ 对任意运动成立，元件不能向系统输入能量。力只依赖 $\mathbf u_A$ 的一个分量，与 $\mathbf d_A$、$R_{AC}$ 无关，故 $u=0$ 时力恒为零：本族不产生静力，不能表达预载。本族也没有频率依赖与滞回，是一个代数律；带串联柔度的阻尼器属于[串联弹簧—黏性阻尼力元](SERIES_SPRING_VISCOUS_DAMPER.md)。
- **耗散势。** $F$ 是一个偶的、非负的、$C^1$ 的分段二次函数的导数：$\Phi(u)=\int_0^{u}F(\xi)\,d\xi=\int_0^{|u|}g(w)\,dw\ge0$，$F=\Phi'$。$\Phi$ 为凸当且仅当 $F$ 单调不减，即所有 $c_i\ge0$；本族不要求这一点，$\Phi$ 一般非凸。
- **非单调与负增量阻尼。** 在各段内部与饱和区，切线阻尼为 $F'(u)=c_i$（$u_{i-1}<|u|<u_i$）与 $F'(u)=0$（$|u|>u_n$）。工作点落在 $c_i<0$ 的段上时，围绕该点的线性化得到负的切线阻尼，尽管总功率仍非正；工作点落在饱和区时切线阻尼为零，本族对线性化阻尼没有贡献；工作点落在首段内部 $|u_*|<u_1$ 时（含 $u_*=0$），线性化阻尼是 $c_1$。对线性化分析而言，本族的阻尼贡献是工作点相关的量，而不是一个常数。
- **有界与饱和。** $|F|\le\max_i g_i$，大速度下力不增长。饱和值 $g_n$ 是末节点力值，不一定是曲线上的最大值。
- **连续、Lipschitz、分段 $C^1$。** $F$ 在整个实轴上连续，Lipschitz 常数为 $\max_i|c_i|$；折点位于 $\pm u_i$ 中两侧斜率不同的那些点，原点不是折点。状态方程右端因此在 $v$ 中连续但仅分段 $C^1$，$u$ 越过一个折点时右端对 $v$ 的 Jacobian 发生跳变。
- **单轴、体固定方向。** 作用轴是 A 的坐标轴，一般不沿两原点连线，因此 $u$ 含运输项、支承矩一般非零；本族不是沿两点连线作用的点对点阻尼器，而是沿参考端固定方向作用的轴向阻尼器。参考端的选择是本构的一部分：改以另一端为参考端，作用轴改为该端刚体上的方向，一般得到不同的元件（共用篇第 6 节）。
- **退化情形。** $n=1$ 时，元件在 $|u|\le u_1$ 内是系数为 $c_1$ 的线性黏性阻尼器，其外饱和于 $g_1$；所有 $g_i=0$ 时 $F\equiv0$，元件对运动没有作用。

## 5. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 力元类型、节点类型与三条定义域要求的契约 | `SaturatedPiecewiseLinearDamper`、`SaturatedPiecewiseLinearDamperPoint`，见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 作用轴编号 $j$ 的类型 | `ForceElementAxis`，见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 半轴曲线 $g$ 的求值与奇延拓 $F(u)$ | `EvaluateValidatedSaturatedDamperCurve`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 标量律 $F(u)$ 的公开入口 | `VehicleForcePlan::SaturatedPiecewiseLinearDamperForce`，见 [`vehicle_force_plan.h`](../../../libs/forces/include/orvd/forces/vehicle_force_plan.h) |
| 取 $\mathbf u_A$ 的第 $j$ 分量、组成 $\mathbf f_A$ 并转入 I | `VehicleForcePlan::CalcAppliedForces` 内的饱和族分支，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 相对运动 $\mathbf u_A$、$\mathbf d_I$、$R_{IA}$ 的一次求取 | `CalcRelativeMotion`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 端点力对与参考端支承矩 | `EmitTranslationalWrenchPair`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 载荷条目与刚体固定点 | `AppliedBodyWrench`、`BodyFixedPoint`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
