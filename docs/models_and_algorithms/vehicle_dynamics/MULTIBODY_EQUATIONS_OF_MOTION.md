[English](MULTIBODY_EQUATIONS_OF_MOTION.en.md)

# 整车多体动力学方程

本篇说明 ORVD 一辆车的多体层如何把刚体、关节与自由体组织成一棵刚体树，如何定义广义位置 $q$、广义速度 $v$ 与二者之间的映射 $\dot q=N(q)v$，以及如何由组件交来的体扳手、多体层自身的重力与关节阻尼求出广义加速度 $\dot v$。已发布的线路几何、不平顺谱、轮轨接触八篇与力元六篇都是组件，它们产生作用于刚体的扳手；本篇把这些扳手与重力、关节阻尼合成完整右端 $[N(q)v;\dot v;\dot z]$，[起动状态装配](STARTUP_STATE_ASSEMBLY.md)构造初值 $y_0$，[轮轨接触力计划的运动学装配](../wheel_rail_contact/CONTACT_FORCE_PLAN_KINEMATICS.md)说明多体状态如何变成每个轮轨接口的接触输入、接触结果如何变回作用于车轮刚体的一条等效扳手，[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)说明求解器如何推进并对完整右端作差分。本篇先给出刚体树的对象与坐标，再给出速率映射、位姿与空间速度、体扳手到广义力的功率共轭，写出 $M(q)\dot v+C(q,v)v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}$，说明前向动力学采用的铰接体算法，最后说明完整右端的装配。多体层的公开接口见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h)，其递推由外置刚体树完成，本篇引用的递推实现位于 [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) 等文件。

## 1. 范围与记号

### 1.1 对象

对象是一辆车的刚体树：有限个刚体、固定在刚体上的坐标系、把两个刚体相连的关节，以及被声明为在世界中自由运动的刚体。多体层接纳五种关系：回转副、移动副、Ball-RPY 球铰、焊接副与自由体声明。它的输入是组件产生的体扳手，每条扳手指明受力刚体、体系中的作用点、表达系、关于作用点的力矩与力；输出是广义位置导数 $N(q)v$ 与广义加速度 $\dot v$。本篇不讨论任何本构律、接触几何、初值构造或时间推进：力元的本构见力元各篇，接触输入的形成见接触力计划篇，初值见起动篇，推进见时间积分篇。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| W、B、P、C | 世界系；刚体 B 及其体坐标系；树中 B 的父节点 P 与子节点 C | 本篇新增 |
| F、M、E | 一个关节的入端坐标系（固定在父刚体上）与出端坐标系（固定在子刚体上）；某条扳手的表达系 | 本篇新增 |
| $B_o$、$\mathbf p_{PoBo}$ | 刚体 B 的体原点；从 $P_o$ 到 $B_o$ 的位置向量，未标表达系时在 W 中表达 | 基座 §4.1 |
| $R_{WB}$、$R_{FM}(q_B)$ | B 在 W 中的姿态；出端相对入端的关节旋转 | 基座 §4.1 |
| $q$、$v$、$z$ | 广义位置、广义速度、力元内部状态 | 基座 §5.1 |
| $q_B$、$v_B$、$n_{v,B}$ | 属于刚体 B 入端关系（关节或自由体声明）的 $q$、$v$ 块及后者的维数 | 本篇新增 |
| $n_q$、$n_v$ | $q$、$v$ 的维数 | 基座 §4.2 |
| $N(q)$、$N^{+}(q)$ | 位置导数映射及其左伪逆 | 基座 §4.5 |
| $(\lambda_0,\boldsymbol\lambda)$、$\hat\lambda$、$Q(\lambda)$ | 自由体的四元数（存储顺序 $w,x,y,z$）、其单位化，以及由它构成的 $4\times3$ 矩阵 | 本篇新增 |
| $\boldsymbol\eta$、$E(\boldsymbol\eta)$、$H(\boldsymbol\eta)$ | Ball-RPY 的三角（滚转、俯仰、偏航）及其角速率映射 | 衬套篇 §2.6–2.7 |
| $\mathbf a$、$\theta$、$d$ | 单轴关节在 F 中的单位轴、回转角、移动位移 | 本篇新增 |
| $m$、$\mathbf c$、$I_{\mathrm c}$、$G_o$ | 质量、体系中的质心偏置、质心惯量、体原点单位惯量 | 本篇新增 |
| $M_B$、$\mathbf c_W$、$G_W$ | B 关于 $B_o$、在 W 中表达的空间惯量及其质心偏置与单位惯量 | 本篇新增 |
| $\mathbf g_W$、$g$、$U$ | 重力加速度向量、其大小、重力势能 | 本篇新增 |
| $V_B$、$A_B$ | B 的空间速度 $(\boldsymbol\omega_{WB};\mathbf v_{WBo})$ 与空间加速度 $(\boldsymbol\alpha_{WB};\mathbf a_{WBo})$，在 W 中表达 | 本篇新增 |
| $\mathcal W_Q^E=(\boldsymbol\tau_Q^E;\mathbf f^E)$ | 关于 Q 取矩、在 E 中表达的扳手，本篇按六维列向量运算 | 基座 §5.2 |
| $\Phi(\mathbf p)$ | 换点算子 $\begin{bmatrix}\mathbf 1&[\mathbf p]_\times\\ \mathbf 0&\mathbf 1\end{bmatrix}$ | 本篇新增 |
| $[\mathbf p]_\times$、$\mathbf 1$ | 反对称矩阵（即基座的 $\operatorname{skew}(\mathbf p)$）与单位矩阵，必要时以下标标明阶数 | 本篇新增 |
| $S_{FM}$、$S_B$ | 关节在 F 中关于 $M_o$ 的运动子空间；同一子空间换到 W、搬到 $B_o$ 后的 $6\times n_{v,B}$ 矩阵 | 本篇新增 |
| $J_B$、$J_Q$ | B 的空间速度、B 上固定点 Q 的空间速度对 $v$ 的 Jacobian | 本篇新增 |
| $M(q)$、$C(q,v)v$ | 系统质量矩阵与速度偏置广义力 | 基座 §5.3 |
| $\tau_{\mathrm g}$、$\tau_{\mathrm d}$、$\tau_{\mathrm{app}}$ | 重力、关节阻尼与外加体扳手对应的广义力 | 基座 §5.3 |
| $\mathcal W_{\mathrm g,Bo}$、$\mathcal W_{\mathrm{app},Bo}$ | 重力关于 $B_o$ 的等效扳手；组件交来的体扳手搬到 $B_o$ 之和 | 本篇新增 |
| $F_{\mathrm{app},B}$、$\tau_{\mathrm{app},B}$ | 累加到 $B_o$ 的全部外加空间力 $\mathcal W_{\mathrm g,Bo}+\mathcal W_{\mathrm{app},Bo}$；作用在 B 入端关系坐标上的广义力（含关节阻尼） | 本篇新增 |
| $F_{\mathrm b,B}$、$A_{\mathrm b,B}$ | B 的速度偏置空间力与偏置空间加速度 | 本篇新增 |
| $P_B$、$P_B^{+}$、$U_B$、$D_B$、$g_B$ | 铰接体惯量、经关节投影后的铰接体惯量、$S_B^{\mathsf T}P_B$、关节惯量、增益 | 本篇新增 |
| $Z_B$、$Z_B^{+}$、$Z_{\mathrm b,B}$、$e_B$ | 残余力、经关节投影后的残余力、铰接偏置力、关节残余广义力 | 本篇新增 |
| $K_B$ | 复合体惯量 | 本篇新增 |
| $c$ | 关节黏性阻尼系数 | 基座 §5.3 |

### 1.3 与基座记号的关系

本篇沿用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)第 4.1 节的旋转矩阵规则与第 5.2 节的扳手记号。多体层把惯性参考称为世界系 W；车辆装配把 W 取为基座第 2.1 节的轨道惯性系 I（重力向量沿 I 的 $+z$ 设置，起动状态也在 I 中给出），因此本篇的 W 与其他各篇的 I 是同一个坐标系。以下符号在本篇局部使用，均在此声明：铰接体递推的运动子空间记 $S$（源码 `H_PB_W`），不用 $H$，因为 $H(\boldsymbol\eta)$ 与 $E(\boldsymbol\eta)$ 在[半角中点 RPY 衬套](../force_elements/HALF_ANGLE_MIDPOINT_RPY_BUSHING.md)第 2.7 节已是角速率映射，本篇第 3.2 节原样引用它们；本篇的 $J$ 是运动学 Jacobian，与[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)中的 $J=\partial f/\partial y$ 无关；$C$ 单独出现时是树中的子节点，带自变量的 $C(q,v)$ 是基座第 5.3 节的系统矩阵；$P$ 单独出现时是父节点，带体下标的 $P_B$ 是铰接体惯量；$V_B$ 是空间速度，不是力元各篇的储能 $\mathcal V$；$g$ 是重力加速度大小，与基座第 2.1 节的纵坡 $g(s)$ 无关，带体下标的 $g_B$ 是递推增益；$\lambda$ 是四元数分量，与时间积分篇稳定性分析中的特征值无关；$N$ 只表示位置导数映射；小写 $\tau$ 是广义力，粗体 $\boldsymbol\tau$ 是空间力矩，与基座第 5.3 节一致。

## 2. 刚体树与广义坐标

### 2.1 刚体、坐标系与关节类型

刚体 B 带一个体坐标系，原点 $B_o$ 是取矩、写惯量与表达状态的基准点。固定坐标系以常量位姿 $(R_{BF},\mathbf p_{BoFo\_B})$ 固定在某个刚体上，只固定在刚体上、不固定在另一个坐标系上。一个关节把父刚体上的坐标系 F 与子刚体上的坐标系 M 相连，关节坐标 $q_B$ 描述 M 相对 F 的位姿 $(R_{FM}(q_B),\mathbf p_{FoMo\_F}(q_B))$，关节速度 $v_B$ 描述 M 相对 F 的空间速度 $V_{FM}=S_{FM}v_B$。哪一端被声明为父端决定了关节坐标的含义与正号，但不决定刚体树从世界向外生长时穿过该关节的方向。

| 类型 | $n_q$ | $n_v$ | 广义位置 | 广义速度 |
|---|---|---|---|---|
| 回转副 | 1 | 1 | 绕 F 中固定单位轴 $\mathbf a$ 的转角 $\theta$，右手定则，$R_{FM}=\exp(\theta[\mathbf a]_\times)$，$\mathbf p_{FoMo}=\mathbf 0$ | $\dot\theta$，$\boldsymbol\omega_{FM\_F}=\dot\theta\,\mathbf a$ |
| 移动副 | 1 | 1 | 沿 $\mathbf a$ 的位移 $d$，$R_{FM}=\mathbf 1$，$\mathbf p_{FoMo\_F}=d\,\mathbf a$ | $\dot d$，$\mathbf v_{FMo\_F}=\dot d\,\mathbf a$ |
| Ball-RPY 球铰 | 3 | 3 | $\boldsymbol\eta=(\text{roll},\text{pitch},\text{yaw})$，$R_{FM}=R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})$，$\mathbf p_{FoMo}=\mathbf 0$ | $\boldsymbol\omega_{FM\_F}$，不是 $\dot{\boldsymbol\eta}$ |
| 焊接副 | 0 | 0 | 无；$R_{FM}=\mathbf 1$，$\mathbf p_{FoMo}=\mathbf 0$，两刚体的相对放置全部由两个固定坐标系给出 | 无 |
| 自由体 | 7 | 6 | 四元数 $(\lambda_0,\boldsymbol\lambda)$ 与体原点在世界中的位置 $\mathbf p_{WoBo\_W}$ | $\boldsymbol\omega_{WB\_W}$ 与 $\mathbf v_{WBo\_W}$，均在世界系表达 |

单轴关节的轴以父端坐标系中的方向给出并单位化，只用其方向；因为 M 只绕该轴相对 F 转动或沿该轴平移，同一根轴在 F 与 M 中的分量相同。Ball-RPY 的合成顺序与基座第 4.3 节相同，也就是衬套篇第 2.6 节的 space-XYZ 角约定，三个角按滚转、俯仰、偏航存放，速度是物理角速度。自由体的四元数按存储值使用，不做单位化；它的两块速度分别是刚体的绝对角速度与体原点的绝对速度，都在世界系中表达（基座第 4.2 节）。

### 2.2 树拓扑

每条关系连接两个不同的刚体，把已经互相可达的两个刚体再连一次会闭合回路，因此模型是一片以世界为根的树：任意两个刚体之间至多一条路径，任何刚体都不能经两条路径到达世界。自由体声明是刚体与世界之间的一条关系，多体层在终结时用一个从世界坐标系到该体体坐标系的六自由度关节实现它，而不是"没有关系"的默认；自由体之外仍可挂关节，声明自由只说明它如何与世界相关。终结时每个刚体都必须可达世界。终结后每个广义坐标恰属于一个关节或一个自由体声明，$q$ 与 $v$ 是这些块按模型自身顺序的拼接，$n_q-n_v$ 等于自由体的个数。刚体树从世界向外为每个刚体指定唯一的入端关系与父节点，本篇的递推都沿这棵树进行。

### 2.3 惯量参数

车辆定义为每个刚体给出质量 $m$、体系中的质心偏置 $\mathbf c$ 与关于质心、在体系中表达的惯量 $I_{\mathrm c}$。多体层以关于体原点的单位惯量存放惯量，装配一次换算：

$$
G_o=\frac{I_{\mathrm c}}{m}+\lvert\mathbf c\rvert^2\mathbf 1-\mathbf c\mathbf c^{\mathsf T},
$$

这是平行轴定理除以质量的形式。由 $(m,\mathbf c,G_o)$ 组成 B 关于 $B_o$、在体系中表达的空间惯量，换到世界系后

$$
M_B=\begin{bmatrix}mG_W&m[\mathbf c_W]_\times\\-m[\mathbf c_W]_\times&m\mathbf 1\end{bmatrix},
\qquad
\mathbf c_W=R_{WB}\mathbf c,
\qquad
G_W=R_{WB}G_oR_{WB}^{\mathsf T},
$$

它把空间加速度 $(\boldsymbol\alpha;\mathbf a_{Bo})$ 映为关于 $B_o$ 的空间力：力矩 $mG_W\boldsymbol\alpha+m\,\mathbf c_W\times\mathbf a_{Bo}$，力 $m\mathbf a_{Bo}+m\,\boldsymbol\alpha\times\mathbf c_W$。多体层要求 $m\ge0$，且由 $G_o$ 移回质心后的中心惯量半正定并满足主惯量的三角不等式；车辆装配另要求每个刚体 $m>0$，且被声明为自由体的刚体的 $I_{\mathrm c}$ 严格正定。后一条保证自由体的六维关节惯量可逆（第 6.4 节）。

### 2.4 重力向量

重力是模型常量，在终结前设定，之后不再改变：$\mathbf g_W=g\,\mathbf e_3$，$g>0$ 是重力加速度大小，向下方向由轨道惯性系的 $+z$ 固定（基座第 2.1 节），调用者只能选择大小。它作用于每个刚体的质心，其关于体原点的扳手与广义力见第 4.5 节。

## 3. 坐标速率映射

$N(q)$ 按关系分块：每个关节或自由体声明的 $q_B$ 块只与自己的 $v_B$ 块相关，$\dot q_B=N_B(q_B)v_B$。反向映射 $v=N^{+}(q)\dot q$ 同样分块。

### 3.1 四元数自由体的 $N(q)$ 与左伪逆

自由体的位置块是四元数 $(\lambda_0,\boldsymbol\lambda)$ 与体原点位置，速度块是 $(\boldsymbol\omega;\mathbf v_{Bo})$，$\boldsymbol\omega=\boldsymbol\omega_{WB\_W}$。单位四元数的姿态满足 $\dot R_{WB}=[\boldsymbol\omega]_\times R_{WB}$，对应的四元数微分方程是 $\dot\lambda=\tfrac12\,\boldsymbol\omega\otimes\lambda$，其中 $\boldsymbol\omega$ 作为纯四元数从左侧相乘（世界系角速度从左乘，体系角速度从右乘）。展开得

$$
\dot\lambda_0=-\tfrac12\,\boldsymbol\lambda\cdot\boldsymbol\omega,
\qquad
\dot{\boldsymbol\lambda}=\tfrac12\left(\lambda_0\boldsymbol\omega+\boldsymbol\omega\times\boldsymbol\lambda\right),
\qquad
N_{\mathrm q}(\lambda)=\tfrac12\,Q(\lambda),
\qquad
Q(\lambda)=\begin{bmatrix}-\boldsymbol\lambda^{\mathsf T}\\ \lambda_0\mathbf 1-[\boldsymbol\lambda]_\times\end{bmatrix}.
$$

$Q$ 的三列与 $\lambda$ 正交且互相正交：$Q(\lambda)^{\mathsf T}\lambda=\mathbf 0$，$Q(\lambda)^{\mathsf T}Q(\lambda)=\lVert\lambda\rVert^2\mathbf 1_3$。于是 $\lambda\cdot\dot\lambda=0$，精确解保持 $\lVert\lambda\rVert$ 不变。实现按存储值使用四元数，既不单位化也不改写它，因此 $\dot q$ 对存储值是线性的：把四元数放大 $s$ 倍，它的四个导数分量也放大 $s$ 倍，而三个平动导数不变。位置块的平动部分 $\dot{\mathbf p}_{WoBo\_W}=\mathbf v_{WBo\_W}$，映射为单位阵。位姿求值只用四元数的方向，旋转矩阵由单位化后的四元数构成。

反向映射是到四元数切空间的左伪逆。记 $\hat\lambda=\lambda/\lVert\lambda\rVert$，实现取

$$
N_{\mathrm q}^{+}(\lambda)=\frac{2}{\lVert\lambda\rVert}\,Q(\hat\lambda)^{\mathsf T}\left(\mathbf 1_4-\hat\lambda\hat\lambda^{\mathsf T}\right),
$$

其中 $(\mathbf 1_4-\hat\lambda\hat\lambda^{\mathsf T})/\lVert\lambda\rVert$ 是单位化映射 $\lambda\mapsto\hat\lambda$ 的导数：它先消去 $\dot\lambda$ 中与四元数平行的分量，再把剩余分量当作单位四元数的导数换成角速度。由 $Q$ 的正交性，$N_{\mathrm q}^{+}N_{\mathrm q}=\mathbf 1_3$ 对任意非零存储值成立，$v\to\dot q\to v$ 是恒等；而 $N_{\mathrm q}N_{\mathrm q}^{+}=\mathbf 1_4-\hat\lambda\hat\lambda^{\mathsf T}$，任意 $\dot q\to v\to\dot q$ 是到切空间的正交投影。这正是基座第 4.5 节所说：与四元数平行的 $\dot q$ 分量不代表物理角速度。

### 3.2 Ball-RPY 的角速率映射与奇点

Ball-RPY 的位置块是 $\boldsymbol\eta=(\eta_1,\eta_2,\eta_3)$，即滚转、俯仰、偏航，速度块是 $\boldsymbol\omega_{FM\_F}$。其合成 $R_{FM}=R_z(\eta_3)R_y(\eta_2)R_x(\eta_1)$ 与衬套篇第 2.6 节的 $R_{AC}$ 同形，因此角速率映射就是该篇第 2.7 节的两个矩阵：

$$
\boldsymbol\omega_{FM\_F}=E(\boldsymbol\eta)\,\dot{\boldsymbol\eta},
\qquad
\dot{\boldsymbol\eta}=H(\boldsymbol\eta)\,\boldsymbol\omega_{FM\_F},
\qquad
E=\begin{bmatrix}\mathrm c_3\mathrm c_2&-\mathrm s_3&0\\ \mathrm s_3\mathrm c_2&\mathrm c_3&0\\ -\mathrm s_2&0&1\end{bmatrix},
\qquad
H=E^{-1}=\begin{bmatrix}\frac{\mathrm c_3}{\mathrm c_2}&\frac{\mathrm s_3}{\mathrm c_2}&0\\ -\mathrm s_3&\mathrm c_3&0\\ \frac{\mathrm c_3\mathrm s_2}{\mathrm c_2}&\frac{\mathrm s_3\mathrm s_2}{\mathrm c_2}&1\end{bmatrix},
$$

$\mathrm c_i=\cos\eta_i$，$\mathrm s_i=\sin\eta_i$。于是 $N_{\mathrm r}(\boldsymbol\eta)=H(\boldsymbol\eta)$，$N_{\mathrm r}^{+}(\boldsymbol\eta)=E(\boldsymbol\eta)$，二者互逆，都不含滚转角。速率映射的实现不构造 $H$，而按 $v=E\dot{\boldsymbol\eta}$ 的三个标量方程逐次求解：$\dot\eta_1=(\mathrm c_3\omega_1+\mathrm s_3\omega_2)/\mathrm c_2$，$\dot\eta_2=-\mathrm s_3\omega_1+\mathrm c_3\omega_2$，$\dot\eta_3=\mathrm s_2\dot\eta_1+\omega_3$，与 $H$ 相乘等价。$\det E=\cos\eta_2$：$\cos\eta_2=0$ 是这组角坐标的奇点，此时 $E$ 降秩、$H$ 无界，映射 $v\to\dot q$ 没有定义，而 $\dot q\to v$ 与位姿求值在任何角上都有定义。局部可逆条件是 $\cos\eta_2\ne0$；角提取所选定的主分支 $\lvert\eta_2\rvert<\tfrac\pi2$ 只是其中一个连通分支，不是可逆域的全部；这一奇点属于角坐标本身，与刚体的运动无关。

### 3.3 单轴关节与焊接副

回转副与移动副的 $\dot q_B=v_B$，$N_B=1$；焊接副没有坐标。因此 $N(q)$ 是分块对角阵，除四元数块 $N_{\mathrm q}$ 与 Ball-RPY 块 $N_{\mathrm r}$ 外其余块都是单位阵；$N^{+}(q)$ 同样分块，$N^{+}N=\mathbf 1_{n_v}$ 在 $N$ 有定义处恒成立，而 $NN^{+}$ 只在没有自由体时是单位阵。

## 4. 位姿、空间速度与载荷

### 4.1 位姿与刚体固定点

位姿从世界向外逐关节合成。对入端坐标系 F 在父刚体 P 上、出端坐标系 M 在刚体 B 上的关节，

$$
R_{WB}=R_{WP}R_{PF}R_{FM}(q_B)R_{MB},
\qquad
\mathbf p_{WoBo}=\mathbf p_{WoPo}+R_{WP}\left(\mathbf p_{PoFo\_P}+R_{PF}\,\mathbf p_{FoMo\_F}(q_B)+R_{PF}R_{FM}\,\mathbf p_{MoBo\_M}\right),
$$

自由体则直接 $R_{WB}=R(\hat\lambda)$、$\mathbf p_{WoBo}=\mathbf p_{WoBo\_W}$。固定坐标系的位姿为 $R_{WF}=R_{WB}R_{BF}$、$\mathbf p_{WoFo}=\mathbf p_{WoBo}+R_{WB}\mathbf p_{BoFo\_B}$。刚体 B 上体坐标为 $\mathbf r_Q^B$ 的固定点在世界中的位置是 $\mathbf p_{WoBo}+R_{WB}\mathbf r_Q^B$。每个坐标系原点都解析为"（刚体，体坐标）"这一对，它是扳手作用点的身份：固定在被焊接刚体上的坐标系仍解析到该刚体自身及其体坐标，焊接不改变端点所属的刚体（力元共用篇第 3.1 节）。若刚体树反向穿过某个回转副、移动副或焊接副，则本节与以下各节以该关节声明的子端为 F、父端为 M，单轴关节的轴取 $-\mathbf a$，坐标 $q_B$ 的取值与正号不变。

### 4.2 空间速度及其搬移

刚体 B 的空间速度 $V_B=(\boldsymbol\omega_{WB\_W};\mathbf v_{WBo\_W})$ 以体原点为基准点。同一刚体上另一点 Q 的速度按 $\mathbf v_{WQ}=\mathbf v_{WBo}+\boldsymbol\omega_{WB}\times\mathbf p_{BoQ}$ 搬移，用换点算子写成

$$
V_Q=\Phi(\mathbf p_{BoQ})^{\mathsf T}V_B,
\qquad
\Phi(\mathbf p)=\begin{bmatrix}\mathbf 1&[\mathbf p]_\times\\ \mathbf 0&\mathbf 1\end{bmatrix},
\qquad
\Phi(\mathbf p)\Phi(\mathbf p')=\Phi(\mathbf p+\mathbf p').
$$

$\Phi$ 本身搬移扳手：$\mathcal W_O=\Phi(\mathbf p_{OQ})\mathcal W_Q$ 就是基座第 5.2 节的 $\boldsymbol\tau_O=\boldsymbol\tau_Q+\mathbf p_{OQ}\times\mathbf f$；$\Phi^{\mathsf T}$ 搬移速度；同一刚体上加速度的搬移另含向心项 $\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf p_{OQ})$，不能只用 $\Phi^{\mathsf T}$，见第 6.2 节偏置加速度中的相应项；空间惯量按 $M_O=\Phi(\mathbf p_{OQ})M_Q\Phi(\mathbf p_{OQ})^{\mathsf T}$ 换点。功率 $\mathcal W_Q\cdot V_Q=\mathcal W_Q^{\mathsf T}\Phi(\mathbf p_{OQ})^{\mathsf T}V_O=\mathcal W_O\cdot V_O$ 与基准点无关。

速度从世界向外递推。关节给出 M 相对 F 的空间速度 $V_{FM}=S_{FM}v_B$，$S_{FM}$ 的列对回转副是 $(\mathbf a;\mathbf 0)$、移动副是 $(\mathbf 0;\mathbf a)$、Ball-RPY 是 $(\mathbf 1_3;\mathbf 0)$ 的三列、自由体是 $\mathbf 1_6$，都不随 $q_B$ 变化。把它搬到 $B_o$、换到世界系，得到 B 的运动子空间，进而得到速度递推与 Jacobian：

$$
S_B=\begin{bmatrix}R_{WF}&\mathbf 0\\ \mathbf 0&R_{WF}\end{bmatrix}\Phi(\mathbf p_{MoBo\_F})^{\mathsf T}S_{FM},
\qquad
V_B=\Phi(\mathbf p_{PoBo})^{\mathsf T}V_P+S_Bv_B,
\qquad
V_B=J_Bv,
\qquad
J_B=\sum_{K\preceq B}\Phi(\mathbf p_{KoBo})^{\mathsf T}S_K\,\Pi_K,
$$

其中 $K\preceq B$ 遍历 B 自身与它到世界路径上的全部刚体，$\Pi_K$ 从 $v$ 中选出 $v_K$ 块；不在这条路径上的坐标对 $V_B$ 没有贡献。$S_B$ 是 $6\times n_{v,B}$ 矩阵，只随位形变化。固定坐标系 F 的空间速度是 $V_F=\Phi(\mathbf p_{BoFo})^{\mathsf T}V_B$：离体原点有偏置的坐标系因 $\boldsymbol\omega\times\mathbf p$ 而有不同的平动速度。

多体层提供坐标系 F 相对坐标系 M、在 E 中表达的空间速度查询，其契约是

$$
V_{MF}^{W}=V_F-\Phi(\mathbf p_{MoFo\_W})^{\mathsf T}V_M,
\qquad
V_{MF}^{E}=\begin{bmatrix}R_{WE}^{\mathsf T}&\mathbf 0\\ \mathbf 0&R_{WE}^{\mathsf T}\end{bmatrix}V_{MF}^{W}.
$$

角分量是 $\boldsymbol\omega_{WF}-\boldsymbol\omega_{WM}$；平动分量是 $F_o$ 在 M 中度量的速度，先把 M 的速度搬到 $F_o$ 再作差，而不是两个原点世界速度的简单之差。这正是[力元连接运动学与空间扳手](../force_elements/FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)第 2.3 节含运输项的相对速度 $\mathbf u$ 的多体来源。

### 4.3 体扳手条目及其到体原点的搬移

一条体扳手条目 `AppliedBodyWrench` 是五元组（刚体 B，体系中的作用点 $\mathbf r_Q^B$，表达系 E，关于 Q 的力矩 $\boldsymbol\tau_Q^E$，力 $\mathbf f^E$）。多体层对每条条目做三步：先算作用点在 E 中相对体原点的位置 $\mathbf p_{BoQ}^E=R_{WE}^{\mathsf T}R_{WB}\,\mathbf r_Q^B$；再由外置刚体树把扳手换到世界系并搬到体原点，

$$
\mathcal W_{Bo}^{W}=\Phi\left(\mathbf p_{BoQ}^W\right)\begin{bmatrix}R_{WE}&\mathbf 0\\ \mathbf 0&R_{WE}\end{bmatrix}\mathcal W_Q^E,
\qquad
\mathbf p_{BoQ}^W=R_{WE}\,\mathbf p_{BoQ}^E=R_{WB}\,\mathbf r_Q^B,
\qquad
\boldsymbol\tau_{Bo}^W=R_{WE}\boldsymbol\tau_Q^E+\mathbf p_{BoQ}^W\times R_{WE}\mathbf f^E,
\qquad
\mathbf f^W=R_{WE}\mathbf f^E;
$$

最后把结果累加到该刚体的外加空间力中，得到 $\mathcal W_{\mathrm{app},Bo}$。换表达系与换取矩点是两个运算，力本身不因换点而变。这一步不做任何广义力投影：条目在这里只是被搬到体原点并累加，投影 $S^{\mathsf T}(\cdot)$ 发生在第 6.2 节叶到根的一趟递推里。力元共用篇第 3.5 节从力元一侧描述的正是同一步。

### 4.4 功率共轭与广义力 $\tau_{\mathrm{app}}=J^{\mathsf T}\mathcal W$

作用在刚体 B 固定点 Q 上的扳手 $\mathcal W_Q$ 向系统输送功率 $\mathcal P=\mathcal W_Q\cdot V_Q$（力元共用篇第 4.1 节）。Q 的空间速度是 $V_Q=J_Qv$，$J_Q=\Phi(\mathbf p_{BoQ})^{\mathsf T}J_B$，因此 $\mathcal P=\left(J_Q^{\mathsf T}\mathcal W_Q\right)\cdot v$：与 $v$ 共轭的广义力是

$$
\tau_{\mathrm{app}}=J_Q^{\mathsf T}\mathcal W_Q=J_B^{\mathsf T}\Phi(\mathbf p_{BoQ})\mathcal W_Q=J_B^{\mathsf T}\mathcal W_{Bo},
$$

也就是说，先把扳手搬到体原点再用体原点的 Jacobian 投影，与直接用作用点的 Jacobian 投影给出同一个广义力。按第 4.2 节 $J_B$ 的结构，$\tau_{\mathrm{app}}$ 中属于路径上刚体 K 的块是 $S_K^{\mathsf T}\Phi(\mathbf p_{KoBo})\mathcal W_{Bo}$：把扳手搬到 $K_o$，再投影到 K 的运动子空间。对全部刚体求和后，每个 K 收到的是其子树中所有刚体的外加扳手搬到 $K_o$ 之和的投影，这可以由叶到根一趟完成：每个节点把自己累加到 $B_o$ 的扳手用 $S_B^{\mathsf T}$ 投影得到自己的块，再把它搬到父原点并加进父节点。实现不显式构造 $J$，而由这一趟递推（重力广义力的查询）或第 6 节铰接体递推中的 $S_B^{\mathsf T}$（前向动力学）逐节点完成同一投影。多体层另提供固定点 Q 的 $J_Q$ 查询，输出角速度与点速度两个 $3\times n_v$ 块。

### 4.5 重力与关节阻尼的广义力

重力作用于质心。关于体原点的等效扳手是力 $m\mathbf g_W$ 与力矩 $\mathbf c_W\times m\mathbf g_W$，$\mathbf c_W=R_{WB}\mathbf c$：

$$
\mathcal W_{\mathrm g,Bo}=\begin{bmatrix}\mathbf c_W\times m\mathbf g_W\\ m\mathbf g_W\end{bmatrix},
\qquad
\tau_{\mathrm g}=\sum_BJ_B^{\mathsf T}\mathcal W_{\mathrm g,Bo},
\qquad
\tau_{\mathrm g}\cdot v=-\frac{dU}{dt},
\qquad
U=-\sum_Bm_B\,\mathbf g_W\cdot\mathbf p_{Wo\,\mathrm{cm},B},
$$

$\mathbf p_{Wo\,\mathrm{cm},B}=\mathbf p_{WoBo}+\mathbf c_W$ 是质心位置。质心偏置 $\mathbf c\ne\mathbf 0$ 时不能只写 $m\mathbf g_W$：力矩项随姿态变化，是重力对转动坐标做功的来源。$U$ 是重力势能，$\tau_{\mathrm g}$ 是保守力。外置刚体树把每个刚体的 $\mathcal W_{\mathrm g,Bo}$ 加进体空间力，并用第 4.4 节的叶到根一趟给出 $\tau_{\mathrm g}$。

关节阻尼是多体层自身提供的唯一速度相关施加广义力（惯性偏置不在此列）。回转副 $\tau_{\mathrm d}=-c\,\dot\theta$，$c\ge0$ 由车辆定义逐关节声明（允许为零），在装配时作为建立关节的参数传入；移动副有同形式的 $\tau_{\mathrm d}=-c\,\dot d$。它直接写在该关节自己的坐标上，是广义力而不是体扳手；Ball-RPY 球铰与自由体没有关节阻尼，球铰若需要转动约束，由力元提供。外置刚体树在 `CalcForceElementsContribution` 中先累加重力体力，末尾经 `AddJointDampingForces` 计入阻尼广义力；两者合起来就是多体层在任何组件载荷之前自带的载荷。

## 5. 动力学方程

### 5.1 $M(q)\dot v+C(q,v)v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}$

对第 4.2 节的 $V_B=J_Bv$ 求导，$A_B=J_B\dot v+\dot J_Bv$，其中 $A_B=(\boldsymbol\alpha_{WB};\mathbf a_{WBo})$，$\mathbf a_{WBo}$ 是材料点 $B_o$ 的加速度。刚体 B 关于 $B_o$ 的 Newton–Euler 方程为

$$
M_BA_B+F_{\mathrm b,B}=F_{\mathrm{tot},B},
\qquad
F_{\mathrm b,B}=m\begin{bmatrix}\boldsymbol\omega\times G_W\boldsymbol\omega\\ \boldsymbol\omega\times(\boldsymbol\omega\times\mathbf c_W)\end{bmatrix},
\qquad
\boldsymbol\omega=\boldsymbol\omega_{WB\_W},
$$

$F_{\mathrm{tot},B}$ 是作用于 B、搬到 $B_o$ 的全部空间力之和，包括组件扳手、重力与关节传来的力。$F_{\mathrm b,B}$ 是速度偏置：转动部分是陀螺力矩 $\boldsymbol\omega\times I_{Bo}\boldsymbol\omega$，质心偏置为零时它一般仍不为零；平动部分是质心相对体原点的向心加速度乘以质量，只有这一项随质心偏置归零。理想关节的约束力成对出现，可以在相邻两个刚体之间传递功率，为零的是成对约束载荷的总功率，即 $\sum_BJ_B^{\mathsf T}(\text{约束力})=\mathbf 0$；以 $J_B^{\mathsf T}$ 左乘并对全部刚体求和，得

$$
M(q)\dot v+C(q,v)\,v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}},
\qquad
M(q)=\sum_BJ_B^{\mathsf T}M_BJ_B,
\qquad
C(q,v)\,v=\sum_BJ_B^{\mathsf T}\left(M_B\dot J_Bv+F_{\mathrm b,B}\right),
$$

$\tau_{\mathrm{app}}=\sum_BJ_B^{\mathsf T}\mathcal W_{\mathrm{app},Bo}$ 是第 4.3 节全部体扳手搬到体原点后的投影，另加直接写在关节坐标上的外加广义力（系统中为空，第 7.1 节）。$M(q)$ 对称，在第 8 节的条件下正定，只依赖 $q$；$C(q,v)v$ 是速度的二次型，多体层只计算这个向量，不声称构造某一个唯一的矩阵 $C(q,v)$。这里 $\dot J_Bv$ 是全树 $\dot v=0$ 时 B 的空间加速度；它由第 6.2 节节点局部的偏置加速度 $A_{\mathrm b,B}$ 沿树累加而成，$\dot J_Bv=\Phi(\mathbf p_{PoBo})^{\mathsf T}\dot J_Pv+A_{\mathrm b,B}$，$\dot J_Wv=\mathbf 0$，二者不能等同，否则父节点已经产生的偏置会被重复计入。

### 5.2 逆动力学形式与偏置项

给定 $\dot v$ 时，同一组方程按逆动力学写成基座第 5.3 节的

$$
\tau_{\mathrm{required}}=M(q)\dot v+C(q,v)v-\tau_{\mathrm g}-\tau_{\mathrm d},
$$

它是为实现 $\dot v$ 还需在关节坐标上施加的广义力。外置刚体树用两趟递推求它：根到叶由 $\dot v$ 递推每个刚体的 $A_B=\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+A_{\mathrm b,B}+S_B\dot v_B$；叶到根求 $F_B=M_BA_B+F_{\mathrm b,B}-F_{\mathrm{app},B}+\sum_C\Phi(\mathbf p_{BoCo})F_C$，即关节必须传给 B 的空间力，再把它搬到关节出端原点 $M_o$、换到 F 中，用 $S_{FM}^{\mathsf T}$ 投影并减去该关节上的外加广义力。用 $S_{FM}^{\mathsf T}$ 在 F 中关于 $M_o$ 投影，与用 $S_B^{\mathsf T}$ 在 W 中关于 $B_o$ 投影给出同一个数，因为 $S_B$ 正是 $S_{FM}$ 经同一换系换点得到的（第 4.2 节），而功率不随基准点与表达系变化。多体层的四个查询中，`CalcVelocityBiasGeneralizedForces` 与 `CalcRequiredGeneralizedForces` 走这两趟递推：前者取 $\dot v=0$ 且不加任何载荷，得到 $C(q,v)v$；后者把重力与阻尼作为已知载荷代入逆动力学，得到 $\tau_{\mathrm{required}}$。另两个不经过逆动力学：`CalcGravityAppliedGeneralizedForces` 对重力体力作第 4.4 节的叶到根投影，得到 $\tau_{\mathrm g}$；`CalcJointDampingAppliedGeneralizedForces` 直接读出第 4.5 节写在关节坐标上的 $\tau_{\mathrm d}$。

### 5.3 质量矩阵的按需装配

$M(q)$ 只在 `CalcGeneralizedMassMatrix` 被查询时装配，按复合体算法在世界系中分两步完成：先由叶到根形成各复合体的惯量，再形成质量矩阵各块。把 B 与其子树中全部刚体焊成一个复合体，其关于 $B_o$ 的惯量由叶到根递推

$$
K_B=M_B+\sum_C\Phi(\mathbf p_{BoCo})K_C\Phi(\mathbf p_{BoCo})^{\mathsf T},
$$

则 $M$ 的对角块与非对角块为

$$
M_{BB}=S_B^{\mathsf T}K_BS_B,
\qquad
M_{KB}=M_{BK}^{\mathsf T}=S_K^{\mathsf T}\Phi(\mathbf p_{KoBo})K_BS_B\quad(K\prec B),
\qquad
M_{KB}=\mathbf 0\quad(K\text{ 与 }B\text{ 不在同一条到世界的路径上}),
$$

这正是把 $M=\sum_BJ_B^{\mathsf T}M_BJ_B$ 按第 4.2 节 $J_B$ 的结构展开后的结果：$K_BS_B$ 是 B 的子树在单位 $\dot v_B$ 下产生的空间力，逐级搬到祖先原点并投影。焊接副没有自己的块，它的刚体只并入复合体。行列按模型的广义速度顺序排列，$M\dot v$ 的分量对平动坐标是力、对转动坐标是力矩。前向动力学路径（第 6 节）既不装配也不分解 $M$，第 7 节的右端求值不经过本节。

## 6. 前向动力学：铰接体算法

前向动力学求解 $M(q)\dot v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}-C(q,v)v$，实现采用铰接体算法：不装配 $M$，而以三趟沿树的递推得到 $\dot v$。全部量在世界系表达、关于体原点取矩；每个节点 B 有父节点 P、子节点 C、运动子空间 $S_B$。算法的出发点是每个刚体的方程都能写成 $F_B=P_BA_B+Z_B$，其中 $F_B$ 是入端关系传给 B 的空间力（关于 $B_o$），$P_B$ 与 $Z_B$ 只依赖 B 的子树；理想关节的条件是 $S_B^{\mathsf T}F_B=\tau_{\mathrm{app},B}$，$\tau_{\mathrm{app},B}$ 是写在 B 入端关系坐标上的广义力（关节阻尼与外加关节力，自由体为零）。

### 6.1 铰接体惯量的叶到根递推

叶节点的 $P_B=M_B$。一般节点先把子节点经关节投影后的惯量搬到 $B_o$ 并与自身惯量相加，再穿过自己的关节投影：

$$
P_B=M_B+\sum_C\Phi(\mathbf p_{BoCo})P_C^{+}\Phi(\mathbf p_{BoCo})^{\mathsf T},
\qquad
U_B=S_B^{\mathsf T}P_B,
\qquad
D_B=U_BS_B=S_B^{\mathsf T}P_BS_B,
$$

$$
g_B=P_BS_BD_B^{-1}=\left(D_B^{-1}U_B\right)^{\mathsf T},
\qquad
P_B^{+}=P_B-g_BU_B=P_B-P_BS_BD_B^{-1}S_B^{\mathsf T}P_B.
$$

$D_B$ 是 $n_{v,B}\times n_{v,B}$ 的关节惯量，对称正定时作 Cholesky 分解（源码 `llt_D_B`），$g_B$ 与 $P_B^{+}$ 都由这个分解求得。$P_B^{+}$ 是父刚体透过关节感受到的 B 子树惯量：沿关节自由方向的响应已被解出并从 $P_B$ 中扣除，$P_B^{+}S_B=\mathbf 0$。这一趟只依赖 $q$。

### 6.2 残余力的叶到根递推

第二趟携带载荷。$F_{\mathrm{app},B}=\mathcal W_{\mathrm g,Bo}+\mathcal W_{\mathrm{app},Bo}$ 是累加到 $B_o$ 的全部外加空间力，即第 4.5 节的重力扳手与第 4.3 节全部体扳手搬到 $B_o$ 之和；$F_{\mathrm b,B}$ 是第 5.1 节的速度偏置；$A_{\mathrm b,B}$ 是节点局部的偏置加速度，即父刚体加速度 $A_P=\mathbf 0$ 且 $\dot v_B=0$ 时 B 的空间加速度；它不含父节点自身的加速度，全树 $\dot v=0$ 时 B 的加速度 $\dot J_Bv$ 由它沿树累加得到（第 5.1 节）。一个反例说明二者的区别：与父刚体在同一原点焊接的刚体局部偏置为零，继承的向心加速度却可以非零。闭式为：

$$
A_{\mathrm b,B}=\begin{bmatrix}\boldsymbol\omega_{WP}\times\boldsymbol\omega_{PB}\\ \boldsymbol\omega_{WP}\times(\boldsymbol\omega_{WP}\times\mathbf p_{PoBo})+2\,\boldsymbol\omega_{WP}\times\mathbf v_{PBo}\end{bmatrix}+\begin{bmatrix}R_{WF}&\mathbf 0\\ \mathbf 0&R_{WF}\end{bmatrix}\begin{bmatrix}\mathbf 0\\ \boldsymbol\omega_{FM}\times(\boldsymbol\omega_{FM}\times\mathbf p_{MoBo\_F})\end{bmatrix},
$$

其中 $(\boldsymbol\omega_{PB};\mathbf v_{PBo})=S_Bv_B$ 是 B 相对父刚体的空间速度（在 W 中），第一项是父运动带来的向心与 Coriolis 项，第二项是关节出端原点 $M_o$ 到体原点的偏置在关节角速度下的向心项；五种关系的 $S_{FM}$ 都不随 $q_B$ 变化，关节自身没有别的偏置项。递推为

$$
Z_B=F_{\mathrm b,B}-F_{\mathrm{app},B}+\sum_C\Phi(\mathbf p_{BoCo})Z_C^{+},
\qquad
e_B=\tau_{\mathrm{app},B}-S_B^{\mathsf T}Z_B,
\qquad
Z_{\mathrm b,B}=P_B^{+}A_{\mathrm b,B},
\qquad
Z_B^{+}=Z_B+Z_{\mathrm b,B}+g_Be_B.
$$

各项的正负与归属按源码：外加空间力以负号进入 $Z_B$，偏置力以正号进入；$e_B$ 是关节上外加广义力与残余力投影之差；$Z_{\mathrm b,B}$ 只依赖 $(q,v)$，$Z_B$、$e_B$、$Z_B^{+}$ 才依赖载荷。推导如下：把 $A_B=A_B^{+}+S_B\dot v_B$（$A_B^{+}$ 见第 6.3 节）代入 $F_B=P_BA_B+Z_B$ 与关节条件 $S_B^{\mathsf T}F_B=\tau_{\mathrm{app},B}$，得 $D_B\dot v_B=e_B-U_BA_B^{+}$；回代得 $F_B=P_B^{+}A_B^{+}+Z_B+g_Be_B$，再以 $A_B^{+}=\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+A_{\mathrm b,B}$ 展开，$F_B=P_B^{+}\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+Z_B^{+}$。把 $F_B$ 搬到 $P_o$ 就是它对父节点方程的贡献 $\Phi(\mathbf p_{PoBo})P_B^{+}\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+\Phi(\mathbf p_{PoBo})Z_B^{+}$，即第 6.1 节与本节求和号中的项。

### 6.3 加速度的根到叶恢复

第三趟从世界（$A_W=\mathbf 0$）向外：

$$
A_B^{+}=\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+A_{\mathrm b,B},
\qquad
\dot v_B=D_B^{-1}\left(e_B-U_BA_B^{+}\right),
\qquad
A_B=A_B^{+}+S_B\dot v_B.
$$

$A_B^{+}$ 是保持当前位形与速度、令本节点广义加速度 $\dot v_B=0$ 时 B 的空间加速度，不是关节锁住时的加速度；$D_B^{-1}$ 由第 6.1 节的 Cholesky 分解施加，源码先求 $\nu_B=D_B^{-1}e_B$，再减去 $g_B^{\mathsf T}A_B^{+}=D_B^{-1}U_BA_B^{+}$。三趟结束后 $\dot v$ 就是 $M(q)^{-1}\left(\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}-C(q,v)v\right)$，各刚体的 $A_B$ 同时得到。

### 6.4 零自由度关节与自由体的退化情形

焊接副 $n_v=0$：$S_B$ 为空，$U_B$、$D_B$、$g_B$、$e_B$ 都不出现，$P_B^{+}=P_B$，$Z_B^{+}=Z_B+P_BA_{\mathrm b,B}$，$A_B=A_B^{+}$。被焊接的刚体只是把自己的惯量与载荷并入父节点，与父刚体一起运动；焊接到世界的刚体 $A_B=\mathbf 0$。

自由体 $S_B=\mathbf 1_6$：$U_B=P_B$，$D_B=P_B$，$g_B=\mathbf 1_6$，$P_B^{+}=\mathbf 0$；其入端关系不带广义力，$\tau_{\mathrm{app},B}=\mathbf 0$，于是 $e_B=-Z_B$，$Z_B^{+}=\mathbf 0$，没有任何惯量或载荷传向世界。父节点是世界时 $A_P=\mathbf 0$、$\boldsymbol\omega_{WP}=\mathbf 0$，且出端坐标系就是体坐标系，故 $A_{\mathrm b,B}=\mathbf 0$，$A_B^{+}=\mathbf 0$，

$$
P_B\,\dot v_B=F_{\mathrm{app},B}-F_{\mathrm b,B}-\sum_C\Phi(\mathbf p_{BoCo})Z_C^{+},
$$

即自由体的六个方程：左端是它与它所挂子树的铰接体惯量，右端是外加载荷、速度偏置与子树传来的残余力。这里 $D_B=P_B$ 的 Cholesky 分解要求 $P_B$ 正定；$P_B$ 中各子树项半正定，$M_B$ 正定当且仅当 $m>0$ 且中心惯量正定，这就是第 2.3 节车辆装配对自由体的要求。

### 6.5 与质量矩阵法的等价性与复杂度

铰接体算法与装配 $M(q)$ 再解线性方程组在精确算术下给出同一个 $\dot v$：第 6.2 节的推导只是把 $M\dot v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}-C(q,v)v$ 按树结构作分块消元，$D_B$ 的 Cholesky 分解对应消元过程中每个关节块的主元。三趟递推中每个节点的工作量只与 $n_{v,B}\le6$ 有关，总代价随刚体数线性增长；装配 $M$ 的代价至多随刚体数平方增长，分解 $M$ 的代价随 $n_v$ 立方增长。两种算法的舍入误差不同，但没有建模上的差别；前向路径不装配、不分解全局质量矩阵。

## 7. 完整右端的装配

### 7.1 三类载荷源

系统连续状态是基座第 5.1 节的 $[q;v;z]$，$z$ 只含串联弹簧黏性阻尼的内力状态。一次右端求值向多体层交付的载荷来自三类计划，每一类都写出若干条第 4.3 节的体扳手条目：

- 车辆力计划：五族力元按力元共用篇第 5.2 节各写两条载荷，同时写出 $\dot z$（[串联弹簧—黏性阻尼力元](../force_elements/SERIES_SPRING_VISCOUS_DAMPER.md)第 3.4 节）。
- 轮轨接触力计划：每个轮轨接口恰一条扳手，作用于车轮刚体、作用点为车轮体原点、在世界系表达，无接触时为零扳手；从多体状态到接触输入、再到这条扳手的过程见接触力计划篇第 7 节。
- 独立旋转车轮的驱动力矩偶：每个通道给出一个保持量 $\tau$（不属于连续状态），写出一对纯力偶，$(+\tau\,\mathbf a;\mathbf 0)$ 作用于车轮刚体、$(-\tau\,\mathbf a;\mathbf 0)$ 作用于指定的反作用刚体，两条都以各自的体原点为作用点、在世界系表达，$\mathbf a=R_{WA}\mathbf e_2$ 是提供轴向的车轴体 A 的体系 $+y$ 轴在世界中的方向。该计划不向车轴体直接施加任何扳手，车轴体与车轮之间的关节约束力由第 6 节的递推另计。

三类条目合成一份列表交给多体层；多体层只接收体扳手，关节力矩与移动副力的输入为空，系统也没有任何调用时外力输入。任何一个计划都可以为空。

### 7.2 一次求值的顺序与状态导数 $[\dot q;\dot v;\dot z]$

给定 $(t,q,v,z)$ 及全部非状态数据，一次求值按以下顺序进行：先由同一份 $(q,v)$ 求各计划的运动学输入并写出全部体扳手，车辆力计划同时写出 $\dot z$；再由多体门面执行第 4.5 节的重力与关节阻尼、第 4.3 节的体扳手搬移与第 6 节的三趟递推，得到 $\dot v$，并按第 3 节由 $v$ 得到 $N(q)v$；最后装配层把三块拼接：

$$
\frac{d}{dt}\begin{bmatrix}q\\v\\z\end{bmatrix}
=\begin{bmatrix}N(q)\,v\\ \dot v(q,v,z)\\ \dot z(q,v,z)\end{bmatrix}.
$$

$\dot v$ 对 $z$ 的依赖只经由串联族的力状态进入体扳手；$\dot z$ 对 $(q,v)$ 的依赖经由力元的相对速度；载荷不依赖 $\dot v$ 或 $\dot z$，三块之间没有代数环。多体层本身不含时间，$t$ 只经由各计划的输入进入右端。

### 7.3 与积分器的接口

积分器给出 $(t,y)$，$y=[q;v;z]$；右端桥接把它写入一个试算上下文，再由装配层按第 7.2 节求 $f(t,y)$，见时间积分篇第 1.1 节。因为 $q$ 块的导数是 $N(q)v$ 而不是 $v$，含自由体时 $q$ 与 $v$ 维数不同，积分器把 $q$、$v$、$z$ 同等地当作状态分量推进；四元数按存储值推进，其模长不由本篇的方程强制为一，见第 8 节。初值 $y_0$ 的构造见起动篇。

## 8. 数学性质与适用条件

- **树拓扑与理想关节。** 模型只表达无闭环的刚体树；关节是理想的，不含摩擦、间隙与弹性，约束力不做功，因此运动方程中不出现约束力，也不需要约束方程。需要闭环、附加约束或柔性的结构不在本篇范围内。
- **世界系表达与体原点取矩。** 递推中的全部空间量在世界系表达、关于体原点取矩；组件交来的扳手可以在任意坐标系中表达、关于任意刚体固定点取矩，进入多体层后统一搬到体原点。这一约定使质心偏置进入 $M_B$、$F_{\mathrm b,B}$ 与重力力矩 $\mathbf c_W\times m\mathbf g_W$ 三处，缺任何一处都不再与关于质心的表述等价。
- **质量矩阵的正定性与关节惯量。** 每个 $M_B$ 半正定，$M(q)=\sum_BJ_B^{\mathsf T}M_BJ_B$ 半正定；铰接体递推要求每个 $D_B=S_B^{\mathsf T}P_BS_B$ 正定，这等价于 $M(q)$ 正定。车辆装配的 $m>0$ 与自由体中心惯量正定保证自由体的 $D_B$ 正定；对由关节相连的刚体，$D_B$ 正定要求其子树关于关节自由方向的惯量非零，例如回转副上不能只挂一个质心恰在轴上的点质量。
- **坐标奇点。** 四元数没有奇点，代价是 $n_q>n_v$ 与一条不被方程强制的模长；Ball-RPY 在 $\cos(\text{pitch})=0$ 处 $N_{\mathrm r}$ 无定义，这是角坐标的奇点而不是动力学的奇点，位姿与 $\dot q\to v$ 在该处仍有定义。
- **四元数模长与线性性。** $\dot q=N(q)v$ 使 $\lambda\cdot\dot\lambda=0$，精确解保持模长；数值推进会使模长漂移，位姿求值只取方向，因此漂移不改变姿态，只改变四元数块的尺度，且该块的导数随尺度线性放大。对完整右端作差分时四元数分量按存储值扰动、不投影回单位球面，见时间积分篇第 2.3 节。
- **能量。** 重力是保守力，$\tau_{\mathrm g}\cdot v=-\dot U$；关节阻尼耗散，每个回转副贡献 $-c\,\dot\theta^2\le0$、每个移动副贡献 $-c\,\dot d^2\le0$；由 Newton–Euler 方程投影得到的 $C(q,v)v$ 满足 $v^{\mathsf T}C(q,v)v=\tfrac12v^{\mathsf T}\dot M(q)v$，因此没有组件载荷与阻尼时动能 $T=\tfrac12v^{\mathsf T}M(q)v$ 与 $U$ 之和守恒。组件扳手的功率由各组件自己的性质决定（力元各篇第 4 节或对应章节）。
- **重力与关节阻尼的归属。** 二者由多体层自带，组件不应再交付重力扳手或关节阻尼，否则被重复计入。
- **球铰的入端方向。** Ball-RPY 球铰只能沿声明的父端到子端的方向被刚体树穿过：其声明的父端必须处于靠近世界的一侧，需要反向穿过球铰的树不在支持范围内。回转副、移动副与焊接副可以朝任一方向穿过，坐标含义不变（第 4.1 节）。
- **刚体与均匀重力。** 刚体无变形；重力是常量均匀场；世界系是惯性系，不含线路以外的加速度效应。

## 9. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 刚体树的元素：刚体、固定坐标系、四种关节与自由体声明；坐标区间 | `MultibodyModel::AddRigidBody`、`AddFixedFrame`、`AddRevoluteJoint`、`AddPrismaticJoint`、`AddBallRpyJoint`、`AddWeldJoint`、`DeclareFreeBody`、`GetJointPositionRange`、`GetFreeBodyPositionRange`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 可达性、自由体的六自由度关节与终结 | `MultibodyModel::Finalize`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 惯量参数 $(m,\mathbf c,G_o)$ 及其可实现性条件 | `RigidBodyInertiaParameters`，见 [`multibody_physical_parameters.h`](../../../libs/multibody_runtime/include/orvd/multibody_runtime/multibody_physical_parameters.h)；`ThrowIfNotRealisableInertia`，见 [`multibody_physical_parameter_validation.cc`](../../../libs/multibody_runtime/src/multibody_physical_parameter_validation.cc) |
| 质心惯量到体原点单位惯量的换算；$m>0$ 与自由体正定要求 | `UnitInertiaAboutBodyOrigin`，见 [`assemble_vehicle_multibody_model.cc`](../../../libs/configuration/src/assemble_vehicle_multibody_model.cc)；`ThrowIfSingularFreeBodyCenterOfMassInertia`，见 [`vehicle_definition_inertia.cc`](../../../libs/configuration/src/vehicle_definition_inertia.cc) |
| 重力向量 $\mathbf g_W=g\,\mathbf e_3$ | `GravitationalAccelerationInInertial`，见 [`track_inertial_frame.cc`](../../../libs/track_geometry/src/track_inertial_frame.cc)；`MultibodyModel::SetGravityVector`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 四元数块 $N_{\mathrm q}$ 与 $N_{\mathrm q}^{+}$ | `CalcQMatrix`、`QuaternionRateToAngularVelocityMatrix`、`DoMapVelocityToQDot`、`DoMapQDotToVelocity`，见 [`quaternion_floating_mobilizer.cc`](../../../external/drake_mbtree/drake/multibody/tree/quaternion_floating_mobilizer.cc) |
| Ball-RPY 块 $N_{\mathrm r}=H$、$N_{\mathrm r}^{+}=E$ | `DoMapVelocityToQDot`、`DoMapQDotToVelocity`，见 [`rpy_ball_mobilizer.cc`](../../../external/drake_mbtree/drake/multibody/tree/rpy_ball_mobilizer.cc) |
| $\dot q=N(q)v$ 与 $v=N^{+}(q)\dot q$ 的整体映射 | `MultibodyModel::MapGeneralizedVelocitiesToPositionDerivatives`、`MapGeneralizedPositionDerivativesToVelocities`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 位姿与坐标系原点的刚体固定点身份 | `MultibodyModel::CalcPoseInWorld`、`CalcFrameOriginAsBodyFixedPoint`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 空间速度、坐标系速度的搬移与相对速度契约 | `MultibodyModel::CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld`、`CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc)；`Frame::CalcSpatialVelocityInWorld`、`Frame::CalcSpatialVelocity`，见 [`frame.cc`](../../../external/drake_mbtree/drake/multibody/tree/frame.cc) |
| 运动子空间 $S_B$ 与速度递推 | `BodyNodeImpl::CalcAcrossNodeJacobianWrtVExpressedInWorld`、`CalcVelocityKinematicsCache_BaseToTip`，见 [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) |
| 体扳手条目及其换系、搬到体原点与累加 | `AppliedBodyWrench`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h)；`MultibodyModel::EvaluateForwardDynamics`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc)；`RigidBody::AddInForce`，见 [`rigid_body.cc`](../../../external/drake_mbtree/drake/multibody/tree/rigid_body.cc) |
| 固定点 Jacobian $J_Q$ | `MultibodyModel::CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 重力扳手 $(\mathbf c_W\times m\mathbf g_W;m\mathbf g_W)$ 与 $\tau=J^{\mathsf T}\mathcal W$ 的叶到根一趟 | `UniformGravityFieldElement::AccumulateGravitySpatialForces`，见 [`uniform_gravity_field_element.cc`](../../../external/drake_mbtree/drake/multibody/tree/uniform_gravity_field_element.cc)；`BodyNodeImpl::CalcSystemJacobianTransposeTimesF_TipToBase`，见 [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) |
| 关节阻尼 $\tau_{\mathrm d}=-c\,\dot\theta$ 及其计入 | `RevoluteJoint::DoAddInDamping`，见 [`revolute_joint.h`](../../../external/drake_mbtree/drake/multibody/tree/revolute_joint.h)；`MultibodyTree::CalcForceElementsContribution`、`AddJointDampingForces`，见 [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc) |
| 速度偏置 $F_{\mathrm b,B}$、偏置加速度 $A_{\mathrm b,B}$ 与逆动力学两趟 | `MultibodyTree::CalcDynamicBiasForces`、`CalcInverseDynamics`，见 [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc)；`BodyNodeImpl::CalcSpatialAccelerationBias`、`CalcInverseDynamics_TipToBase`，见 [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) |
| 动力学查询：$M(q)$、$C(q,v)v$、$\tau_{\mathrm g}$、$\tau_{\mathrm d}$、$\tau_{\mathrm{required}}$ | `MultibodyModel::CalcGeneralizedMassMatrix`、`CalcVelocityBiasGeneralizedForces`、`CalcGravityAppliedGeneralizedForces`、`CalcJointDampingAppliedGeneralizedForces`、`CalcRequiredGeneralizedForces`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 复合体惯量 $K_B$ 与质量矩阵分块 | `MultibodyTree::CalcMassMatrix`，见 [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc)；`BodyNodeImpl::CalcMassMatrixContributionViaWorld_TipToBase`，见 [`body_node_impl_mass_matrix.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl_mass_matrix.cc) |
| 铰接体算法三趟：$P_B$、$D_B$、$g_B$、$P_B^{+}$；$Z_B$、$e_B$、$Z_B^{+}$；$\dot v_B$、$A_B$ | `BodyNodeImpl::CalcArticulatedBodyInertiaCache_TipToBase`、`CalcArticulatedBodyForceCache_TipToBase`、`CalcArticulatedBodyAccelerations_BaseToTip`，见 [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc)；`BodyNode::CalcArticulatedBodyHingeInertiaMatrixFactorization`，见 [`body_node.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node.cc)；`MultibodyTree::CalcArticulatedBodyForceCache`、`CalcArticulatedBodyAccelerations`，见 [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc) |
| 多体门面的 $[N(q)v;\dot v]$ | `MultibodyModel::CalcGeneralizedVelocityDerivatives`、`CalcStateTimeDerivatives`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 驱动力矩偶 $(\pm\tau\,\mathbf a;\mathbf 0)$ | `IndependentWheelActiveTorquePlan::CalcAppliedForces`，见 [`independent_wheel_active_torque_plan.cc`](../../../libs/forces/src/independent_wheel_active_torque_plan.cc)；`EmitCoupleWrenchPair`，见 [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h) |
| 三类载荷源与 $[N(q)v;\dot v;\dot z]$ 的拼接 | `CompiledSystemPlan::CalcStateTimeDerivatives`，见 [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc) |
| 积分器给出的 $(t,y)$ 到试算上下文 | `SystemRhsBridge::CalcTimeDerivatives`，见 [`system_rhs_bridge.cc`](../../../libs/integrators/src/system_rhs_bridge.cc) |
