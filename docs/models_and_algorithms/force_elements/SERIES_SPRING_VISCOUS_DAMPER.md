[English](SERIES_SPRING_VISCOUS_DAMPER.en.md)

# 串联弹簧—黏性阻尼力元

本篇是力元本构文档中的一族，说明一个线性弹簧与一个线性黏性阻尼器串联、沿参考端坐标系某一根轴作用的力元。串联使二者共享同一个力而变形各异，力因此不再是相对运动的代数函数，而是一个一阶内部状态。本篇由串联的同力条件与变形相加条件推出 Maxwell 内力方程，定义松弛时间，给出阶跃与正弦激励下的解析响应，说明这个力状态如何进入系统连续状态的 $z$ 块、其导数如何与两条载荷在同一次求值中给出，并证明本族满足[力元连接运动学与空间扳手](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)第 4.5 节的组织原则。两端相对运动、三种扳手施加方式与功率恒等式均在该共用篇推得，本篇只引用不重推。元件类型见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) 的 `SeriesSpringViscousDamper`；本构求值与状态导数在 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) 的 `VehicleForcePlan::CalcAppliedForces` 中形成。

## 1. 对象与记号

### 1.1 元件

元件连接参考端坐标系 A 与对端坐标系 C，二者分别固定在刚体 $\mathcal A$ 与 $\mathcal C$ 上，含义与共用篇第 1.1 节相同。元件由刚度为 $k$ 的线性弹簧与阻尼为 $c$ 的线性黏性阻尼器首尾相接构成，只沿 A 的第 $j$ 根坐标轴 $\mathbf e_j$（$j\in\{1,2,3\}$）传力；沿另外两根轴它不传任何力。$k$、$c$ 与作用轴都在 A 中陈述，因此和共用篇的其他族一样，参考端的选择是本构的一部分。

元件传递的力记为 $F$，它是本族唯一的内部状态。$F>0$ 时参考端所受的力沿 $+\mathbf e_j$，对端所受的力沿 $-\mathbf e_j$；对端沿 $+\mathbf e_j$ 相对参考端的运动驱动 $F$ 增大，即参考端所受的力随对端相对运动的方向，这与[三向平动弹簧—阻尼力元](TRANSLATIONAL_SPRING_DAMPER.md)中 $\mathbf f_A$ 的正向一致。

### 1.2 记号表

| 记号 | 含义 | 来源 |
|---|---|---|
| A、C、$\mathcal A$、$\mathcal C$、I | 参考端、对端、两刚体与惯性系 | 共用篇 §1.1 |
| $\mathbf d_A$、$\mathbf d_I$ | 两端原点的相对位置，分别在 A、I 中表达 | 共用篇 §2.1 |
| $R_{IA}$、$R_{AC}$ | 参考端在 I 中的姿态，对端在参考端中的相对姿态 | 共用篇 §1.2 |
| $\mathbf u_A$ | 对端原点相对参考端（相对刚体 $\mathcal A$）的速度，在 A 中表达，$\mathbf u_A=\dot{\mathbf d}_A$ | 共用篇 §2.3 |
| $\boldsymbol\omega_A$、$\boldsymbol\omega_{rel,A}$ | 刚体 $\mathcal A$ 的角速度（在 I 中表达），C 相对 A 的角速度（在 A 中表达） | 共用篇 §1.2 |
| $\mathbf f_A$、$\mathbf f_I$ | 参考端所受的力，分别在 A、I 中表达 | 共用篇 §1.3 |
| $\mathcal W_Q^E[\mathcal A]$ | 作用于刚体 $\mathcal A$、关于点 Q 取矩、在 E 中表达的扳手 | 共用篇 §1.3 |
| $\mathbf r_{A_o}^{\mathcal A}$、$\mathbf r_{C_o}^{\mathcal C}$ | 两端原点在各自刚体中的体坐标 | 共用篇 §3.1 |
| $\mathcal P$ | 力元向两刚体输送的总功率 | 共用篇 §1.2 |
| $\mathbf e_j$ | A 的第 $j$ 个标准基向量，即元件的作用轴 | 基座 §4.1 |
| $[q;v;z]$、$N(q)$ | 系统连续状态与位置导数映射 | 基座 §5.1、§4.5 |
| $k$、$c$ | 串联刚度与串联阻尼，正标量 | 基座 §5.3 |
| $u$ | 两端沿作用轴的相对速度，$u=\mathbf e_j^{\mathsf T}\mathbf u_A$ | 本篇新增 |
| $F$ | 元件传递的力，即内部力状态 | 本篇新增 |
| $\delta_k$、$\delta_c$ | 弹簧与阻尼器各自的伸长，只在推导中出现 | 本篇新增 |
| $\tau_{\mathrm r}$ | 松弛时间 $c/k$ | 本篇新增 |
| $\mathcal V$、$\mathcal D$ | 弹簧储能与阻尼器耗散功率 | 共用篇 §1.2 |
| $\omega$、$\hat u$、$\vartheta$ | 正弦激励的角频率、速度幅值与力相对速度的相位滞后 | 本篇新增 |
| $\Delta$、$u_0$、$F_0$ | 相对位置阶跃的幅值、相对速度阶跃的幅值、阶跃前的力 | 本篇新增 |
| $\sigma$ | 卷积积分的哑时间变量 | 本篇新增 |

### 1.3 与共用篇及基座记号的关系

本篇的旋转矩阵、扳手、表达系下标与共用篇第 1.3 节完全一致；$R_{IA}\mathbf e_j$ 按基座第 4.1 节指 $R_{IA}$ 的第 $j$ 列。以下符号在本篇局部使用，与其他篇的同名符号无关，均在此声明：标量 $u$ 是共用篇相对速度 $\mathbf u_A$ 的第 $j$ 个分量，是速度，不是[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 1.2 节的欧氏位移 $u$，该篇第 1.1 节把同一个量写作 $v_{\mathrm{rel}}$；$F$ 沿用该篇第 1.1 节的力状态记号，与轮轨接触链的 $F_e$、$F_d$ 无关；$\delta_k$、$\delta_c$ 是元件内部两段的伸长，与接触链的穿透 $\delta$ 无关；$\tau_{\mathrm r}$ 是标量时间，与共用篇的力矩 $\boldsymbol\tau$ 及基座第 5.3 节的广义力 $\tau$ 无关；$\omega$ 是标量激励频率，与角速度 $\boldsymbol\omega_A$ 及其分量无关。功率沿用共用篇的约定：以参考端所受载荷为正向量，$\mathcal P$ 是力元向两刚体输送的功率，$-\mathcal P$ 是力元吸收的功率。

## 2. 模型与推导

### 2.1 由串联条件到 Maxwell 内力方程

弹簧与阻尼器串联，意味着两个条件同时成立：二者传递同一个力 $F$，二者的伸长相加等于元件的总伸长。设弹簧伸长为 $\delta_k$、阻尼器伸长为 $\delta_c$，则

$$
F=k\,\delta_k,
\qquad
F=c\,\dot\delta_c,
\qquad
\dot\delta_k+\dot\delta_c=u.
$$

第三式的右端是两端沿作用轴的相对速度 $u=\mathbf e_j^{\mathsf T}\mathbf u_A$：元件的总伸长率就是对端相对参考端沿 $\mathbf e_j$ 的速度分量。共用篇第 6 节已说明运动学层不提供自然长度，总伸长本身因此没有零点；本族的本构只用到它的变化率，零点从不出现。

对第一式求导，用第二式消去 $\dot\delta_c=F/c$，再用第三式，

$$
\dot F=k\,\dot\delta_k=k\left(u-\dot\delta_c\right)=k\left(u-\frac{F}{c}\right),
$$

即 Maxwell 内力方程

$$
\dot F=k\,u-\frac{k}{c}\,F.
$$

定义松弛时间

$$
\tau_{\mathrm r}=\frac{c}{k},
$$

方程可写成 $\tau_{\mathrm r}\dot F+F=c\,u$。它是以 $F$ 为未知量的一阶线性常微分方程，$u$ 是由多体运动决定的输入。推导中除以了 $c$，并以 $k$ 把 $\dot\delta_k$ 换成 $\dot F$，这两步分别要求 $c\ne0$ 与 $k\ne0$；两者任一为零的退化在第 4 节讨论。

力之所以成为状态，是因为串联结构多出一个运动学层看不见的自由度：多体状态 $(q,v)$ 只决定 $\delta_k+\delta_c$ 的变化率，不决定 $\delta_k$ 与 $\delta_c$ 各自的值；给定同样的相对运动历史，力还取决于弹簧此刻分到多少伸长。这个自由度可以用 $\delta_k$ 表示，也可以用与之成正比的 $F=k\,\delta_k$ 表示；本族取 $F$，因为载荷直接需要它。$\delta_c$ 从不需要单独求出：它的变化率 $F/c$ 由 $F$ 决定，而它的绝对值与总伸长一样没有零点。

### 2.2 变形度量与速度输入

本族不读任何位移：$\mathbf d_A$、$R_{AC}$ 与 $\boldsymbol\omega_{rel,A}$ 都不进入本构。唯一的运动学输入是 $u=\mathbf e_j^{\mathsf T}\mathbf u_A$，其中 $\mathbf u_A$ 是共用篇第 2.3 节含运输项的相对速度，即 $\dot{\mathbf d}_A$。因此 $u=\dot d_{A,j}$ 是相对位置 $\mathbf d_A$ 第 $j$ 个分量的时间导数，而不是惯性系速度差 $\dot{\mathbf d}_I$ 沿作用轴的投影；二者按共用篇第 6 节只在垂直于两原点连线的分量上不同，作用轴不沿连线时不可互换。$\mathbf d_A$ 只在写出支承矩时出现，见第 2.4 节。

### 2.3 解析响应

以下各式是本构方程的符号解，用来陈述性质。取 $t_0$ 时刻的力为 $F(t_0)$，常数变易法给出对任意可积输入 $u$ 的解

$$
F(t)=F(t_0)\,e^{-(t-t_0)/\tau_{\mathrm r}}+k\int_{t_0}^{t}e^{-(t-\sigma)/\tau_{\mathrm r}}\,u(\sigma)\,d\sigma.
$$

核函数 $k\,e^{-t/\tau_{\mathrm r}}$ 是松弛函数，即元件对相对位置单位阶跃的力响应。它说明当前的力是相对速度历史的指数加权积分，权重在 $\tau_{\mathrm r}$ 的时间尺度上遗忘。

**相对位置阶跃（应力松弛）。** 设两端沿作用轴的相对位置在 $t=0$ 突增 $\Delta$，此前的力为 $F_0$，此后 $u=0$。突变瞬间阻尼器传递的力有限，其伸长率也有限，因此阻尼器来不及伸长，突增全部落在弹簧上，$F(0^+)=F_0+k\,\Delta$；此后

$$
F(t)=\left(F_0+k\,\Delta\right)e^{-t/\tau_{\mathrm r}},\qquad t>0.
$$

力以 $\tau_{\mathrm r}$ 为时间常数衰减到零：元件在静止位形下不传递任何力，没有静刚度。

**相对速度阶跃。** 设 $t\ge0$ 时 $u=u_0$ 为常数，$F(0)=F_0$，则

$$
F(t)=c\,u_0+\left(F_0-c\,u_0\right)e^{-t/\tau_{\mathrm r}},\qquad t\ge0.
$$

从 $F_0=0$ 出发时，初始斜率 $\dot F(0)=k\,u_0$ 是纯弹簧的响应，稳态值 $c\,u_0$ 是纯阻尼器的响应；由前者过渡到后者所需的时间以 $\tau_{\mathrm r}$ 度量。

**正弦稳态。** 设 $u(t)=\hat u\sin\omega t$，衰减的齐次部分消失后

$$
F(t)=\frac{c\,\hat u}{1+(\omega\tau_{\mathrm r})^2}\left(\sin\omega t-\omega\tau_{\mathrm r}\cos\omega t\right)
=\frac{c\,\hat u}{\sqrt{1+(\omega\tau_{\mathrm r})^2}}\,\sin\left(\omega t-\vartheta\right),
\qquad
\tan\vartheta=\omega\tau_{\mathrm r},
$$

其中 $0\le\vartheta<\pi/2$ 是力相对速度的相位滞后。以复相量写，力与速度之比为 $c/(1+\mathrm i\omega\tau_{\mathrm r})$；再除以位移相量 $\hat u/(\mathrm i\omega)$ 得复刚度

$$
k^*(\omega)=\frac{\mathrm i\omega c}{1+\mathrm i\omega\tau_{\mathrm r}}
=k\,\frac{(\omega\tau_{\mathrm r})^2}{1+(\omega\tau_{\mathrm r})^2}
+\mathrm i\,k\,\frac{\omega\tau_{\mathrm r}}{1+(\omega\tau_{\mathrm r})^2}.
$$

实部是储能刚度，随 $\omega\tau_{\mathrm r}$ 从零单调增到 $k$；虚部是损耗刚度，在 $\omega\tau_{\mathrm r}=1$ 处取最大值 $k/2$。$\omega\tau_{\mathrm r}\ll1$ 时 $k^*\approx\mathrm i\omega c$，元件表现为阻尼 $c$；$\omega\tau_{\mathrm r}\gg1$ 时 $k^*\to k$，元件表现为刚度 $k$。一个周期内的平均耗散功率为

$$
\bar{\mathcal D}=\frac{c\,\hat u^2}{2\left(1+(\omega\tau_{\mathrm r})^2\right)}.
$$

### 2.4 载荷与支承矩

本构给出的是参考端所受的力，在 A 中只有第 $j$ 个分量：

$$
\mathbf f_A=F\,\mathbf e_j,
\qquad
\mathbf f_I=R_{IA}\mathbf f_A=F\,R_{IA}\mathbf e_j.
$$

载荷按共用篇第 3.2 节的端点力对加参考端支承矩施加：

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

支承矩在 A 中是 $F\,\mathbf d_A\times\mathbf e_j$。它没有沿 $\mathbf e_j$ 的分量，在两端原点的连线与作用轴平行或 $F=0$ 时为零；作用轴是 A 的坐标轴而不是连线方向，一般情形下支承矩非零。

### 2.5 功率原则、储能与耗散

本族采用共用篇第 4.5 节表中的第一行：力对施于两端原点并配参考端支承矩，速度输入为 $\mathbf u_A$。共用篇第 4.2 节已证明这种施加方式下两刚体收到的总功率为 $\mathcal P=-\mathbf f_A\cdot\mathbf u_A$。代入 $\mathbf f_A=F\,\mathbf e_j$，

$$
\mathcal P=-F\,\mathbf e_j^{\mathsf T}\mathbf u_A=-F\,u.
$$

另一方面，元件按本构吸收的功率是它传递的力与总伸长率的乘积 $F\,u$。两者恰好互为相反数，本族因此满足组织原则：本构律读取的速度 $u$ 正是与扳手施加位置共轭的量。若去掉支承矩，端点功率按共用篇第 4.2 节变为 $-\mathbf f_I\cdot\dot{\mathbf d}_I=-F\,\mathbf e_j^{\mathsf T}R_{IA}^{\mathsf T}\dot{\mathbf d}_I$，与 $-F\,u$ 相差支承矩的功率 $F\,\boldsymbol\omega_A\cdot\left(\mathbf d_I\times R_{IA}\mathbf e_j\right)$，本构所记的功与元件对两刚体所做的功便不再相同。支承矩因此是使两种功率相等的必要项，而不是只为配平净力矩而添加的修正。

把吸收功率按串联条件拆开，

$$
F\,u=F\dot\delta_k+F\dot\delta_c
=k\,\delta_k\dot\delta_k+c\,\dot\delta_c^{\,2}
=\frac{d}{dt}\left(\frac{F^2}{2k}\right)+\frac{F^2}{c}.
$$

定义储能与耗散功率

$$
\mathcal V=\frac{F^2}{2k},
\qquad
\mathcal D=\frac{F^2}{c}=c\,\dot\delta_c^{\,2},
$$

则

$$
-\mathcal P=\dot{\mathcal V}+\mathcal D.
$$

直接由内力方程验证：$\dot{\mathcal V}=F\dot F/k=F\,u-F^2/c$。储能只是状态 $F$ 的函数，与多体位形无关，弹性部分的功率因此是全导数；$c>0$ 时耗散 $\mathcal D\ge0$ 恒成立。对任一时段积分得 $\int_{t_0}^{t}(-\mathcal P)\,d\sigma\ge\mathcal V(t)-\mathcal V(t_0)\ge-\mathcal V(t_0)$：元件能向两刚体净输出的能量不超过初始时刻弹簧中的储能 $F(t_0)^2/(2k)$，元件是无源的。

## 3. 计算实现

### 3.1 相对运动的取用

每次求值，元件先按共用篇第 5.1 节取得一份相对运动。本族只消费其中 $\mathbf u_A$ 的第 $j$ 个分量作为 $u$，以及写出载荷所需的 $R_{IA}$、$\mathbf d_I$ 与两端原点的刚体固定点；$\mathbf d_A$、$R_{AC}$、$\boldsymbol\omega_{rel,A}$ 不参与。作用轴是元件的一个离散属性，三个取值分别对应 A 的第一、二、三轴。

### 3.2 本构求值与状态导数

求值输入包括当前的力状态 $F$，它来自系统连续状态 $z$ 块中属于该元件的那一个分量。元件在同一次求值中写出两样东西：状态导数

$$
\dot F=k\,u-\frac{k}{c}\,F,
$$

写入 $\dot z$ 中对应的分量；以及代数输出 $\mathbf f_A=F\,\mathbf e_j$，转为 $\mathbf f_I$ 后交给载荷写入。两者的依赖关系不同：标量力只依赖状态 $F$，不含相对速度；状态导数依赖 $u$ 与 $F$。相对速度因此不直接产生力，只通过改变 $F$ 的演化间接影响力，标量本构对速度没有代数直通项。空间扳手则仍代数地依赖位形：$\mathbf f_I=F\,R_{IA}(q)\mathbf e_j$ 随姿态转动，支承矩 $\mathbf d_I(q)\times\mathbf f_I$ 随力臂变化。这两条计算之间也没有代数环：载荷不依赖 $\dot F$，$\dot F$ 不依赖载荷，二者由同一份相对运动与同一个 $F$ 各自算出。

### 3.3 扳手的写出

$\mathbf f_I=F\,R_{IA}\mathbf e_j$ 与 $\mathbf d_I$ 交给共用篇第 5.2 节的第一条路径，写出 $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ 与 $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$ 两条载荷。这与[三向平动弹簧—阻尼力元](TRANSLATIONAL_SPRING_DAMPER.md)的写出路径相同，区别只在 $\mathbf f_A$ 由状态给出且只有一个非零分量。

### 3.4 与系统状态和积分器的衔接

基座第 5.1 节的连续状态 $[q;v;z]$ 中，$z$ 块由全部串联元件的力状态依次排列而成，每个元件恰占一个分量，没有串联元件的系统 $z$ 块为空。完整状态导数按

$$
\frac{d}{dt}\begin{bmatrix}q\\v\\z\end{bmatrix}
=\begin{bmatrix}N(q)\,v\\ \dot v(q,v,z)\\ \dot z(q,v,z)\end{bmatrix}
$$

装配：力元求值一次性给出全部载荷与 $\dot z$；多体层把载荷按共用篇第 3.5 节换点到刚体原点并解前向动力学得到 $\dot v$；系统装配层把 $[N(q)v;\dot v]$ 与 $\dot z$ 拼接。$\dot v$ 对 $z$ 的依赖来自 $\mathbf f_A=F\,\mathbf e_j$，$\dot z$ 对 $v$ 的依赖来自 $u$，$\dot z$ 对 $z$ 自身的依赖是对角的，每个元件的对角元为 $-1/\tau_{\mathrm r}$。积分器把 $F$ 当作与 $q$、$v$ 同等的状态分量推进，初值问题因此需要 $F(t_0)$：它不能由 $(q,v)$ 代数确定，是初始状态的一部分。$z$ 块在各积分方法中的位置见[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 1.1 节。

## 4. 数学性质与适用条件

- **参数域与状态的存在性。** 本族定义在 $k>0$、$c>0$ 上，此时松弛时间 $\tau_{\mathrm r}=c/k$ 有限且为正。$k=0$ 时弹簧传不了力，$F=k\,\delta_k\equiv0$；$c=0$ 时阻尼器在有限伸长率下传不了力，$F=c\,\dot\delta_c\equiv0$。两种情形下元件传递的力都被代数地钉在零，不存在需要积分的内部自由度。内力方程本身也随之失效：$k=0$ 时它退化为 $\dot F=0$，与运动脱钩，其解是任意常数而非串联条件所要求的零；$c=0$ 时 $k/c$ 没有定义，对应松弛时间为零的瞬时松弛。因此 $k$、$c$ 任一为零的元件是代数元件，不属于本族，也不占 $z$ 块的分量。
- **两个无穷极限也是代数元件。** $k\to\infty$（$c$ 固定，$\tau_{\mathrm r}\to0$）时，经过一段以 $\tau_{\mathrm r}$ 度量的初始松弛层后 $F\to c\,u$，是沿单轴的纯黏性阻尼器，力是 $v$ 的代数函数；$c\to\infty$（$k$ 固定，$\tau_{\mathrm r}\to\infty$）时 $\dot F=k\,\dot d_{A,j}$，积分得 $F(t)=F(t_0)+k\,[d_{A,j}(t)-d_{A,j}(t_0)]$，是沿单轴带常值力 $F(t_0)-k\,d_{A,j}(t_0)$ 的纯弹簧，力是 $q$ 的代数函数。两个极限分别是三向平动族沿一根轴的阻尼与弹簧，本族恰是介于二者之间、$0<\tau_{\mathrm r}<\infty$ 的情形。
- **稳定性与无源性。** 状态方程的特征值为 $-1/\tau_{\mathrm r}<0$，齐次解单调衰减；$\mathcal V\ge0$ 与 $\mathcal D\ge0$ 使元件无源。$k<0$ 或 $c<0$ 会使特征值为正或使耗散变号，元件主动做功，不在本族的定义域内。
- **无静刚度。** 相对位置保持不变时力以 $\tau_{\mathrm r}$ 衰减到零，元件不承担任何静载，需要静态支承的方向必须由其他元件或约束提供。本族适合表示带串联柔度的黏性阻尼器：$\omega\tau_{\mathrm r}\gg1$ 时趋于刚度 $k$，$\omega\tau_{\mathrm r}\ll1$ 时趋于阻尼 $c$。
- **力对速度跳变连续。** $F$ 是 $u$ 的积分量：$u$ 可积时 $F$ 连续，$u$ 连续时 $F$ 连续可微。相对速度发生跳变时，代数阻尼器的力随之跳变，本族的力只改变斜率。
- **状态方程的刚性。** $-1/\tau_{\mathrm r}$ 是系统 Jacobian 中 $\partial\dot F/\partial F$ 的对角元，也是孤立内力方程的特征值，但不必是完整耦合系统的特征值：$F$ 通过载荷影响 $\dot v$，$v$ 通过 $u$ 影响 $\dot F$，耦合后的特征值一般同时受运动方程支配（读同一 $u$ 的两个同参数元件，其内力差 $\dot D=-D/\tau_{\mathrm r}$ 与运动脱钩，是耦合系统恰好保留该特征值的例子）。$\tau_{\mathrm r}$ 远小于多体运动的时间尺度时，时间尺度分离使一组快特征值接近 $-1/\tau_{\mathrm r}$，系统呈刚性；隐式方法对此的处理见[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 2 节。
- **单轴、固定于参考端。** 作用轴 $\mathbf e_j$ 随刚体 $\mathcal A$ 转动，不随两原点连线转动；元件不感知垂直于该轴的相对运动，也不传递沿这些方向的力。互换两端后作用轴变为 C 的第 $j$ 轴，除非 $R_{AC}\mathbf e_j=\mathbf e_j$ 恒成立，否则得到不同的元件。支承矩 $F\,\mathbf d_A\times\mathbf e_j$ 随两原点垂直于作用轴的偏距增长。
- **线性与叠加。** 方程对 $(u,F)$ 线性且时不变，第 2.3 节的卷积解对任意可积输入成立，多个输入的响应可以叠加。
- **初值的物理含义。** 从平衡起步（$u\equiv0$ 且力已松弛）时 $F(t_0)=0$；从匀速相对运动的稳态起步时 $F(t_0)=c\,u_0$。任何其他初值都会经历一段以 $\tau_{\mathrm r}$ 度量的暂态。

## 5. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 元件类型：两端、作用轴、$k$ 与 $c$ | `SeriesSpringViscousDamper`、`ForceElementAxis`，见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 相对运动记录，$u$ 取自其中 $\mathbf u_A$ 的第 $j$ 个分量 | `VehicleForcePlan::CalcAppliedForces` 内的 `CalcRelativeMotion`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 状态导数 $\dot F=k\,u-(k/c)F$ 与代数输出 $\mathbf f_A=F\,\mathbf e_j$ | `VehicleForcePlan::CalcAppliedForces` 的串联族分支，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 端点力对与参考端支承矩 | `VehicleForcePlan::CalcAppliedForces` 内的 `EmitTranslationalWrenchPair`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 求值契约：$F$ 为输入、$\dot F$ 为输出、每元件一个状态分量 | `VehicleForcePlan::CalcAppliedForces`、`VehicleForcePlan::series_spring_damper_force_state_count`，见 [`vehicle_force_plan.h`](../../../libs/forces/include/orvd/forces/vehicle_force_plan.h) |
| $z$ 块在 $[q;v;z]$ 中的区间 | `SystemInstance::series_spring_damper_force_state_range`，见 [`system_instance.h`](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
| 载荷与 $\dot z$ 拼成完整状态导数 | `CompiledSystemPlan::CalcStateTimeDerivatives`，见 [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc) |
| 载荷条目 | `AppliedBodyWrench`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
