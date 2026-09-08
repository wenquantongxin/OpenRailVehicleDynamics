[English](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md)

# 力元连接运动学与空间扳手

本篇是力元本构文档的共用篇。它定义并推导两端连接坐标系之间的相对位置、相对姿态、相对平动速度与相对角速度，给出力元向两个刚体施加空间扳手的三种方式及其换点规则，并证明每种施加方式下两刚体收到的总功率等于对应的本构功率。五个本构族——[三向平动弹簧—阻尼力元](TRANSLATIONAL_SPRING_DAMPER.md)、[侧滚弹簧—阻尼力偶](ROLL_SPRING_DAMPER_COUPLE.md)、[串联弹簧—黏性阻尼力元](SERIES_SPRING_VISCOUS_DAMPER.md)、[奇对称饱和分段线性阻尼力元](SATURATED_PIECEWISE_LINEAR_DAMPER.md)与[半角中点 RPY 衬套](HALF_ANGLE_MIDPOINT_RPY_BUSHING.md)——都以本篇的记号陈述自己的变形度量、速度输入与载荷，并各自证明满足第 4.5 节的组织原则。相对运动由 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) 中 `VehicleForcePlan::CalcAppliedForces` 调用的 `CalcRelativeMotion` 一次求得。载荷条目的类型定义见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h)。

## 1. 对象与记号

### 1.1 两端、两刚体与三个坐标系

一个力元连接两个坐标系：参考端坐标系 A 与对端坐标系 C。二者分别刚性固定在多体模型的两个不同刚体上，本篇记这两个刚体为 $\mathcal A$ 与 $\mathcal C$。A 的原点 $A_o$ 是 $\mathcal A$ 上的一个固定材料点，C 的原点 $C_o$ 是 $\mathcal C$ 上的一个固定材料点；连接坐标系的姿态一般不同于所在刚体体坐标系的姿态，原点也不必与体坐标系原点重合。惯性系 I 见[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)第 2.1 节。

本目录内的字母 C 指力元的对端坐标系，与轮轨接触链中作为接触坐标系的 C（基座第 2.8 节）是局部重名；两条链不会出现在同一篇中。字母 B 预留给[半角中点 RPY 衬套](HALF_ANGLE_MIDPOINT_RPY_BUSHING.md)引入的半角中间系，本篇不构造它。

本篇给出的相对运动记录统一以参考端坐标系 A 为表达系；各族在此基础上自行选择本构坐标——平动族与侧滚族直接在 A 中陈述常数，串联族与饱和族的标量律沿 A 的一根轴陈述、只读取该轴上的速度分量，衬套族把平动常数放在半角中间系 B、把转动常数放在一组角坐标上。惯性系 I 只在推导与载荷写入时出现。

### 1.2 记号表

| 记号 | 含义 |
|---|---|
| A、C | 参考端与对端连接坐标系 |
| $\mathcal A$、$\mathcal C$ | 承载 A 与 C 的两个刚体 |
| I | 惯性系 |
| $\mathbf p_A$、$\mathbf p_C$ | 原点 $A_o$、$C_o$ 在 I 中的位置 |
| $\mathbf d_I$、$\mathbf d_A$ | 相对位置 $\mathbf p_C-\mathbf p_A$，分别在 I 与 A 中表达 |
| $R_{IA}$、$R_{IC}$、$R_{AC}$ | A、C 在 I 中的姿态，以及 C 在 A 中的相对姿态 |
| $\mathbf v_{Ao}$、$\mathbf v_{Co}$ | 原点 $A_o$、$C_o$ 相对 I 的速度，在 I 中表达 |
| $\boldsymbol\omega_A$、$\boldsymbol\omega_C$ | 刚体 $\mathcal A$、$\mathcal C$ 相对 I 的角速度，在 I 中表达 |
| $\mathbf u_A$、$\mathbf u_I$ | 对端原点相对参考端的速度，分别在 A 与 I 中表达 |
| $\boldsymbol\omega_{rel,A}$、$\boldsymbol\omega_{rel,I}$ | C 相对 A 的角速度，分别在 A 与 I 中表达 |
| $\mathbf x_m$ | 两端原点的瞬时世界中点 |
| $\mathbf u_{m,I}$、$\mathbf u_{m,A}$ | 两刚体在 $\mathbf x_m$ 处材料点的相对速度，分别在 I 与 A 中表达 |
| $\mathbf v^{(\mathcal A)}(\mathbf x)$ | 刚体 $\mathcal A$ 上与空间点 $\mathbf x$ 重合的材料点的速度 |
| $\mathbf f$、$\boldsymbol\tau$、$\mathbf m$ | 参考端所受的力、力矩与纯力偶，表达系由末下标标出 |
| $\mathcal W_Q^E[\mathcal A]$ | 作用于刚体 $\mathcal A$、关于点 Q 取矩、在 E 中表达的空间扳手 |
| $\mathbf x_{\mathcal A}$、$R_{I\mathcal A}$ | 刚体 $\mathcal A$ 体坐标系原点在 I 中的位置与姿态 |
| $\mathbf r_Q^{\mathcal A}$ | 点 Q 在刚体 $\mathcal A$ 体坐标系中的坐标 |
| $\mathcal P$ | 力元向两刚体输送的总功率 |
| $\mathcal V$、$\mathcal D$ | 力元的储能函数与耗散率；各族篇在各自的变量上定义，需要区分多个部分时加下标 |
| $\mathbf a\circ\mathbf b$ | 两个向量的逐分量乘积 |
| $\operatorname{skew}(\mathbf w)$ | 满足 $\operatorname{skew}(\mathbf w)\mathbf x=\mathbf w\times\mathbf x$ 的反对称矩阵 |

### 1.3 与基座记号的关系

本篇沿用基座第 4.1 节的旋转矩阵规则：$R_{AB}$ 把 B 中分量变到 A 中，$R_{AC}=R_{AB}R_{BC}$，矩阵元 $R_{ij}$ 按一基编号。基座的长下标写法与本篇简写的对应为 $\mathbf v_{Ao}=\mathbf v_{IAo\_I}$、$\boldsymbol\omega_A=\boldsymbol\omega_{IA\_I}$、$\mathbf u_A=\mathbf v_{ACo\_A}$、$\boldsymbol\omega_{rel,A}=\boldsymbol\omega_{AC\_A}$。共用运动学只涉及这三个坐标系，衬套族另引入半角中间系 B；$\mathbf u$ 与 $\boldsymbol\omega_{rel}$ 始终是相对 A 度量的量，其下标只标表达系。

扳手沿用基座第 5.2 节的 $\mathcal W_Q^E$ 与换点公式 $\boldsymbol\tau_O^E=\boldsymbol\tau_Q^E+\mathbf p_{OQ}^E\times\mathbf f^E$，并作两处扩充，均在此声明：为标明受力刚体，扳手记号后加方括号，$\mathcal W_Q^E[\mathcal A]$ 是作用于刚体 $\mathcal A$ 的扳手；写出扳手分量时，分量向量的表达系按基座第 4.1 节以末下标标出（如 $\mathbf f_I$），不再重复上标。

以下符号在本目录内另有含义并与其他目录区分：粗体 $\mathbf d$ 是两端原点的相对位置向量，不是轮轨接触链已释放的标量穿透；$\mathbf u$ 是相对速度，与[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 1.2 节的欧氏位移 $u$ 无关；功率写作 $\mathcal P$，以避开轮侧作用点 `P`。力元级刚度与阻尼按基座第 5.3 节一律小写 $k$、$c$ 或其向量形式 $\mathbf k$、$\mathbf c$。

未加作用端标记的 $\mathbf f$、$\boldsymbol\tau$、$\mathbf m$ 一律指参考端所受载荷。两端所受的力互为相反数，两端所受的纯力偶互为相反数，两端的扳手关于同一取矩点亦互为相反数；但各端关于自身作用点写出的力矩不是——端点力对中参考端另带支承矩 $\mathbf d_I\times\mathbf f_I$ 而对端为零（第 3.2 节）。某族需要以对端所受载荷陈述本构时，把作用端写成第一个下标、表达系写成第二个下标，例如 $\mathbf f_{C,B}$ 是作用于 C 端、在 B 中表达的力。

## 2. 相对运动

### 2.1 相对位置与相对姿态

两端原点的相对位置在 I 与 A 中分别为

$$
\mathbf d_I=\mathbf p_C-\mathbf p_A,
\qquad
\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I.
$$

对端在参考端中的相对姿态为

$$
R_{AC}=R_{IA}^{\mathsf T}R_{IC}.
$$

$\mathbf d_A$ 是平动变形度量的公共来源：它是原点之差，不含任何自然长度或零位移向量；两原点重合时 $\mathbf d_A=\mathbf 0$。平动族直接以它为变形度量，衬套族改用它在 B 中的表示，串联族与饱和族的标量律不读取位移、只在支承矩 $\mathbf d_I\times\mathbf f_I$ 中用到它。$R_{AC}$ 是各族转动变形度量的唯一来源，具体取矩阵元还是取某种角坐标由各族自行定义。

### 2.2 刚体材料点的速度搬移

设刚体 $\mathcal A$ 的角速度为 $\boldsymbol\omega_A$，其上材料点 $A_o$ 的速度为 $\mathbf v_{Ao}$。刚体上任一材料点，以及把刚体刚性延拓后与任一空间点 $\mathbf x$ 重合的材料点，速度为

$$
\mathbf v^{(\mathcal A)}(\mathbf x)=\mathbf v_{Ao}+\boldsymbol\omega_A\times\left(\mathbf x-\mathbf p_A\right).
$$

对 $\mathcal C$ 同理，$\mathbf v^{(\mathcal C)}(\mathbf x)=\mathbf v_{Co}+\boldsymbol\omega_C\times\left(\mathbf x-\mathbf p_C\right)$。这条搬移公式是本篇的基本工具：第 2.3 节用它解释运输项，第 2.5 节用它构造中点速度，第 4 节用它计算扳手功率。

### 2.3 含运输项的相对速度

姿态矩阵的时间导数为 $\dot R_{IA}=\operatorname{skew}(\boldsymbol\omega_A)R_{IA}$，从而 $\dot R_{IA}^{\mathsf T}=-R_{IA}^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_A)$。对 $\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I$ 求导，

$$
\dot{\mathbf d}_A
=R_{IA}^{\mathsf T}\dot{\mathbf d}_I-R_{IA}^{\mathsf T}\left(\boldsymbol\omega_A\times\mathbf d_I\right)
=R_{IA}^{\mathsf T}\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right).
$$

本篇把这个量定义为对端原点相对参考端的速度：

$$
\mathbf u_I=\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I,
\qquad
\mathbf u_A=R_{IA}^{\mathsf T}\mathbf u_I=\dot{\mathbf d}_A.
$$

$-\boldsymbol\omega_A\times\mathbf d_I$ 是运输项。按第 2.2 节，$\mathbf v_{Ao}+\boldsymbol\omega_A\times\mathbf d_I=\mathbf v^{(\mathcal A)}(\mathbf p_C)$ 是刚体 $\mathcal A$ 上此刻与 $C_o$ 重合的材料点的速度，所以 $\mathbf u_I$ 是 $C_o$ 相对于刚体 $\mathcal A$ 的速度，而不是相对于点 $A_o$ 的速度。这正是多体层相对空间速度查询的契约：参考系的速度先搬到被测原点，再作差。$R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ 与 $\mathbf u_A$ 只在垂直于 $\mathbf d$ 的分量上不同，因为 $\mathbf d_I\cdot(\boldsymbol\omega_A\times\mathbf d_I)=0$，二者沿两原点连线的分量相同。

### 2.4 相对角速度与相对姿态的导数

相对角速度是两刚体角速度之差：

$$
\boldsymbol\omega_{rel,I}=\boldsymbol\omega_C-\boldsymbol\omega_A,
\qquad
\boldsymbol\omega_{rel,A}=R_{IA}^{\mathsf T}\boldsymbol\omega_{rel,I}.
$$

它与相对姿态的导数由

$$
\dot R_{AC}=\operatorname{skew}(\boldsymbol\omega_{rel,A})\,R_{AC}
$$

联系。推导如下：$\dot R_{AC}=\dot R_{IA}^{\mathsf T}R_{IC}+R_{IA}^{\mathsf T}\dot R_{IC}=R_{IA}^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_C-\boldsymbol\omega_A)R_{IC}$，再用恒等式 $R^{\mathsf T}\operatorname{skew}(\mathbf w)R=\operatorname{skew}(R^{\mathsf T}\mathbf w)$ 与 $R_{IC}=R_{IA}R_{AC}$ 即得。任何由 $R_{AC}$ 定义的转动坐标，其时间导数都从这条式子出发；角坐标的导数一般不等于 $\boldsymbol\omega_{rel,A}$ 的分量，二者之间的映射由采用该坐标的族给出。

### 2.5 瞬时中点处的材料相对速度

两端原点的瞬时世界中点为

$$
\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I=\tfrac12\left(\mathbf p_A+\mathbf p_C\right).
$$

它不是任何一个刚体上的固定材料点；每一时刻，两刚体各有一个材料点与它重合。用第 2.2 节的搬移公式，这两个材料点的速度分别为 $\mathbf v^{(\mathcal A)}(\mathbf x_m)=\mathbf v_{Ao}+\tfrac12\boldsymbol\omega_A\times\mathbf d_I$ 与 $\mathbf v^{(\mathcal C)}(\mathbf x_m)=\mathbf v_{Co}-\tfrac12\boldsymbol\omega_C\times\mathbf d_I$，其差为

$$
\mathbf u_{m,I}
=\mathbf v^{(\mathcal C)}(\mathbf x_m)-\mathbf v^{(\mathcal A)}(\mathbf x_m)
=\mathbf v_{Co}-\mathbf v_{Ao}-\tfrac12\left(\boldsymbol\omega_A+\boldsymbol\omega_C\right)\times\mathbf d_I.
$$

与第 2.3 节的 $\mathbf u_I$ 相减并转入 A，

$$
\mathbf u_{m,A}=\mathbf u_A-\tfrac12\,\boldsymbol\omega_{rel,A}\times\mathbf d_A.
$$

$\mathbf u_{m,A}$ 是两刚体在公共中点处的材料相对速度。它一般不是任何位移量的时间导数，只有 $\boldsymbol\omega_{rel,A}\times\mathbf d_A=\mathbf 0$ 时才与 $\dot{\mathbf d}_A$ 重合。

## 3. 扳手的施加与换点

### 3.1 载荷条目与刚体固定点

力元向多体层交付的每一条载荷都是一个作用于某刚体的空间扳手：指明刚体、刚体固定点 Q 的体坐标、表达系 E、关于 Q 取矩的力矩与力，即 $\mathcal W_Q^E[\mathcal A]$。本目录的三种施加方式都在 I 中表达，每个力元恰写两条：一条给 $\mathcal A$，一条给 $\mathcal C$。作用点 Q 一律以刚体固定点交付；世界空间中的一个点 $\mathbf x$ 在刚体 $\mathcal A$ 上的体坐标为

$$
\mathbf r_Q^{\mathcal A}=R_{I\mathcal A}^{\mathsf T}\left(\mathbf x-\mathbf x_{\mathcal A}\right).
$$

两端原点 $A_o$、$C_o$ 的体坐标是常量；第 3.4 节的中点则每次求值重新换算。

### 3.2 端点力对与参考端支承矩

设 $\mathbf f_I$ 是参考端所受的力。第一种施加方式在两端原点施加等大反向的力，并在参考端附加支承矩：

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(\mathbf 0,\ -\mathbf f_I\right).
$$

支承矩 $\mathbf d_I\times\mathbf f_I$ 关于 $A_o$ 取矩，施加在刚体 $\mathcal A$ 上。这一对载荷的合力为零；关于任意点 O 的合力矩为 $(\mathbf p_A-\mathbf x_O)\times\mathbf f_I+\mathbf d_I\times\mathbf f_I-(\mathbf p_C-\mathbf x_O)\times\mathbf f_I=\mathbf 0$，因此成对扳手对系统不产生净力和净力矩。若没有支承矩，只要 $\mathbf f_I$ 不平行于 $\mathbf d_I$，两端点力就留下一个净力偶 $-\mathbf d_I\times\mathbf f_I$；但平衡只是支承矩的一个后果，它的必要性由第 4.2 节的功率恒等式给出。

各族给出的是 A 中的 $\mathbf f_A$，写入前按 $\mathbf f_I=R_{IA}\mathbf f_A$ 转入 I。

### 3.3 纯力偶对

设 $\mathbf m_I$ 是参考端所受的纯力偶。第二种施加方式在两端原点施加等大反向的纯力偶，不含力：

$$
\mathcal W_{A_o}^{I}[\mathcal A]=\left(\mathbf m_I,\ \mathbf 0\right),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=\left(-\mathbf m_I,\ \mathbf 0\right).
$$

纯力偶与取矩点无关，因此换点不改变它；载荷条目仍记录两端原点作为形式上的作用点。合力与合力矩均为零。各族给出的是 A 中的 $\mathbf m_A$，按 $\mathbf m_I=R_{IA}\mathbf m_A$ 转入 I。

### 3.4 同施于瞬时中点的扳手对

设 $\mathbf f_I$、$\boldsymbol\tau_I$ 是参考端所受的力与力矩。第三种施加方式把两个扳手都施加在第 2.5 节的瞬时世界中点 $\mathbf x_m$，且不附加支承矩：

$$
\mathcal W_{m}^{I}[\mathcal A]=\left(\boldsymbol\tau_I,\ \mathbf f_I\right),
\qquad
\mathcal W_{m}^{I}[\mathcal C]=\left(-\boldsymbol\tau_I,\ -\mathbf f_I\right).
$$

两个扳手关于同一点取矩、在同一表达系中互为相反数，合力与合力矩为零。$\mathbf x_m$ 换算到两个刚体上的体坐标分别为 $\mathbf r_m^{\mathcal A}=R_{I\mathcal A}^{\mathsf T}(\mathbf x_m-\mathbf x_{\mathcal A})$ 与 $\mathbf r_m^{\mathcal C}=R_{I\mathcal C}^{\mathsf T}(\mathbf x_m-\mathbf x_{\mathcal C})$；这两个点都不是连接坐标系的原点，且随相对运动而变。采用这种方式的族以对端所受载荷 $\mathbf f_{C,I}=-\mathbf f_I$、$\boldsymbol\tau_{C,I}=-\boldsymbol\tau_I$ 陈述本构时，只是把上式两个扳手的记号互换，施加位置不变。

### 3.5 换算到刚体原点

多体层把每条载荷从作用点 Q 移到所在刚体的体坐标系原点 $O_{\mathcal A}$。按基座第 5.2 节，$\mathbf p_{OQ}$ 是从新取矩点指向旧取矩点的向量，对刚体 $\mathcal A$ 上的点 Q，

$$
\boldsymbol\tau_{O_{\mathcal A}}=\boldsymbol\tau_Q+\left(R_{I\mathcal A}\mathbf r_Q^{\mathcal A}\right)\times\mathbf f,
$$

力本身不变。三种施加方式的差别正体现在这一步：端点力对的支承矩与力臂项 $(R_{I\mathcal A}\mathbf r_{A_o}^{\mathcal A})\times\mathbf f_I$ 叠加；纯力偶不产生力臂项；中点扳手对的力臂从各自刚体原点指向同一个空间点。换点与换表达系是两个不同运算，本篇所有换点都在 I 中完成，换表达系只发生在 A 与 I 之间，以及衬套族的 B 与 A 之间。

## 4. 功率恒等式与组织原则

### 4.1 单个刚体上扳手的功率

作用于刚体 $\mathcal A$、关于其材料点 Q 取矩的扳手 $(\boldsymbol\tau_Q,\mathbf f)$ 向该刚体输送的功率为

$$
\mathcal P=\mathbf f\cdot\mathbf v^{(\mathcal A)}(\mathbf x_Q)+\boldsymbol\tau_Q\cdot\boldsymbol\omega_A.
$$

它不随取矩点改变：把 Q 换成 O 时，$\mathbf v^{(\mathcal A)}(\mathbf x_Q)=\mathbf v^{(\mathcal A)}(\mathbf x_O)+\boldsymbol\omega_A\times\mathbf p_{OQ}$ 与 $\boldsymbol\tau_O=\boldsymbol\tau_Q+\mathbf p_{OQ}\times\mathbf f$ 带来的两项 $\mathbf f\cdot(\boldsymbol\omega_A\times\mathbf p_{OQ})$ 与 $(\mathbf p_{OQ}\times\mathbf f)\cdot\boldsymbol\omega_A$ 相等而抵消。它也不随表达系改变，因为点积在旋转下不变；下文的功率因此可以在 I 中推导、在 A 中陈述。推导反复使用混合积恒等式

$$
\boldsymbol\omega\cdot\left(\mathbf d\times\mathbf f\right)=\mathbf f\cdot\left(\boldsymbol\omega\times\mathbf d\right).
$$

### 4.2 端点力对

对第 3.2 节的一对载荷，两刚体的总功率为

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v_{Ao}+\boldsymbol\omega_A\cdot\left(\mathbf d_I\times\mathbf f_I\right)-\mathbf f_I\cdot\mathbf v_{Co}
=-\mathbf f_I\cdot\left(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I\right)
=-\mathbf f_A\cdot\mathbf u_A.
$$

第二个等号用了混合积恒等式，第三个等号用了第 2.3 节的定义与点积的旋转不变性。于是端点功率恰是参考端力与 $\dot{\mathbf d}_A$ 的负内积。去掉支承矩，总功率变成 $-\mathbf f_I\cdot(\mathbf v_{Co}-\mathbf v_{Ao})=-\mathbf f_I\cdot\dot{\mathbf d}_I$，它与在 A 中陈述、以 $\mathbf u_A$ 为速度输入的本构不共轭，二者相差支承矩的功率 $\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)$。支承矩因此是使端点功率等于本构功率的必要项，配平净力矩只是它的附带结果。

这条恒等式只说明功率的配对：以 $\mathbf u_A=\dot{\mathbf d}_A$ 为速度输入的本构，其功率 $-\mathbf f_A\cdot\dot{\mathbf d}_A$ 恰是两刚体收到的功率。弹性部分是否为某个储能函数的全导数是另一件事：它要求 $\mathbf f_{\mathrm{el}}(\mathbf d_A)$ 是一个标量函数的梯度，即满足可积性条件 $\partial f_{\mathrm{el},i}/\partial d_{A,j}=\partial f_{\mathrm{el},j}/\partial d_{A,i}$；$\mathbf f_{\mathrm{el}}=k\,(d_{A,2},0,0)^{\mathsf T}$ 这样的律有同样的功率配对却没有势函数。采用这种施加方式的三个族在各自篇中分别以势梯度或内变量的能量恒等式论证储能与耗散。

### 4.3 纯力偶对

对第 3.3 节的一对纯力偶，

$$
\mathcal P
=\mathbf m_I\cdot\boldsymbol\omega_A-\mathbf m_I\cdot\boldsymbol\omega_C
=-\mathbf m_I\cdot\boldsymbol\omega_{rel,I}
=-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}.
$$

纯力偶的功率与作用点无关，物理力偶与相对角速度共轭。经一组角坐标 $\boldsymbol\eta$ 的速率映射 $\dot{\boldsymbol\eta}=H\boldsymbol\omega_{rel,A}$，同一功率也可写成对端所受广义力矩与广义速率的内积：$\mathcal P=\boldsymbol\tau_\eta\cdot\dot{\boldsymbol\eta}$，其中 $\boldsymbol\tau_\eta$ 是作用于对端 C 的广义力矩，参考端所受的物理力偶为 $\mathbf m_A=-H^{\mathsf T}\boldsymbol\tau_\eta$——这是同一配对换了变量，衬套族的转动部分即如此陈述。若一族的力偶只沿 A 的第一轴，$\mathbf m_A=m\,\mathbf e_1$，则 $\mathcal P=-m\,\omega_{rel,A,1}$：与之共轭的速率是 $\boldsymbol\omega_{rel,A}$ 的第一分量，侧滚族的阻尼输入取的正是这一分量。

### 4.4 中点扳手对

对第 3.4 节的一对扳手，两刚体在 $\mathbf x_m$ 处的材料点速度按第 2.5 节取值，

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v^{(\mathcal A)}(\mathbf x_m)+\boldsymbol\tau_I\cdot\boldsymbol\omega_A
-\mathbf f_I\cdot\mathbf v^{(\mathcal C)}(\mathbf x_m)-\boldsymbol\tau_I\cdot\boldsymbol\omega_C
=-\mathbf f_I\cdot\mathbf u_{m,I}-\boldsymbol\tau_I\cdot\boldsymbol\omega_{rel,I}.
$$

在 A 中即 $\mathcal P=-\mathbf f_A\cdot\mathbf u_{m,A}-\boldsymbol\tau_A\cdot\boldsymbol\omega_{rel,A}$；以对端所受载荷陈述时 $\mathcal P=\mathbf f_{C,A}\cdot\mathbf u_{m,A}+\boldsymbol\tau_{C,A}\cdot\boldsymbol\omega_{rel,A}$。与力共轭的是中点材料相对速度 $\mathbf u_m$，不是 $\mathbf u_A$。把同一个力改施于两端原点而不加支承矩，功率变为 $-\mathbf f_I\cdot\dot{\mathbf d}_I$；改施于两端原点并加支承矩，功率变为 $-\mathbf f_A\cdot\mathbf u_A$。三者两两之差分别是 $\mathbf f$ 与 $\tfrac12\boldsymbol\omega_{rel}\times\mathbf d$、$\boldsymbol\omega_A\times\mathbf d$ 这类向量的内积，力沿两原点连线时为零，一般不为零。

### 4.5 组织原则：速度输入与施力位置成对选定

以上三条恒等式有同一个结构：端点功率等于参考端所受载荷与对端相对参考端的某个速度的负内积，而这个速度由扳手的完整分配决定，即力施于何处与随之施加何种力矩共同决定——施于两端原点并配支承矩时是 $\mathbf u_A$，施为纯力偶时是 $\boldsymbol\omega_{rel,A}$，施于瞬时中点时是 $\mathbf u_{m,A}$ 与 $\boldsymbol\omega_{rel,A}$。本目录据此立一条组织原则：**每一族把本构律中的速度输入与扳手的施加位置成对选定，使本构功率恰等于两刚体收到的端点功率。** 按第 3.5 节的换点公式把力搬到另一点并同时补上相应力矩，功率不变；一族若只改变施力位置而不补力矩、也不换速度输入，或反之，功率恒等式一般即失效（沿力的作用线换点等特殊情形除外）：元件在两刚体上做的功与其本构所记的功不再相同，储能与耗散的核算随之失去意义。三种配对的对应关系为：

| 施加方式 | 速度输入 | 端点功率 |
|---|---|---|
| 两端原点力对加参考端支承矩 | $\mathbf u_A=\dot{\mathbf d}_A$ | $-\mathbf f_A\cdot\mathbf u_A$ |
| 两端纯力偶对 | $\boldsymbol\omega_{rel,A}$ | $-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}$ |
| 同施于瞬时中点的扳手对 | $\mathbf u_{m,A}$ 与 $\boldsymbol\omega_{rel,A}$ | $-\mathbf f_A\cdot\mathbf u_{m,A}-\boldsymbol\tau_A\cdot\boldsymbol\omega_{rel,A}$ |

各族篇引用本节，说明自己采用哪一行，并证明自己的速度输入确实是该行的量。本篇只保证功率相等；弹性部分是否为某个储能函数的全导数，还取决于两件事——该族的变形度量与速度输入是否互为导数，以及弹性律在该变形度量上是否可积（第 4.2 节）；带内变量的族另需计入内变量的储能。这些都由各族自行论证。

## 5. 计算实现

### 5.1 相对运动的一次求取

每次求值，每个力元先由两端坐标系得到一份相对运动，包含 $\mathbf d_A$、$R_{AC}$、$\mathbf u_A$、$\boldsymbol\omega_{rel,A}$，以及写入载荷所需的 $\mathbf d_I$、$R_{IA}$、$\mathbf p_A$ 与两端原点的刚体固定点。多体层提供三类查询：坐标系在 I 中的位姿给出 $(R_{IA},\mathbf p_A)$ 与 $(R_{IC},\mathbf p_C)$；C 相对 A、在 A 中表达的空间速度查询直接给出 $\boldsymbol\omega_{rel,A}$ 与 $\mathbf u_A$，其平动分量按契约是 $C_o$ 在 A 中度量的速度，运输项已在其中；坐标系原点到刚体固定点的解析给出 $(\mathcal A,\mathbf r_{A_o}^{\mathcal A})$ 与 $(\mathcal C,\mathbf r_{C_o}^{\mathcal C})$。$\mathbf d_I$、$\mathbf d_A$ 与 $R_{AC}$ 按第 2.1 节由两个位姿相减、相乘得到。这一步对五族相同，族的差别只在如何消费这份相对运动。

### 5.2 三种扳手对的写入

每个力元恰写两条载荷，表达系都是 I。

- 端点力对加支承矩：调用者给出 $\mathbf f_I=R_{IA}\mathbf f_A$ 与 $\mathbf d_I$；写入 $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf d_I\times\mathbf f_I,\ \mathbf f_I)$ 与 $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ \mathbf 0,\ -\mathbf f_I)$。平动族、串联族与饱和族沿用这一路径，后两者的 $\mathbf f_A$ 只有指定轴上一个非零分量。
- 纯力偶对：调用者给出 $\mathbf m_I=R_{IA}\mathbf m_A$；写入 $(\mathcal A,\ \mathbf r_{A_o}^{\mathcal A},\ \mathbf m_I,\ \mathbf 0)$ 与 $(\mathcal C,\ \mathbf r_{C_o}^{\mathcal C},\ -\mathbf m_I,\ \mathbf 0)$。侧滚族沿用这一路径。
- 中点扳手对：由 $\mathbf x_m=\mathbf p_A+\tfrac12\mathbf d_I$ 与两个刚体的世界位姿求 $\mathbf r_m^{\mathcal A}$、$\mathbf r_m^{\mathcal C}$；以对端所受载荷 $\mathbf f_{C,I}$、$\boldsymbol\tau_{C,I}$ 为输入，写入 $(\mathcal A,\ \mathbf r_m^{\mathcal A},\ -\boldsymbol\tau_{C,I},\ -\mathbf f_{C,I})$ 与 $(\mathcal C,\ \mathbf r_m^{\mathcal C},\ \boldsymbol\tau_{C,I},\ \mathbf f_{C,I})$。衬套族沿用这一路径。

### 5.3 多体层的消费与状态导数装配

多体层逐条读取载荷，把关于 Q 的力矩按第 3.5 节移到所在刚体的原点，并入前向动力学的外力，得到 $\dot v$。带内部状态的族在同一次求值中同时写出其状态导数：串联族的力状态构成基座第 5.1 节的 $z$ 块，其导数与两条载荷由同一份相对运动算出，系统装配层把 $[N(q)v;\dot v]$ 与 $\dot z$ 拼成完整的状态导数。$z$ 块在积分器中的位置见[时间积分方法](../numerical_methods/TIME_INTEGRATION_METHODS.md)第 1.1 节。运动学层本身不保存任何历史量：$\mathbf d_A$、$R_{AC}$、$\mathbf u_A$、$\boldsymbol\omega_{rel,A}$ 全部由当前 $(q,v)$ 决定。

## 6. 数学性质与适用条件

- **无自然长度。** 平动变形度量是原点之差 $\mathbf d_A$，运动学层不提供自然长度、零位移向量或安装位形。一族的无力位形由其本构律自身表达，例如一个常值力项；两原点重合当且仅当 $\mathbf d_A=\mathbf 0$。
- **两端必须在不同刚体上。** 两端同在一个刚体时 $\mathbf d_A$、$R_{AC}$ 为常量，$\mathbf u_A=\mathbf 0$、$\boldsymbol\omega_{rel,A}=\mathbf 0$，而三种成对扳手落在同一刚体上时合力、合力矩为零，这样的元件对运动没有任何作用。两端也不能取惯性系本身，因为没有可以接受扳手的刚体。
- **参考端的选择是本构的一部分。** 本条针对以 A 为本构系、按端点力对施力的平动律，不覆盖以中点施力的衬套族——后者的平动部分对两端命名不变，见[半角中点 RPY 衬套](HALF_ANGLE_MIDPOINT_RPY_BUSHING.md)第 4.2 节。对前者，相对运动在 A 中表达，本构常数在 A 中陈述，支承矩落在 A 端。互换两端后相对位置向量反向并改在 C 中表达，$R_{AC}$ 转置，一个在 A 中对角的刚度或阻尼在 C 中一般不再对角，各向同性的系数矩阵不受这一表示上的影响；但运输项 $-\boldsymbol\omega_A\times\mathbf d_I$ 只含参考端刚体的角速度，互换两端后它变为 $-\boldsymbol\omega_C\times\mathbf d_I'$，以 $\mathbf u_A$ 为输入的阻尼律即使各向同性也随之改变，在这类律中只有沿连线的中心弹性力对互换不变（[三向平动弹簧—阻尼力元](TRANSLATIONAL_SPRING_DAMPER.md)第 2.8 节）。在同一刚体上取不同姿态的参考端坐标系，也会得到不同的元件。
- **运输项只影响垂直于连线的分量。** $\mathbf u_A$ 与 $R_{IA}^{\mathsf T}\dot{\mathbf d}_I$ 沿 $\mathbf d$ 的分量相同，横向分量相差 $\boldsymbol\omega_A\times\mathbf d$ 在 A 中的表示。只沿连线作用的一维元件不区分二者；本目录的三向对角律与单轴律都以 $\mathbf u_A$ 为速度输入，坐标轴不沿连线时二者一般不同（$\boldsymbol\omega_A=\mathbf 0$ 或 $\boldsymbol\omega_A\parallel\mathbf d$ 时仍相同）。
- **相对角速度与角坐标不可互换。** $\boldsymbol\omega_{rel,A}$ 是几何量，不依赖任何角坐标的选择；由 $R_{AC}$ 提取的任何角坐标，其导数与 $\boldsymbol\omega_{rel,A}$ 之间存在位形相关的映射，且该映射有自己的奇点。以 $\boldsymbol\omega_{rel,A}$ 分量为速率输入的族没有这类奇点。
- **中点速度不是位移的导数。** $\mathbf u_{m,A}$ 与 $\dot{\mathbf d}_A$ 相差 $\tfrac12\boldsymbol\omega_{rel,A}\times\mathbf d_A$。以 $\mathbf u_m$ 为速度输入、以 $\mathbf d$ 的某种表示为变形度量的族，其弹性功率一般不是储能的全导数；功率恒等式仍然成立，但储能核算需要额外条件。
- **功率与表达系无关。** 第 4 节所有功率式在 I、A 或任何中间系中取值相同，各族可在自己最方便的表达系中陈述功率。

## 7. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 两端与五个本构族的类型 | `ForceElementEnd`、`VehicleForceElementCollection`，见 [`vehicle_force_elements.h`](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 相对运动 $\mathbf d_A$、$R_{AC}$、$\mathbf u_A$、$\boldsymbol\omega_{rel,A}$ 的一次求取 | `VehicleForcePlan::CalcAppliedForces` 内的 `CalcRelativeMotion`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 相对空间速度的契约，平动分量为 $C_o$ 在 A 中度量的速度 | `MultibodyModel::CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 坐标系世界位姿与原点的刚体固定点解析 | `MultibodyModel::CalcPoseInWorld`、`MultibodyModel::CalcFrameOriginAsBodyFixedPoint`，见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 载荷条目与刚体固定点 | `AppliedBodyWrench`、`BodyFixedPoint`，见 [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| 端点力对与参考端支承矩 | `VehicleForcePlan::CalcAppliedForces` 内的 `EmitTranslationalWrenchPair`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 纯力偶对 | `internal::EmitCoupleWrenchPair`，见 [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h) |
| 同施于瞬时中点的扳手对 | `VehicleForcePlan::CalcAppliedForces` 内的 `EmitMidpointBushingWrenchPair`，见 [`vehicle_force_plan.cc`](../../../libs/forces/src/vehicle_force_plan.cc) |
| 载荷换点到刚体原点并进入前向动力学 | `MultibodyModel::CalcStateTimeDerivatives`，见 [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| 载荷与 $\dot z$ 拼成完整状态导数 | `CompiledSystemPlan::CalcStateTimeDerivatives`，见 [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc) |
