[English](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md)

# 力元连接运动学与空间扳手

本篇推导两端连接坐标系的相对运动、成对载荷及功率关系。各力元在这些关系上定义变形度量、速度输入和本构律。

## 1. 对象与记号

### 1.1 两端、两刚体与三个坐标系

参考端坐标系 A 与对端坐标系 C 分别固定在刚体 $\mathcal A$ 与 $\mathcal C$ 上，原点为安装点 $A_o$ 与 $C_o$。连接坐标系的姿态与原点可独立于所在刚体的体坐标系。I 为惯性系，旋转矩阵 $R_{IA}$ 将 A 中的分量变换到 I。

本构可以沿参考架的坐标轴，也可以沿两安装点的瞬时连线定义。两种方向由各力元分别声明。

### 1.2 记号表

| 记号 | 含义 |
|---|---|
| $\mathbf p_A,\mathbf p_C$ | 两安装点在 I 中的位置 |
| $\mathbf d_I,\mathbf d_A$ | 相对位置，在 I 与 A 中表达 |
| $R_{IA},R_{IC},R_{AC}$ | 两端姿态与相对姿态 |
| $\mathbf v_{Ao},\mathbf v_{Co}$ | 两安装点相对 I 的速度，在 I 中表达 |
| $\boldsymbol\omega_A,\boldsymbol\omega_C$ | 两刚体相对 I 的角速度，在 I 中表达 |
| $\mathbf u_A,\mathbf u_I$ | 对端原点相对参考架的速度，在 A 与 I 中表达 |
| $\boldsymbol\omega_{rel,A},\boldsymbol\omega_{rel,I}$ | C 相对 A 的物理角速度 |
| $l,\mathbf e_I,\mathbf e_A$ | 安装点间距及由 A 指向 C 的单位方向 |
| $\mathbf f,\mathbf m$ | 参考端所受的力与纯力偶，末下标表示表达系 |
| $\mathcal W_Q^E[\mathcal A]$ | 作用于刚体 $\mathcal A$、关于 Q 取矩、在 E 中表达的扳手 |
| $\mathbf x_{\mathcal A},R_{I\mathcal A},\mathbf r_Q^{\mathcal A}$ | 刚体原点、姿态及 Q 的体坐标 |
| $\mathbf v^{(\mathcal A)}(\mathbf x)$ | 刚体 $\mathcal A$ 在空间点 $\mathbf x$ 处的材料速度 |
| $\mathcal P,\mathcal V,\mathcal D$ | 向两刚体输送的总功率、元件储能与耗散率 |

### 1.3 坐标与符号

采用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)的旋转、扳手与 SI 单位。$\operatorname{skew}(\mathbf w)\mathbf x=\mathbf w\times\mathbf x$，$\mathbf a\circ\mathbf b$ 表示逐分量乘积。力元标量刚度和阻尼记为 $k,c$；矩阵或向量系数由各篇定义。

$\mathbf v_{Ao}=\mathbf v_{IAo\_I}$，$\boldsymbol\omega_A=\boldsymbol\omega_{IA\_I}$，$\mathbf u_A=\mathbf v_{ACo\_A}$，$\boldsymbol\omega_{rel,A}=\boldsymbol\omega_{AC\_A}$。C 在本篇表示对端坐标系。各端扳手按“力矩、力”的次序书写，统一到同一取矩点后互为相反数。

## 2. 相对运动

### 2.1 相对位置与相对姿态

$$
\mathbf d_I=\mathbf p_C-\mathbf p_A,\qquad
\mathbf d_A=R_{IA}^{\mathsf T}\mathbf d_I,\qquad
R_{AC}=R_{IA}^{\mathsf T}R_{IC}.
$$

平动弹性与参考架衬套以 $\mathbf d_A$ 为变形度量；点对点元件还使用其模长和方向。侧滚力偶的转动度量由 $R_{AC}$ 给出。

### 2.2 刚体材料点的速度搬移

刚体 $\mathcal A$ 在空间点 $\mathbf x$ 处的材料速度为

$$
\mathbf v^{(\mathcal A)}(\mathbf x)
=\mathbf v_{Ao}+\boldsymbol\omega_A\times(\mathbf x-\mathbf p_A).
$$

该式同样适用于刚体的刚性延拓。对 $\mathcal C$ 使用对应的位置和角速度。

### 2.3 含运输项的相对速度

由 $\dot R_{IA}=\operatorname{skew}(\boldsymbol\omega_A)R_{IA}$，对相对位置求导：

$$
\dot{\mathbf d}_A
=R_{IA}^{\mathsf T}
(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I).
$$

定义

$$
\mathbf u_I=\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I,\qquad
\mathbf u_A=R_{IA}^{\mathsf T}\mathbf u_I=\dot{\mathbf d}_A.
$$

运输项将参考架的速度搬移到 $C_o$。因此 $\mathbf u_I=\mathbf v_{Co}-\mathbf v^{(\mathcal A)}(\mathbf p_C)$。它与惯性速度差在垂直于连线的分量上相差 $\boldsymbol\omega_A\times\mathbf d_I$。

### 2.4 相对角速度与相对姿态的导数

$$
\boldsymbol\omega_{rel,I}=\boldsymbol\omega_C-\boldsymbol\omega_A,\qquad
\boldsymbol\omega_{rel,A}=R_{IA}^{\mathsf T}\boldsymbol\omega_{rel,I}.
$$

对 $R_{AC}=R_{IA}^{\mathsf T}R_{IC}$ 求导，并用旋转保持叉乘的性质，得到

$$
\dot R_{AC}
=R_{IA}^{\mathsf T}\operatorname{skew}(\boldsymbol\omega_C-\boldsymbol\omega_A)R_{IC}
=\operatorname{skew}(\boldsymbol\omega_{rel,A})R_{AC}.
$$

物理相对角速度与所选角坐标的导数通过位形相关的速率映射联系。

### 2.5 连线长度与长度变化率

当 $l=\|\mathbf d_I\|>0$ 时，

$$
\mathbf e_I=\frac{\mathbf d_I}{l},\qquad
\mathbf e_A=\frac{\mathbf d_A}{l},\qquad
\dot l=\mathbf e_I\cdot(\mathbf v_{Co}-\mathbf v_{Ao})
=\mathbf e_A\cdot\mathbf u_A.
$$

最后一个等号来自 $\mathbf d_I\cdot(\boldsymbol\omega_A\times\mathbf d_I)=0$。长度变化率既可由两安装点的惯性速度差求得，也可由参考架内相对速度求得。

## 3. 扳手的施加与换点

### 3.1 载荷条目与刚体固定点

每条扳手指定受力刚体、作用点、表达系以及关于该点的力矩和力。空间点 $\mathbf x$ 在刚体 $\mathcal A$ 上的体坐标为

$$
\mathbf r_Q^{\mathcal A}
=R_{I\mathcal A}^{\mathsf T}(\mathbf x-\mathbf x_{\mathcal A}).
$$

以下载荷均施于两端安装点，其体坐标为常量，扳手在 I 中表达。

### 3.2 端点力对与参考端支承矩

参考端受力 $\mathbf f_I=R_{IA}\mathbf f_A$，成对扳手为

$$
\mathcal W_{A_o}^{I}[\mathcal A]
=(\mathbf d_I\times\mathbf f_I,\ \mathbf f_I),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]
=(\mathbf 0,\ -\mathbf f_I).
$$

合力为零。关于任意点 O 的合矩为

$$
(\mathbf p_A-\mathbf x_O)\times\mathbf f_I
+\mathbf d_I\times\mathbf f_I
-(\mathbf p_C-\mathbf x_O)\times\mathbf f_I=\mathbf 0.
$$

参考端支承矩同时使载荷与参考架内位移导数功率共轭，见第 4.2 节。

### 3.3 纯力偶对

$$
\mathcal W_{A_o}^{I}[\mathcal A]=(\mathbf m_I,\ \mathbf 0),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=(-\mathbf m_I,\ \mathbf 0),\qquad
\mathbf m_I=R_{IA}\mathbf m_A.
$$

纯力偶与取矩点无关，两端合力和合矩均为零。它可以与第 3.2 节的载荷相加。

### 3.4 沿连线的轴向力对

以拉力 $F>0$ 表示两端相互拉近：

$$
\mathbf f_I=F\mathbf e_I,\qquad
\mathcal W_{A_o}^{I}[\mathcal A]=(\mathbf 0,\ F\mathbf e_I),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=(\mathbf 0,\ -F\mathbf e_I).
$$

由于 $\mathbf d_I\times\mathbf e_I=\mathbf 0$，两端力关于任意点的合矩为零。

### 3.5 换算到刚体原点

将关于 Q 的力矩移到刚体原点 $O_{\mathcal A}$：

$$
\boldsymbol\tau_{O_{\mathcal A}}
=\boldsymbol\tau_Q+(R_{I\mathcal A}\mathbf r_Q^{\mathcal A})\times\mathbf f.
$$

力保持不变。端点力产生的力臂矩与该端显式力矩一起进入多体方程；纯力偶在换点后保持原值。

## 4. 功率恒等式与组织原则

### 4.1 单个刚体上扳手的功率

$$
\mathcal P_{\mathcal A}
=\mathbf f\cdot\mathbf v^{(\mathcal A)}(\mathbf x_Q)
+\boldsymbol\tau_Q\cdot\boldsymbol\omega_A.
$$

由速度搬移和力矩换点公式，以及

$$
\boldsymbol\omega\cdot(\mathbf d\times\mathbf f)
=\mathbf f\cdot(\boldsymbol\omega\times\mathbf d)
$$

可知功率在换点前后相等。旋转保持点积，故功率也与表达系的选择无关。

### 4.2 端点力对

第 3.2 节的载荷给出

$$
\mathcal P
=\mathbf f_I\cdot\mathbf v_{Ao}
+\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I)
-\mathbf f_I\cdot\mathbf v_{Co}
=-\mathbf f_A\cdot\mathbf u_A.
$$

相对位移 $\mathbf d_A$ 与力 $\mathbf f_A$ 功率共轭。若弹性力为储能梯度 $\mathbf f_{\mathrm{el}}=\partial\mathcal V/\partial\mathbf d_A$，其向两体输送的功率为 $-\dot{\mathcal V}$；光滑弹性律的可积性要求 $\partial f_{\mathrm{el},i}/\partial d_{A,j}=\partial f_{\mathrm{el},j}/\partial d_{A,i}$。

### 4.3 纯力偶对

$$
\mathcal P
=\mathbf m_I\cdot(\boldsymbol\omega_A-\boldsymbol\omega_C)
=-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}.
$$

物理力偶与相对角速度功率共轭。若以角坐标表示功率，其变换遵循[多体动力学](../vehicle_dynamics/MULTIBODY_EQUATIONS_OF_MOTION.md)第 3.2 节的速率映射与共轭力矩关系。

### 4.4 轴向力对

第 3.4 节与第 2.5 节给出

$$
\mathcal P
=F\mathbf e_I\cdot(\mathbf v_{Ao}-\mathbf v_{Co})
=-F\dot l.
$$

轴向拉力与安装点距离的变化率功率共轭。

### 4.5 速度输入与施力位置的配对

每一族将本构中的速度输入与载荷的作用点、支承矩成对确定，使两体端点总功率与本构功率一致。

| 施加方式 | 共轭速度 | 两体总功率 |
|---|---|---|
| 端点力对及参考端支承矩 | $\mathbf u_A=\dot{\mathbf d}_A$ | $-\mathbf f_A\cdot\mathbf u_A$ |
| 纯力偶对 | $\boldsymbol\omega_{rel,A}$ | $-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}$ |
| 沿连线的力对 | $\dot l$ | $-F\dot l$ |

载荷可以叠加，对应功率随之相加。具有储能 $\mathcal V$ 与耗散率 $\mathcal D\ge0$ 的力元满足 $\mathcal P+\dot{\mathcal V}+\mathcal D=0$。带内部状态的元件将其内部储能一并计入。

## 5. 计算实现

### 5.1 相对运动的一次求取

两端位姿给出 $\mathbf d_I,\mathbf d_A,R_{AC}$；相对空间速度给出 $\mathbf u_A,\boldsymbol\omega_{rel,A}$；安装点解析给出受力刚体及体固坐标。空间速度的平动分量已含第 2.3 节的运输项。这些量由当前多体位姿和速度确定。

### 5.2 成对载荷

- **端点力对与支承矩**：三向平动、定轴串联和饱和阻尼采用第 3.2 节的形式。
- **纯力偶**：侧滚元采用第 3.3 节的形式。
- **沿连线力对**：点对点串联元采用第 3.4 节的形式。

参考架衬套叠加前两种形式。每个元件向两刚体各给出一条合成扳手。

### 5.3 状态导数装配

两类串联元件在同次本构求值中给出载荷和内力导数。系统状态按 $[q;v;z]$ 组织，$z$ 汇集各串联元件以 N 为单位的力状态，完整导数为 $[N(q)v;\dot v;\dot z]$。

## 6. 数学性质与适用条件

- **变形基准。** 相对运动以安装点的位置和姿态为基准。元件的无力状态由其本构和初始内力确定；平动元的名义力作为本构常量参与求值。
- **端点。** 两端安装在不同的受力刚体上。点对点方向要求 $l>0$；参考架形式在两点重合时仍有定义。
- **参考架。** 各向异性系数的方向随参考架运动。交换两端同时改变表达系、运输项和支承矩归属，因此参考端的选取属于本构定义。
- **轴向运动。** 运输项在连线方向的投影为零。点对点元件交换两端后，$l,\dot l$ 保持，方向向量反向，物理载荷保持。
- **转动。** 物理角速度由两刚体运动确定，直接用作速率输入时不引入角坐标奇点；角坐标的速率映射有其坐标适用域。

## 7. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 端点和力元类型 | `ForceElementEnd`、`VehicleForceElementCollection`，见 [vehicle_force_elements.h](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 相对运动与成对载荷 | `CalcRelativeMotion`、`EmitTranslationalWrenchPair`、`VehicleForcePlan::CalcAppliedForces`，见 [vehicle_force_plan.cc](../../../libs/forces/src/vehicle_force_plan.cc) |
| 纯力偶 | `internal::EmitCoupleWrenchPair`，见 [body_wrench_pair.h](../../../libs/forces/src/body_wrench_pair.h) |
| 位姿、相对空间速度与作用点 | `MultibodyModel` 的相应查询，见 [multibody_model.h](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| 扳手表示 | `AppliedBodyWrench`，见 [multibody_applied_forces.h](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h) |
| 完整状态导数 | `CompiledSystemPlan::CalcStateTimeDerivatives`，见 [compiled_system_plan.cc](../../../libs/system_assembly/src/compiled_system_plan.cc) |
