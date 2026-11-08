[English](REFERENCE_FRAME_LINEAR_BUSHING.en.md)

# 参考架线性衬套

衬套连接两个刚体上的安装架，在参考架内定义三向平动弹性、平动阻尼和转动阻尼。相对运动、扳手及功率记号沿用[力元连接运动学与空间扳手](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)。

## 1. 定义

参考端 A 与对端 C 的原点分别为安装点。设

$$
\mathbf d_A=R_{IA}^{\mathsf T}(\mathbf p_C-\mathbf p_A),\qquad
\mathbf u_A=\dot{\mathbf d}_A
=R_{IA}^{\mathsf T}
(\mathbf v_{Co}-\mathbf v_{Ao}-\boldsymbol\omega_A\times\mathbf d_I),
$$

$$
\boldsymbol\omega_{rel,A}
=R_{IA}^{\mathsf T}(\boldsymbol\omega_C-\boldsymbol\omega_A).
$$

系数为参考架内的常对角矩阵：

$$
K_t=\operatorname{diag}(k_1,k_2,k_3),\qquad
C_t=\operatorname{diag}(c_1,c_2,c_3),\qquad
C_r=\operatorname{diag}(c_{r,1},c_{r,2},c_{r,3}).
$$

$k_i$ 的单位为 N/m，$c_i$ 为 N·s/m，$c_{r,i}$ 为 N·m·s/rad；各系数有限且非负。下标 $t,r$ 分别表示平动与转动；$C_t,C_r$ 为元件阻尼矩阵。

参考端的本构力与阻尼力偶为

$$
\mathbf f_A=K_t\mathbf d_A+C_t\mathbf u_A,\qquad
\mathbf m_A=C_r\boldsymbol\omega_{rel,A}.
$$

平动弹性以两安装点重合为零位。转动项直接使用物理相对角速度。

## 2. 载荷与功率推导

### 2.1 参考架内的速度

参考架随刚体转动，故

$$
\frac{\mathrm d}{\mathrm dt}
\left(R_{IA}^{\mathsf T}\mathbf d_I\right)
=R_{IA}^{\mathsf T}
\left(\dot{\mathbf d}_I-\boldsymbol\omega_A\times\mathbf d_I\right)
=\mathbf u_A.
$$

平动阻尼与该位移导数配对。转动阻尼作用于 C 相对 A 的物理角速度，三向系数沿 A 的轴定义。

### 2.2 真实安装点的扳手

记 $\mathbf f_I=R_{IA}\mathbf f_A$、$\mathbf m_I=R_{IA}\mathbf m_A$。在两安装点施加

$$
\mathcal W_{A_o}^{I}[\mathcal A]
=(\mathbf d_I\times\mathbf f_I+\mathbf m_I,\ \mathbf f_I),
\qquad
\mathcal W_{C_o}^{I}[\mathcal C]
=(-\mathbf m_I,\ -\mathbf f_I).
$$

参考端承担支承矩 $\mathbf d_I\times\mathbf f_I$。关于任意点 O，两体合矩为

$$
(\mathbf p_A-\mathbf x_O)\times\mathbf f_I
+\mathbf d_I\times\mathbf f_I+\mathbf m_I
-(\mathbf p_C-\mathbf x_O)\times\mathbf f_I-\mathbf m_I=\mathbf 0.
$$

合力也为零。

### 2.3 储能与耗散

两体获得的总功率为

$$
\begin{aligned}
\mathcal P
&=\mathbf f_I\cdot(\mathbf v_{Ao}-\mathbf v_{Co})
+\boldsymbol\omega_A\cdot(\mathbf d_I\times\mathbf f_I+\mathbf m_I)
-\boldsymbol\omega_C\cdot\mathbf m_I\\
&=-\mathbf f_A\cdot\mathbf u_A
-\mathbf m_A\cdot\boldsymbol\omega_{rel,A}.
\end{aligned}
$$

由于 $K_t$ 在参考架内为常对称矩阵，

$$
\mathcal V=\tfrac12\mathbf d_A^{\mathsf T}K_t\mathbf d_A,\qquad
\dot{\mathcal V}=\mathbf u_A^{\mathsf T}K_t\mathbf d_A.
$$

阻尼耗散率为

$$
\mathcal D
=\mathbf u_A^{\mathsf T}C_t\mathbf u_A
+\boldsymbol\omega_{rel,A}^{\mathsf T}C_r\boldsymbol\omega_{rel,A}\ge0,
\qquad
\mathcal P+\dot{\mathcal V}+\mathcal D=0.
$$

因此元件吸收的功率由弹性储能变化和阻尼耗散组成。

## 3. 性质与适用条件

- 系数轴随参考架运动，各向异性在 A 中定义；参考端的选取同时确定运输项和支承矩归属。
- 两安装点重合时，平动弹性力和支承矩为零，平动与转动阻尼仍由相对速度决定。
- 半正定系数允许某些方向自由运动；正定阻尼在相应非零相对速度下产生正耗散。
- 力和力偶由当前相对运动代数确定。该元件具有平动储能，连续状态由多体位置与速度承载。
- 在同一空间刚体变换下同时变换两端，参考架内本构量保持，惯性系载荷随之旋转。

## 4. 源码映射

| 理论对象 | 实现 |
|---|---|
| 两端与三组对角系数 | `ReferenceFrameLinearBushing`，见 [vehicle_force_elements.h](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 相对运动、支承矩与阻尼力偶 | `CalcRelativeMotion`、`EmitTranslationalWrenchPair`、`VehicleForcePlan::CalcAppliedForces`，见 [vehicle_force_plan.cc](../../../libs/forces/src/vehicle_force_plan.cc) |
