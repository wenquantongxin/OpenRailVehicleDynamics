[English](POINT_TO_POINT_SERIES_SPRING_BILINEAR_DAMPER.en.md)

# 点对点串联弹簧—双斜率阻尼力元

线性弹簧与奇对称双斜率阻尼器串联，共同传递沿两安装点连线的轴向力。该力作为内部状态，与两刚体运动共同演化。端点和表达系沿用[力元连接运动学与空间扳手](FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)。

## 1. 定义

### 1.1 几何与参数

$$
\mathbf d_I=\mathbf p_C-\mathbf p_A,\qquad
l=\|\mathbf d_I\|>0,\qquad
\mathbf e_I=\mathbf d_I/l,\qquad
\dot l=\mathbf e_I\cdot(\mathbf v_{Co}-\mathbf v_{Ao})
=\mathbf e_A\cdot\mathbf u_A.
$$

$\mathbf e_I$ 从参考端 A 指向对端 C；$F>0$ 为拉力。弹簧刚度为 $k$，内、外段阻尼斜率为 $c,c_2$，正拐点力为 $f_b$。四个参数均有限且严格为正，单位依次为 N/m、N·s/m、N·s/m、N。拐点速度为 $v_b=f_b/c$。

### 1.2 阻尼律及其反函数

以阻尼器伸长率 $v_d$ 为输入：

$$
D(v_d)=
\begin{cases}
c\,v_d,& |v_d|\le v_b,\\
\operatorname{sgn}(v_d)\left[f_b+c_2(|v_d|-v_b)\right],& |v_d|>v_b.
\end{cases}
$$

两段在拐点连续，外段沿斜率 $c_2$ 延伸。$D$ 为严格递增奇函数，在实数上具有唯一反函数：

$$
D^{-1}(F)=
\begin{cases}
F/c,& |F|\le f_b,\\
\operatorname{sgn}(F)\left[f_b/c+(|F|-f_b)/c_2\right],& |F|>f_b.
\end{cases}
$$

## 2. 串联关系与内部状态

### 2.1 内力方程

记弹簧伸长为 $\delta_s$、阻尼器伸长为 $\delta_d$。串联条件为

$$
F=k\delta_s=D(\dot\delta_d),\qquad
\dot\delta_s+\dot\delta_d=\dot l.
$$

用 $\dot\delta_d=D^{-1}(F)$ 消去阻尼器速度，得到

$$
\dot F=k\left(\dot l-D^{-1}(F)\right).
$$

初值 $F(t_0)$ 与两刚体的初始位姿和速度共同确定演化。以弹簧伸长为状态时，$\delta_s=F/k$，有

$$
\dot\delta_s=\dot l-D^{-1}(k\delta_s).
$$

两种状态表示由常量 $k$ 线性换算。本族采用以 N 为单位的内力 $F$，各元件占系统 $z$ 块的一个分量。

### 2.2 安装点载荷

$$
\mathcal W_{A_o}^{I}[\mathcal A]=(\mathbf 0,\ F\mathbf e_I),\qquad
\mathcal W_{C_o}^{I}[\mathcal C]=(\mathbf 0,\ -F\mathbf e_I).
$$

两力共线，合力与关于任意点的合矩均为零。长度率对运输项的投影为零，故参考架内速度与安装点惯性速度差给出同一个 $\dot l$。

### 2.3 储能与耗散

两体接收的功率、弹簧储能和阻尼耗散率分别为

$$
\mathcal P=-F\dot l,\qquad
\mathcal V=\frac{F^2}{2k},\qquad
\mathcal D=F D^{-1}(F)\ge0.
$$

由内力方程，

$$
\dot{\mathcal V}=\frac{F\dot F}{k}
=F\dot l-F D^{-1}(F),\qquad
\mathcal P+\dot{\mathcal V}+\mathcal D=0.
$$

$\mathcal D\ge0$ 来自 $D^{-1}(F)$ 与 $F$ 同号。拉伸和压缩具有相同的耗散性质。

## 3. 响应性质

### 3.1 固定长度下的松弛

取 $\dot l=0$。在 $|F|\le f_b$ 内，

$$
F(t)=F(t_0)e^{-k(t-t_0)/c}.
$$

若初值在外段，令 $r=|F|>f_b$，则

$$
\dot r=-k\left[\frac{f_b}{c}+\frac{r-f_b}{c_2}\right].
$$

令 $r_*=f_b(1-c_2/c)$，同一外段内的解为

$$
r(t)=r_*+[r(t_0)-r_*]e^{-k(t-t_0)/c_2}.
$$

该式用至 $r=f_b$，随后接续内段指数衰减。$r_*$ 是外段方程的代数常数；由于 $r_*<f_b$，力幅值在外段持续减小，并最终趋于零。两段的局部松弛时间分别为 $c/k$ 与 $c_2/k$。

### 3.2 恒定长度率与线性特例

当 $\dot l=u_0$ 恒定时，稳态力满足

$$
F_\infty=D(u_0).
$$

取 $c=c_2$，全域阻尼律变为 $D(v_d)=c\,v_d$，内力方程为 $\dot F=k\dot l-(k/c)F$。它与[定轴串联弹簧—黏性阻尼](SERIES_SPRING_VISCOUS_DAMPER.md)具有相同的标量形式；本元件的作用方向随连线运动，定轴元件使用参考架指定轴的位移导数及其支承矩。

### 3.3 连续性与适用域

$D$ 与 $D^{-1}$ 在拐点连续，两侧导数有限；当 $c\ne c_2$ 时导数在拐点改变。内力方程关于 $F$ 连续且 Lipschitz。在线速连续、安装点分离的区间内，给定初值确定唯一局部解；连续位姿和速度对应连续的力及力导数。$l=0$ 时连线方向无定义。

两端交换时，$l,\dot l,F$ 保持，$\mathbf e_I$ 反向，作用在原两刚体上的载荷保持。

## 4. 源码映射

| 理论对象 | 实现 |
|---|---|
| 两端和四个参数 | `PointToPointSeriesSpringBilinearDamper`，见 [vehicle_force_elements.h](../../../libs/forces/include/orvd/forces/vehicle_force_elements.h) |
| 反函数、长度率、载荷与内力导数 | `InverseBilinearDamping`、`VehicleForcePlan::CalcAppliedForces`，见 [vehicle_force_plan.cc](../../../libs/forces/src/vehicle_force_plan.cc) |
| 初始内力 | `PointToPointSeriesSpringBilinearDamperForceState`，见 [resolved_startup_state.h](../../../libs/configuration/include/orvd/configuration/resolved_startup_state.h) |
| 连续状态中的力分量 | `SystemInstance::series_spring_damper_force_state_range`，见 [system_instance.h](../../../libs/system_assembly/include/orvd/system_assembly/system_instance.h) |
