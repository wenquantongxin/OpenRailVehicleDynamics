[English](WHEEL_RAIL_POSE_REDUCTION.en.md)

# 轮轨位姿归约与不平顺输入

本篇说明 ORVD 如何把轮对在轨型系中的三维状态归约为接触几何使用的四个位姿标量：配对滚转、配对摇头、横向偏移与竖向抬升；同时说明轨道方向、高低不平顺如何进入该归约。理论模型由 [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc)、[roll_yaw_pitch.cc](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc)、[track_irregularity_field.cc](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) 与 [rail_gauge_datum.cc](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc) 实现。

## 1. 范围

归约的输入由轮对放置、平面站位速率、横竖向不平顺的位移与时间变化率，以及每一侧轮轨对的固定几何量组成。输出只描述当前横截面内的相对位姿，不包含轨型面的完整刚体位姿、有效取样站位或轮轨材料点相对速度；这些量由车辆力组装层并行形成。

不平顺的谱与随机实现见[轨道不平顺谱及其空间随机实现](../track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md)，四个位姿标量的几何消费见[接触几何](CONTACT_GEOMETRY.md)，接触系与蠕滑率见[蠕滑率与接触系](CREEPAGE_AND_CONTACT_FRAME.md)。

## 2. 记号

坐标与正号沿用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)。轨型系记为 $T$，其 $x$ 轴沿线路前进方向、$y$ 轴向右、$z$ 轴向下。

| 记号 | 含义 | 实现量 |
|---|---|---|
| $W$ | 不旋转轮型面系，姿态为 $R_{TW}$ | WheelsetPlacement 的姿态来源 |
| $y_w,z_w$ | 轮对刚体原点在 $T$ 中的横、竖坐标 | lateral_meters, vertical_meters |
| $\phi_w,\psi_w$ | $R_{TW}$ 的 X-Z-Y 滚转与摇头 | roll_radians, yaw_radians |
| $\dot s$ | 带符号的平面站位速率 | track_station_rate_meters_per_second |
| $y_\epsilon,z_\epsilon$ | 横向与竖向不平顺位移 | TrackIrregularity 的位移量 |
| $\dot y_\epsilon,\dot z_\epsilon$ | 沿车辆行进采样不平顺的时间变化率 | TrackIrregularity 的速率量 |
| $\phi_r$ | 该侧带符号的轨底坡滚转 | rail_roll_radians |
| $y_r,z_r$ | 轨型面原点在 $T$ 中的固定基准 | rail_lateral_datum_meters, rail_vertical_datum_meters |
| $\sigma$ | 沿车轴带符号的轮型面横向基准 | wheel_lateral_datum_meters |
| $r_0,r$ | 标称与俯仰修正后的滚动半径 | nominal_rolling_radius_meters |
| $T_c$ | 轨底坡系，$R_{TT_c}=R_x(\phi_r)$ | 轨底坡横截面 |
| $T_\ell$ | 局部切向轨系 | 方向、高低与轨底坡的合成姿态 |
| $\phi_p,\psi_p$ | 配对滚转与配对摇头 | ContactPoseScalars |
| $\ell_p,h_p$ | 配对系横向偏移与向上为正的竖向抬升 | ContactPoseScalars |
| $s_c,s_e$ | 共享载体站位与该侧有效型面站位 | 力组装层的站位量 |

## 3. 模型

### 3.1 三套坐标系

归约有意区分三套坐标系：

1. 轨型系 $T$ 承载轮对放置、轨型面基准与不平顺位移。
2. 轨底坡系 $T_c$ 是钢轨横截面的自身轴系，用来表达轮、轨型面基准之间的横竖分离。
3. 局部切向轨系 $T_\ell$ 还包含方向与高低不平顺斜率，用来度量配对滚转。

三者不能合一。接触几何在轨底坡横截面比较两条型面曲线，而公法线与冲角必须相对钢轨的真实局部切向方向定义。

### 3.2 受保护的斜率角与配对摇头

实现用带分母地板的比值形成斜率角：

$$
\operatorname{SafeAtan2Ratio}(a,b)=
\begin{cases}
0, & |b|<\varepsilon_{\dot s},\\
\operatorname{atan2}(a,b), & |b|\geq\varepsilon_{\dot s}.
\end{cases}
$$

方向与高低不平顺角分别为

$$
\psi_\epsilon=\operatorname{SafeAtan2Ratio}(\dot y_\epsilon,\dot s),
\qquad
\theta_\epsilon=-\operatorname{SafeAtan2Ratio}(\dot z_\epsilon,\dot s).
$$

$z_T$ 向下，因此正的 $dz/ds$ 表示轨面沿站位下降，对应绕 $+y_T$ 的负俯仰。配对摇头是轮对相对钢轨实际横向方向的冲角：

$$
\psi_p=\psi_w-\psi_\epsilon.
$$

这里使用速率之比而不是分别传递空间斜率，使行进方向的符号集中在 $\dot s$ 中。

### 3.3 局部切向轨系与配对滚转

局部切向轨系为

$$
R_{TT_\ell}=R_z(\psi_\epsilon)R_y(\theta_\epsilon)R_x(\phi_r).
$$

记其第 1、2 列为 $\mathbf c_1,\mathbf c_2$。轮对车轴方向是 $R_{TW}$ 的第 1 列：

$$
\mathbf a=
\begin{bmatrix}
-\sin\psi_w\\
\cos\phi_w\cos\psi_w\\
\sin\phi_w\cos\psi_w
\end{bmatrix}.
$$

于是配对滚转为

$$
\phi_p=\operatorname{atan2}
\left(\mathbf c_2^\mathsf T\mathbf a,\,
      \mathbf c_1^\mathsf T\mathbf a\right).
$$

这等价于从相对姿态 $R_{TT_\ell}^{\mathsf T}R_{TW}$ 提取 X-Z-Y 滚转，但实现只形成所需的两列和两个点积。

### 3.4 滚动半径的俯仰修正

高低不平顺使横截面相对轮对竖向杠杆发生俯仰。其投影半径为

$$
r=r_0\cos\theta_\epsilon.
$$

该修正由独立模型门控制；关闭时取 $r=r_0$。小斜率下，忽略投影会引入约 $r_0\theta_\epsilon^2/2$ 的虚假轮轨分离。

### 3.5 轮、轨型面基准

轮型面基准从轮对中心沿车轴伸出 $\sigma$，再沿轮对自身竖向下移 $r$。令 $\rho=\sigma\cos\psi_w$，则

$$
y_{wd}=y_w+\rho\cos\phi_w-r\sin\phi_w,
\qquad
z_{wd}=z_w+\rho\sin\phi_w+r\cos\phi_w.
$$

这里必须使用轮对自身角 $\phi_w,\psi_w$，因为这两个杠杆属于轮对，而不是轮轨相对姿态。轨型面实际基准为

$$
y_{rd}=y_r+y_\epsilon,\qquad z_{rd}=z_r+z_\epsilon.
$$

### 3.6 分离量与配对编码

先把 $T$ 中的横竖分离转入轨底坡系：

$$
\begin{bmatrix}\ell_c\\v_c\end{bmatrix}
=
\begin{bmatrix}
\cos\phi_r & \sin\phi_r\\
-\sin\phi_r & \cos\phi_r
\end{bmatrix}
\begin{bmatrix}
y_{wd}-y_{rd}\\z_{wd}-z_{rd}
\end{bmatrix}.
$$

再编码到配对系：

$$
\ell_p=\cos\phi_p\,\ell_c+\sin\phi_p\,v_c,
\qquad
h_p=\sin\phi_p\,\ell_c-\cos\phi_p\,v_c.
$$

第二个矩阵的行列式为 $-1$ 且自乘为单位阵，所以接触几何可用同一表达式解码回 $(\ell_c,v_c)$。$h_p$ 向上为正；负的 $h_p$ 因而表示更深的几何穿透。

### 3.7 固定几何量

固定量不是系统状态，而是每一侧轮轨对的几何定义。轨距 $G$ 在轨冠以下规定深度的两轨轨距面之间度量。记右、左轨距面相对各自轨型面原点的偏距为 $\delta_{\mathrm R}$、$\delta_{\mathrm L}$，轨底坡幅值为 $\phi_c$，则

$$
y_r^{\mathrm R}=\frac{G}{2}+\delta_{\mathrm R},
\qquad
y_r^{\mathrm L}=-\left(\frac{G}{2}+\delta_{\mathrm L}\right),
\qquad
\phi_r^{\mathrm R}=-\phi_c,
\qquad
\phi_r^{\mathrm L}=+\phi_c.
$$

每一侧先把作者轨型点按相应的 $\mp\phi_c$ 滚转，再按滚转后的横坐标排序并依次连成分段线性折线；$\delta_{\mathrm R}$ 或 $\delta_{\mathrm L}$ 由该折线与测量线的向轨道中心一侧交点确定。由此，轨距定义不依赖后续选择的型面插值器。只有镜像对称轨型或等价的交点对称条件成立时，才能进一步写成 $\delta_{\mathrm R}=\delta_{\mathrm L}=\delta$。$z_r$ 给出轨型面的竖向参考偏置，$\sigma$ 按左右侧取相反符号，$r_0$ 与接触几何中的标称半径保持同一物理含义。

### 3.8 纵向原点

轨型截面的纵向原点有两种数学约定。轨道站位约定保持给定轨型原点不变；型面坐标约定沿轨型面自身第一轴平移，使轮刚体原点到轨型面原点的局部纵向坐标等于 $d=s_e-s_c$。

记原轨型面原点为 $\mathbf o$、姿态为 $R_{TR}$、轮刚体原点为 $\mathbf w$，则

$$
\mathbf o'=\mathbf o+
R_{TR}
\begin{bmatrix}
d-\left(R_{TR}^{\mathsf T}(\mathbf o-\mathbf w)\right)_x\\0\\0
\end{bmatrix},
$$

并有

$$
\left(R_{TR}^{\mathsf T}(\mathbf o'-\mathbf w)\right)_x=d.
$$

参照点必须是轮刚体原点；若改用已含车轴伸展的轮型面基准，非零摇头下会重复施加纵向站位修正。

### 3.9 不平顺场与有效站位

横、竖向不平顺分别由独立自然三次样条 $\eta_y(s)$、$\eta_z(s)$ 表示，两者可以有不同节点和定义域。记通道定义域为闭区间 $I_q=[s_0^q,s_n^q]$，线路自身的定义区间为 $I_T$。对任一通道 $q\in\{y,z\}$，实际进入本接触组装的场为

$$
q_\epsilon(s)=
\begin{cases}
\eta_q(s), & s\in I_q\cap I_T,\\
0, & \text{otherwise},
\end{cases}
\qquad
q_\epsilon'(s)=
\begin{cases}
\eta_q'(s), & s\in I_q\cap I_T,\\
0, & \text{otherwise}.
\end{cases}
$$

不平顺包装层先在 $I_q$ 外给出零位移、零斜率，而接触组装再在 $I_T$ 外把该通道置零；这不同于底层自然样条的端值平延。属于两个闭区间的边界节点仍取样条节点值与单侧内部斜率；若在任一激活边界上的位移非零，拼接后的场在那里不连续。

方向修正在共享站位 $s_c$ 单独取横向斜率；该侧的位移、两条斜率及其时间变化率随后在有效站位 $s_e$ 重新取样，并形成

$$
\dot y_\epsilon=y_\epsilon'(s_e)\dot s,\qquad
\dot z_\epsilon=z_\epsilon'(s_e)\dot s.
$$

有效站位由共享载体站位、配对方向与线路平面曲率共同确定。以载体站位处方向斜率修正摇头后，

$$
\psi_e=\psi_w-
\operatorname{SafeAtan2Ratio}\!\left(y_\epsilon'(s_c)\dot s,\dot s\right),
$$

$$
s_e=s_c+
\frac{-\sigma\sin\psi_e}
     {1-\kappa\left(y_w+\sigma\cos\psi_e\right)}.
$$

这构成“先求方向、再选该侧截面”的两阶段站位选择。$s_c$ 与 $s_e$ 的线路区间判定彼此独立：前者有效并不推出后者有效；任一站位落在 $I_T$ 外时，相应阶段的不平顺量按上式为零，即使不平顺样条本身在那里仍有定义。

### 3.10 X-Z-Y 姿态与速率

对 $R=R_x(\phi)R_z(\psi)R_y(\theta)$，实现采用

$$
\phi=\operatorname{atan2}(R_{21},R_{11}),
$$

$$
\psi=\operatorname{atan2}\!\left(-R_{01},
\sqrt{R_{00}^2+R_{02}^2}\right),
\qquad
\theta=\operatorname{atan2}(R_{02},R_{00}).
$$

角速度的正向关系为

$$
\boldsymbol\omega
=\dot\phi\,\mathbf e_x
+\dot\psi\,R_x(\phi)\mathbf e_z
+\dot\theta\,R_x(\phi)R_z(\psi)\mathbf e_y.
$$

令 $\tau=\omega_y\cos\phi+\omega_z\sin\phi$，逆映射是

$$
\dot\psi=-\omega_y\sin\phi+\omega_z\cos\phi,
\qquad
\dot\theta=\frac{\tau}{\cos\psi},
\qquad
\dot\phi=\omega_x+\tau\tan\psi.
$$

## 4. 算法结构

BuildContactPoseScalars 的计算顺序是：

1. 由不平顺速率与站位速率形成 $\psi_\epsilon,\theta_\epsilon$。
2. 由局部切向轨系和车轴方向提取 $\phi_p$，并令 $\psi_p=\psi_w-\psi_\epsilon$。
3. 计算俯仰修正半径 $r$。
4. 放置轮、轨型面基准并形成它们在 $T$ 中的分离。
5. 把分离转到 $T_c$，再用自逆反射编码为 $(\ell_p,h_p)$。

归约本身只含常数次三角函数、点积和二阶线性变换。有效站位与不平顺样条求值位于上游；X-Z-Y 姿态与速率解析也都是常数次运算。

## 5. 非光滑性与理论适用条件

- 当 $|\dot s|$ 穿过分母地板时，斜率角从零切换到 atan2 值；若分子不为零，该切换不连续。
- atan2 在负实轴上有 $\pm\pi$ 分支。倒行时，速率零的符号也会影响所得分支，因此倒行不能简单套用前进的平直轨极限。
- 平直轨且前进时，$\theta_\epsilon=0$、$\psi_p=\psi_w$；在 $\cos\psi_w>0$ 时有 $\phi_p=\phi_w-\phi_r$。
- 不平顺在支撑交集 $I_q\cap I_T$ 的边界由零函数切换到样条；非零端值或端点斜率会产生跳变。
- X-Z-Y 解析在 $\cos\psi=0$ 处奇异，角速率逆映射同样含这一奇异性。
- 有效站位映射要求 $1-\kappa(y_w+\sigma\cos\psi_e)\ne0$；该分母趋近零时，横向偏置到中心线站位的局部映射发生几何奇异。
- 钢轨在本模型中是线路携带的几何构造，没有独立惯量或材料点速度。不平顺改变轨型面位置与姿态，但不由本归约生成轨侧材料速度。
- 同一不平顺斜率在位姿归约中经速率比进入，在轨型刚体姿态中可直接由空间斜率进入。前进且速度高于地板时两者一致；静止与倒行时二者代表不同的建模延拓。

## 6. 实现映射

| 理论对象 | 主要实现 |
|---|---|
| 四标量位姿归约与配对滚转 | [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) |
| X-Z-Y 姿态解析与速率映射 | [roll_yaw_pitch.cc](../../../libs/wheel_rail_contact/src/roll_yaw_pitch.cc) |
| 两通道不平顺场 | [track_irregularity_field.cc](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) |
| 轨距面与左右轨基准 | [rail_gauge_datum.cc](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc) |
| 有效站位、型面放置与输入形成 | [wheel_rail_contact_force_plan.cc](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) |
| 四标量的下游解码 | [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
