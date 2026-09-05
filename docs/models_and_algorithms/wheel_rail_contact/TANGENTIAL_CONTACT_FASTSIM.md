[English](TANGENTIAL_CONTACT_FASTSIM.en.md)

# 切向接触力：FASTSIM 条带推进

本篇说明 ORVD 如何在一个椭圆接触斑上，由法向载荷、蠕滑率、Kalker 系数与摩擦律求得纵横两个切向力分量。实现沿用 [Kalker 的 FASTSIM](https://doi.org/10.1080/00423118208968684) 所代表的局部柔度与条带推进思想；本仓库采用的压力形状、条带细化和离散推进以 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) 为准。

## 1. 模型范围与记号

接触斑为椭圆，沿滚动方向与横向的半轴分别为 $a$、$b$。使用归一化坐标

$$
u=\frac{x}{a},\qquad v=\frac{y}{b},\qquad u^2+v^2\le1.
$$

法向载荷为 $N$，取含阻尼的法向总力；纵向、横向和自旋蠕滑率为 $\xi_x$、$\xi_y$、$\xi_{sp}$，其定义见 [蠕滑率与接触坐标系](CREEPAGE_AND_CONTACT_FRAME.md)。摩擦系数 $\mu$ 每个接触斑只求一次，并由斑内全部胞元共享。Kalker 系数给出三个柔度尺度 $L_x$、$L_y$、$L_{sp}$；其形成见 [Kalker 线性蠕滑系数](KALKER_COEFFICIENTS.md)。本篇假定 $N$、$a$、$b$、$\mu$ 均为正，且半轴比落在所选 Kalker 系数函数的定义域内。

## 2. 应力积累模型

### 2.1 局部柔度

令 $G=E/[2(1+\nu)]$、$\kappa=a/b$，实现使用下列表达式。这里的 $\kappa$ 沿用 Kalker 系数篇的局部记号，表示半轴比而不是线路平面曲率。

$$
L_x=\frac{8a}{3C_{11}G},\qquad
L_y=\frac{8a}{3C_{22}G},\qquad
L_{sp}=\frac{\pi a\sqrt{\kappa}}{4C_{23}G}.
$$

这些量把蠕滑率换成沿材料行程的切应力积累率。对横坐标为 $y$ 的条带，

$$
r_x(y)=\frac{\xi_x}{L_x}-\frac{\xi_{sp} y}{L_{sp}},
\qquad
r_y(x)=\frac{\xi_y}{L_y}+\frac{\xi_{sp} x}{L_{sp}}.
$$

$r_x$ 在一条带内为常数；有自旋时，$r_y$ 沿纵向位置线性变化。因此“应力沿行程线性增长”只对 $\tau_x$，或对 $\xi_{sp}=0$ 时的 $\tau_y$ 成立，不能无条件推广到两个分量。

### 2.2 黏着区内的连续表达

在归一化横坐标 $v$ 的条带上，半弦长为

$$
h(v)=\sqrt{1-v^2}.
$$

材料从前缘 $x_\ell=ah$ 向后缘运动并在前缘以零切应力进入。尚未达到摩擦上限时，

$$
\begin{aligned}
\tau_x(x,y)
&=-\int_x^{x_\ell}r_x(y)\,dx'
=-r_x(y)(x_\ell-x),\\
\tau_y(x,y)
&=-\int_x^{x_\ell}r_y(x')\,dx'\\
&=-\frac{\xi_y}{L_y}(x_\ell-x)
-\frac{\xi_{sp}}{2L_{sp}}(x_\ell^2-x^2).
\end{aligned}
$$

所以有自旋时 $\tau_y$ 是 $x$ 的二次函数。源码在每个输运步的中点求 $r_y$；由于 $r_y$ 对 $x$ 为线性函数，中点积分在单个黏着步上精确积分这一线性率。

负号来自推进方向与坐标方向的关系：正的平动蠕滑率产生负的对应切向应力和合力。

### 2.3 应力积累率零点

当 $\xi_{sp}\ne0$ 时，同时令 $r_x=0$ 与 $r_y=0$ 得到

$$
x_p=-\frac{\xi_yL_{sp}}{\xi_{sp}L_y},
\qquad
y_p=\frac{\xi_xL_{sp}}{\xi_{sp}L_x},
$$

归一化后为

$$
u_p=-\frac{\xi_yL_{sp}}{\xi_{sp}L_y\,a},
\qquad
v_p=\frac{\xi_xL_{sp}}{\xi_{sp}L_x\,b}.
$$

这两个量在源码中命名为 `spin_pole_longitudinal` 与 `spin_pole_lateral`。从实现公式看，它们是两个应力积累率的共同零点，而不是无条件的“刚性滑移零点”：刚性滑移场不含 $L_x$、$L_y$、$L_{sp}$，只有在附加的柔度关系成立时，两种零点才会重合。

## 3. 压力、摩擦与黏滑分区

### 3.1 抛物面压力

切向求解器使用归一化抛物面压力

$$
p(u,v)=p_0^{\mathrm F}(1-u^2-v^2),
\qquad
p_0^{\mathrm F}=\frac{2N}{\pi ab},
$$

并满足

$$
\iint_{u^2+v^2<1}p(u,v)\,ab\,du\,dv=N.
$$

这不是法向 Hertz 解中的半椭球压力，而是本切向近似采用的独立形状。本篇以 $p_0^{\mathrm F}$ 表示 FASTSIM 抛物面压力峰值；[法向接触力](NORMAL_CONTACT_FORCE.md)篇则以 $p_0^{\mathrm H}=3F_e/(2\pi ab)$ 表示 Hertz 峰值。二者之比为

$$
\frac{p_0^{\mathrm F}}{p_0^{\mathrm H}}=\frac{4N}{3F_e},
$$

仅在 $N=F_e$ 时等于 $4/3$。法向篇的峰值只由弹性法向力 $F_e$ 构成，而本篇的 $N$ 是含阻尼的法向总力，所以不能把前者直接代入本篇的摩擦界。压力分布不仅需要积分为 $N$：它还逐点决定摩擦上限

$$
\tau_{\max}(u,v)=\mu p(u,v),
$$

从而决定每个胞元何时由黏着转入滑移、黏滑边界的位置以及未完全饱和时的合力。两个具有相同积分但不同空间形状的压力场，一般不会给出相同的切向解。

### 3.2 黏着与滑移

对一个输运步，先按积累率形成试算应力 $\boldsymbol\tau^*$。若

$$
\|\boldsymbol\tau^*\|\le\mu p,
$$

胞元保持黏着并接受试算值；否则沿试算方向径向投影到摩擦圆：

$$
\boldsymbol\tau=\mu p\,
\frac{\boldsymbol\tau^*}{\|\boldsymbol\tau^*\|}.
$$

径向投影保持应力方向，且每个胞元都满足自身的局部摩擦界。最终合力由各胞元应力乘其面积后求和。

记离散切向合力为 $\mathbf F_t=[F_x,F_y]^{\mathsf T}$。整斑充分滑移时，每个胞元都有 $\|\boldsymbol\tau_{ij}\|=\mu p_{ij}$，因此

$$
\|\mathbf F_t\|\leq\sum_{i,j}\mu p_{ij}\,\Delta A_{ij}.
$$

等号要求所有胞元的应力方向一致；无自旋的充分滑移满足这一条件。等宽条带上的中点求积不保持连续压力的积分归一化，右端可以大于 $\mu N$，离散合力因而也可以超过 $\mu N$。有自旋时各胞元方向不必一致，只保留上述标量求积界。被超出的是连续压力积分的离散近似，而不是任一胞元的局部摩擦界，因此 $\mu N$ 不是离散合力的严格上界。

### 3.3 随滑动速度下降的摩擦系数

`FrictionCoefficientAt` 先由两个平动蠕滑率形成滑动速度

$$
v_s=\max(|v_{\mathrm{ref}}|,v_{\min})
\sqrt{\xi_x^2+\xi_y^2},
$$

其中 $v_{\mathrm{ref}}$ 是[蠕滑率与接触坐标系](CREEPAGE_AND_CONTACT_FRAME.md)篇中已经托底的参考速度 $V$，也就是蠕滑率定义式的除数；$v_{\min}$ 是摩擦律自带的另一个参考速度地板，与蠕滑侧的托底门槛 $V_{\min}$ 是两个独立的量。两个地板的相对大小决定这条式子读出什么。只要 $v_{\min}\le V_{\min}$，由 $|v_{\mathrm{ref}}|\ge V_{\min}$ 知外层取大返回 $|v_{\mathrm{ref}}|$，它与蠕滑率中的除数相消：

$$
v_s=|V|\sqrt{\left(\frac{v_{C,x}}{V}\right)^2+\left(\frac{v_{C,y}}{V}\right)^2}
=\sqrt{v_{C,x}^2+v_{C,y}^2},
$$

即 $v_s$ 等于斑参考点处面内平动相对速度的模长，蠕滑侧的托底被精确约掉。反之，若 $v_{\min}>V_{\min}$，则在 $|v_{\mathrm{ref}}|<v_{\min}$ 的区间上外层取大生效，$v_s=v_{\min}\sqrt{\xi_x^2+\xi_y^2}$ 不再等于这一平动相对速度模长；取大因子在 $|v_{\mathrm{ref}}|=v_{\min}$ 处有导数折点，并通常把该折点传给 $v_s$。

得到 $v_s$ 后，摩擦系数按下式计算

$$
\mu(v_s)=\mu_0\left[(1-A)e^{-Bv_s}+A\right].
$$

$\mu(0)=\mu_0$。当 $B>0$ 时，$v_s\to\infty$ 有 $\mu\to A\mu_0$；进一步在 $\mu_0>0$、$0\leq A<1$ 时，曲线随 $v_s$ 严格下降（$A=1$ 时退化为常数）。因此“下降摩擦”这一物理解读以这些参数条件为前提；公式本身并不会强制它们。自旋蠕滑率不进入这条斑级摩擦律。该函数形式与 [Polach 轮轨接触模型](https://komunikacie.uniza.sk/artkey/csl-200101-0002_contact-of-wheel-and-rail-in-computer-simulation-of-vehicle-dynamics-and-axle-drive-dynamics.php) 中使用的下降摩擦曲线同型；ORVD 的离散接触解仍由本篇其余公式定义。

## 4. 条带与胞元离散

### 4.1 条带布置

`LayStrips` 在 $v\in[-1,1]$ 上布置条带。本节出现三个离散量：每条带的纵向胞元数 $n_x$、横向条带数 $n_y$，以及细化目标宽度 $w_\ast$（在斑宽取为 $2$ 的归一化单位下度量）。三者都是求积分辨率而不是物理量，只决定网格，不进入连续模型。应力积累率零点不在单位圆内或不存在时，使用 $n_y$ 条等宽条带。若

$$
u_p^2+v_p^2<1,
$$

则从两个横向边缘分别向内推进，并在接近 $v_p$ 时逐次把条带宽度减半，直到宽度落入 $[w_\ast,2w_\ast]$ 为止。若几何级数式推进无法把最窄条带带进这个区间，算法改用覆盖全斑的备用布置：$w_\ast$ 不小于半斑宽时只用一条全宽条带，否则使用近等宽条带并把包含 $v_p$ 的一条二分。细化只改变横向求积网格，不改变连续模型中的 $r_x$、$r_y$ 或摩擦律。

因为 $(u_p,v_p)$ 随蠕滑率变化，横向求积节点也随状态变化。单位圆内外的严格分支以及备用网格的整组替换没有插值连接；条带集合改变时，离散合力可能出现导数折点，也可能发生有限跳变，其幅度没有统一的小量上界。

### 4.2 纵向推进

对中心为 $v_j$、归一化宽度为 $w_j$ 的条带，半弦为 $h_j=\sqrt{1-v_j^2}$。纵向划分步长

$$
\Delta u=\frac{2h_j}{n_x},
\qquad
\Delta A=a\Delta u\,b w_j.
$$

材料从 $u=h_j$ 开始，第一步走到第一个胞元中心，长度为半个胞元；后续步骤在相邻胞元中心之间前进一个整胞元。每个胞元的面积权重始终是完整的 $\Delta A$。实现的核心递推为


```text
p0_F = 2 * N / (pi * a * b)
tau_x = 0; tau_y = 0
u_previous = h
u = h - 0.5 * delta_u
for each longitudinal cell:
    step = a * (u_previous - u)
    u_mid = 0.5 * (u_previous + u)
    r_x = xi_x / L_x - xi_sp * (b * v) / L_sp
    r_y = xi_y / L_y + xi_sp * (a * u_mid) / L_sp
    trial_x = tau_x - r_x * step
    trial_y = tau_y - r_y * step
    p = p0_F * max(0, 1 - v*v - u*u)
    (tau_x, tau_y) = radial_projection_to_radius_mu_p(trial_x, trial_y)
    F_x += tau_x * delta_A
    F_y += tau_y * delta_A
    u_previous = u
    u = u - delta_u
```

### 4.3 小蠕滑极限

无自旋且整个斑黏着时，连续模型回到 Kalker 线性力。等宽条带的中点求积给出有限分辨率因子

$$
F_x=-G\,a\,b\,C_{11}\xi_x\left(1+\frac{w^2}{8}\right),
\qquad
F_y=-G\,a\,b\,C_{22}\xi_y\left(1+\frac{w^2}{8}\right),
\qquad
w=\frac{2}{n_y}.
$$

当 $n_y\to\infty$ 时，该因子趋于 $1$，得到连续线性极限。

## 5. 近似与非光滑性

本实现的主要近似包括：以三个局部柔度代替完整弹性耦合；采用抛物面压力作为局部摩擦界；由斑级平动滑动速度计算一个 $\mu$ 并在斑内共享；以条带和胞元作中点离散；在应力积累率零点附近自适应改变横向条带。

相应的非光滑来源包括：胞元从黏着切换到滑移时径向投影的导数改变；应力积累率零点穿过单位圆或条带布局改变；$v_{\min}>V_{\min}$ 时摩擦律参考速度地板的折点（$v_{\min}\le V_{\min}$ 时这一取大为恒等，见 §3.3）；Kalker 折线节点及有限表与渐近式的拼接。蠕滑率自身在 $V_0=0$ 与 $|V_0|=V_{\min}$ 处的非光滑属[蠕滑率与接触坐标系](CREEPAGE_AND_CONTACT_FRAME.md)篇，经 $\xi_x$、$\xi_y$、$\xi_{sp}$ 传入本篇。力本身在单个黏滑投影处连续，但对状态的导数一般不连续。

该模型输出 $F_x$、$F_y$，不输出斑内关于法向的直接自旋力矩，也不求解条带之间的非局部弹性耦合。这些是模型范围，而不是从离散结果中可以恢复的遗漏量。

## 6. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 摩擦律与斑级摩擦系数 | `FrictionLaw`、`FrictionCoefficientAt`，见 [`tangential_contact_force.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/tangential_contact_force.h) 与 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| 柔度、积累率与胞元推进 | `TangentialContactSolver::Solve`，见 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| 应力积累率零点与条带布置 | `spin_pole_longitudinal`、`spin_pole_lateral`、`TangentialContactSolver::LayStrips`，见 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| Kalker 系数 | `KalkerCoefficientTable::At`，见 [`kalker_coefficient_table.cc`](../../../libs/wheel_rail_contact/src/kalker_coefficient_table.cc) |
| 离散分辨率与条带细化 | `TangentialContactConfiguration`、`TangentialContactSolver::LayStrips`，见 [`tangential_contact_force.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/tangential_contact_force.h) 与 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| 输入与输出量 | `TangentialContactPatch`、`TangentialContactResult`，见 [`tangential_contact_force.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/tangential_contact_force.h) |
