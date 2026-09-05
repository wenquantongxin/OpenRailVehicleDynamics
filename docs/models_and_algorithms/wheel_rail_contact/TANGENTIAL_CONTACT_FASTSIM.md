[English](TANGENTIAL_CONTACT_FASTSIM.en.md)

# 切向接触力：FASTSIM 条带推进

本篇说明 ORVD 如何在一个椭圆接触斑上，由法向载荷、蠕滑率、Kalker 系数与摩擦律求得纵横两个切向力分量。实现沿用 [Kalker 的 FASTSIM](https://doi.org/10.1080/00423118208968684) 所代表的局部柔度与条带推进思想；本仓库采用的压力形状、条带细化和离散推进以 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) 为准。

## 1. 模型范围与记号

接触斑为椭圆，沿滚动方向与横向的半轴分别为 $a$、$b$。使用归一化坐标

$$
u=\frac{x}{a},\qquad v=\frac{y}{b},\qquad u^2+v^2\le1.
$$

法向载荷为 $N$，摩擦系数为 $\mu$，纵向、横向和自旋蠕滑率为 $\xi_x$、$\xi_y$、$\varphi$。Kalker 系数给出三个柔度尺度 $L_x$、$L_y$、$L_\varphi$；其形成见 [Kalker 线性蠕滑系数](KALKER_COEFFICIENTS.md)。本篇假定 $N$、$a$、$b$、$\mu$ 均为正，且半轴比属于 Kalker 系数模型的正有限定义域。

## 2. 应力积累模型

### 2.1 局部柔度

令 $G=E/[2(1+\nu)]$、$\kappa=a/b$，实现使用下列表达式。这里的 $\kappa$ 沿用 Kalker 系数篇的局部记号，表示半轴比而不是线路平面曲率。

$$
L_x=\frac{8a}{3C_{11}G},\qquad
L_y=\frac{8a}{3C_{22}G},\qquad
L_\varphi=\frac{\pi a\sqrt{\kappa}}{4C_{23}G}.
$$

这些量把蠕滑率换成沿材料行程的切应力积累率。对横坐标为 $y$ 的条带，

$$
r_x(y)=\frac{\xi_x}{L_x}-\frac{\varphi y}{L_\varphi},
\qquad
r_y(x)=\frac{\xi_y}{L_y}+\frac{\varphi x}{L_\varphi}.
$$

$r_x$ 在一条带内为常数；有自旋时，$r_y$ 沿纵向位置线性变化。因此“应力沿行程线性增长”只对 $\tau_x$，或对 $\varphi=0$ 时的 $\tau_y$ 成立，不能无条件推广到两个分量。

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
-\frac{\varphi}{2L_\varphi}(x_\ell^2-x^2).
\end{aligned}
$$

所以有自旋时 $\tau_y$ 是 $x$ 的二次函数。源码在每个输运步的中点求 $r_y$；由于 $r_y$ 对 $x$ 为线性函数，中点积分在单个黏着步上精确积分这一线性率。

负号来自推进方向与坐标方向的关系：正的平动蠕滑率产生负的对应切向应力和合力。

### 2.3 应力积累率零点

当 $\varphi\ne0$ 时，同时令 $r_x=0$ 与 $r_y=0$ 得到

$$
x_p=-\frac{\xi_yL_\varphi}{\varphi L_y},
\qquad
y_p=\frac{\xi_xL_\varphi}{\varphi L_x},
$$

归一化后为

$$
u_p=-\frac{\xi_yL_\varphi}{\varphi L_y\,a},
\qquad
v_p=\frac{\xi_xL_\varphi}{\varphi L_x\,b}.
$$

这两个量在源码中命名为 `spin_pole_longitudinal` 与 `spin_pole_lateral`。从实现公式看，它们是两个应力积累率的共同零点，而不是无条件的“刚性滑移零点”：刚性滑移场不含 $L_x$、$L_y$、$L_\varphi$，只有在附加的柔度关系成立时，两种零点才会重合。

## 3. 压力、摩擦与黏滑分区

### 3.1 抛物面压力

切向求解器使用归一化抛物面压力

$$
p(u,v)=p_0(1-u^2-v^2),
\qquad
p_0=\frac{2N}{\pi ab},
$$

并满足

$$
\iint_{u^2+v^2<1}p(u,v)\,ab\,du\,dv=N.
$$

这不是法向 Hertz 解中的半椭球压力，而是本切向近似采用的独立形状。压力分布不仅需要积分为 $N$：它还逐点决定摩擦上限

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

### 3.3 随滑动速度下降的摩擦系数

`FrictionCoefficientAt` 先由两个平动蠕滑率形成滑动速度

$$
v_s=\max(|v_{\mathrm{ref}}|,v_{\min})
\sqrt{\xi_x^2+\xi_y^2},
$$

再计算

$$
\mu(v_s)=\mu_0\left[(1-A)e^{-Bv_s}+A\right].
$$

$\mu(0)=\mu_0$。当 $B>0$ 时，$v_s\to\infty$ 有 $\mu\to A\mu_0$；进一步在 $\mu_0>0$、$0\leq A<1$ 时，曲线随 $v_s$ 严格下降（$A=1$ 时退化为常数）。因此“下降摩擦”这一物理解读以这些参数条件为前提；公式本身并不会强制它们。自旋蠕滑率不进入这条斑级摩擦律。该函数形式与 [Polach 轮轨接触模型](https://komunikacie.uniza.sk/artkey/csl-200101-0002_contact-of-wheel-and-rail-in-computer-simulation-of-vehicle-dynamics-and-axle-drive-dynamics.php) 中使用的下降摩擦曲线同型；ORVD 的离散接触解仍由本篇其余公式定义。

## 4. 条带与胞元离散

### 4.1 条带布置

`LayStrips` 在 $v\in[-1,1]$ 上布置条带。应力积累率零点不在单位圆内或不存在时，使用等宽条带。若

$$
u_p^2+v_p^2<1,
$$

则从两个横向边缘分别向内推进，并在接近 $v_p$ 时逐次把条带宽度减半。若几何级数式推进无法得到目标尺度附近的条带，算法改用覆盖全斑的备用布置：目标宽度不小于半斑宽时只用一条全宽条带，否则使用近等宽条带并把包含 $v_p$ 的一条二分。细化只改变横向求积网格，不改变连续模型中的 $r_x$、$r_y$ 或摩擦律。

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
tau_x = 0; tau_y = 0
u_previous = h
u = h - 0.5 * delta_u
for each longitudinal cell:
    step = a * (u_previous - u)
    u_mid = 0.5 * (u_previous + u)
    r_x = xi_x / L_x - phi * (b * v) / L_phi
    r_y = xi_y / L_y + phi * (a * u_mid) / L_phi
    trial_x = tau_x - r_x * step
    trial_y = tau_y - r_y * step
    p = p_0 * max(0, 1 - v*v - u*u)
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

本实现包含四项主要近似：以三个局部柔度代替完整弹性耦合；采用抛物面压力作为局部摩擦界；以条带和胞元作中点离散；在应力积累率零点附近自适应改变横向条带。

相应的非光滑来源包括：胞元从黏着切换到滑移时径向投影的导数改变；应力积累率零点穿过单位圆或条带布局改变；参考速度地板的折点；Kalker 折线节点及有限表与渐近式的拼接。力本身在单个黏滑投影处连续，但对状态的导数一般不连续。

该模型输出 $F_x$、$F_y$，不输出斑内关于法向的直接自旋力矩，也不求解条带之间的非局部弹性耦合。这些是模型范围，而不是从离散结果中可以恢复的遗漏量。

## 6. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 摩擦系数 | `FrictionCoefficientAt`，见 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
| 柔度、积累率与胞元推进 | `TangentialContactSolver::Solve` |
| 应力积累率零点与条带布置 | `spin_pole_longitudinal`、`spin_pole_lateral`、`TangentialContactSolver::LayStrips` |
| Kalker 系数 | `KalkerCoefficientTable::At`，见 [`kalker_coefficient_table.cc`](../../../libs/wheel_rail_contact/src/kalker_coefficient_table.cc) |
| 输入与输出量 | `TangentialContactPatch`、`TangentialContactResult`，见 [`tangential_contact_force.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/tangential_contact_force.h) |
