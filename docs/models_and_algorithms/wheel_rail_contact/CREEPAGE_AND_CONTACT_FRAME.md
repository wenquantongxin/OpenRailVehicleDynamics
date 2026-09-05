[English](CREEPAGE_AND_CONTACT_FRAME.en.md)

# 蠕滑率与接触坐标系

本篇说明 ORVD 如何把接触处的相对运动写入接触坐标系，并形成纵向蠕滑率、横向蠕滑率、自旋蠕滑率与法向接近速度。理论定义与实现分别落在 [`contact_creepage.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h) 和 [`contact_creepage.cc`](../../../libs/wheel_rail_contact/src/contact_creepage.cc)。

## 1. 范围与记号

接触几何给出斑位置、局部滚动半径与轨面坡角；本篇从这些量出发定义运动学量。法向力见[法向接触力](NORMAL_CONTACT_FORCE.md)，Kalker 系数与切向力分别见 [Kalker 线性蠕滑系数](KALKER_COEFFICIENTS.md)和 [FASTSIM 切向接触](TANGENTIAL_CONTACT_FASTSIM.md)，最终的力与扳手组装见[接触模型组装与成对扳手](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.md)。

除非另有说明，向量都在轨型系 `T` 中表达。使用以下记号：

| 记号 | 含义 |
|---|---|
| $R_{TC}$ | 接触系 `C` 到轨型系 `T` 的旋转 |
| $\alpha$ | 接触系角，即斑形心处轨面坡角加带侧别符号的轨底坡 |
| $\mathbf n_T$ | 在 `T` 中表达的接触法向 |
| $\mathbf v_T$、$\boldsymbol\omega_T$ | 接触处轮相对轨的平动速度与角速度 |
| $\dot\ell$ | 接触沿线路前进的路径速率 |
| $\Omega$ | 车轮绕车轴的自转率；按本仓库坐标约定，前进滚动时为负 |
| $r$ | 接触处未变形的局部滚动半径 |
| $V_0$、$V$ | 托底前、托底后的蠕滑参考速度 |
| $\xi_x$、$\xi_y$、$\varphi$ | 纵向、横向与自旋蠕滑率 |
| $v_n$ | 法向接近速度；接近为正 |

## 2. 接触坐标系与相对运动

### 2.1 由轨面坡角构造接触系

`MakeContactFrame` 构造绕轨型系 `+x` 轴的纯滚转：

$$
R_{TC}(\alpha)=
\begin{bmatrix}
1&0&0\\
0&\cos\alpha&-\sin\alpha\\
0&\sin\alpha&\cos\alpha
\end{bmatrix},\qquad
\mathbf n_T=R_{TC}\mathbf e_z=
\begin{bmatrix}0\\-\sin\alpha\\\cos\alpha\end{bmatrix}.
$$

因此接触系的纵轴与轨型系纵轴完全重合。实现采用 `ContactPatch::rail_slope_angle_radians`，而不是 `common_normal_angle_radians`；前者描述钢轨参考表面的朝向，后者描述轮轨型面的几何公法线，两者不能互换。

相对速度与相对角速度通过旋转的转置写入接触系：

$$
\mathbf v_C=R_{TC}^{\mathsf T}\mathbf v_T,
\qquad
\boldsymbol\omega_C=R_{TC}^{\mathsf T}\boldsymbol\omega_T.
$$

特别地，$v_{C,x}=v_{T,x}$；横向分量、法向分量和法向自旋分量则随 $\alpha$ 混合。

### 2.2 钢轨材料参考点处的运动

装配层在钢轨材料参考点 `R` 处形成相对运动。若轮型面系 W 的基准点位置与速度为 $\mathbf o_W$、$\mathbf v_o$，轮体角速度为 $\boldsymbol\omega$，`R` 在轨型系中的位置为 $\mathbf x_R$，则

$$
\mathbf v_T=\mathbf v_o+\boldsymbol\omega\times(\mathbf x_R-\mathbf o_W),
\qquad
\boldsymbol\omega_T=\boldsymbol\omega.
$$

这里钢轨是由线路携带的静止几何，不是多体系统中带状态的刚体，因而钢轨材料速度取零。点 `R` 用于形成相对材料速度；它与力的作用点 `P` 不同，二者的关系见[接触模型组装与成对扳手](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.md)。

## 3. 蠕滑率

### 3.1 参考速度

轮缘速度与未托底参考速度定义为

$$
v_{\mathrm{rim}}=-\Omega r,
\qquad
V_0=\frac{1}{2}\left(\dot\ell+v_{\mathrm{rim}}\right).
$$

纯滚动时 $\dot\ell=v_{\mathrm{rim}}$。两者的平均值用作把相对滑动归一化为蠕滑率的速度尺度。为使静止附近的除法有定义，实现以 $V_{\min}>0$ 对其作带符号托底：

$$
V=
\begin{cases}
V_0,&|V_0|\ge V_{\min},\\
\operatorname{copysign}(V_{\min},V_0),&0<|V_0|<V_{\min},\\
+V_{\min},&V_0=0.
\end{cases}
$$

这是一项明确的低速建模约定，而不是平滑正则化。$V$ 在 $V_0=0$ 处改变符号，在 $|V_0|=V_{\min}$ 处连续但导数不连续。

必须限定“过零翻号”的条件。若某个分子在 $V_0=0$ 两侧保持同一个非零值，则其商在 $V$ 变号时随之翻号；若分子同时过零、改变符号或改变大小，则不能据参考速度单独判断相应蠕滑率的符号。因而实际车辆反向过程中，三个蠕滑率不具有无条件同时翻号的不变量。

### 3.2 三个蠕滑率的定义

实现使用

$$
\xi_x=\frac{v_{C,x}}{V},
\qquad
\xi_y=\frac{v_{C,y}}{V},
\qquad
\varphi=\frac{\omega_{C,z}}{V}.
$$

$\xi_x$ 与 $\xi_y$ 无量纲；$\varphi$ 的量纲为 $\mathrm{m}^{-1}$。由于接触系是纯滚转，纵向蠕滑率的分子不受 $\alpha$ 影响。自旋则包含车轮角速度在接触法向上的投影：

$$
\omega_{C,z}=-\sin\alpha\,\omega_{T,y}+\cos\alpha\,\omega_{T,z}.
$$

### 3.3 法向接近速度

法向接近速度与接触系中的法向速度分量相同：

$$
v_n=\mathbf n_T\cdot\mathbf v_T
=-\sin\alpha\,v_{T,y}+\cos\alpha\,v_{T,z}
=v_{C,z}.
$$

轨型系竖轴向下，$\mathbf n_T$ 指入钢轨，所以车轮沿法向接近钢轨时 $v_n>0$。法向接触律用这一符号定义阻尼项。

### 3.4 滚动半径约定

参考速度中的 $r$ 是接触处未变形的局部轮半径：

$$
r=r_0+h_w(y_w),
$$

其中 $r_0$ 是标称滚动半径，$h_w(y_w)$ 是轮型面在接触站位处的高度。本实现不从 $r$ 中减去弹性穿透。力作用点可以采用 $r-\delta_{\mathrm{eq}}/2$，但那是作用点位置的约定，不改变蠕滑率的参考速度。

## 4. 数值结构与适用条件

计算顺序为：构造 $R_{TC}$ 与 $\mathbf n_T$，把相对运动转入接触系，形成 $V$，再计算三个蠕滑率；法向接近速度由同一接触系直接投影。这样法向力、蠕滑率与最终力向量使用完全相同的 $\alpha$。

对有界的分子，参考速度托底使精确静止附近的比值保持有界，但保留两类非光滑性：$V_0=0$ 处的符号跳变，以及 $|V_0|=V_{\min}$ 处的导数折点。该模型适合具有明确行进方向的滚动工况；若研究持续低速换向，应把这一托底视为模型的一部分，并评估它对力与数值 Jacobian 的影响。

## 5. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 接触系与法向 | `ContactFrame`、`MakeContactFrame`，见 [`contact_creepage.cc`](../../../libs/wheel_rail_contact/src/contact_creepage.cc) |
| 相对运动与蠕滑率 | `ContactRelativeMotion`、`Creepages`、`ComputeCreepages`，见 [`contact_creepage.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h) |
| 法向接近速度 | `ComputeNormalApproachSpeed`，见 [`contact_creepage.cc`](../../../libs/wheel_rail_contact/src/contact_creepage.cc) |
| `R` 点处的刚体速度 | `WheelRailContactModel::Evaluate`，见 [`wheel_rail_contact_model.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc) |
| $\alpha$ 与局部半径的几何来源 | `ContactPatch`，见 [`contact_geometry.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_geometry.h) |
