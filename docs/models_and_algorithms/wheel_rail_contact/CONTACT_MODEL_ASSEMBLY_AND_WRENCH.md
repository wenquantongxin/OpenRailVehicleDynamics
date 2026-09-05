[English](CONTACT_MODEL_ASSEMBLY_AND_WRENCH.en.md)

# 单轮接触模型组装与成对扳手

本篇说明 `WheelRailContactModel` 如何把接触几何、法向力、蠕滑率、Kalker 系数与 FASTSIM 切向力组装成作用于一个车轮—钢轨接触斑的空间扳手。重点是各物理阶段之间传递什么、钢轨材料参考点 `R` 与轮侧力作用点 `P` 为何不同，以及力和力矩如何换点、换基。

## 1. 五阶段物理链

单个车轮对单根钢轨的接触由五个理论阶段构成：

1. **接触几何**：由轮轨型面和相对位姿得到零个或多个接触斑，以及每个斑的位置、宽度、面积、穿透、局部半径和表面方向。
2. **法向接触**：由斑几何和法向接近速度得到法向力 $N$、等效穿透 $\delta_{\mathrm{eq}}$ 与接触椭圆半轴 $a$、$b$。
3. **蠕滑率**：由接触系中的相对平动、相对转动和滚动参考速度得到 $\xi_x$、$\xi_y$ 与 $\varphi$。
4. **Kalker 系数**：由半轴比 $a/b$ 与材料泊松比得到 $C_{11}$、$C_{22}$、$C_{23}$，并由此形成切向局部柔度。
5. **切向接触**：由 $N$、$a$、$b$、摩擦系数、蠕滑率与局部柔度求得接触系中的 $F_x$、$F_y$。

随后，组装层把 $(F_x,F_y,N)$ 转为轨型系中的力，放置到轮侧作用点 `P`，并形成成对扳手。各阶段的内部理论分别见[接触几何](CONTACT_GEOMETRY.md)、[法向接触力](NORMAL_CONTACT_FORCE.md)、[蠕滑率与接触坐标系](CREEPAGE_AND_CONTACT_FRAME.md)、[Kalker 线性蠕滑系数](KALKER_COEFFICIENTS.md)和 [FASTSIM 切向接触](TANGENTIAL_CONTACT_FASTSIM.md)。

## 2. 输入运动学与静止钢轨假设

接触几何只消费轮轨型面的相对位姿标量；力学组装还需要钢轨型面在轨型系 `T` 中的刚体放置，以及轮型面基准点的位置、姿态、速度和轮体角速度。路径速率与车轮自转率另行进入蠕滑参考速度。

在本模型中，钢轨是线路携带的几何约束，不是车辆多体树中具有状态与惯量的刚体。钢轨材料速度因而取零。接触处的相对速度完全由轮刚体运动在指定钢轨材料点处的速度给出。这一假设不表示钢轨动力学效应一般不存在，而是说明它们不属于当前接触元的自由度。

## 3. 两个不同的点：R 与 P

### 3.1 钢轨材料参考点 R

几何阶段给出 `R` 在钢轨型面自身坐标中的坐标 $\mathbf r_R$。由钢轨型面原点 $\mathbf o_{\mathrm{rail}}$ 与姿态 $R_{T\mathrm{rail}}$ 放置到轨型系：

$$
\mathbf x_R=\mathbf o_{\mathrm{rail}}+R_{T\mathrm{rail}}\mathbf r_R.
$$

若轮型面系 W 的基准点位置与速度为 $\mathbf o_W$、$\mathbf v_o$，轮体实际角速度为 $\boldsymbol\omega$，则轮材料在 `R` 处的速度以及相对角速度为

$$
\Delta\mathbf v
=\mathbf v_o+\boldsymbol\omega\times(\mathbf x_R-\mathbf o_W),
\qquad
\Delta\boldsymbol\omega=\boldsymbol\omega.
$$

`R` 的用途是使轮、轨材料速度在同一个空间位置上比较。它不是轮面上的力作用点，也不是压力形心。

### 3.2 轮侧力作用点 P

设接触斑在轮型面中的纵向坐标、横向站位和未变形局部半径为 $x_w$、$y_w$、$r$。本实现以下式定义力的取矩点：

$$
\mathbf r_P=
\begin{bmatrix}
x_w\\y_w\\r-\frac{1}{2}\delta_{\mathrm{eq}}
\end{bmatrix},
\qquad
\mathbf x_P=\mathbf o_W+R_{TW}\mathbf r_P,
$$

其中 $R_{TW}$ 是不含轮自旋的轮型面姿态。轮自旋不改变轴对称型面的几何放置，但包含在 $\boldsymbol\omega$ 中并影响材料速度。

`P` 首先是扳手取矩点的模型约定。在相同 $(x_w,y_w)$ 处，未变形回转面下半部的向下径向坐标应为

$$
z_{\mathrm{rev}}=\sqrt{\max(0,r^2-x_w^2)},
$$

而上式采用 $r-\delta_{\mathrm{eq}}/2$。因此 $x_w\ne0$ 时，`P` 一般不是精确回转面材料点，而是把局部型面沿纵向挤出后，再沿轮型面 $z$ 轴内移半个等效穿透所得的作用点；在 $x_w=0$ 时，其未变形部分才与最低径向位置重合。这一区分确定后续扳手换点的力臂，但不改变接触几何求得的 $x_w$。

`R` 与 `P` 来自不同构造：前者服务相对速度，后者服务力作用位置。把二者合并会改变将接触力搬移到车轮刚体原点时的力臂和力矩。

## 4. 接触系中的力

接触系 `C` 由斑的 `rail_slope_angle_radians` 构成，其到轨型系的旋转为 $R_{TC}$。同一个接触系用于法向接近速度、蠕滑率和最终力变换，避免三部分采用不同的表面方向。

法向 $+z_C$ 指入钢轨，因此钢轨作用于车轮的法向分量为 $-N$。接触系与轨型系中的力为

$$
\mathbf f_C=
\begin{bmatrix}F_x\\F_y\\-N\end{bmatrix},
\qquad
\mathbf f_T=R_{TC}\mathbf f_C.
$$

当前模型不输出接触斑绕自身法向的直接自旋力矩，所以在作用点 `P` 处取

$$
\mathbf m_P=\mathbf 0.
$$

车辆在其他参考点看到的接触力矩来自 $\mathbf f_T$ 的力臂，而不是由此处补造一个斑内自旋力矩。

## 5. 空间扳手代数

### 5.1 换取矩点

空间扳手记为 $\mathcal W=(\mathbf m,\mathbf f)$。若它原来关于点 $\mathbf a$ 取矩，改为关于点 $\mathbf b$ 取矩而表达系不变，则

$$
\mathbf f_b=\mathbf f_a,
\qquad
\mathbf m_b=\mathbf m_a+(\mathbf a-\mathbf b)\times\mathbf f_a.
$$

这对应 `TransportWrench`。叉积中的向量从新取矩点指向旧取矩点。

### 5.2 换表达基

若旋转 $R$ 把旧表达系中的分量写入新表达系，而取矩点不变，则

$$
\mathbf f'=R\mathbf f,
\qquad
\mathbf m'=R\mathbf m.
$$

这对应 `RotateWrench`。换点与换基是不同操作；当旋转还改变点坐标时，二者不能在没有明确几何关系的情况下互换顺序。

### 5.3 成对扳手

`MakePairedContactWrench` 从作用于车轮的一组力、力矩构造两半相互作用，二者关于同一个点 `P`、在同一个表达系中：

$$
\mathcal W_{\mathrm{rail\ on\ wheel}}=(\mathbf 0,\mathbf f_T),
\qquad
\mathcal W_{\mathrm{wheel\ on\ rail}}=(\mathbf 0,-\mathbf f_T).
$$

成对存放确保作用与反作用来自同一组数。若两半随后被搬移到不同的参考点，各自新增的力矩必须按各自力臂计算，不能只在一个点取负后假定所有参考点上的力矩仍逐项互消。

## 6. 组装算法

对一次给定轮轨状态，`WheelRailContactModel::Evaluate` 的物理计算顺序可概括为：

```text
patches = contact_geometry(relative_pose)
for each geometric patch:
    x_R = place_rail_reference_point(patch, rail_profile_frame)
    relative_motion = wheel_rigid_motion_at(x_R) - stationary_rail_motion
    frame = contact_frame(patch.rail_slope_angle)

    v_n = dot(frame.normal, relative_motion.velocity)
    normal = normal_contact(patch.geometry, v_n)
    if normal load is not positive: continue

    creepages = contact_creepages(relative_motion, frame, patch.local_radius)
    mu = friction_law(creepages)
    (F_x, F_y) = tangential_contact(normal, creepages, mu, a / b)

    f_C = (F_x, F_y, -N)
    f_T = frame.rotation * f_C
    x_P = place_wheel_side_application_point(patch, normal.equivalent_penetration)
    emit paired_wrench(point = x_P, force = f_T, direct_moment = 0)
```

法向载荷不为正时不计算切向力，因为没有法向约束就没有库仑摩擦容量。多斑情况下，每个斑独立走完以上链条；车辆力计划再把各斑扳手搬移到车轮或轮对的所需参考点并累加。

## 7. 模型范围与近似

该组装继承各阶段的适用条件与非光滑性：接触斑出现、消失或合并；法向力的单边接触；低速蠕滑参考速度约定；Kalker 折线与渐近拼接；FASTSIM 的黏滑切换和自适应条带。组装层不会消除这些特征。

当前模型还作出四项明确取舍：钢轨材料在接触元中静止；斑间不直接弹性耦合；斑内直接自旋力矩为零；`P` 采用上述型面挤出式作用点，而不是非零 $x_w$ 处的精确回转面材料点。这些量若要进入车辆动力学，需要扩展相应物理阶段，不能从现有成对力扳手中唯一恢复。

## 8. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 五阶段组装与 `R`、`P` 的形成 | `WheelRailContactModel::Evaluate`，见 [`wheel_rail_contact_model.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc) |
| 模型输入和逐斑结果 | `WheelRailContactInput`、`WheelRailContactPatchResult`，见 [`wheel_rail_contact_model.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/wheel_rail_contact_model.h) |
| 接触系与相对运动 | `ContactFrame`、`ContactRelativeMotion`，见 [`contact_creepage.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_creepage.h) |
| 空间扳手换点与换基 | `TransportWrench`、`RotateWrench`，见 [`contact_wrench.cc`](../../../libs/wheel_rail_contact/src/contact_wrench.cc) |
| 成对扳手 | `PairedContactWrench`、`MakePairedContactWrench`，见 [`contact_wrench.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/contact_wrench.h) |
