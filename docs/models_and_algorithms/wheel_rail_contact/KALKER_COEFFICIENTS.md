[English](KALKER_COEFFICIENTS.en.md)

# Kalker 线性蠕滑系数

本篇说明 ORVD 切向接触模型使用的三个 Kalker 系数，以及它们如何由泊松比与接触椭圆半轴比得到。有限表、插值与细长椭圆渐近式实现在 [`kalker_coefficient_table.cc`](../../../libs/wheel_rail_contact/src/kalker_coefficient_table.cc)；这些系数随后进入 [FASTSIM 切向接触](TANGENTIAL_CONTACT_FASTSIM.md)。

## 1. 记号与作用

接触椭圆沿滚动方向和横向的半轴分别为 $a$、$b$，定义

$$
\kappa=\frac{a}{b},\qquad \kappa>0,
$$

其中当前数值模型以正的有限半轴比为定义域。本篇局部用 $\kappa$ 表示接触椭圆半轴比；它与线路几何文档中表示平面曲率的同形符号无关。材料参数为杨氏模量 $E$ 与泊松比 $\nu$，剪切模量为

$$
G=\frac{E}{2(1+\nu)}.
$$

实现提供三个无量纲系数：纵向系数 $C_{11}$、横向系数 $C_{22}$ 和横向—自旋耦合系数 $C_{23}$。切向求解器把它们换成三个柔度尺度：

$$
L_x=\frac{8a}{3C_{11}G},\qquad
L_y=\frac{8a}{3C_{22}G},\qquad
L_\varphi=\frac{\pi a\sqrt{\kappa}}{4C_{23}G}.
$$

$C_{11}$ 和 $C_{22}$ 分别控制平动蠕滑引起的纵向、横向应力积累；$C_{23}$ 控制自旋与两方向应力积累之间的耦合。此实现不使用 $C_{33}$：自旋效应通过斑内条带推进处理，并且结果不包含接触斑绕法向的直接自旋力矩。

在连续、全黏着的小蠕滑极限下，相应线性合力为

$$
F_x=-G\,a\,b\,C_{11}\xi_x,
\qquad
F_y=-G\,a\,b\,C_{22}\xi_y
-G\,(ab)^{3/2}C_{23}\varphi,
$$

纵向合力中的自旋项因接触斑关于横轴对称而积分为零；横向合力保留 $C_{23}$ 控制的自旋耦合。`FASTSIM` 的离散条带求积会在有限分辨率下引入其自身的求积因子；无自旋离散表达见相邻文档。

## 2. 有限表与双轴插值

### 2.1 表格结构

源码中的 `kLongitudinal`、`kLateral` 与 `kLateralSpin` 是三张 $3\times19$ 的常量表。泊松比节点为

$$
(\nu_0,\nu_1,\nu_2)=(0,\,0.25,\,0.5),
$$

半轴比节点为

$$
\begin{aligned}
\{\kappa_i\}_{i=0}^{18}={}&\{0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,\\
&1/0.9,1/0.8,1/0.7,1/0.6,1/0.5,1/0.4,1/0.3,1/0.2,1/0.1\}.
\end{aligned}
$$

节点网格在 $\kappa\mapsto1/\kappa$ 下成对，但这不意味着三个系数或在 $\kappa$ 上构造的折线插值也具有倒数对称性。三张表的具体数值由源码常量数组给出，是当前数值模型的一部分。

### 2.2 泊松比方向的坍缩

材料的泊松比在模型构造后保持不变，因此实现先用三个节点上的二次 Lagrange 插值消去这一维。权重为

$$
\ell_j(\nu)=\prod_{k\ne j}\frac{\nu-\nu_k}{\nu_j-\nu_k},
\qquad j,k\in\{0,1,2\},
$$

每个半轴比节点上的坍缩值为

$$
\bar C(\kappa_i;\nu)=\sum_{j=0}^{2}\ell_j(\nu)C(\kappa_i;\nu_j),
\qquad i=0,\ldots,18.
$$

同一组三个权重用于 $C_{11}$、$C_{22}$ 与 $C_{23}$ 的全部列。在泊松比节点上，权重退化为 Kronecker delta，原表行被精确选中。实现只在 $0\le\nu\le0.5$ 的材料范围内使用这项插值。

### 2.3 半轴比方向的插值

当 $\kappa_i\le\kappa<\kappa_{i+1}$ 时，查询沿 $\kappa$ 本身作分段线性插值：

$$
\bar C(\kappa;\nu)=\bar C(\kappa_i;\nu)
+\left[\bar C(\kappa_{i+1};\nu)-\bar C(\kappa_i;\nu)\right]
\frac{\kappa-\kappa_i}{\kappa_{i+1}-\kappa_i}.
$$

自变量不是 $\ln\kappa$，也不是 $\min(\kappa,1/\kappa)$。因此网格的倒数配对不能用来把 $\kappa>1$ 的查询改写成 $1/\kappa$ 的查询。

## 3. 细长椭圆渐近式

有限表覆盖闭区间 $0.1\le\kappa\le10$。当前轮轨接触模型对表外、仍为正有限数的半轴比采用细长椭圆渐近式。先定义

$$
\sigma=\min\left(\kappa,\frac{1}{\kappa}\right),
\qquad
\Lambda=\ln\frac{16}{\sigma^2}.
$$

### 3.1 $0<\kappa<0.1$

当接触椭圆沿滚动方向较短时，源码使用

$$
\begin{aligned}
C_{11}&=\frac{\pi^2}{4(1-\nu)},\\
C_{22}&=\frac{\pi^2}{4},\\
C_{23}&=\frac{\pi\sqrt{\sigma}}{3(1-\nu)}
\left[1+\nu\left(\frac{\Lambda}{2}+\ln4-5\right)\right].
\end{aligned}
$$

前两个系数在这一渐近表达中不再依赖形状；耦合系数仍依赖细长度。

### 3.2 $\kappa>10$

当接触椭圆沿滚动方向较长时，令

$$
S=\Lambda-2\nu,
\qquad
W=(1-\nu)\Lambda+2\nu.
$$

则源码使用

$$
\begin{aligned}
C_{11}&=\frac{2\pi}{S\sigma}\left(1+\frac{3-\ln4}{S}\right),\\
C_{22}&=\frac{2\pi}{\sigma W}\left(1+\frac{(1-\nu)(3-\ln4)}{W}\right),\\
C_{23}&=\frac{2\pi}{3\sigma^{3/2}\left[(1-\nu)\Lambda-2+4\nu\right]}.
\end{aligned}
$$

两个渐近区域不是彼此简单交换纵横方向后的镜像。

### 3.3 拼接与适用性

有限表与渐近表达在 $\kappa=0.1$ 和 $\kappa=10$ 处硬切换，没有混合区。因此当前系数函数在表内节点处通常只有分段光滑性，在两个表域端点还可能有函数值跳跃。这个跳跃属于有限表与渐近近似的算法拼接，不应解释成真实接触力学在该椭圆形状处发生物理突变。

渐近式用于有限表无法描述的细长椭圆。随着 $\sigma\to0$，对数项和幂次项体现了细长极限的奇异尺度；该表达不应外推为退化线接触或零面积接触的模型。

## 4. 数值算法

构造阶段先计算三个 $\ell_j(\nu)$，再逐列形成三张一维坍缩表。每次斑求解只计算一次 $\kappa=a/b$：表域内用有序节点定位相邻列并共享同一个插值分数，表域外进入第 3 节的渐近表达。

两个端点 $\kappa=0.1$ 与 $\kappa=10$ 直接返回首、末节点值。对严格内点 $0.1<\kappa<10$，算法可写为

```text
high = first index with kappa_node[high] > kappa
low = high - 1
t = (kappa - kappa_node[low]) / (kappa_node[high] - kappa_node[low])
C = C_collapsed[low] + (C_collapsed[high] - C_collapsed[low]) * t
```

三个系数共享 `high`、`low` 与 $t$。该结构保留了原始节点值，也使对 $\kappa$ 的导数在一般节点处不连续；下游切向力可能继承这些折点以及表—渐近拼接处的跳跃。零蠕滑等退化状态可以把系数变化完全遮蔽。

## 5. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| 三个系数与表对象 | `KalkerCoefficients`、`KalkerCoefficientTable`，见 [`kalker_coefficient_table.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/kalker_coefficient_table.h) |
| 有限表与节点 | `kLongitudinal`、`kLateral`、`kLateralSpin`、`kSemiAxisRatioNodes`，见 [`kalker_coefficient_table.cc`](../../../libs/wheel_rail_contact/src/kalker_coefficient_table.cc) |
| 泊松比坍缩 | `PoissonInterpolationWeights`、`CollapsePoissonAxis` |
| 半轴比查询 | `KalkerCoefficientTable::At` |
| 细长椭圆表达 | `AsymptoticCoefficients` |
| 柔度消费者 | `TangentialContactSolver::Solve`，见 [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
