[中文](KALKER_COEFFICIENTS.md)

# Kalker linear creepage coefficients

This chapter describes the three Kalker coefficients used by ORVD's tangential contact model and how they are obtained from Poisson's ratio and the contact-ellipse semi-axis ratio. The finite table, interpolation, and slender-ellipse asymptotic expressions are implemented in [`kalker_coefficient_table.cc`](../../../libs/wheel_rail_contact/src/kalker_coefficient_table.cc); the coefficients then enter [FASTSIM tangential contact](TANGENTIAL_CONTACT_FASTSIM.en.md).

## 1. Notation and role

Let $a$ and $b$ be the contact ellipse's semi-axes in the rolling and lateral directions, respectively, and define

$$
\kappa=\frac{a}{b},\qquad \kappa>0,
$$

where the current numerical model has positive finite semi-axis ratio as its domain. This document locally uses $\kappa$ for the contact-ellipse semi-axis ratio; it is unrelated to the identically shaped symbol for planar curvature in the track-geometry documents. With Young's modulus $E$ and Poisson's ratio $\nu$, the shear modulus is

$$
G=\frac{E}{2(1+\nu)}.
$$

The implementation supplies three dimensionless coefficients: longitudinal $C_{11}$, lateral $C_{22}$, and lateral-spin coupling $C_{23}$. The tangential solver converts them into three flexibility scales:

$$
L_x=\frac{8a}{3C_{11}G},\qquad
L_y=\frac{8a}{3C_{22}G},\qquad
L_\varphi=\frac{\pi a\sqrt{\kappa}}{4C_{23}G}.
$$

$C_{11}$ and $C_{22}$ control longitudinal and lateral stress build-up due to translational creepage; $C_{23}$ controls the coupling between spin and stress build-up in both directions. This implementation does not use $C_{33}$: spin is handled by the within-patch strip march, and the result contains no direct spin moment about the patch normal.

In the continuous, fully adhering small-creepage limit, the corresponding linear resultants are

$$
F_x=-G\,a\,b\,C_{11}\xi_x,
\qquad
F_y=-G\,a\,b\,C_{22}\xi_y
-G\,(ab)^{3/2}C_{23}\varphi,
$$

The spin term in the longitudinal resultant integrates to zero by transverse symmetry of the patch, whereas the lateral resultant retains the $C_{23}$ spin coupling. Finite-resolution `FASTSIM` strip quadrature introduces its own quadrature factor; the no-spin discrete expression is given in the adjacent chapter.

## 2. Finite table and interpolation along two axes

### 2.1 Table structure

`kLongitudinal`, `kLateral`, and `kLateralSpin` in the source are three constant $3\times19$ tables. Their Poisson-ratio nodes are

$$
(\nu_0,\nu_1,\nu_2)=(0,\,0.25,\,0.5),
$$

and their semi-axis-ratio nodes are

$$
\begin{aligned}
\{\kappa_i\}_{i=0}^{18}={}&\{0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0,\\
&1/0.9,1/0.8,1/0.7,1/0.6,1/0.5,1/0.4,1/0.3,1/0.2,1/0.1\}.
\end{aligned}
$$

The node grid is paired under $\kappa\mapsto1/\kappa$, but this does not imply reciprocal symmetry of the three coefficients or of the piecewise-linear interpolants constructed in $\kappa$. The actual values of the three tables are given by the constant source arrays and form part of the current numerical model.

### 2.2 Collapse along Poisson's-ratio axis

Poisson's ratio remains fixed after model construction, so the implementation first removes that dimension by quadratic Lagrange interpolation through the three nodes. The weights are

$$
\ell_j(\nu)=\prod_{k\ne j}\frac{\nu-\nu_k}{\nu_j-\nu_k},
\qquad j,k\in\{0,1,2\},
$$

and the collapsed value at each semi-axis-ratio node is

$$
\bar C(\kappa_i;\nu)=\sum_{j=0}^{2}\ell_j(\nu)C(\kappa_i;\nu_j),
\qquad i=0,\ldots,18.
$$

The same three weights are shared by all columns of $C_{11}$, $C_{22}$, and $C_{23}$. At a Poisson-ratio node the weights reduce to a Kronecker delta and select the original row exactly. The implementation uses this interpolation only over the material range $0\le\nu\le0.5$.

### 2.3 Interpolation along the semi-axis-ratio axis

For $\kappa_i\le\kappa<\kappa_{i+1}$, lookup is piecewise linear in $\kappa$ itself:

$$
\bar C(\kappa;\nu)=\bar C(\kappa_i;\nu)
+\left[\bar C(\kappa_{i+1};\nu)-\bar C(\kappa_i;\nu)\right]
\frac{\kappa-\kappa_i}{\kappa_{i+1}-\kappa_i}.
$$

The independent variable is neither $\ln\kappa$ nor $\min(\kappa,1/\kappa)$. The reciprocal pairing of the grid therefore cannot be used to replace a query at $\kappa>1$ by one at $1/\kappa$.

## 3. Slender-ellipse asymptotic expressions

The finite table covers the closed interval $0.1\le\kappa\le10$. For positive finite semi-axis ratios outside the table, the current wheel-rail contact model uses slender-ellipse asymptotic expressions. Define

$$
\sigma=\min\left(\kappa,\frac{1}{\kappa}\right),
\qquad
\Lambda=\ln\frac{16}{\sigma^2}.
$$

### 3.1 $0<\kappa<0.1$

When the contact ellipse is short in the rolling direction, the source uses

$$
\begin{aligned}
C_{11}&=\frac{\pi^2}{4(1-\nu)},\\
C_{22}&=\frac{\pi^2}{4},\\
C_{23}&=\frac{\pi\sqrt{\sigma}}{3(1-\nu)}
\left[1+\nu\left(\frac{\Lambda}{2}+\ln4-5\right)\right].
\end{aligned}
$$

The first two coefficients lose their shape dependence in this asymptotic expression, while the coupling coefficient retains a dependence on slenderness.

### 3.2 $\kappa>10$

When the contact ellipse is long in the rolling direction, let

$$
S=\Lambda-2\nu,
\qquad
W=(1-\nu)\Lambda+2\nu.
$$

The source then uses

$$
\begin{aligned}
C_{11}&=\frac{2\pi}{S\sigma}\left(1+\frac{3-\ln4}{S}\right),\\
C_{22}&=\frac{2\pi}{\sigma W}\left(1+\frac{(1-\nu)(3-\ln4)}{W}\right),\\
C_{23}&=\frac{2\pi}{3\sigma^{3/2}\left[(1-\nu)\Lambda-2+4\nu\right]}.
\end{aligned}
$$

The two asymptotic regions are not simple mirror images obtained by swapping the longitudinal and lateral directions.

### 3.3 Junctions and applicability

The finite table and asymptotic expressions switch directly at $\kappa=0.1$ and $\kappa=10$, with no blending region. The current coefficient function is therefore generally only piecewise smooth at interior table nodes and may also have value jumps at the two table boundaries. Such a jump belongs to the algorithmic junction between a finite table and an asymptotic approximation; it must not be interpreted as a physical discontinuity of real contact mechanics at that ellipse shape.

The asymptotic expressions cover slender ellipses beyond the finite table. As $\sigma\to0$, their logarithmic and power terms display the singular scales of the slender limit; they should not be extrapolated into a model of degenerate line contact or zero-area contact.

## 4. Numerical algorithm

At construction, the implementation first evaluates the three $\ell_j(\nu)$ and then collapses each column into three one-dimensional tables. Each patch computes $\kappa=a/b$ once. Lookup locates adjacent columns and shares a single interpolation fraction inside the table; outside it, the algorithm evaluates the expressions in Section 3.

The two endpoints $\kappa=0.1$ and $\kappa=10$ return the first and last node values directly. For a strict interior point $0.1<\kappa<10$, the algorithm is

```text
high = first index with kappa_node[high] > kappa
low = high - 1
t = (kappa - kappa_node[low]) / (kappa_node[high] - kappa_node[low])
C = C_collapsed[low] + (C_collapsed[high] - C_collapsed[low]) * t
```

All three coefficients share `high`, `low`, and $t$. This structure preserves the original node values and makes the derivative with respect to $\kappa$ discontinuous at general nodes. The downstream tangential force may inherit those corners and the jumps at table-asymptotic junctions; degenerate states such as zero creepage can mask coefficient changes completely.

## 5. Source mapping

| Theoretical object | Primary implementation |
|---|---|
| Coefficient triple and table object | `KalkerCoefficients`, `KalkerCoefficientTable`; see [`kalker_coefficient_table.h`](../../../libs/wheel_rail_contact/include/orvd/wheel_rail_contact/kalker_coefficient_table.h) |
| Finite tables and nodes | `kLongitudinal`, `kLateral`, `kLateralSpin`, `kSemiAxisRatioNodes`; see [`kalker_coefficient_table.cc`](../../../libs/wheel_rail_contact/src/kalker_coefficient_table.cc) |
| Poisson-ratio collapse | `PoissonInterpolationWeights`, `CollapsePoissonAxis` |
| Semi-axis-ratio lookup | `KalkerCoefficientTable::At` |
| Slender-ellipse expressions | `AsymptoticCoefficients` |
| Flexibility consumer | `TangentialContactSolver::Solve`; see [`tangential_contact_force.cc`](../../../libs/wheel_rail_contact/src/tangential_contact_force.cc) |
