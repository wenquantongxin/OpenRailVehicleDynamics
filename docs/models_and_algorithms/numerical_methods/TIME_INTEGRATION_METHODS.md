[English](TIME_INTEGRATION_METHODS.en.md)

# BDF、Radau5、Newmark 与 Zhai 时间积分方法

本文说明 BDF、三阶段五阶 Radau IIA、Newmark 与 Zhai 简单显式法如何把连续动力学方程推进为离散状态，并讨论它们的误差、稳定性以及与 ORVD 状态结构的相容关系。ORVD 当前以 CVODE BDF2 为默认后端，也保留源码树内的 BDF5 与 Radau5 实现；Newmark 和 Zhai 尚未实现，本文将二者标为 **仅理论**。

## 1. 方程形态与共同记号

### 1.1 一阶状态方程

BDF 和 Radau5 直接作用于一阶初值问题

$$
\dot y=f(t,y),
\qquad
y(t_0)=y_0,
\qquad
y\in\mathbb R^N.
$$

ORVD 的连续状态写成

$$
y=
\begin{bmatrix}
q\\v\\z
\end{bmatrix},
\qquad
\dot y=
\begin{bmatrix}
N(q)v\\a(t,q,v,z)\\g(t,q,v,z)
\end{bmatrix}.
$$

其中 $q$ 是广义位置，$v$ 是广义速度，$z$ 是 Maxwell 串联弹簧黏性阻尼器等力元的一阶内变量。自由体以四元数表示姿态时，$q$ 与 $v$ 不等维，位形运动学为 $\dot q=N(q)v$，不能把完整状态简化成 $\dot q=v$。状态与导数的代码映射见 [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h)；例如 Maxwell 力状态满足

$$
\dot F=K v_{\mathrm{rel}}-\frac{K}{C}F.
$$

### 1.2 二阶机械方程

经典 Newmark 和 Zhai 方法通常从二阶机械方程出发：

$$
M(q)a+C(q,v)v+f_{\mathrm{int}}(q,v,z)=p(t),
\qquad
\dot q=v.
$$

这一形式天然适合位移、速度和加速度同维的欧氏坐标。对一般多体系统，位形必须通过切空间增量与 retraction 更新；一阶内变量 $z$ 也需要单独的离散方程。若不作这些扩展，把某个一阶积分公式直接作用于 $[q;v;z]$，得到的是另一种一阶状态方法，而不是经典 Newmark 或 Zhai 方法。

### 1.3 误差尺度

不同状态分量具有不同量纲。自适应方法可由相对容差和逐分量绝对容差形成权重

$$
w_i=\frac{1}{\operatorname{rtol}|y_i|+\operatorname{atol}_i},
\qquad
\lVert e\rVert_{\mathrm{WRMS}}
=\sqrt{\frac{1}{N}\sum_{i=1}^{N}(w_i e_i)^2}.
$$

局部误差估计在这种加权范数下决定步长调整。不同方法使用的误差估计器与控制器并不相同，因此相同的 `rtol` 和 `atol` 不代表相同的全局误差。

## 2. BDF：隐式线性多步法

[CVODE 的数学说明](https://sundials.readthedocs.io/en/latest/cvode/Mathematics_link.html)给出了其变步长、变阶 BDF 形式。$k$ 阶 BDF 用当前未知状态和若干历史状态逼近新端点导数；固定等步长时可写成

$$
\sum_{j=0}^{k}\alpha_j y_{n+1-j}
=h f(t_{n+1},y_{n+1}).
$$

### 2.1 固定等步长公式

BDF1–BDF5 的等步长公式为

$$
\begin{aligned}
\text{BDF1:}\quad
&y_{n+1}-y_n=h f_{n+1},\\
\text{BDF2:}\quad
&\frac{3}{2}y_{n+1}-2y_n+\frac{1}{2}y_{n-1}=h f_{n+1},\\
\text{BDF3:}\quad
&\frac{11}{6}y_{n+1}-3y_n+\frac{3}{2}y_{n-1}-\frac{1}{3}y_{n-2}=h f_{n+1},\\
\text{BDF4:}\quad
&\frac{25}{12}y_{n+1}-4y_n+3y_{n-1}-\frac{4}{3}y_{n-2}+\frac{1}{4}y_{n-3}=h f_{n+1},\\
\text{BDF5:}\quad
&\frac{137}{60}y_{n+1}-5y_n+5y_{n-1}-\frac{10}{3}y_{n-2}+\frac{5}{4}y_{n-3}-\frac{1}{5}y_{n-4}=h f_{n+1}.
\end{aligned}
$$

这些系数只适用于固定等步长。变步长 BDF 必须根据最近的步长历史重新形成差分系数；“最大二阶”或“最大五阶”只限制可选阶数，不表示每一步都采用该阶数。

### 2.2 隐式端点与步长控制

把历史项收集后，新端点可表示为非线性残差

$$
R(y_{n+1})=y_{n+1}-\gamma f(t_{n+1},y_{n+1})-a_n=0,
$$

其中 $a_n$ 由已接受的历史决定，$\gamma$ 由当前步长和 BDF 系数决定。Newton 或 modified Newton 迭代求解

$$
\left(I-\gamma J\right)\delta=-R,
\qquad
J=\frac{\partial f}{\partial y},
\qquad
y^{(m+1)}=y^{(m)}+\delta.
$$

一个自适应 BDF 步依次完成历史外推、非线性求解、局部误差估计以及步长和阶数选择。只有接受新端点后，方法历史才滚动到下一步；状态方程或外部保持量改变后，旧历史不再表示同一初值问题，必须从当前端点重建。

### 2.3 精度与稳定性

- 对足够光滑的问题，$k$ 阶 BDF 的局部截断误差为 $O(h^{k+1})$，全局误差为 $O(h^k)$。
- BDF1 和 BDF2 是 A-stable；BDF3–BDF5 不再覆盖整个左半平面，但仍可用于刚性问题。
- BDF 是多步法，需要启动历史；接触切换或力律折点会削弱高阶收敛。
- 稠密输出来自最近内部步的历史多项式，其定义域不能超出该内部步。

### 2.4 ORVD 中的实现

[`cvode_continuous_state_advancer.cc`](../../../libs/integrators/src/cvode_continuous_state_advancer.cc) 以 `CV_BDF` 构造 CVODE 后端，并分别把最大阶数固定为 2 或 5。公共系统推进器选择 BDF2；BDF5 是源码树内部的具名实现。[`system_rhs_bridge.cc`](../../../libs/integrators/src/system_rhs_bridge.cc) 把完整 $[q;v;z]$ 状态映射到多体动力学右端，[`continuous_state_advancer.h`](../../../libs/integrators/include/orvd/integrators/continuous_state_advancer.h) 则承接接受内步和最近一步稠密输出所需的共同数学结果。

## 3. Radau5：三阶段五阶 Radau IIA

本文所称 Radau5 是[Hairer 的 RADAU5](https://www.unige.ch/~hairer/software.html)所采用的三阶段五阶 Radau IIA 配点法。令

$$
c_1=\frac{4-\sqrt6}{10},
\qquad
c_2=\frac{4+\sqrt6}{10},
\qquad
c_3=1.
$$

### 3.1 Butcher 表与阶段方程

方法的 Butcher 表为

$$
\begin{array}{c|ccc}
c_1 & \frac{88-7\sqrt6}{360}
    & \frac{296-169\sqrt6}{1800}
    & \frac{-2+3\sqrt6}{225}\\
c_2 & \frac{296+169\sqrt6}{1800}
    & \frac{88+7\sqrt6}{360}
    & \frac{-2-3\sqrt6}{225}\\
1   & \frac{16-\sqrt6}{36}
    & \frac{16+\sqrt6}{36}
    & \frac{1}{9}\\
\hline
    & \frac{16-\sqrt6}{36}
    & \frac{16+\sqrt6}{36}
    & \frac{1}{9}
\end{array}.
$$

三个阶段同时满足

$$
Y_i=y_n+h\sum_{j=1}^{3}a_{ij}f(t_n+c_jh,Y_j),
\qquad i=1,2,3.
$$

权重等于 $A$ 的最后一行，因此方法 stiffly accurate：

$$
y_{n+1}=y_n+h\sum_{i=1}^{3}b_i f(t_n+c_i h,Y_i)=Y_3.
$$

### 3.2 耦合 Newton 求解

由于 $A$ 不是下三角矩阵，三个阶段构成耦合非线性系统。简化 Newton 的原始线性化为

$$
\left(I_3\otimes I_N-hA\otimes J\right)\Delta=-R.
$$

直接分解该 $3N\times3N$ 系统并非必要。利用 $A^{-1}$ 的一个实特征值和一对共轭复特征值，可以把它变换为一个实 $N\times N$ 系统与一个复 $N\times N$ 系统。阶段状态仍分别求值右端，Jacobian 与线性分解则可在若干 Newton 迭代或相邻步骤间复用。

### 3.3 误差控制与稠密输出

Radau5 主公式为五阶，stage order 为 3。完整求解器还必须规定阶段初值、局部误差估计、步长控制以及配点多项式稠密输出；这些算法共同决定实际的 Radau5 推进，而不只是一张 Butcher 表。高阶结论要求解足够光滑，接触状态切换处不能假定仍观察到五阶收敛。

### 3.4 稳定性

对线性测试方程 $y'=\lambda y$，令 $\zeta=h\lambda$，稳定函数为

$$
\mathcal R(\zeta)=
\frac{1+\frac{2}{5}\zeta+\frac{1}{20}\zeta^2}
{1-\frac{3}{5}\zeta+\frac{3}{20}\zeta^2-\frac{1}{60}\zeta^3}.
$$

该方法是 A-stable，并且当左半平面的 $|\zeta|\to\infty$ 时有 $\mathcal R(\zeta)\to0$，所以也是 L-stable。L-stability 使强衰减刚性模态在大步长极限下被压低，但不消除非光滑点带来的阶数退化。

### 3.5 ORVD 中的实现

[`radau5_core.cc`](../../../external/radau5/src/radau5_core.cc) 实现正向一阶常微分方程、$M=I$、稠密 Jacobian、三阶段五阶 Radau IIA、自适应误差控制和最近成功步的配点稠密输出。它用一个实线性系统和一个复线性系统完成简化 Newton 迭代。[`radau5_continuous_state_advancer.cc`](../../../libs/integrators/src/radau5_continuous_state_advancer.cc) 把该核心接到与 BDF 相同的完整一阶状态右端。Radau5 已有源码树实现，但不是公共默认后端。

## 4. Newmark：二阶机械系统的一步法族（仅理论）

[Newmark 方法](https://doi.org/10.1061/JMCEA3.0000098)以端点加速度参数化位移和速度更新。给定 $u_n$、$v_n$、$a_n$，其基本公式为

$$
u_{n+1}=u_n+h v_n+h^2\left[\left(\frac12-\beta\right)a_n+\beta a_{n+1}\right],
$$

$$
v_{n+1}=v_n+h\left[(1-\gamma)a_n+\gamma a_{n+1}\right].
$$

### 4.1 隐式平衡

隐式 Newmark 还要求新端点满足动力学平衡

$$
R_{n+1}=p_{n+1}-M a_{n+1}-C v_{n+1}-f_{\mathrm{int}}(u_{n+1},v_{n+1},z_{n+1})=0.
$$

初始加速度由初始动力学平衡确定，例如

$$
M a_0=p_0-Cv_0-f_{\mathrm{int}}(u_0,v_0,z_0).
$$

### 4.2 线性系统的有效刚度

对常量 $M,C,K$、$f_{\mathrm{int}}=Ku$ 且 $\beta>0$ 的系统，定义

$$
\kappa_0=\frac{1}{\beta h^2},
\qquad
\kappa_1=\frac{\gamma}{\beta h},
$$

则以 $u_{n+1}$ 为未知量的有效刚度为

$$
K_{\mathrm{eff}}=K+\kappa_1C+\kappa_0M.
$$

非线性系统以端点平衡残差迭代；若 $M$、$C$ 或载荷依赖状态，一致切线还要包含这些项对 $u_{n+1}$ 的导数。$\beta=0$ 属于显式 Newmark 分支，不能使用含 $1/\beta$ 的有效刚度公式。

### 4.3 参数、精度与稳定性

- $\gamma=1/2$ 时，标准 Newmark 家族为二阶；$\gamma>1/2$ 引入算法耗散并通常降为一阶。
- 对线性无阻尼系统，$2\beta\ge\gamma\ge1/2$ 是常用的无条件稳定条件。
- $\beta=1/4,\gamma=1/2$ 是平均加速度法；它对线性系统无条件稳定且没有算法高频耗散。
- $\beta=1/6,\gamma=1/2$ 是线性加速度法，其稳定性受步长限制。

### 4.4 与 ORVD 状态的关系

经典公式假设 $u$、$v$、$a$ 同维且 $\dot u=v$。若将 Newmark 用于 ORVD 完整车辆，必须用切空间速度和加速度形成位形增量，再通过 retraction 更新含四元数和 Ball-RPY 的 $q$；同时还要为 $z$ 定义与端点平衡耦合的离散方程。因此，欧氏线性结构的有效刚度公式不能直接充当完整 ORVD 多体模型的离散方程。

## 5. Zhai 简单显式二步法（仅理论）

Zhai 在 1996 年提出的简单显式法使用当前和前一端点的加速度：

$$
u_{n+1}=u_n+h v_n+\left(\frac12+\psi\right)h^2a_n-\psi h^2a_{n-1},
$$

$$
v_{n+1}=v_n+(1+\phi)h a_n-\phi h a_{n-1}.
$$

随后由动力学方程显式求新加速度：

$$
a_{n+1}=M^{-1}\left[p_{n+1}-C v_{n+1}-f_{\mathrm{int}}(u_{n+1},v_{n+1})\right].
$$

### 5.1 启动与历史

二步法在初始时没有 $a_{-1}$。常用自启动取 $\phi=\psi=0$：

$$
u_1=u_0+h v_0+\frac12h^2a_0,
\qquad
v_1=v_0+h a_0,
$$

其中 $a_0$ 由初始动力学平衡给出。正常步骤常取 $\phi=\psi=1/2$，并在形成 $u_{n+1}$、$v_{n+1}$ 后计算 $a_{n+1}$。只有新端点成为方法历史后，$a_n$ 与 $a_{n-1}$ 才向前滚动；改变步长或状态方程时，等步长二步系数不能继续沿用。

### 5.2 精度与稳定性

- $\phi=\psi=1/2$ 的常用形式为二阶，在线性无阻尼分析中没有数值耗散，但存在相位误差。
- 对无阻尼线性振子，该参数组合的稳定条件为 $h\omega<2$，等价于 $h<T_{\min}/\pi$。
- 最高可解析频率会限制显式步长；轮轨接触刚度、悬挂刚度和一阶内变量都可能贡献高频时间尺度。
- 原始公式是固定等步长方法；变步长、缩短末步和稠密输出都需要另行定义相应的数学公式。

### 5.3 与 ORVD 状态的关系

对 ORVD 完整车辆，Zhai 方法同样需要在切空间中构造位形增量并 retraction 到新 $q$，还需要为 Maxwell 等内变量 $z$ 指定离散方法。把 AB2 直接作用于完整 $[q;v;z]$ 会得到普通一阶多步法，不能称为 Zhai 简单显式法。

## 6. 方法比较

| 方法 | 基本方程 | 主阶数 | 隐式性 | 稳定性要点 | 历史结构 | ORVD 实现状态 |
|---|---|---:|---|---|---|---|
| CVODE BDF | 完整一阶 $\dot y=f(t,y)$ | 1–5 | 隐式 | BDF1–2 为 A-stable | 多步历史 | BDF2 默认；BDF5 源码树内实现 |
| Radau5 | 完整一阶 $\dot y=f(t,y)$ | 5 | 三阶段全隐式 | A-stable、L-stable | 单步阶段与线性化历史 | 源码树内实现，非默认 |
| Newmark | 二阶机械平衡 | 通常 2 | 常用形式隐式 | 取决于 $\beta,\gamma$ | 单步端点量 | **仅理论** |
| Zhai 简单显式法 | 二阶机械加速度 | 2 | 显式 | 受最高频率限制 | 两步加速度历史 | **仅理论** |

BDF 和 Radau5 可直接消费 ORVD 的完整一阶右端。Newmark 与 Zhai 的原始公式利用二阶机械结构；把它们用于含流形位形和一阶内变量的车辆模型时，必须先明确扩展后的离散方程，否则方法名称与实际算法不再等价。
