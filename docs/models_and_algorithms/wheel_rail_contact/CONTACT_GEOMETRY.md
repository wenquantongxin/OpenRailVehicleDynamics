[English](CONTACT_GEOMETRY.en.md)

# 接触几何

本篇说明 ORVD 轮轨接触模型的几何部分：给定位姿归约产生的四个位姿标量，`ContactGeometrySolver::Solve` 返回若干接触斑。每个斑包含位置、穿透、横向宽度、三维纵向长度、局部角度、轮轨曲率与钢轨材料参考点；力、材料与载荷由后续模型处理。核心实现位于 [`contact_geometry.cc`](../../../libs/wheel_rail_contact/src/contact_geometry.cc)。

## 1. 范围

模型依次回答三个问题：

1. 回转车轮在给定摇头下向钢轨横截面呈现什么可见轮廓。
2. 投影轮廓与轨面在哪里互穿，以及互穿区如何分成接触岛。
3. 每个岛的几何尺度、形心、法向、曲率和纵向弦长是什么。

模型不计算压力分布、法向力或切向力。不平顺已在上游位姿归约中进入轮轨相对放置。输入位姿见[轮轨位姿归约与不平顺输入](WHEEL_RAIL_POSE_REDUCTION.md)，法向载荷见[法向接触力](NORMAL_CONTACT_FORCE.md)，接触系与蠕滑率见[蠕滑率与接触系](CREEPAGE_AND_CONTACT_FRAME.md)。

## 2. 记号

型面横向坐标为 $Y$，竖向向下为正。为避免与线路姿态混淆，本篇把位姿滚转记为 $\varphi$、位姿摇头记为 $\beta$。

| 记号 | 含义 | 实现量 |
|---|---|---|
| $\eta$ | 侧解析后的轮型面横向站位 | wheel_station_meters |
| $h(\eta)$ | 自然三次样条轮面高度 | wheel_spline_ |
| $\hat h(\eta)$ | 由节点值与自然样条节点斜率构造的 Hermite 轮面 | wheel_surface_ |
| $R_0$ | 标称滚动半径 | nominal_rolling_radius_meters |
| $r(\eta)$ | 局部轮径 $R_0+h(\eta)$ | local_radius |
| $\tau$ | 从车轴正下方起算的周向角 | angle |
| $\tau_s(\eta)$ | 可见轮廓的周向角 | silhouette_angle |
| $u_y,u_z^\uparrow$ | 位姿横向偏移与向上为正的竖向抬升 | ContactPoseScalars |
| $d_y,d_z$ | 轮基准相对轨基准的横、竖平移 | lateral_offset, vertical_offset |
| $z_r(Y)$ | 钢轨横截面高度 | rail_surface_ |
| $H(Y)$ | 投影轮廓的单值上包络 | envelope cubic segments |
| $\eta(Y)$ | 包络横坐标到轮站位的分段线性映射 | envelope_station_ |
| $g(Y)$ | 轮包络减轨面的竖向互穿 | union_gap_ |
| $\epsilon$ | 接触判定间隙阈值 | contact_gap_epsilon_meters |
| $\delta_m$ | 接触岛合并的谷深阈值 | island_merge_gap_tolerance_meters |
| $\delta_v,\delta_n$ | 竖向与法向穿透 | patch penetration fields |
| $Y_c,Z_c$ | 重叠区域的面积形心 | centroid fields |
| $\alpha_r,\alpha_s,\gamma$ | 轨面角、含轨底坡的接触系角、公法线角 | patch angle fields |
| $\varsigma$ | 侧号，右侧 $+1$、左侧 $-1$ | `WheelSide` 的数学编码 |
| $L$ | 接触斑的三维纵向最长弦 | longitudinal_length_meters |

## 3. 模型

### 3.1 轮面插值、位姿与投影

轮型面点先按左右侧解析并按横向站位升序排列；可选的等弧长重扫只改变后续使用的节点集。由这组节点构造两套相关但职责不同的轮面表示：

- `wheel_spline_` 是自然三次样条，用于轮廓高度、轮廓斜率以及三维纵向解析。
- `wheel_surface_` 通过 `FromNodalSlopes` 取得同一节点值与自然样条在节点处的斜率，提供共享的表面值、导数和曲率接口。`FromNodalSlopes` 直接使用调用者给出的斜率，不进行保形限斜。

两者在节点区间之外都保持端点值为常数，并在严格端外给出零的一阶、二阶导数；共存原因不是外推规则不同，而是消费者和接口不同。若自然样条没有把近似等距点列替换为理想网格、两者实际使用完全相同的节点横坐标与区间长度，则区间内由同一节点值与斜率确定的 Hermite 三次段相同。自然样条对“近似等距”的节点可启用理想网格 $x_0+ih$，而 `wheel_surface_` 接收原始节点，因此在这种路径下二者不保证逐点完全相同。轨面 $z_r(Y)$ 也由自然样条节点斜率经同一 `FromNodalSlopes` 路径构造，并非经过保形限斜的曲线。

在轮型面基准系中，回转面点为

$$
\mathbf p_w(\eta,\tau)=
\begin{bmatrix}
r(\eta)\sin\tau\\
\eta\\
r(\eta)\cos\tau-R_0
\end{bmatrix},
\qquad
r(\eta)=R_0+h(\eta).
$$

位姿偏移先由配对编码还原为平面平移：

$$
d_y=u_y\cos\varphi+u_z^\uparrow\sin\varphi,
\qquad
d_z=u_y\sin\varphi-u_z^\uparrow\cos\varphi.
$$

该二阶变换是行列式为 $-1$ 的自逆反射。令 $x_w=r\sin\tau$、$z_w=r\cos\tau-R_0$，经摇头和滚转投影到轨型横截面：

$$
\widetilde y=\sin\beta\,x_w+\cos\beta\,\eta,
$$

$$
Y=\cos\varphi\,\widetilde y-\sin\varphi\,z_w+d_y,
\qquad
Z=\sin\varphi\,\widetilde y+\cos\varphi\,z_w+d_z.
$$

### 3.2 可见轮廓

令 $\chi(\eta)=-\tan\beta\,h'(\eta)$。当 $|\chi|\leq1$ 时，回转面法向与轨道纵向正交给出可见轮廓条件 $\sin\tau_s=\chi$。实现把同一表达式延拓到全部有限姿态：

$$
\sin\tau_s(\eta)=
\operatorname{clamp}\!\left(-\tan\beta\,h'(\eta),-1,1\right),
$$

$$
\cos\tau_s=\sqrt{\max(0,1-\sin^2\tau_s)}.
$$

当 $\beta=0$ 时，$\tau_s=0$，可见轮廓就是最低周向位置上的轮型面。非零摇头使有坡度的型面点沿周向移动，同时改变投影的 $Y$ 与 $Z$。若 $|\chi|>1$，夹制取 $\sin\tau_s=\pm1$；这是选择周向边界方向的数值延拓，不再是原始法向正交方程的实根。

在等距轮站位上预先求得 $h(\eta_i)$ 与 $h'(\eta_i)$。两者都来自 `wheel_spline_`，从而使可见轮廓与三维纵向解析使用同一自然样条求值语义。`wheel_surface_` 则服务于通用表面与曲率计算；这里没有“保形规则把轮面节点斜率压平”的过程。

### 3.3 分箱上包络

投影点 $(Y_i,Z_i,\eta_i)$ 可能因摇头和滚转而在 $Y$ 方向折叠，不再是单值函数。模型以宽度 $w_b$ 分箱：

$$
n_b=\left\lceil\frac{Y_{\max}-Y_{\min}}{w_b}\right\rceil+1,
$$

$$
k(Y)=
\min\!\left(
\left\lfloor\frac{Y-Y_{\min}}{w_b}\right\rfloor,
n_b-1
\right).
$$

每个箱只保留 $Z$ 最大的投影点，即最外层、能够接触钢轨的分支。保留的是样本本身的 $(Y_i,Z_i,\eta_i)$，不是箱中心；$w_b$ 是折叠竞争尺度，不是重新采样步长。

包络节点的斜率由保形三次规则计算。注意保形限斜只用于这个投影上包络 $H(Y)$，不用于上一节的 wheel_surface_ 或 rail_surface_。相邻包络节点之间采用 Hermite 三次段。令

$$
t=\frac{Y-Y_j}{\Delta_j},\qquad \Delta_j=Y_{j+1}-Y_j,
$$

则

$$
H(Y)=c_0+c_1t+c_2t^2+c_3t^3,
$$

$$
\begin{aligned}
c_0&=Z_j,&
c_1&=\Delta_jm_j,\\
c_2&=-3Z_j+3Z_{j+1}-2\Delta_jm_j-\Delta_jm_{j+1},\\
c_3&=2Z_j-2Z_{j+1}+\Delta_jm_j+\Delta_jm_{j+1}.
\end{aligned}
$$

从 $Y$ 回到轮站位的 $\eta(Y)$ 采用分段线性插值。投影折叠处的站位可跳变；对它施加平滑三次插值会生成轮面上并不存在的中间站位。

### 3.4 互穿与接触岛

在轮包络与轨面共同覆盖区间，把两组节点横坐标合并为有序共同网格。网格上的竖向互穿为

$$
g_i=H(Y_i)-z_r(Y_i).
$$

严格条件 $g_i>\epsilon$ 定义接触。满足条件的极大连续网格段构成原始接触岛；在实现可表示的范围内，这些岛按横坐标扫描顺序保留。

相邻两岛之间的谷底为

$$
v_k=\min_{e_k\leq i\leq s_{k+1}}g_i.
$$

若 $v_k>-\delta_m$，两岛合并。判据取谷的竖向深度，而不是两岛的横向距离。合并岛边缘在相邻异号网格值之间用一次割线插值：

$$
Y_e=Y_a+
\frac{(\epsilon-g_a)(Y_b-Y_a)}{g_b-g_a}.
$$

当分母为零时取中点。该一次插值与岛发现阶段采用的分段线性符号模型一致。

### 3.5 逐岛求积

每个保留的合并岛在 $[Y_L,Y_R]$ 上使用自己的均匀求积网格。定义

$$
o(Y)=\max(0,H(Y)-z_r(Y)).
$$

竖向穿透 $\delta_v$ 是求积站位上的最大 $o$，最深站位取首个最大者。复合梯形法在同一站位序列上计算

$$
A=\int_{Y_L}^{Y_R}o\,dY,
\qquad
W_a=\int_{Y_L}^{Y_R}\sqrt{1+H'^2}\,dY,
$$

$$
A_a=\int_{Y_L}^{Y_R}o\sqrt{1+H'^2}\,dY,
\qquad
M_y=\int_{Y_L}^{Y_R}Yo\,dY,
$$

$$
M_z=\int_{Y_L}^{Y_R}
\frac{(z_r+o)^2-z_r^2}{2}\,dY.
$$

于是

$$
Y_c=\frac{M_y}{A},
\qquad
Z_c=\frac{M_z}{A}.
$$

$A$ 是横截面重叠面积，$W_a$ 是轮包络弧宽，$A_a$ 是弧加权重叠面积，$(Y_c,Z_c)$ 是重叠区域的面积形心。岛发现网格与岛内求积网格解决不同的离散问题，因此求积所得最深点不必与共同网格的最大互穿点相同。

### 3.6 法向穿透、角度与曲率

最深求积站位处的轨面斜率把竖向穿透投影到法向：

$$
\delta_n=
\frac{\delta_v}
{\sqrt{1+z_r'(Y_d)^2}}.
$$

形心处的轮站位和局部轮径为

$$
\eta_c=\eta(Y_c),
\qquad
r_c=R_0+\hat h(\eta_c).
$$

定义轨面自身坡角、公法线角及含轨底坡的接触系角：

$$
\alpha_r=\arctan z_r'(Y_c),
$$

$$
\gamma=
\operatorname{atan2}\!\left(
\cos\beta\,\sin(\alpha_r-\varphi),
\cos(\alpha_r-\varphi)
\right),
$$

$$
\alpha_s=\alpha_r-\varsigma c_r,
$$

其中 $\varsigma=+1$ 表示右侧、$\varsigma=-1$ 表示左侧，$c_r$ 是几何模型采用的轨底坡幅值。$\gamma$ 不再重复加入轨底坡，因为位姿滚转已含钢轨姿态；$\alpha_s$ 则用于构造接触力坐标系。

接触斑在轮上的纵向坐标为

$$
x_c=r_c\,
\operatorname{clamp}\!\left(-\tan\beta\tan\gamma,-1,1\right).
$$

轮轨曲率均按平面曲线公式计算：

$$
\kappa=\frac{z''}{(1+z'^2)^{3/2}}.
$$

轮侧在 $\eta_c$ 对 $\hat h$ 求导，轨侧在 $Y_c$ 对 $z_r$ 求导。二者属于各自型面坐标，不应被解释为同一全局曲率分量。

### 3.7 钢轨材料参考点

接触斑携带一个未变形轨面上的材料参考点，用于后续在同一空间位置形成两体速度。它由轮侧几何代表点的纵横坐标投影到轨系，再竖直落到轨面：

$$
x_R=\cos\beta\,x_c-\sin\beta\,\eta_c,
$$

$$
Y_R=
\cos\varphi(\sin\beta\,x_c+\cos\beta\,\eta_c)
-\sin\varphi(r_c-R_0)+d_y,
$$

$$
Z_R=z_r(Y_R).
$$

这是钢轨参考点而非轮面点或压力形心，因此其竖向坐标直接取未变形轨面高度。

### 3.8 三维纵向长度

横截面包络折叠掉了轮的周向自由度。对每个岛内求积站位，纵向解析重新令 $\tau$ 为自由变量，并定义

$$
o(\eta,\tau)=Z(\eta,\tau)-z_r(Y(\eta,\tau)).
$$

从可见轮廓角 $\tau_s$ 向前后搜索，使两侧都括住 $o=0$ 的根。初始角步长采用局部圆估计并设有限下界：

$$
\Delta_0=
\max\!\left(
\Delta_{\min},
\sqrt{\frac{2o(\eta,\tau_s)}{r(\eta)}}
\right),
$$

之后倍增至规定的角域上界。两侧根括住后分别二分。所得站位弦长为

$$
\ell(\eta)=
\left|
r(\eta)\cos\beta\,
(\sin\tau_a-\sin\tau_b)
\right|,
$$

接触斑的三维纵向长度取

$$
L=\max_\eta\ell(\eta).
$$

若没有站位形成有效的双侧根括区，则 $L=0$，其理论含义是几何纵向测量不可得，而不是一个物理上必为零长的接触斑。

二分按纵向弦长绝对误差停止。令 $p=|r\cos\beta|$，由于正弦是 1-Lipschitz 函数，

$$
|x(\tau)-x(\tau')|\leq p|\tau-\tau'|.
$$

在精确算术中，若两侧真实根始终留在各自的异号括区内，而且细化由 $p\Delta\tau$ 条件终止，则取中点后两根对弦长的合计误差不超过规定分辨率。当前模型把 20 nm 作为绝对弦长目标；每侧另有 36 次二分的硬上限，因此先达到该上限时，20 nm 不是无条件误差保证。这个条件界使用每个站位的实际局部半径，不依赖特定轮型曲率。

为减少中点三角函数求值，二分可用端点方向向量之和与偶次多项式近似归一化。令

$$
h=\frac{\tau_{\mathrm{in}}-\tau_{\mathrm{out}}}{2},
$$

$$
P_8(h)=
\frac12\left[
1+h^2\left(
\frac12+h^2\left(
\frac5{24}+h^2\left(
\frac{61}{720}+\frac{1385}{40320}h^2
\right)\right)\right)\right]
\approx\frac1{2\cos h}.
$$

则

$$
\sin\tau_m\approx
(\sin\tau_{\mathrm{out}}+\sin\tau_{\mathrm{in}})P_8(h),
$$

$$
\cos\tau_m\approx
(\cos\tau_{\mathrm{out}}+\cos\tau_{\mathrm{in}})P_8(h).
$$

在搜索规定的角宽内，该近似有显式局部误差界；它可能在重叠量极接近零时改变一次二分分支，因此属于带误差预算的数值近似，而不是代数恒等替换。

对尚未完整解析的站位，只要真实根仍在括区内，精确算术中可由根括区得到几何竞争上界：

$$
U=p(\tau_{\max}-\tau_{\min}).
$$

若 $U$ 已不大于当前最长弦，该站位在上述条件下不可能改变 $L$，可以停止细化。实现以普通 binary64 直接计算 $U$，没有定向舍入；因此筛除与中点近似同属有限精度算法的一部分，而不是区间算术意义下的严格证明。

## 4. 算法结构

`ContactGeometrySolver::Solve` 的主要顺序为：

1. 解码位姿偏移并投影预先采样的轮廓点。
2. 按横向位置分箱，保留每箱最外层点。
3. 为上包络构造保形三次段，并建立分段线性的 $\eta(Y)$。
4. 合并轮包络与轨型节点，在共同网格上计算互穿并发现可保留的原始岛。
5. 按谷深阈值合并相邻岛，以割线插值得到岛边缘。
6. 在每个保留的合并岛上求积，形成面积、宽度、形心、穿透和最深站位。
7. 计算角度、局部半径、轮轨曲率和钢轨材料参考点。
8. 恢复周向自由度并求三维纵向最长弦。

若轮廓采样数为 $n_s$、包络节点数为 $n_e$、轨节点数为 $n_r$、合并岛数为 $n_i$、每岛求积站位数为 $N_q$，主要工作量由投影和分箱 $O(n_s)$、共同网格归并 $O(n_e+n_r)$ 与岛内求积/纵向解析 $O(n_iN_q)$ 构成。二分深度由长度误差目标与局部投影半径共同决定。

## 5. 离散近似、非光滑性与适用条件

- 可见轮廓只在有限个轮站位采样；窄于采样尺度的型面特征可能被漏过。
- 精确法向正交形式要求 $|\tan\beta\,h'(\eta)|\leq1$；超出时采用 $\sin\tau_s=\pm1$ 的夹制延拓。
- 分箱上包络以 $w_b$ 判定投影分支之间的横向竞争；改变 $w_b$ 会改变折叠分支选择。
- 接触岛由共同节点网格发现，而斑的积分量由另一均匀网格近似；两套网格不应混作同一离散化。
- 离散求解器按扫描顺序最多保留 64 个原始岛，并最多输出前 16 个合并斑。本篇公式完整覆盖未触及上限的模型域；超过上限时，算法只继续合并所保留的 64 岛前缀，并从所得合并岛中形成至多 16 个斑。
- 严格接触阈值、岛合并阈值、首个最大值选择、分箱胜者选择与最长弦的 max 都引入分段切换和非光滑性。
- $\eta(Y)$ 在投影折叠处允许跳变；分段线性映射保留这个几何事实。
- 轮轨曲率要求局部一阶、二阶导数及曲率分母有意义。近尖点、近竖直切线或非光滑测量型面会使局部曲率成为不稳定描述。
- 纵向解析假设从可见轮廓向前、后能在有限角域内各找到一次离面根；$L=0$ 表示这一测量不可得，后续法向模型采用自己的解析纵向尺度。
- 模型把接触斑描述为横截面重叠岛与每站位周向弦的组合，不求解三维弹性自由边界；法向和切向模型据此再构造等效接触尺度。

## 6. 实现映射

| 理论对象 | 主要实现 |
|---|---|
| 轮轨型面与导数接口 | [natural_cubic_spline.cc](../../../libs/wheel_rail_contact/src/natural_cubic_spline.cc)、[monotone_cubic_interpolant.cc](../../../libs/wheel_rail_contact/src/monotone_cubic_interpolant.cc) |
| 轮廓投影、包络、接触岛与求积 | [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
| 纵向根括取、二分、筛除与弦长 | [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) 中的 ResolveLongitudinalLength |
| 几何输入位姿 | [wheel_rail_pose.cc](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc) |
| 几何斑的法向消费 | [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc) |
