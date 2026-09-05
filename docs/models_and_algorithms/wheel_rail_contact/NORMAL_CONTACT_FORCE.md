[English](NORMAL_CONTACT_FORCE.en.md)

# 法向接触力

本篇说明 ORVD 如何把单个接触斑的几何量转成法向载荷：由重叠截面构造等效穿透，由椭圆 Hertz 关系得到弹性力与最大压力，再按当前割线刚度加入速度阻尼。核心实现位于 [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc)，其在完整接触链中的消费位于 [wheel_rail_contact_model.cc](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc)。

## 1. 范围

法向律读取六个几何量：横截面重叠面积、轮包络弧宽、三维纵向长度、竖向穿透、局部滚动半径与公法线角；另读取法向接近速度。它输出法向总力、弹性与阻尼分量、纵横半轴、等效穿透和最大压力。

几何量的构造见[接触几何](CONTACT_GEOMETRY.md)，接近速度与接触系见[蠕滑率与接触系](CREEPAGE_AND_CONTACT_FRAME.md)，半轴与法向力的切向消费见[切向接触](TANGENTIAL_CONTACT_FASTSIM.md)。

模型采用相同材料的轮与轨、刚性轨面和常数穿透等效系数。它不是完整三维弹性自由边界求解，也不是 Hunt–Crossley 型穿透幂次阻尼律。

## 2. 记号

| 记号 | 含义 | 实现量 |
|---|---|---|
| $A$ | 平面横截面重叠面积 | cross_section_area_square_meters |
| $W$ | 轮包络弧宽，作为等面积圆弧段的弦长 | arc_width_meters |
| $L_3$ | 接触几何给出的三维纵向长度 | longitudinal_length_meters |
| $d$ | 竖向穿透 | vertical_penetration_meters |
| $R$ | 接触处局部滚动半径 | rolling_radius_meters |
| $\theta_n$ | 公法线角 | common_normal_angle_radians |
| $R_{\mathrm{eff}}$ | 解析纵向基线的有效半径 | effective_radius |
| $h$ | 与 $(A,W)$ 等面积、等弦的圆弧段高度 | segment_height |
| $f$ | 穿透等效系数 | penetration_equivalence_factor |
| $\delta_{\mathrm{eq}}$ | 等效穿透 | equivalent_penetration_meters |
| $E,\nu$ | 两体共同的杨氏模量与泊松比 | ContactMaterial |
| $E^*$ | 两同材料体的等效接触模量 | equivalent_modulus_pascals |
| $a,b$ | 纵向与横向半轴 | result semi-axis fields |
| $K(m)$ | 以参数 $m$ 表示的第一类完全椭圆积分 | CompleteEllipticIntegralFirstKind |
| $p_0$ | 最大接触压力 | maximum_pressure_pascals |
| $F_e,F_d,N$ | 弹性力、阻尼力与法向总力 | NormalContactResult |
| $v_n$ | 接近为正的法向相对速度 | approach_speed_meters_per_second |
| $k_s$ | 当前割线刚度 $F_e/\delta_{\mathrm{eq}}$ | secant_stiffness |

## 3. 模型

### 3.1 从几何互穿到等效穿透

接触几何给出的 $d$ 与 $A$ 描述两条未变形型面的几何互穿；Hertz 理论中的趋近量描述两弹性体远端的相对接近。模型用等面积圆弧段把二者连接起来。

以轮包络弧宽 $W$ 为弦、以 $h$ 为弧高，过弦端点与弧顶的圆半径为

$$
R_c=\frac{W^2}{8h}+\frac h2.
$$

对应圆心角和圆弧段面积为

$$
\vartheta=2\arccos\frac{R_c-h}{R_c},
$$

$$
A_{\mathrm{seg}}(h,W)
=\frac12R_c^2(\vartheta-\sin\vartheta).
$$

在本模型采用的高度区间 $0<h\leq W$ 内，通过单调反解

$$
A_{\mathrm{seg}}(h,W)=A
$$

得到 $h$，再定义

$$
\delta_{\mathrm{eq}}=f h.
$$

这个精确反解要求目标面积不超过

$$
A_{\max}(W)=A_{\mathrm{seg}}(W,W)\approx1.05246\,W^2.
$$

这里使用的是平面面积 $A$，不是弧加权面积。系数 $f$ 把完整条带关系中的形状效应压缩为常数，是本法向律的主要模型近似。浅弧段有

$$
A_{\mathrm{seg}}\sim\frac23Wh,
\qquad
h\sim\frac{3A}{2W},
$$

但实际计算使用完整圆弧段表达式。

### 3.2 纵向尺度与解析基线

横向半轴直接由弧宽给出：

$$
b=\frac W2.
$$

模型对每个斑先建立解析纵向基线。以余弦地板 $\epsilon_{\cos}>0$ 正则化近直角公法线，

$$
R_{\mathrm{eff}}
=
\frac{R}
{\max(|\cos\theta_n|,\epsilon_{\cos})},
$$

$$
L_{\mathrm{base}}
=
2\sqrt{\max(0,2R_{\mathrm{eff}}d)}.
$$

若接触几何给出的 $L_3$ 有限且为正，则采用 $L=L_3$；否则采用 $L=L_{\mathrm{base}}$。纵向半轴为

$$
a=\frac L2.
$$

因此 $L_3=0$ 的含义是三维几何长度不可用，而不是强迫法向斑为零长；解析基线使该情况下的纵向半轴仍有定义。

### 3.3 椭圆 Hertz 关系

对两体具有相同的 $E,\nu$，

$$
\frac1{E^*}=
\frac{2(1-\nu^2)}{E}.
$$

输出半轴始终保持固定语义：$a$ 为纵向，$b$ 为横向，不因大小关系而交换。椭圆积分内部另取

$$
a_{\max}=\max(a,b),
\qquad
a_{\min}=\min(a,b),
$$

$$
\rho=\frac{a_{\min}}{a_{\max}},
\qquad
m=1-\rho^2.
$$

这里 $m$ 是偏心率的平方，即椭圆积分的参数，而不是模数。第一类完全椭圆积分定义为

$$
K(m)=
\int_0^{\pi/2}
\frac{d\varphi}
{\sqrt{1-m\sin^2\varphi}}.
$$

最大压力与弹性法向力为

$$
p_0=
\frac{\delta_{\mathrm{eq}}E^*}
     {a_{\min}K(m)},
$$

$$
F_e=
\frac23\pi a_{\max}a_{\min}p_0.
$$

等价地，

$$
F_e=
\frac{2\pi}{3}
\frac{a_{\max}E^*\delta_{\mathrm{eq}}}{K(m)}.
$$

圆斑极限 $a=b$ 时 $m=0$、$K(0)=\pi/2$，于是

$$
F_e=\frac43E^*a\,\delta_{\mathrm{eq}}.
$$

对固定半轴，$F_e$ 与 $\delta_{\mathrm{eq}}$ 线性；完整轮轨系统还会因接触几何随状态改变斑形状和尺度而呈现非线性。

### 3.4 阻尼与不可受拉条件

当前割线刚度为

$$
k_s=\frac{F_e}{\delta_{\mathrm{eq}}}.
$$

阻尼系数按其平方根缩放：

$$
c=c_{\mathrm{ref}}
\sqrt{\frac{k_s}{k_{\mathrm{ref}}}},
\qquad
F_d=cv_n.
$$

只有当等效穿透超过小的激活阈值时才计算这一项，从而避免在趋近零穿透处形成割线刚度。总法向力满足不可受拉条件：

$$
N=\max(0,F_e+F_d).
$$

若分离速度足以使未裁剪总和为负，局部法向律令 $N=0$，并把所报告的阻尼分量改写为 $F_d=-F_e$，保持

$$
F_e+F_d=N.
$$

从局部法向律看，这仍是具有有效几何但法向载荷为零的单边接触结果。完整物理链只让 $N>0$ 的斑进入蠕滑与切向接触阶段；因此 $N=0$ 表示几何候选在力学装配处失活，而不是一个仍具库仑摩擦容量的接触斑。

### 3.5 数学定义域

法向律的有效几何域要求

$$
0<A\leq A_{\max}(W),\qquad W>0,\qquad d>0,\qquad R>\epsilon_R,
$$

且这些量有限。圆弧段等效还要求 $f>0$；材料域要求 $E>0$、$0\leq\nu<1/2$；阻尼尺度要求 $c_{\mathrm{ref}}>0$、$k_{\mathrm{ref}}>0$。$L_3$ 不属于必需域，因为缺失时有解析基线。

在这些条件下，$a,b,\delta_{\mathrm{eq}},K(m),p_0,F_e$ 均为正。公法线接近直角时，余弦地板把无限增长的 $R_{\mathrm{eff}}$ 替换为有限正则化；这是一种模型延拓，不是直角接触的精确渐近解。

## 4. 数值算法

### 4.1 圆弧段高度反解

SolveCircularSegmentHeight 在 $[h_{\min},W]$ 内对 $A_{\mathrm{seg}}(h,W)-A$ 二分，其中 $h_{\min}=10^{-15}\,\mathrm m$，因为 $h=0$ 时 $R_c$ 的表达式退化。中点面积大于目标时收缩上界，否则收缩下界，直到高度区间小于绝对长度容差或达到有限迭代上限。若 $A$ 超出该数值区间映射出的面积范围，所得高度饱和在相应端点附近，而不再满足精确反解等式。

前向面积计算把反余弦自变量夹在 $[-1,1]$ 内。二分采用绝对而非相对高度容差，因此其分辨尺度不随弦长改变。

### 4.2 第一类完全椭圆积分

实现先把 $m$ 限制在 $[0,1)$ 的有限闭子区间，以避开 $m\to1$ 的对数发散。随后采用算术几何平均：

$$
a_0=1,\qquad g_0=\sqrt{1-m},
$$

$$
a_{j+1}=\frac{a_j+g_j}{2},
\qquad
g_{j+1}=\sqrt{a_jg_j},
$$

$$
K(m)=
\frac{\pi}
{2\,\operatorname{AGM}(1,\sqrt{1-m})}.
$$

算术几何平均二次收敛，以算术、几何均值之差作为停止量，并设置有限迭代上限。

### 4.3 单斑求值顺序

NormalContactLaw::Solve 的理论顺序为：

1. 检查几何量是否位于正有限定义域。
2. 形成 $b$、解析纵向基线，并由可用的 $L_3$ 覆盖基线以得到 $a$。
3. 反解等面积圆弧段并形成 $\delta_{\mathrm{eq}}$。
4. 局部排序半轴，计算 $m$ 与 $K(m)$。
5. 计算 $p_0$、$F_e$ 与割线刚度。
6. 在激活区间内加入阻尼，并对总力施加不可受拉裁剪。
7. 完整接触模型仅对 $N>0$ 的斑继续计算蠕滑与切向力。

每斑的工作由一次有界二分、一次有界算术几何平均和常数次标量运算组成。

## 5. 非光滑性与理论适用条件

- $A$、$W$、$d$ 或 $R$ 跨越有效域边界时，法向接触状态发生分支切换。
- $L_3$ 在可用与不可用之间切换时，$a$ 从三维测量切换到解析基线，力可以不连续。
- 阻尼在等效穿透激活阈值处是分段定义；除非 $v_n=0$，该处通常不光滑。
- 不可受拉裁剪在 $F_e+F_d=0$ 处连续但导数不连续；组装层同时在这里改变接触斑数。
- $a=b$ 时局部主、次轴排序换支，但对称的椭圆积分与力值连续。
- 模型假设每个几何岛可由半轴 $(a,b)$ 的等效椭圆代表，并用横截面等面积圆弧段代理弹性趋近量。强共形接触、多点耦合、塑性、粗糙度与材料非线性不在该理论中。
- 阻尼是以当前割线刚度缩放的线性速度项。它适合表示接触耗散的低阶闭合关系，不应解释为由材料黏弹性推导出的唯一规律。

## 6. 实现映射

| 理论对象 | 主要实现 |
|---|---|
| 圆弧段面积与高度反解 | CircularSegmentArea、SolveCircularSegmentHeight，见 [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc) |
| 等效模量、Hertz 力与阻尼 | NormalContactLaw，见 [normal_contact_force.cc](../../../libs/wheel_rail_contact/src/normal_contact_force.cc) |
| 几何量的来源 | [contact_geometry.cc](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |
| 接近速度、正载荷门与切向衔接 | [wheel_rail_contact_model.cc](../../../libs/wheel_rail_contact/src/wheel_rail_contact_model.cc) |
