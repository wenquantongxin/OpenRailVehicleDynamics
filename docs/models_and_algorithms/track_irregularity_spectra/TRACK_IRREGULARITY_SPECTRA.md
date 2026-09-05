[English](TRACK_IRREGULARITY_SPECTRA.en.md)

# 轨道不平顺谱及其空间随机实现

本篇说明 ORVD 如何把单边空间功率谱密度（power spectral density, PSD）转化为有限长度的横向、竖向轨道不平顺场。重点是谱变量与单位、有限频带、随机相位谐波和、确定性的种子到相位映射、通道相关性以及双端 `smoothstep5` 包络；最后以紧凑映射连接这些理论对象与源码。

## 1. 模型对象与范围

轨道不平顺建模包含三个彼此独立的选择：谱公式规定各空间频率上的二阶统计能量，有限频带规定保留的最长与最短波长，随机实现规定各离散频率的相位。相同的谱和频带可以产生无穷多条 realization；改变随机种子不改变理论 PSD，改变截止频率则会改变方差与动力学含义。

本篇只讨论叠加在理想线路上的平稳随机几何不平顺。线路平面曲线、纵坡、竖曲线和超高属于[线路几何与轨道坐标系](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.md)，焊缝、波磨、擦伤等局部确定性缺陷也不能由平稳 PSD 唯一描述。横向位移沿轨型系向右为正，竖向位移向下为正，站位沿用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)中的平面投影里程。

当前实现可以由 AAR5/AAR6 谱生成一次双通道 realization，也可以解析三套已经门控的冻结 field；两条路径最终都形成同一种 `TrackIrregularityField`，冻结点列不会再叠加第二层包络。

AAR5 与 AAR6 在这里表示两组轨道平顺性谱参数，不表示车辆类别，也不是对任意线路、速度和车辆频带都适用的普遍描述。

## 2. 空间频率与单边 PSD

### 2.1 循环空间频率、角波数与波长

令 $f$ 为循环空间频率，单位为 $\mathrm{cycles/m}$；令 $\Omega$ 为空间角波数，单位为 $\mathrm{rad/m}$；令 $\lambda$ 为空间波长。三者满足

$$
\Omega=2\pi f,
\qquad
\lambda=\frac{1}{f}=\frac{2\pi}{\Omega}.
$$

设 $S_f(f)$ 与 $S_\Omega(\Omega)$ 分别是以 $f$ 和 $\Omega$ 为自变量的单边 PSD。换元必须保持方差：

$$
\sigma^2
=\int_{f_{\min}}^{f_{\max}}S_f(f)\,df
=\int_{\Omega_{\min}}^{\Omega_{\max}}S_\Omega(\Omega)\,d\Omega.
$$

因为 $d\Omega=2\pi\,df$，两种表示之间的关系是

$$
S_f(f)=2\pi S_\Omega(2\pi f),
\qquad
S_\Omega(\Omega)=\frac{1}{2\pi}S_f\!\left(\frac{\Omega}{2\pi}\right).
$$

因此不能只把频率轴乘以 $2\pi$ 而保持 PSD 纵轴不变，也不能把针对 $f$ 展开的多项式系数直接用于 $\Omega$。本篇的单边谱只积分正空间频率；双边谱必须先按其自身归一化约定换成一致的单边表示。

### 2.2 从空间频率到时间频率

若车辆以速度大小 $v_s$ 通过空间波长 $\lambda$，对应的时间频率为

$$
f_t=v_s f=\frac{v_s}{\lambda}.
$$

速度改变的是空间激励到车辆时间响应的映射，不会改变线路本身的空间 PSD。

## 3. 简化 FRA/AAR 谱

### 3.1 横向与竖向单截止谱

ORVD 实现的 AAR5/AAR6 模型采用单截止角波数谱。此类以 PSD、粗糙度参数和截止频率表征随机轨道几何的方法可参见 [FRA 的轨道几何统计表示报告](https://rosap.ntl.bts.gov/view/dot/9617)。横向与竖向谱分别为

$$
S_{\mathrm{lat}}(\Omega)
=\frac{kA_a\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)},
\qquad
S_{\mathrm{ver}}(\Omega)
=\frac{kA_v\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)}.
$$

实现采用下列参数；$A_a$ 与 $A_v$ 是传统表值，单位为 $\mathrm{cm^2\,rad/m}$，形成 SI 制长度 PSD 时还要乘以 $10^{-4}$ 完成 $\mathrm{cm^2\to m^2}$ 换算。

| 等级 | $A_a$ | $A_v$ | $\Omega_c$ / $\mathrm{rad\,m^{-1}}$ | $k$ |
|---|---:|---:|---:|---:|
| AAR5 | `0.0762` | `0.2095` | `0.8245` | `0.25` |
| AAR6 | `0.0339` | `0.0339` | `0.8245` | `0.25` |

令 $d=10^{-4}$，并以 $A$ 表示相应方向的 $A_a$ 或 $A_v$，同一谱可写成多项式商

$$
S_\Omega(\Omega)=\frac{b_0}{a_2\Omega^2+\Omega^4},
\qquad
a_2=\Omega_c^2,
\qquad
b_0=kAd\,\Omega_c^2.
$$

`AarTrackClass` 只选择 $A_a$、$A_v$、$k$ 与 $\Omega_c$ 所确定的谱形和幅值，并不隐式规定唯一的 $f_{\min}$、$f_{\max}$ 或离散频率数。

### 3.2 Gauge 与 cross-level（仅理论）

一种常见的简化参考式把 gauge 或 cross-level 写成双截止谱，其中 $\Omega_s$ 是第二个截止角波数：

$$
S_{gcl}(\Omega)=
\frac{4kA_v\Omega_c^2}
{(\Omega^2+\Omega_c^2)(\Omega^2+\Omega_s^2)}.
$$

其多项式形式满足

$$
a_0=\Omega_c^2\Omega_s^2,
\qquad
a_2=\Omega_c^2+\Omega_s^2,
\qquad
a_4=1,
\qquad
b_0=4kA_v10^{-4}\Omega_c^2.
$$

若 cross-level 以两参考点的高差 $u$ 表示，而所需量是小角度滚转 $\phi$，还必须给出参考基长 $b_{\mathrm{ref}}$：

$$
\phi\simeq\frac{u}{b_{\mathrm{ref}}},
\qquad
S_\phi=\frac{S_u}{b_{\mathrm{ref}}^2}.
$$

Gauge 与 cross-level 是不同的几何量，共享有理函数外形不代表物理等价。当前生成器只实现横向、竖向位移通道；这里的双截止谱和相关多通道生成均为理论说明，不能从现有两通道结果中恢复。

## 4. 有限频带与方差

单截止谱在低角波数处按 $\Omega^{-2}$ 增长，因此若令下截止趋于零，其方差发散。正的 $f_{\min}$ 是模型定义的一部分，而不是可以无代价删除的数值细节。高角波数端按 $\Omega^{-4}$ 衰减，但 $f_{\max}$ 仍决定最短波长以及轨道、轮轨接触和车辆模型会接收到的最高空间频率。

对 $0<\Omega_{\min}<\Omega_{\max}$，定义

$$
\mathcal F(\Omega)=-\frac{1}{\Omega}
-\frac{1}{\Omega_c}\arctan\!\left(\frac{\Omega}{\Omega_c}\right).
$$

单截止谱在有限带内的连续方差为

$$
\sigma_{\mathrm{continuous}}^2
=kA10^{-4}
\left[\mathcal F(\Omega_{\max})-\mathcal F(\Omega_{\min})\right],
\qquad
\Omega_{\min,\max}=2\pi f_{\min,\max}.
$$

降低 $f_{\min}$ 会加入长波能量，提高 $f_{\max}$ 会加入短波能量，所以谱公式、有限带和 realization 必须共同定义一个随机工况。空间输出网格的间距 $\Delta s$ 还应满足 Nyquist 必要条件

$$
\Delta s\,f_{\max}<\frac12.
$$

实现采用严格不等式，以避免恰好位于 Nyquist 频率的随机相位谐波退化。有限长度 realization 的样本均方值一般不会精确等于连续积分方差；端部包络还会进一步改变全区间统计量。

## 5. 随机相位谐波实现

### 5.1 离散频率网格与谐波和

对包含两端点且 $N\ge2$ 的等距频率网格

$$
f_j=f_{\min}+(j-1)\Delta f,
\qquad
\Delta f=\frac{f_{\max}-f_{\min}}{N-1},
\qquad
j=1,\ldots,N,
$$

ORVD 采用随机相位谐波和

$$
r_{\mathrm{raw}}(s)=\sum_{j=1}^{N}
\sqrt{2S_f(f_j)\Delta f}
\cos\!\left(2\pi f_j\xi+\theta_j\right),
\qquad
\xi=s-s_0,
$$

其中 $s_0$ 是 placement 起点，$\theta_j$ 在 $[0,2\pi)$ 上均匀分布。每一项的相位期望为零、方差为 $S_f(f_j)\Delta f$，独立相位下总期望方差为

$$
\sigma_{\mathrm{discrete}}^2
=\sum_{j=1}^{N}S_f(f_j)\Delta f.
$$

生成器对端点与内部频率采用相同的矩形权重，而不是对两端使用梯形半权，因此 $\sigma_{\mathrm{discrete}}^2$ 与上一节的连续积分是两个不同但随频率网格细化而接近的量。相位坐标使用局部站位 $\xi$，所以保持谱、频率网格和种子不变而整体移动 placement，只会把同一 realization 随起点平移。

### 5.2 从 realization seed 到相位

一个无符号 64 位 realization seed 先经固定领域分离得到横向和竖向 channel seed。`SplitMix64` 的映射为

```text
x = x + 0x9E3779B97F4A7C15
x = (x XOR (x >> 30)) * 0xBF58476D1CE4E5B9
x = (x XOR (x >> 27)) * 0x94D049BB133111EB
x = x XOR (x >> 31)
```

所有运算都按无符号 64 位模 $2^{64}$ 进行，两个通道定义为

```text
lateral_seed  = SplitMix64(realization_seed XOR 0x4C41544552414C00)
vertical_seed = SplitMix64(realization_seed XOR 0x564552544943414C)
```

两个领域常量分别编码 `LATERAL` 加空字节与 `VERTICAL`。上述加法、奇数乘法和异或移位都可逆，因而整个 `SplitMix64` 是双射；不同的领域输入会得到不同的两个 channel seed。

每个通道用独立的 `std::mt19937_64` 引擎，按频率从低到高消费随机字。对第 $j$ 个 64 位输出 $x_j$，相位映射明确为

$$
u_j=(x_j\mathbin{\texttt{>>}}11)2^{-53},
\qquad
\theta_j=2\pi u_j.
$$

这一定义直接取高 53 位，不依赖标准库分布对象的具体算法，也不使用时钟或进程全局随机状态。相同的生成规格与 realization seed 因而确定同一组伪随机相位。

### 5.3 谐振器递推

设相邻站位的相位增量为 $\Delta\alpha_j=2\pi f_j\Delta s$。源码不在每个站位重新调用三角函数，而是按

$$
\begin{bmatrix}
c_{n+1}\\s_{n+1}
\end{bmatrix}
=
\begin{bmatrix}
\cos\Delta\alpha_j&-\sin\Delta\alpha_j\\
\sin\Delta\alpha_j&\cos\Delta\alpha_j
\end{bmatrix}
\begin{bmatrix}
c_n\\s_n
\end{bmatrix}
$$

推进 $c_n=\cos\alpha_{j,n}$ 与 $s_n=\sin\alpha_{j,n}$。在实数算术中这与直接谐波式等价；浮点递推会积累漂移，因此实现每 256 个站位间隔用解析相位重新锚定。

### 5.4 横向—竖向相关性

若两个通道复用完全相同的相位，AAR6 因横向、竖向 PSD 相同而会逐点相同；AAR5 的两个谱只在幅值参数上不同，序列将成为固定比例，比例为

$$
\sqrt{\frac{A_v}{A_a}}
=\sqrt{\frac{0.2095}{0.0762}}
\approx1.65811454.
$$

这种锁相不是 AAR 谱规定的物理相关性。领域分离使当前两个通道使用不同的伪随机相位流，但不同 seed 并不保证任一有限区间上的样本相关系数恰好为零。若要描述已知互谱的多方向场，应给出正半定互谱矩阵并进行联合谱分解与生成；当前实现不包含这一算法，仅理论上可扩展。

## 6. Placement 与双端 `smoothstep5`

令 placement 为 $[s_0,s_1]$，fade-in 与 fade-out 长度分别为 $L_{\mathrm{in}}$、$L_{\mathrm{out}}$。五次平滑步进为

$$
q(u)=6u^5-15u^4+10u^3,
\qquad
0\le u\le1.
$$

部署包络定义为

$$
w(s)=
\begin{cases}
0, & s\le s_0,\\
q\!\left((s-s_0)/L_{\mathrm{in}}\right),
& s_0<s<s_0+L_{\mathrm{in}},\\
1, & s_0+L_{\mathrm{in}}\le s\le s_1-L_{\mathrm{out}},\\
q\!\left((s_1-s)/L_{\mathrm{out}}\right),
& s_1-L_{\mathrm{out}}<s<s_1,\\
0, & s\ge s_1.
\end{cases}
$$

最终 realization 是

$$
r(s)=w(s)r_{\mathrm{raw}}(s).
$$

该构造要求 $s_1>s_0$、$L_{\mathrm{in}}>0$、$L_{\mathrm{out}}>0$ 且 $L_{\mathrm{in}}+L_{\mathrm{out}}\le s_1-s_0$。等号对应两个 fade 只在一个满幅点相接，不产生负长度平台。

五次多项式满足

$$
q(0)=0,
\quad q(1)=1,
\quad q'(0)=q'(1)=q''(0)=q''(1)=0.
$$

由于有限谐波和本身光滑，解析乘积 $r(s)$ 在零段、fade 与满幅段的连接处具有连续的位移、一阶坡度和二阶导数。`Smoothstep5` 在 $u>0.5$ 时利用关于 $u=0.5$ 的对称形式求值，以减少靠近 1 时直接计算多项式的相消。

生成点列先乘包络，再裁到 $[s_0,s_1]$ 并构造自然三次样条 `TrackIrregularityField`。该 field 在严格定义域外返回零位移与零坡度；但自然样条的端点导数来自离散插值，并不因解析 $q'(0)=q'(1)=0$ 而必然逐位为零。

## 7. 理论假设与适用范围

原始随机相位和在无限站位观念下具有由输入 PSD 决定的二阶统计结构；有限频带、有限频率数和有限站位区间把它变成一个离散近似。乘以 placement 包络后，场在过渡区不再平稳，因此谱与方差的平稳解释只直接适用于未加窗的谐波和或满幅平台。

PSD 只规定二阶统计量，不保存某条真实线路的确定相位，也不描述孤立缺陷。随机相位、横竖通道无指定互谱、单截止谱以及自然样条重建都是模型假设；研究对象若依赖确定性缺陷、非平稳演化或方向间相干，需要另行扩展模型。

## 8. 源码映射

| 理论对象 | 主要实现 |
|---|---|
| AAR 参数、$S_\Omega\to S_f$ 与连续频带方差 | `AarSingleCutoffPsdParametersFor`、`EvaluateAarOneSidedSpatialPsd`、`ContinuousBandVariance`，见 [`aar_track_irregularity_generator.cc`](../../../libs/track_irregularity/src/aar_track_irregularity_generator.cc) |
| 领域分离和 seed 到相位映射 | `DeriveAarTrackIrregularityChannelSeeds`、`SplitMix64`、`ReproduciblePhaseGenerator` |
| 谐波和、离散方差与周期重锚 | `AccumulateHarmonicChannel` |
| 五次包络与双通道生成 | `Smoothstep5`、`TrackIrregularityPlacementWeight`、`GenerateAarTrackIrregularity`；类型定义见 [`aar_track_irregularity_generator.h`](../../../libs/track_irregularity/include/orvd/track_irregularity/aar_track_irregularity_generator.h) |
| 生成点列到车辆场 | `ResolveTrackIrregularityField`，见 [`resolve_track_irregularity_field.cc`](../../../libs/configuration/src/resolve_track_irregularity_field.cc)；域外语义见 [`track_irregularity_field.cc`](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) |
