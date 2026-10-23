[English](TRACK_IRREGULARITY_SPECTRA.en.md)

# 轨道不平顺谱及其空间随机实现

本篇说明如何把单边空间功率谱密度（power spectral density, PSD）转化为有限长度的轨道不平顺场，并说明这个场如何落到左右两根钢轨上。内容按"物理量与单位 → 谱族 → 有限频带 → 随机实现 → 空间包络与几何施加"的顺序展开，末节把这些理论对象映射到源码。

## 1. 不平顺分量与坐标约定

轨道不平顺是叠加在理想线路上的平稳随机几何偏差。本篇用四个分量描述它，全部在轨型系中表达，自变量是[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)中的平面投影站位 $s$，正号约定与该篇 §2.9 一致。

| 分量 | 记号 | 单位 | 含义与正号 |
|---|---|---|---|
| 横向 | $y(s)$ | m | 两轨整体沿轨型系横向的位移，向右为正 |
| 竖向 | $z(s)$ | m | 两轨整体沿轨型系竖向的位移，向下为正 |
| 轨距 | $g(s)$ | m | 轨距相对名义值的变窄量，正值变窄，左右轨各向中心线移动 $g/2$ |
| 滚转 | $\phi(s)$ | rad | 两轨组成的轨平面绕轨型系 $+x$ 的转角，正转使右轨下沉、左轨抬升 |

交叉水平（cross-level）是滚转的另一种表达：记两轨参考点的高差为 $u$，右轨低于左轨时为正，则小角度下 $u$ 与 $\phi$ 由第 2.3 节的基长相联系。轨距与交叉水平都是长度，横向与竖向也是长度，滚转是角度；四个分量各自独立定义在站位上。

线路平面曲线与超高属于[线路几何与轨道坐标系](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.md)，纵坡与竖曲线属于[轨道竖向剖面建模及其三维耦合](../track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.md)；焊缝、波磨、擦伤等局部确定性缺陷由专门的确定性模型描述。本篇只处理平稳随机部分。

一个随机不平顺工况由三个相互独立的选择共同定义：谱公式规定各空间频率上的二阶统计能量，有限频带规定保留的最长与最短波长，随机实现规定各离散频率的相位。相同的谱和频带可以产生无穷多条 realization；改变随机种子只改变相位，改变截止频率则改变方差与动力学含义。

每个分量有两种来源：由谱生成一条 realization（第 5 节），或以给定的站位点列供给。给定点列自带相位、门控与插值历史，按原样使用。两种来源最终进入同一种场对象（第 6 节）。AAR5、AAR6 与 ERRI Low／High 在本篇中是谱参数集的名称，ERRI Low／High 区分不平顺幅值等级。

## 2. 谱变量、单位与换算

### 2.1 循环空间频率、角波数与波长

令 $f$ 为循环空间频率，单位为 $\mathrm{cycles/m}$；令 $\Omega$ 为空间角波数，单位为 $\mathrm{rad/m}$；令 $\lambda$ 为空间波长。三者满足

$$
\Omega=2\pi f,
\qquad
\lambda=\frac{1}{f}=\frac{2\pi}{\Omega}.
$$

### 2.2 单边谱与方差保持换元

设 $S_f(f)$ 与 $S_\Omega(\Omega)$ 分别是以 $f$ 和 $\Omega$ 为自变量的单边 PSD。换元保持方差：

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

换元同时作用于频率轴与 PSD 纵轴，针对 $f$ 展开的多项式系数与针对 $\Omega$ 展开的系数由此互相换算。本篇的单边谱只积分正空间频率；双边谱先按其归一化约定换成一致的单边表示再使用。

若车辆以速度大小 $v_s$ 通过空间波长 $\lambda$，对应的时间频率为

$$
f_t=v_s f=\frac{v_s}{\lambda}.
$$

速度决定空间激励到车辆时间响应的映射，线路本身的空间 PSD 与速度无关。

### 2.3 高差谱与滚转角谱的换算

交叉水平以高差 $u$ 描述，滚转以角度 $\phi$ 描述。设高差的两个测点横向相距 $b_{\mathrm{ref}}$，小角度下

$$
\phi=\frac{u}{b_{\mathrm{ref}}},
\qquad
S_\phi(\Omega)=\frac{S_u(\Omega)}{b_{\mathrm{ref}}^2}.
$$

$b_{\mathrm{ref}}$ 是高差测量的参考基长，与轨距是两个不同的量。以标准轨为例，轨距 $1.435\,\mathrm m$ 在轨顶面下的规定深度处量取两轨内侧面的距离，而交叉水平与超高的参考基长取两轨走行面中心的名义间距 $1.5\,\mathrm m$；把超高转成线路滚转角用的也是这个基长。一旦把高差谱按 $b_{\mathrm{ref}}$ 换成角谱，后续生成、插值与施加都直接针对角度进行。

直接以角度定义的谱把归一化长度写在自身的定义里。第 3.2.2 节的 ERRI 交叉水平角谱就是这种写法，它采用两轨中心间距的一半 $b=0.75\,\mathrm m$。

## 3. 谱模型

本篇的有理谱一律写成同一个多项式商模板

$$
S_\Omega(\Omega)=\frac{b_0+b_2\Omega^2}{a_0+a_2\Omega^2+a_4\Omega^4+a_6\Omega^6},
$$

其中 $S_\Omega$ 取 SI 制，长度谱的单位为 $\mathrm{m^2/(rad/m)}$，角谱的单位为 $\mathrm{rad^2/(rad/m)}$。位移谱只用到 $b_0$ 与 $a_0$、$a_2$、$a_4$；ERRI 交叉水平角谱用到 $b_2$ 与 $a_6$。两个谱族都以截止角波数的乘积因子给出，模板系数由因子展开直接得到。每个谱族按同一顺序叙述：所描述的量与单位、闭式、参数表、模板系数。

### 3.1 AAR 谱

AAR 谱族以粗糙度系数 $A_a$、$A_v$、截止角波数 $\Omega_c$、$\Omega_s$ 和常数 $k$ 描述六个轨道等级，闭式与参数表见 Lei 的 [High Speed Railway Track Dynamics](https://doi.org/10.1007/978-981-10-2039-1) 第 5 章。闭式以 $\mathrm{cm^2/(rad/m)}$ 为单位，$A_a$ 与 $A_v$ 以 $\mathrm{cm^2\,rad/m}$ 为单位；形成 SI 制谱时乘以 $10^{-4}$。本篇实现的两个等级参数如下。

| 等级 | $A_a$ | $A_v$ | $\Omega_c$ / $\mathrm{rad\,m^{-1}}$ | $\Omega_s$ / $\mathrm{rad\,m^{-1}}$ | $k$ |
|---|---:|---:|---:|---:|---:|
| AAR5 | 0.0762 | 0.2095 | 0.8245 | 0.8209 | 0.25 |
| AAR6 | 0.0339 | 0.0339 | 0.8245 | 0.4380 | 0.25 |

#### 3.1.1 横向与竖向

横向与竖向位移谱为单截止谱：

$$
S_{y}(\Omega)
=\frac{kA_a\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)},
\qquad
S_{z}(\Omega)
=\frac{kA_v\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)}.
$$

以 $A$ 表示相应方向的 $A_a$ 或 $A_v$，SI 制模板系数为

$$
a_0=0,
\qquad
a_2=\Omega_c^2,
\qquad
a_4=1,
\qquad
b_0=kA\cdot10^{-4}\,\Omega_c^2.
$$

$a_0=0$ 是该谱在低角波数端按 $\Omega^{-2}$ 增长的来源。

#### 3.1.2 轨距与交叉水平

轨距变窄量 $g$ 与交叉水平高差 $u$ 共用一个双截止谱，第二截止角波数 $\Omega_s$ 随等级取值：

$$
S_{g}(\Omega)=S_{u}(\Omega)
=\frac{4kA_v\Omega_c^2}
{(\Omega^2+\Omega_c^2)(\Omega^2+\Omega_s^2)}.
$$

这一形式同时见于 Xin 等的 [Research on characterization methods for track irregularities](https://doi.org/10.1007/s40534-024-00367-z)。SI 制模板系数为

$$
a_0=\Omega_c^2\Omega_s^2,
\qquad
a_2=\Omega_c^2+\Omega_s^2,
\qquad
a_4=1,
\qquad
b_0=4kA_v\cdot10^{-4}\,\Omega_c^2.
$$

滚转角谱由第 2.3 节的基长换算得到，模板系数中只有分子变化：$b_{0,\phi}=b_0/b_{\mathrm{ref}}^2$。

### 3.2 ERRI B176 谱

ERRI B176 谱族以 $A_a$、$A_v$ 和三个截止角波数 $\Omega_r$、$\Omega_c$、$\Omega_s$ 描述 Low 与 High 两个幅值等级，参数表见 Lei 的 [High Speed Railway Track Dynamics](https://doi.org/10.1007/978-981-10-2039-1) 第 5 章。闭式直接以 SI 制书写。

| 等级 | $A_a$ / $\mathrm{m\,rad}$ | $A_v$ / $\mathrm{m\,rad}$ | $\Omega_r$ / $\mathrm{rad\,m^{-1}}$ | $\Omega_c$ / $\mathrm{rad\,m^{-1}}$ | $\Omega_s$ / $\mathrm{rad\,m^{-1}}$ |
|---|---:|---:|---:|---:|---:|
| ERRI Low | $2.119\times10^{-7}$ | $4.032\times10^{-7}$ | 0.0206 | 0.8246 | 0.4380 |
| ERRI High | $6.125\times10^{-7}$ | $10.80\times10^{-7}$ | 0.0206 | 0.8246 | 0.4380 |

#### 3.2.1 横向与竖向位移谱

$$
S_{y}(\Omega)
=\frac{A_a\Omega_c^2}
{(\Omega^2+\Omega_r^2)(\Omega^2+\Omega_c^2)},
\qquad
S_{z}(\Omega)
=\frac{A_v\Omega_c^2}
{(\Omega^2+\Omega_r^2)(\Omega^2+\Omega_c^2)}.
$$

以 $A$ 表示相应方向的 $A_a$ 或 $A_v$，模板系数为

$$
a_0=\Omega_r^2\Omega_c^2,
\qquad
a_2=\Omega_r^2+\Omega_c^2,
\qquad
a_4=1,
\qquad
b_0=A\,\Omega_c^2.
$$

代入参数表得到的系数如下，四组谱共用同一分母。

| 参数集 | 方向 | $b_0$ | $a_0$ | $a_2$ | $a_4$ |
|---|---|---:|---:|---:|---:|
| ERRI Low | 横向 | $1.440846\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |
| ERRI Low | 竖向 | $2.741619\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |
| ERRI High | 横向 | $4.164787\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |
| ERRI High | 竖向 | $7.343623\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |

Low 与 High 在同一方向上只有 $b_0$ 不同，谱形相同；若频率网格和相位也相同，High realization 等于 Low realization 乘以 $\sqrt{b_{0,\mathrm{High}}/b_{0,\mathrm{Low}}}$。

#### 3.2.2 交叉水平角谱

ERRI B176 直接定义滚转角 $\phi$ 的谱，归一化长度取两轨中心间距的一半 $b=0.75\,\mathrm m$，并引入第三个截止角波数 $\Omega_s$；同一形式见 Xin 等的 [Research on characterization methods for track irregularities](https://doi.org/10.1007/s40534-024-00367-z) 式 9，该文把 $b$ 定义为左右轮心距的一半，并把角谱单位写成 $\mathrm{1/(rad/m)}$，与本篇的 $\mathrm{rad^2/(rad/m)}$ 同义：

$$
S_{\phi}(\Omega)
=\frac{A_v\Omega_c^2\,\Omega^2}
{b^2(\Omega^2+\Omega_r^2)(\Omega^2+\Omega_c^2)(\Omega^2+\Omega_s^2)},
\qquad
[S_\phi]=\mathrm{rad^2/(rad/m)}.
$$

模板系数为

$$
b_0=0,
\qquad
b_2=\frac{A_v\Omega_c^2}{b^2},
\qquad
a_0=\Omega_r^2\Omega_c^2\Omega_s^2,
\qquad
a_2=\Omega_r^2\Omega_c^2+\Omega_r^2\Omega_s^2+\Omega_c^2\Omega_s^2,
\qquad
a_4=\Omega_r^2+\Omega_c^2+\Omega_s^2,
\qquad
a_6=1.
$$

代入参数表：$b_2$ 在 Low 为 $4.87399\times10^{-7}$、在 High 为 $1.305533\times10^{-6}$；$a_0=5.535659\times10^{-5}$，$a_2=0.1308172$，$a_4=0.8722335$。本篇收录这个角谱以完整给出该族的角谱约定；当前随机实现生成横向与竖向位移通道，滚转角通道以点列供给。

### 3.3 谱形性质与导数矩

低角波数端由分母常数项决定。单截止谱 $a_0=0$，按 $\Omega^{-2}$ 增长，其位移方差在下截止趋于零时发散，因此正的 $f_{\min}$ 是这个谱的组成部分。双截止谱与 ERRI 位移谱的 $a_0>0$，低端趋于有限平台

$$
S_\Omega(0)=\frac{b_0}{a_0}.
$$

有限平台表示长波能量有界，具体的下截止仍由工况给定，并且连续 PSD 在零点的值与 realization 中的常量分量是两回事：随机实现只含严格为正的频率。

高角波数端由分母最高次项决定。本篇的全部位移谱与 ERRI 角谱都按 $\Omega^{-4}$ 衰减。对第 $n$ 阶空间导数，其方差矩含 $\int\Omega^{2n}S_\Omega(\Omega)\,d\Omega$：位移与一阶坡度的高频端收敛，二阶导数的高频端发散。因此有限的 $f_{\max}$ 是几何模型的一部分，它规定轨道、轮轨接触与车辆模型接收的最高空间频率。

## 4. 有限频带、采样间距与方差

对 $0<\Omega_{\min}<\Omega_{\max}$，$\Omega_{\min,\max}=2\pi f_{\min,\max}$。单截止谱的有限带连续方差由

$$
\mathcal F(\Omega)=-\frac{1}{\Omega}
-\frac{1}{\Omega_c}\arctan\!\left(\frac{\Omega}{\Omega_c}\right)
$$

给出：

$$
\sigma_{\mathrm{continuous}}^2
=kA\cdot10^{-4}
\left[\mathcal F(\Omega_{\max})-\mathcal F(\Omega_{\min})\right].
$$

双截止谱与 ERRI 位移谱的分母是两个截止因子的乘积。记两个截止角波数中较小者为 $\Omega_1$、较大者为 $\Omega_2$（AAR 轨距谱取 $\Omega_s$ 与 $\Omega_c$，ERRI 位移谱取 $\Omega_r$ 与 $\Omega_c$），则

$$
\frac{b_0}{(\Omega^2+\Omega_1^2)(\Omega^2+\Omega_2^2)}
=\frac{b_0}{\Omega_2^2-\Omega_1^2}
\left[\frac{1}{\Omega^2+\Omega_1^2}-\frac{1}{\Omega^2+\Omega_2^2}\right],
$$

有限带连续方差为

$$
\sigma_{\mathrm{continuous}}^2
=\frac{b_0}{\Omega_2^2-\Omega_1^2}
\left[
\frac{1}{\Omega_1}\arctan\frac{\Omega}{\Omega_1}
-\frac{1}{\Omega_2}\arctan\frac{\Omega}{\Omega_2}
\right]_{\Omega_{\min}}^{\Omega_{\max}}.
$$

ERRI 角谱的分母是三个截止因子的乘积，同样的部分分式分解给出三个反正切项。

降低 $f_{\min}$ 加入长波能量，提高 $f_{\max}$ 加入短波能量，谱公式、有限带和 realization 共同定义一个随机工况。空间输出网格的间距 $\Delta s$ 满足 Nyquist 条件

$$
\Delta s\,f_{\max}<\frac12,
$$

实现采用严格不等式：恰好位于 Nyquist 频率的谐波在网格上退化为幅值取决于相位的交错序列，严格不等式把这种退化排除在外。有限长度 realization 的样本均方值围绕第 5.1 节的离散谱和方差波动，离散方差随频率网格细化趋近连续积分；端部包络再改变全区间统计量。

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

每个分量采用随机相位谐波和

$$
r_{\mathrm{raw}}(s)=\sum_{j=1}^{N}
\sqrt{2S_f(f_j)\Delta f}
\cos\!\left(2\pi f_j\xi+\theta_j\right),
\qquad
\xi=s-s_0,
$$

其中 $s_0$ 是放置起点，$\theta_j$ 在 $[0,2\pi)$ 上均匀分布。对相位取期望时，每个谐波项的均值为零、方差为 $S_f(f_j)\Delta f$；各相位相互独立时，谐波和的均值为零，方差为

$$
\sigma_{\mathrm{discrete}}^2
=\sum_{j=1}^{N}S_f(f_j)\Delta f.
$$

端点与内部频率采用相同的矩形权重，$\sigma_{\mathrm{discrete}}^2$ 随频率网格细化趋近第 4 节的连续积分。相位坐标使用局部站位 $\xi$，保持谱、频率网格和种子不变而整体移动放置区间，得到同一 realization 随起点的平移。

等距频率网格还带来一条恒等式。记谐波和的复形式为 $Z(\xi)=\sum_{j=1}^{N}\sqrt{2S_f(f_j)\Delta f}\,e^{\mathrm i(2\pi f_j\xi+\theta_j)}$，取 $T=1/\Delta f$，由 $f_jT=f_{\min}/\Delta f+(j-1)$ 得 $Z(\xi+T)=e^{\mathrm i2\pi f_{\min}/\Delta f}Z(\xi)$：平移 $T$ 给复谐波和乘上一个常相位因子，$|Z|$ 以 $T$ 为周期。实信号 $r_{\mathrm{raw}}=\operatorname{Re}Z$ 的周期由 $f_{\min}/\Delta f$ 决定：该比值为既约分数 $p/q$ 时周期为 $qT$，为无理数时谐波和是准周期的。

### 5.2 从实现种子到相位

一个无符号 64 位实现种子通过固定的领域分离派生各通道的通道种子：每个通道与一个领域常量结合后经同一个双射混合映射，得到互不相同的通道种子。每个通道以自己的通道种子驱动一条确定性伪随机序列，按频率从低到高逐个产生 $[0,1)$ 上的均匀数 $u_j$，再映射为

$$
\theta_j=2\pi u_j.
$$

这一构造只依赖生成规格与实现种子，相同的规格与种子确定同一组相位。具体的混合映射、领域常量与伪随机引擎在第 8 节给出。

### 5.3 谐振器递推

设相邻站位的相位增量为 $\Delta\alpha_j=2\pi f_j\Delta s$。实现按

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

推进 $c_n=\cos\alpha_{j,n}$ 与 $s_n=\sin\alpha_{j,n}$，在每个站位上避免重新调用三角函数。实数算术下递推与直接谐波式等价；浮点递推每隔固定站位数用解析相位重新锚定，把漂移控制在舍入量级。

### 5.4 多通道相关性

各通道的相位流由不同的通道种子驱动。本篇把各通道相位相互独立作为建模假设，在该假设下任一有限区间上的样本相关系数围绕零波动。谱族只规定各分量的自谱，通道之间的互谱是另一项建模选择：若各通道复用同一组相位，AAR6 的横向与竖向会逐点相同，AAR5 的两者成固定比例 $\sqrt{A_v/A_a}$，这对应完全相干的互谱；本篇选择零互谱。轨距与交叉水平共用一个谱形同样是谱族的建模选择，二者是不同的几何量。需要描述方向间相干时，给定正半定互谱矩阵并做联合谱分解即可在同一框架内生成多通道场。

## 6. 空间包络、插值与轨型几何施加

### 6.1 五次包络

令放置区间为 $[s_0,s_1]$，淡入与淡出长度分别为 $L_{\mathrm{in}}$、$L_{\mathrm{out}}$。五次平滑步进为

$$
q(u)=6u^5-15u^4+10u^3,
\qquad
0\le u\le1,
$$

它满足

$$
q(0)=0,
\quad q(1)=1,
\quad q'(0)=q'(1)=q''(0)=q''(1)=0.
$$

放置包络定义为

$$
w(s)=
\begin{cases}
0, & s\le s_0,\\
q\!\left((s-s_0)/L_{\mathrm{in}}\right),
& s_0<s<s_0+L_{\mathrm{in}},\\
1, & s_0+L_{\mathrm{in}}\le s\le s_1-L_{\mathrm{out}},\\
q\!\left((s_1-s)/L_{\mathrm{out}}\right),
& s_1-L_{\mathrm{out}}<s<s_1,\\
0, & s\ge s_1,
\end{cases}
$$

最终 realization 是

$$
r(s)=w(s)\,r_{\mathrm{raw}}(s).
$$

该构造要求 $s_1>s_0$、$L_{\mathrm{in}}>0$、$L_{\mathrm{out}}>0$ 且 $L_{\mathrm{in}}+L_{\mathrm{out}}\le s_1-s_0$，等号对应两个淡变段在一个满幅点相接。有限谐波和本身光滑，解析乘积 $r(s)$ 在零段、淡变段与满幅段的连接处具有连续的位移、一阶坡度和二阶导数。

### 6.2 插值与定义域

生成点列先乘包络，再裁到 $[s_0,s_1]$。每个分量在自己的站位网格上构造自然三次样条，分量之间的网格相互独立；样条的值与一阶导数即该分量的位移（或角度）与坡度。每个分量的首末节点是它的作用边界，边界之外值与坡度均为零。给定的点列按同样方式插值，它自带的门控就是它的包络。自然边界条件约束样条在首末节点的二阶导数为零；首末节点的一阶导数由离散插值决定，与解析包络的 $q'=0$ 之间存在随网格间距减小的插值误差。

### 6.3 左右轨放置与朝向

四个分量在每根钢轨各自的站位上采样，并共同决定该轨轨型参考点的位置与朝向。记名义轨型原点在轨型系中的位置为 $\boldsymbol p_0$，它由轨距基准与轨底坡确定，见[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md) §2.4。放置约定为先绕轨型系 $x$ 轴旋转滚转角，再沿横向与竖向平移：

$$
\boldsymbol p=R_x(\phi)\,\boldsymbol p_0+\left(0,\;y\pm\frac{g}{2},\;z\right)^{\mathsf T},
$$

轨距项左轨取正号、右轨取负号，使正的 $g$ 把两轨各拉近中心线 $g/2$；正的 $\phi$ 使右轨下沉、左轨抬升。轨型朝向由轨底坡角加 $\phi$ 的滚转、由横向坡度 $y'\pm g'/2$ 构成的偏航以及由竖向坡度 $z'$ 构成的俯仰组成。把轨距项沿旋转后的轨平面分配，与上式的差别是 $\phi\,g/2$ 量级的项；把横向与竖向平移也放到旋转之前，差别还含 $\phi\,y$ 与 $\phi\,z$ 量级的项。这些差别都是两个小量的乘积，对各小扰动联合计阶为二阶。这些位置与朝向如何进入轮轨位姿，见[轮轨位姿归约与不平顺输入](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.md)。

## 7. 理论假设与适用范围

原始随机相位和在无限站位观念下具有由输入 PSD 决定的二阶统计结构；有限频带、有限频率数和有限站位区间把它变成离散近似。乘以放置包络后，场在淡变段不再平稳，谱与方差的平稳解释适用于未加窗的谐波和与满幅平台。

PSD 规定二阶统计量，随机相位、各通道独立、所选谱族以及自然样条重建构成本篇的模型假设。确定性缺陷、非平稳演化与方向间相干各有相应的扩展路径：局部缺陷以确定性几何叠加，非平稳性以沿站位变化的谱参数描述，方向间相干以互谱矩阵引入。

## 8. 数值实现与源码映射

通道种子的混合映射是 `SplitMix64`：

```text
x = x + 0x9E3779B97F4A7C15
x = (x XOR (x >> 30)) * 0xBF58476D1CE4E5B9
x = (x XOR (x >> 27)) * 0x94D049BB133111EB
x = x XOR (x >> 31)
```

所有运算按无符号 64 位模 $2^{64}$ 进行。横向与竖向通道的领域常量分别编码 `LATERAL` 加空字节与 `VERTICAL`：

```text
lateral_seed  = SplitMix64(realization_seed XOR 0x4C41544552414C00)
vertical_seed = SplitMix64(realization_seed XOR 0x564552544943414C)
```

加法、奇数乘法和异或移位都可逆，整个映射是双射。每个通道用独立的 `std::mt19937_64` 引擎，对第 $j$ 个 64 位输出 $x_j$ 取高 53 位形成均匀数，$u_j=(x_j\mathbin{\texttt{>>}}11)\,2^{-53}$。谐振器递推每 256 个站位用解析相位重新锚定。`Smoothstep5` 在 $u>0.5$ 时利用关于 $u=0.5$ 的对称形式求值，减少靠近 1 时多项式的相消。

| 理论对象 | 主要实现 |
|---|---|
| AAR 单截止谱参数、$S_\Omega\to S_f$ 与连续频带方差 | `AarSingleCutoffPsdParametersFor`、`EvaluateAarOneSidedSpatialPsd`、`ContinuousBandVariance`，见 [`aar_track_irregularity_generator.cc`](../../../libs/track_irregularity/src/aar_track_irregularity_generator.cc) |
| ERRI B176 位移谱参数、角波数谱、$S_\Omega\to S_f$ 与连续频带方差 | `ErriB176DisplacementPsdParametersFor`、`EvaluateErriB176AngularWavenumberPsd`、`EvaluateErriB176OneSidedSpatialPsd`、`ContinuousBandVariance`，见 [`erri_b176_track_irregularity_generator.cc`](../../../libs/track_irregularity/src/erri_b176_track_irregularity_generator.cc) |
| 领域分离、种子到相位映射、谐波和与周期重锚 | `DeriveTrackIrregularityChannelSeeds`、`SplitMix64`、`ReproduciblePhaseGenerator`、`AccumulateHarmonicChannel`、`GenerateTrackIrregularityCore`，见 [`track_irregularity_generation.cc`](../../../libs/track_irregularity/src/track_irregularity_generation.cc) |
| 五次包络与横向、竖向通道生成 | `Smoothstep5`、`TrackIrregularityPlacementWeight`、`GenerateAarTrackIrregularity`、`GenerateErriB176TrackIrregularity`；公共类型见 [`track_irregularity_generation.h`](../../../libs/track_irregularity/include/orvd/track_irregularity/track_irregularity_generation.h) |
| 生成点列或给定点列到场 | `ResolveTrackIrregularityField`，见 [`resolve_track_irregularity_field.cc`](../../../libs/configuration/src/resolve_track_irregularity_field.cc)；`LoadTrackIrregularityFieldFromDataRoot`，见 [`load_track_irregularity_field.cc`](../../../libs/configuration/src/load_track_irregularity_field.cc) |
| 四个分量的样条求值与定义域外为零 | `TrackIrregularityField::LateralDisplacementMeters`、`VerticalDisplacementMeters`、`GaugeNarrowingMeters`、`RollRadians` 及对应的坡度函数，见 [`track_irregularity_field.cc`](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) |
| 左右轨放置与朝向 | `ApplyGaugeAndRollIrregularity`，见 [`wheel_rail_pose.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc)；按站位采样与坡度合成在 [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) |
