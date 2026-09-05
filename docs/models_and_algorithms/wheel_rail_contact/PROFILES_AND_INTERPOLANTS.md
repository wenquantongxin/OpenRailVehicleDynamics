[English](PROFILES_AND_INTERPOLANTS.en.md)

# 型面点列与插值

本篇说明 ORVD 轮轨接触层把一条测量型面变成可求值曲面所经过的数学步骤：点列的侧解析、两种分段三次插值、型面弧长求积与反查、车轮型面的等弧长重扫，以及从钢轨型面形状导出的轨距基准。每个公式要么直接对应源码中的计算，要么给出该计算依据的定义；最后以一张紧凑映射表连接理论对象与实现位置。

## 1. 范围

本篇覆盖 `orvd::wheel_rail_contact` 中承载上述数学对象的六个实现单元：`ProfilePoints` 与 `SideResolvedProfile`、`NaturalCubicSpline`、`MonotoneCubicInterpolant`、型面弧长算法、`WheelProfilePreprocessing` 和 `ComputeRailGaugeDatum`。讨论终止于接触几何所消费的三类量：车轮控制节点、建在节点上的曲线表示和每根钢轨的轨距基准；不展开文件格式、配置项或接口清单。

本篇明确不做以下内容。接触几何如何把两条曲面投影、分箱、取上包络、切接触岛与接触斑，属于 [`CONTACT_GEOMETRY.md`](CONTACT_GEOMETRY.md)；轮对相对钢轨的位姿归约与 `WheelRailPoseConstants` 如何消费轨距基准，属于 [`WHEEL_RAIL_POSE_REDUCTION.md`](WHEEL_RAIL_POSE_REDUCTION.md)；法向力、蠕滑率、Kalker 系数与 FASTSIM 各有自己的文档。轨道不平顺序列的插值也不属于型面，见 [`TRACK_IRREGULARITY_SPECTRA.md`](../track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md)。

实现中的理论数据流是：作者点列先解析到物理侧；车轮点列可沿第一条自然样条按等弧长重扫，钢轨点列则直接侧解析；两组节点各形成一条 `NaturalCubicSpline`，并以该样条的节点斜率形成一条 `MonotoneCubicInterpolant`。钢轨作者点列另经滚转和规定深度截线得到 `RailGaugeDatum`。接触几何此后消费这些派生表示，而不再直接查询作者点列。

## 2. 记号

坐标系、左右侧与正号沿用[坐标与记号约定](../CONVENTIONS_AND_NOTATION.md)第 2.5 节：型面坐标是型面自身坐标系中的米，横向跨越轨道，竖向向下为正；`WheelSide::kRight` 保留资产书写的符号，`WheelSide::kLeft` 是镜像。本篇新增的记号如下。

| 记号 | 含义 | 代码对应 |
|---|---|---|
| $y$、$z$ | 型面横坐标与竖坐标，单位 m，竖向向下为正 | `lateral_meters`、`vertical_meters` |
| $n$，$y_0\lt y_1\lt\cdots\lt y_{n-1}$，$z_i$ | 节点个数、严格递增的节点与节点值 | `knots`、`values` |
| $h_i=y_{i+1}-y_i$ | 第 $i$ 个节点区间的宽度 | `spacings` |
| $\delta_i=(z_{i+1}-z_i)/h_i$ | 第 $i$ 个区间的割线斜率 | `secants` |
| $m_i$ | 节点一阶导，即节点斜率 | `nodal_slopes` |
| $M_i$ | 节点二阶导，即矩 | `moments` |
| $t=(y-y_i)/h_i$ | 段内局部参数 | `local_parameter` |
| $\varsigma$ | 侧号，右侧 $+1$、左侧 $-1$ | `ResolveForSide` 中的 `sign` |
| $\sigma(a,b)$、$\Sigma_j$、$L$ | 区间弧长、各节点处的累积弧长、总弧长 | `IntegrateProfileArcLength`、`cumulative`、`total` |
| $\Delta\sigma$ | 等弧长重扫步长 | `equal_arc_length_rescan_step_meters` |
| $G$、$d$、$\gamma$、$\rho$ | 轨距、轨距测量深度、轨底坡幅值、按侧带符号的钢轨滚转 | `track_gauge_meters`、`gauge_measuring_depth_meters`、`rail_cant_radians`、`roll_radians` |

## 3. 模型

### 3.1 点列、角色与侧解析

型面就是点列。`ProfilePoints` 持有作者写下的点列，顺序与符号完全保留，连同角色 `ProfileRole`（`kWheel` 或 `kRail`）和标识符。角色决定可施加的几何运算：等弧长预处理定义在车轮型面上，轨距基准定义在钢轨型面上，接触几何也区分轮面与轨面。型面本身不携带站位、左右侧或车辆信息。

可用点列满足：两列长度相同、至少有两个有限点、标识符非空，并且横坐标沿作者顺序严格朝一个方向单调，升序或降序均可。不能先把折返点列排序后再解释，因为排序会把原来不同的几何折线改造成另一条曲面；重复横坐标也使 $z(y)$ 不再单值。这一前提意味着模型不能表达横向有倒扣的形状（见第 6 节）。

`ResolveForSide(side)` 产生 `SideResolvedProfile`：按有符号横坐标升序排列的同一组点。以 $\pi$ 记按 $\varsigma\,y$ 升序的排列，

$$
y^{(\varsigma)}_i=\varsigma\,y_{\pi(i)},\qquad z^{(\varsigma)}_i=z_{\pi(i)},\qquad \varsigma=\begin{cases}+1,&\texttt{kRight}\\ -1,&\texttt{kLeft}\end{cases}
$$

对应 `ResolveForSide` 中 `sign * lateral_meters_[order[index]]`。右侧只可能改变顺序，不改变任何符号；左侧把横坐标取反后再升序。轨型系横轴指向右侧是这一约定的依据。本模块的每一个插值器都建在侧解析点列上，从不建在作者点列上，因为插值器需要严格递增的横坐标，而作者点列不被要求具备。

车轮预处理只在物理右侧重扫，再把结果镜像到左侧（第 3.5 节）。原因是网格相位：等弧长重扫从点列第一点起步、以整数倍步长布点，不能整除步长的余量落在最后一个区间；若对已镜像的左侧点列独立重扫，余量区间会移到左型面的另一端，左右两侧就不再共享同一网格相位。

### 3.2 自然三次样条

`NaturalCubicSpline` 是型面曲面的基础表示。每一段是局部参数 $t=(y-y_i)/h_i$ 上的三次多项式，以缩放 Hermite 形式存储：

$$
p_i(t)=c_{0,i}+c_{1,i}\,t+c_{2,i}\,t^{2}+c_{3,i}\,t^{3},\qquad t=\frac{y-y_i}{h_i}\in[0,1]
$$

$$
\begin{aligned}
c_{0,i}&=z_i,\\
c_{1,i}&=h_i\,m_i,\\
c_{2,i}&=-3z_i+3z_{i+1}-2h_i\,m_i-h_i\,m_{i+1},\\
c_{3,i}&=2z_i-2z_{i+1}+h_i\,m_i+h_i\,m_{i+1}
\end{aligned}
$$

四个系数依次是 `SegmentCoefficients` 的 `constant`、`linear`、`quadratic`、`cubic`。一阶与二阶导由同一组系数给出，对应 `EvaluateFirstDerivative` 与 `EvaluateSecondDerivative`：

$$
z'(y)=\frac{1}{h_i}\left(c_{1,i}+2c_{2,i}\,t+3c_{3,i}\,t^{2}\right),\qquad
z''(y)=\frac{1}{h_i^{2}}\left(2c_{2,i}+6c_{3,i}\,t\right)
$$

节点斜率 $m_i$ 由两条构造路径之一求出。在严格等距的数据上二者解析等价；若数据只是落入下式的有限精度等距判据，等距路径仍以 $h_0$ 代替全部真实间距，两条路径便不再解完全相同的插值问题。路径由 `SpacingsAreUniform` 根据数据决定：

$$
\left|h_i-h_0\right|\le\tau_{\mathrm{abs}}+\tau_{\mathrm{rel}}\,|h_0|\qquad\forall\,i
$$

其中 $\tau_{\mathrm{abs}}$ 与 $\tau_{\mathrm{rel}}$ 分别是绝对和相对容差。通过检测的网格在理想化网格 $y_0+i\,h_0$ 上构造和定位；真实 $y_i$ 与理想节点之间的偏差因此成为该快路径的建模近似，而不仅是存储细节。

等距路径（`SolveNodalSlopesUniform`）直接对斜率解一个与间距无关的三对角系统，自然边界条件以斜率表达，全程不出现矩：

$$
\begin{aligned}
2m_0+m_1&=\frac{3}{h}\,(z_1-z_0),\\
m_{i-1}+4m_i+m_{i+1}&=\frac{3}{h}\,(z_{i+1}-z_{i-1}),\qquad 1\le i\le n-2,\\
m_{n-2}+2m_{n-1}&=\frac{3}{h}\,(z_{n-1}-z_{n-2})
\end{aligned}
$$

$n=2$ 时两端斜率都取割线。系统的对角元不依赖间距，避免了把很小的 $h$ 直接放入矩系统主对角线。

一般路径（`SolveNodalSlopesGeneral`）解经典的内点矩系统，两端矩不进入系统而直接为零，这使自然边界条件精确成立而不是近似成立：

$$
h_{i-1}M_{i-1}+2\,(h_{i-1}+h_i)\,M_i+h_iM_{i+1}=6\,(\delta_i-\delta_{i-1}),\qquad 1\le i\le n-2,\qquad M_0=M_{n-1}=0
$$

再由矩得到段形式需要的斜率：

$$
m_i=\delta_i-\frac{h_i}{6}\,(2M_i+M_{i+1}),\quad 0\le i\le n-2,\qquad
m_{n-1}=\delta_{n-2}+\frac{h_{n-2}}{6}\,(M_{n-2}+2M_{n-1})
$$

节点范围之外的行为是刻意选择的，不是多项式延拓。型面在最后一个测量点之外没有意义，把三次多项式延伸过轮缘根部会产生一个看似合理、实际不存在的曲面。因此：

$$
z(y)=\begin{cases}z_0,&y\le y_0\\ z_{n-1},&y\ge y_{n-1}\end{cases},\qquad
z'(y)=0\quad(y\lt y_0\ \lor\ y\gt y_{n-1}),\qquad
z''(y)=0\quad(y\le y_0\ \lor\ y\ge y_{n-1})
$$

值在端外平延；一阶导只在严格端外为零，在边界节点本身取内侧单侧斜率，因此当该斜率非零时，一阶导在相应端点产生阶跃；二阶导在边界节点上及端外精确为零，这是把自然边界条件做成精确值而不是留下舍入噪声。三个判据的严格与非严格之别是实现的合同，不是疏忽。

`nodal_slopes()` 给出构造所得的节点斜率，供另一个 Hermite 表示在相同节点上复用。节点值、节点斜率和区间宽度共同唯一确定每段三次多项式；第 3.3 节区分“理想分段多项式相同”和“有限精度公开求值逐点相同”这两个层次。

### 3.3 保形三次插值

`MonotoneCubicInterpolant` 与自然三次样条使用同一种缩放 Hermite 段形式（第 3.2 节的四个系数公式相同），区别在节点斜率的来源和端外导数规则。以 `FromValues` 的限斜率规则构造时，它通常只有 $C^1$ 而不是 $C^2$，但每个单调数据区间保持单调且不越出两端节点值张成的区间。这个保形结论也适用于外部斜率已经满足相应单调 Hermite 条件的 `FromNodalSlopes` 输入，却不适用于任意有限斜率；后者可能过冲。这里的取舍针对轮缘根部近乎不连续的斜率变化：$C^2$ 插值可能以值幅很小而导数幅很大的纹波响应，而导数直接决定接触角。

`FromValues` 由值本身导出节点斜率，规则是 `ComputeShapePreservingNodalSlopes`。内点：

$$
m_i=\begin{cases}
0,&\delta_{i-1}\,\delta_i\le 0\\
\dfrac{w_{\mathrm L}+w_{\mathrm R}}{\dfrac{w_{\mathrm L}}{\delta_{i-1}}+\dfrac{w_{\mathrm R}}{\delta_i}},&\text{otherwise}
\end{cases},\qquad
w_{\mathrm L}=2h_i+h_{i-1},\quad w_{\mathrm R}=h_i+2h_{i-1}
$$

相邻两条割线异号或任一为零时斜率置零，这是保持平段为平、保持单调段不反向的机制；其余情形是加权调和平均，对应源码中 `left_weight`、`right_weight` 与 `(left_weight + right_weight) / (left_weight / left_secant + right_weight / right_secant)`。源码注释强调每个权重上加倍的区间是它所除割线远侧的那个区间，配反了在等距网格上不可见、在其他任何网格上都错。端点用一侧三点公式并作两级限幅，对应 `EndpointSlope`：

$$
\tilde m_0=\frac{(2h_0+h_1)\,\delta_0-h_0\,\delta_1}{h_0+h_1},\qquad
m_0=\begin{cases}0,&\tilde m_0\,\delta_0\le 0\\ 3\delta_0,&\delta_0\,\delta_1\lt 0\ \land\ |\tilde m_0|\gt 3|\delta_0|\\ \tilde m_0,&\text{otherwise}\end{cases}
$$

右端以 $(h_{n-2},h_{n-3},\delta_{n-2},\delta_{n-3})$ 代入同一规则；$n=2$ 时两端斜率都取割线。第二级限幅是防止端段像无限幅的一侧公式那样过冲的机制。源码未给这一斜率规则冠以方法名，本篇不补。

`FromNodalSlopes` 直接使用给定斜率，保形性由这些斜率负责。在精确算术中，若施主与受主使用同一组节点、节点值和区间宽度，那么两端值与两端斜率唯一确定同一个 Hermite 三次段，两者的**理想分段多项式**相同。施主自然样条走等距路径时每段使用理想宽度 $h_0$，而 `MonotoneCubicInterpolant` 使用真实 $h_i$，所以此时连段多项式本身也一般不同。

即使理想分段多项式相同，两种公开求值器也不保证对每个浮点输入逐位相同。`NaturalCubicSpline::Locate` 会把距节点不超过 `kLocalParameterSnap` 的局部参数吸到节点；`MonotoneCubicInterpolant::Locate` 只把局部参数夹到 $[0,1]$。因此节点邻域内可能选取不同的 $t$，右端值的求值路径也不同。接触几何中的 `SurfaceThroughNodes` 仍有意先建 `NaturalCubicSpline`，再以 `spline.nodal_slopes()` 和 `OutsideDerivativeRule::kZeroOutsideKnots` 建 `MonotoneCubicInterpolant`：后者无条件共享前者的节点导数；只有实际节点与区间宽度也一致时，两者才共享理想段形状；两种表示无论如何都不能被描述为有限精度下逐点同一函数。`wheel_spline_` 用于轮廓值和斜率，`wheel_surface_` 则用于接触处的局部轮径与曲率。

端外导数有两条规则，由构造参数 `OutsideDerivativeRule` 选择。默认 `kHoldEndpointDerivative` 在端外保持端点导数，与平延的值刻意不一致，这是消费者外推一个它知道会继续的曲面时想要的读数；`kZeroOutsideKnots` 给出一致的读数：一阶导在严格端外为零，二阶导在边界节点上及端外为零。两个判据一严格一非严格，与自然样条相同，理由也相同。值在两条规则下都平延。

### 3.4 型面弧长

型面曲线是样条 $z(y)$ 的图像，其弧长是

$$
\sigma(a,b)=\int_a^b\sqrt{1+z'(y)^2}\,dy
$$

被积函数在样条段内光滑，跨节点处失去高阶光滑性，这决定了整个设计：求积规则逐节点区间应用，从不跨越节点。一条规则铺满整个型面会在每个节点处损失大部分精度，更重要的是，累积表与站点搜索必须以同一种方式积分，否则它们认可的站点会彼此漂移。`IntegrateProfileArcLength(profile, from, to)` 在单个区间上用 16 点 Gauss–Legendre 规则，规则对称，八个正节点各在区间中点两侧求值一次：

$$
\sigma(a,b)\approx\eta\sum_{k=1}^{8}w_k\left[\sqrt{1+z'(c-\eta\,\xi_k)^2}+\sqrt{1+z'(c+\eta\,\xi_k)^2}\right],\qquad c=\frac{a+b}{2},\quad \eta=\frac{b-a}{2}
$$

对应 `half_width * sum`，其中每一项用 `std::hypot(1.0, profile.EvaluateFirstDerivative(...))` 形成。区间为空或反向（`!(to > from)`）时返回零；函数不知道节点在哪里，不跨节点是调用方的责任。累积表由 `AccumulateProfileArcLength` 逐区间形成，$\Sigma_0=0$，$\Sigma_{j+1}=\Sigma_j+\sigma(y_j,y_{j+1})$。反问题"从首节点起弧长达到 $\sigma^\star$ 处的横坐标"由 `FindLateralCoordinateAtArcLength` 回答，它把答案括在一个节点区间内并对该括区做固定次数的二分（第 4.4 节）。

### 3.5 车轮型面预处理

`WheelProfilePreprocessing` 把作者写下的车轮型面变成接触几何采样的轮廓。步长为零时保留作者节点；步长为正时以沿第一条自然三次样条按恒定弧长布下的节点替换作者节点，再通过这些替换节点建第二条自然三次样条。型面转折急的地方等弧节点会聚拢，因此轮廓在轮缘根部以与踏面相同的曲线长度密度解析，而不是以资产作者恰好写下的密度解析。

重扫总在物理右侧顺序上进行，镜像作用于其结果：在已镜像点列上重扫会把余量区间、从而整个网格的相位移到左型面的另一端（第 3.1 节）。左侧节点由右侧节点反序并取反横坐标得到：

$$
y^{\mathrm L}_j=-\,y^{\mathrm R}_{N-1-j},\qquad z^{\mathrm L}_j=z^{\mathrm R}_{N-1-j},\qquad 0\le j\le N-1
$$

节点布置的目标弧长是步长的整数倍 $\sigma_k=k\,\Delta\sigma$，$k=1,2,\dots$，只要 $\sigma_k\lt L-\tau_{\mathrm{end}}$ 就布一个节点，$\tau_{\mathrm{end}}$ 是 `RescanEndTolerance` 给出的终止容差。首末两个节点直接复制物理端点而不参加弧长反查，由此保留资产声明的支持域端点；样条公开求值在两端也直接返回所存节点值。

在重扫节点上重新构造第二条自然样条，得到接触几何实际消费的连续轮廓。它通过重扫节点，但一般不再通过作者的中间节点；因此等弧长重扫是一次重新离散和重新插值，不是原样条的无损重参数化。

### 3.6 轨距基准

轨距在两条钢轨的轨距面之间、在轨顶以下规定深度处测量，不在轨顶之间测量。因此钢轨型面原点的横向位置不是简单的半轨距，而是半轨距加上轨距面到型面原点的距离；该距离由钢轨型面、测量深度与轨底坡共同决定。

按侧带符号的滚转是 $\rho=-\varsigma\,\gamma$：正轨底坡下右轨向轨道中心倾斜，在横轴向右、竖轴向下的系中这是绕前进轴的负滚转，左轨是其镜像。作者点列先绕原点滚转：

$$
\begin{bmatrix}y'_i\\ z'_i\end{bmatrix}=\begin{bmatrix}\cos\rho&-\sin\rho\\ \sin\rho&\cos\rho\end{bmatrix}\begin{bmatrix}y_i\\ z_i\end{bmatrix}
$$

对应 `rolled_lateral` 与 `rolled_vertical`。轨顶是滚转后竖坐标的最小值（竖轴向下），测量水平线是 $z_{\mathrm{level}}=\min_i z'_i+d$。构造刻意是型面折线上的分段线性，而不建在任何插值器上：轨距面是施加于测量点的测量约定，若经插值，答案会依赖于恰好选了哪种插值器。把点按滚转后横坐标排序，对相邻两点 $(a,b)$，当 $(z'_a-z_{\mathrm{level}})(z'_b-z_{\mathrm{level}})\le 0$ 时记录一个交点：

$$
y_\times=\begin{cases}\dfrac{y'_a+y'_b}{2},&z'_a=z'_b\\ y'_a+\dfrac{(z_{\mathrm{level}}-z'_a)\,(y'_b-y'_a)}{z'_b-z'_a},&\text{otherwise}\end{cases}
$$

恰好落在测量水平线上的线段贡献其中点。轨距面是面向轨道中心的那个交点：右轨取交点横坐标的最小值，左轨取最大值。三个输出为

$$
\text{offset}=-\varsigma\,y_{\mathrm{face}},\qquad
y_{\mathrm{datum}}=\varsigma\left(\frac{G}{2}+\text{offset}\right),\qquad
\rho=-\varsigma\,\gamma
$$

依次是 `gauge_face_offset_meters`、`lateral_datum_meters`、`roll_radians`。该构造要求 $G>0$、$d>0$，并要求滚转后的折线与测量水平线至少相交一次。左右滚转总满足 $\rho_{\mathrm L}=-\rho_{\mathrm R}$；但左右偏移相等、横向基准互为相反数并不是任意作者轨型的不变量。只有当作者轨型关于 $y=0$ 镜像对称（更一般地，两次相反滚转后的选定轨距面交点满足相应镜像关系）时，才有

$$
\operatorname{offset}_{\mathrm L}=\operatorname{offset}_{\mathrm R},\qquad
y_{\mathrm{datum,L}}=-y_{\mathrm{datum,R}}.
$$

## 4. 算法

### 4.1 侧解析

作者点列的数学前提是两列等长、至少含两个有限点，且横坐标沿作者顺序严格单调。`ResolveForSide` 建一个索引数组，以 $\varsigma y$ 为键排序后按序取出；横坐标两两不同，因此结果唯一。该过程复杂度为 $O(n\log n)$，输出横坐标严格递增，供后续所有插值器使用。

### 4.2 样条构造与求值

样条构造先形成正间距 $h_i$，再检测等距、求节点斜率并形成每段系数。一般路径的 `SolveTridiagonal` 使用不选主元的 Thomas 消去，并以固定主元阈值排除数值奇异的消元。这个阈值是绝对量，因此极小间距的一般网格可能受其影响，而等距路径的无量纲主对角系统不受同样缩放影响。等距路径的系统规模为 $n$，一般矩路径为 $n-2$；$n=2$ 时无需解系统。两条构造路径均为 $O(n)$。

`Locate` 把横坐标映射到段号与局部参数。等距路径对 $(y-y_0)/h_0$ 取 `std::floor`，为 $O(1)$；一般路径用 `std::upper_bound` 在节点中二分，为 $O(\log n)$。随后对局部参数作吸附：$t\le$ `kLocalParameterSnap` 归到本段起点 $t=0$，$t\ge 1-$ `kLocalParameterSnap` 归到下一段起点（最后一段取 $t=1$）。一般路径由此在节点处稳定返回表值；等距路径的定位基于理想化网格，只有真实节点相对理想节点的局部偏差不超过吸附阈值时才有同一结论。值在端外平延；一阶导在严格端外为零而在边界取内侧单侧斜率；二阶导在边界及端外为零。

端外平延可能在两个边界节点引入非光滑点：当内侧单侧斜率非零时，一阶导在相应边界跳到端外的零；若该斜率为零，则一阶导连续。二阶导在边界返回零与自然边界条件一致；一般路径直接令 $M_0=M_{n-1}=0$，等距路径的两条边界方程在精确算术下等价。两条构造路径在严格等距数据上解析等价，但定位表达式和舍入序列不同，有限精度结果仍可能不同。

### 4.3 保形斜率与段定位

`ComputeShapePreservingNodalSlopes` 先形成全部 $h_i$ 与 $\delta_i$，再按第 3.3 节的一侧端点公式和加权调和平均求 $m_i$，总复杂度为 $O(n)$。接触几何对已经满足有限、严格递增等前提的临时节点复用同一数学内核；是否重复执行前置检查不改变斜率公式。

`MonotoneCubicInterpolant::Locate` 对真实节点恒用二分搜索，不采用等距快路径，也不使用节点吸附容差。局部参数只按

$$
t=\operatorname{clamp}\!\left(\frac{y-y_i}{h_i},0,1\right)
$$

限制在段内。左端取首段 $t=0$，右端取末段 $t=1$；因此左端值按 $c_0=z_0$ 形成，右端值按 $c_0+c_1+c_2+c_3$ 形成。后者在精确算术中等于 $z_{n-1}$，但浮点求和不承诺与存储端值逐位一致。段定位为 $O(\log n)$。

### 4.4 弧长求积与反查

`IntegrateProfileArcLength` 的 16 点规则以八个正节点与权重存储，节点关于区间中点对称使用，全部权重之和为二是读者可对表核对的恒等式。两个数组 `kGaussLegendreAbscissae` 与 `kGaussLegendreWeights` 的字面量如下：

| $k$ | $\xi_k$ | $w_k$ |
|---|---|---|
| 1 | `0.095012509837637440185319335424958` | `0.189450610455068496285396723208283` |
| 2 | `0.281603550779258913230460501460496` | `0.182603415044923588866763667969219` |
| 3 | `0.458016777657227386342419442983577` | `0.169156519395002538189312079030359` |
| 4 | `0.617876244402643748446671764048791` | `0.149595988816576732081501730547479` |
| 5 | `0.755404408355003033895101194847442` | `0.124628971255533872052476282192017` |
| 6 | `0.865631202387831743880467897712393` | `0.095158511682492784809925107602246` |
| 7 | `0.944575023073232576077988415534608` | `0.062253523938647892862843836994378` |
| 8 | `0.989400934991649932596154173450333` | `0.027152459411754094851780572456018` |

`AccumulateProfileArcLength` 对每个严格递增的节点区间调用一次单区间积分，共需 $O(16\,n)$ 次导数求值。

给定 $\sigma^\star\in[\Sigma_0,\Sigma_{n-1}]$，`FindLateralCoordinateAtArcLength` 用 `std::upper_bound` 找到唯一的累积弧长区间 $j$，令局部目标为 $\sigma^\star-\Sigma_j$，再在 $[y_j,y_{j+1}]$ 上作固定 64 次二分。每次比较 $\sigma(y_j,y_{\mathrm{mid}})$ 与局部目标，最后返回括区中点。由于 $\sqrt{1+z'^2}>0$，弧长关于 $y$ 严格递增，根在该区间内唯一；每次积分都留在同一个样条段内。固定次数使离散求逆成为输入的确定函数，但返回值仍受求积和浮点舍入影响，不应称为解析精确根。

### 4.5 等弧长重扫

令重扫步长 $\Delta\sigma\ge0$。当 $\Delta\sigma=0$ 时，接触几何直接采用侧解析的作者节点；当 $\Delta\sigma>0$ 时，`LayControlNodes` 执行以下流程：

```
physical = authored.ResolveForSide(kRight)
first_pass = NaturalCubicSpline(physical.y, physical.z)
cumulative = AccumulateProfileArcLength(first_pass, physical.y)
L = cumulative.back()
tol = 16 * eps * max(1, |L|, |step|)
nodes = [(physical.y.front, physical.z.front)]
for k = 1, 2, ...:
    target = k * step
    if target >= L - tol: break
    y_k = FindLateralCoordinateAtArcLength(first_pass, physical.y, cumulative, target)
    nodes.append((y_k, first_pass.Evaluate(y_k)))
nodes.append((physical.y.back, physical.z.back))        // copied, not re-evaluated
if side == kLeft: nodes = reverse(nodes) with y -> -y
```

终止容差 `RescanEndTolerance(total, step)` 为 $16\,\varepsilon\max(1,|L|,|\Delta\sigma|)$；它防止总弧长恰为步长整数倍时再生成一个与物理端点几乎重合的节点。最后一个区间是余量：循环在 $k\Delta\sigma\ge L-\tau_{\mathrm{end}}$ 处停止，所以其弧长不超过 $\Delta\sigma+\tau_{\mathrm{end}}$。节点数约为 $L/\Delta\sigma$，每个内部节点需要一次固定迭代弧长反查。

### 4.6 轨距基准的构造

`ComputeRailGaugeDatum` 先按侧形成 $\rho$，滚转作者点列，并按滚转后的横坐标排序；再以最小竖坐标加 $d$ 形成测量水平线，扫描相邻点对的交点，最后按侧选取面向轨道中心的极值交点。水平线与一段重合时取该段中点，避免零分母。排序占 $O(n\log n)$，随后扫描为 $O(n)$。这里得到的是滚转后、按横坐标重排的折线截面；若滚转使原作者邻接关系改变，它并不等同于原折线整体刚性旋转后的拓扑连接。

## 5. 实现映射

下表只定位承载核心数学关系的实现，不作为接口或配置参考。

| 理论对象或算法 | 主要实现 | 源文件 |
|---|---|---|
| 作者点列与左右侧解析 | `ProfilePoints`、`SideResolvedProfile` | [`profile_points.cc`](../../../libs/wheel_rail_contact/src/profile_points.cc) |
| 自然三次样条、等距判定与节点吸附 | `NaturalCubicSpline`、`SpacingsAreUniform`、`Locate` | [`natural_cubic_spline.cc`](../../../libs/wheel_rail_contact/src/natural_cubic_spline.cc) |
| 保形斜率与 Hermite 插值 | `ComputeShapePreservingNodalSlopes`、`MonotoneCubicInterpolant` | [`monotone_cubic_interpolant.cc`](../../../libs/wheel_rail_contact/src/monotone_cubic_interpolant.cc) |
| 单段弧长、累计弧长与反查 | `IntegrateProfileArcLength`、`AccumulateProfileArcLength`、`FindLateralCoordinateAtArcLength` | [`profile_arc_length.cc`](../../../libs/wheel_rail_contact/src/profile_arc_length.cc) |
| 等弧长重扫与左右镜像 | `WheelProfilePreprocessing::LayControlNodes` | [`wheel_profile_preprocessing.cc`](../../../libs/wheel_rail_contact/src/wheel_profile_preprocessing.cc) |
| 滚转折线上的轨距面与轨距基准 | `ComputeRailGaugeDatum` | [`rail_gauge_datum.cc`](../../../libs/wheel_rail_contact/src/rail_gauge_datum.cc) |
| 接触几何对样条与 Hermite 曲面的组合 | `SurfaceThroughNodes`、`ResolveWheelNodes` | [`contact_geometry.cc`](../../../libs/wheel_rail_contact/src/contact_geometry.cc) |

## 6. 理论假设与适用范围

- 型面被表示为单值图像 $z(y)$，作者横坐标必须严格单调。横向有倒扣、同一 $y$ 对应多个 $z$ 的轨头或轮缘形状不能由一条本模型型面表达。
- 自然三次样条在内部为 $C^2$，采用零端矩；测量跨度之外的常值延拓使一阶导通常在两个边界跳变。端外延拓不代表存在额外材料。
- 等距快路径把落入容差的网格理想化为 $y_0+i h_0$。真实节点偏离理想网格时，这是一项有限精度几何近似；节点吸附也使它与一般路径的逐点浮点函数不同。
- `FromValues` 的斜率规则产生保形的分段 Hermite 曲线。`FromNodalSlopes` 只有在给定斜率满足相应单调 Hermite 条件时才有同一性质；任意有限斜率可能过冲。
- 借用自然样条节点斜率时，相同节点、值和区间宽度只保证精确算术中的分段多项式相同。节点吸附、局部参数夹制、右端求值顺序以及等距网格理想化都可能使公开求值结果不逐位相同。
- 弧长由每个样条段上的固定 16 点 Gauss–Legendre 规则近似，反查由固定 64 次二分近似。由于被积函数严格为正，弧长映射严格递增，反问题在给定节点区间内唯一；这并没有把求积结果变成解析弧长。
- 等弧长重扫沿第一条样条选节点，再由这些节点构造第二条样条；第二条样条一般不通过作者中间点，也不与第一条样条完全相同。
- 轨距基准以滚转后按横坐标排序的测量折线和规定深度截线定义，不以自然样条或保形插值定义。左右滚转角必然反号；左右偏移相等和基准互为相反数还需要作者轨型的镜像对称或等价的交点对称条件。
