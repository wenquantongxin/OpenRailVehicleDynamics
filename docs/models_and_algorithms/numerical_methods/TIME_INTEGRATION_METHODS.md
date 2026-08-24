# BDF、Radau5、Newmark 与 Zhai 时间积分方法

本文说明 ORVD 当前使用或计划研究的四类时间积分方法，重点记录它们从连续方程形成一个数值步的
方式、误差与稳定性特征，以及放入轨道车辆动力学模型时必须保留的边界。本文不是实现进度表；
Radau5 的实施顺序见[时间积分器迁移路书](../../planning/integrator_migration/INTEGRATOR_MIGRATION_ROADMAP.md)。

本文所称 **Radau5** 专指三阶段、五阶 Radau IIA 方法。仓库历史材料中的 **Radau3** 目前只冻结了
名称与固定步语境，其来源和 Butcher 表仍待专项审计；不得用本文的 Radau5 身份静默回填或替代它。
本文所称 **Zhai 方法** 专指 1996 年论文中的简单显式二步法，不把同一论文提出的
predictor-corrector 家族混入这一名称。

## 1. 方程形态与共同记号

### 1.1 一阶状态方程

BDF 和 Radau5 直接求解一阶初值问题：

\[
\dot y=f(t,y),
\qquad
y(t_0)=y_0,
\qquad
y\in\mathbb R^N.
\]

ORVD 的连续状态不是单一位移向量，而是：

\[
y=
\begin{bmatrix}
q\\v\\z
\end{bmatrix},
\qquad
\dot y=
\begin{bmatrix}
N(q)v\\a(t,q,v,z)\\g(t,q,v,z)
\end{bmatrix}.
\]

其中 \(q\) 是广义位置，\(v\) 是广义速度，\(z\) 是 Maxwell 串联弹簧黏性阻尼器等一阶内变量。
自由体采用四元数时 `q` 与 `v` 不等维，且 \(\dot q=N(q)v\)；因此不能把整个状态静默改写为
`qdot=v`。当前映射合同见
[multibody_model.h](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h)，Maxwell 力状态满足：

\[
\dot F=K v_{\mathrm{rel}}-\frac{K}{C}F,
\]

实现见 [vehicle_force_plan.cc](../../../libs/forces/src/vehicle_force_plan.cc)。

### 1.2 二阶机械方程

经典 Newmark 和 Zhai 方法通常从以下二阶形式出发：

\[
M(q)\,a+C(q,v)\,v+f_{\mathrm{int}}(q,v,z)=p(t),
\qquad
\dot q=v.
\]

这一写法天然适合欧氏位移坐标且位移、速度、加速度同维的结构模型。一般多体系统需要把
`q` 的更新改成位形流形上的 retraction，并另行规定 \(z\) 的离散方程。否则得到的只是把某个
一阶方法套到完整状态上，不能继续称为经典 Newmark 或 Zhai 方法。

### 1.3 误差尺度

对不同量纲的状态分量，常用相对容差和逐分量绝对容差形成权重：

\[
w_i=\frac{1}{\operatorname{rtol}|y_i|+\operatorname{atol}_i},
\qquad
\lVert e\rVert_{\mathrm{WRMS}}
=\sqrt{\frac{1}{N}\sum_{i=1}^N(w_i e_i)^2}.
\]

自适应积分器通常在该类加权范数中接受或拒绝试步。不同积分器的误差估计器并不相同；相同
`rtol/atol` 不能自动解释为相同全局误差。

## 2. BDF：隐式线性多步法

### 2.1 离散公式

\(k\) 阶 BDF 用当前未知状态和若干历史状态逼近当前导数。等步长时可写为：

\[
\sum_{j=0}^{k}\alpha_j y_{n+1-j}
=h f(t_{n+1},y_{n+1}).
\]

BDF1–BDF5 的等步长公式是：

\[
\begin{aligned}
\text{BDF1:}\quad
&y_{n+1}-y_n=h f_{n+1},\\
\text{BDF2:}\quad
&\frac{3}{2}y_{n+1}-2y_n+\frac{1}{2}y_{n-1}=h f_{n+1},\\
\text{BDF3:}\quad
&\frac{11}{6}y_{n+1}-3y_n+\frac{3}{2}y_{n-1}-\frac{1}{3}y_{n-2}
=h f_{n+1},\\
\text{BDF4:}\quad
&\frac{25}{12}y_{n+1}-4y_n+3y_{n-1}
-\frac{4}{3}y_{n-2}+\frac{1}{4}y_{n-3}
=h f_{n+1},\\
\text{BDF5:}\quad
&\frac{137}{60}y_{n+1}-5y_n+5y_{n-1}
-\frac{10}{3}y_{n-2}+\frac{5}{4}y_{n-3}-\frac{1}{5}y_{n-4}
=h f_{n+1}.
\end{aligned}
\]

这些系数只适用于固定等步长。CVODE 使用 fixed-leading-coefficient 形式，并按最近步长历史重新
形成变步长系数；“最大二阶”或“最大五阶”不表示求解过程中始终保持该阶数。

### 2.2 一个 BDF 步怎样计算

把历史项收集后，当前步可归一化为非线性残差：

\[
R(y_{n+1})
=y_{n+1}-\gamma f(t_{n+1},y_{n+1})-a_n=0,
\]

其中 \(a_n\) 只含已知历史，\(\gamma\) 同当前步长和 BDF 系数有关。Newton 或 modified Newton
迭代求解：

\[
\left(I-\gamma J\right)\delta=-R,
\qquad
J=\frac{\partial f}{\partial y},
\qquad
y^{(m+1)}=y^{(m)}+\delta.
\]

一个完整自适应步骤通常包括：

1. 从历史多项式预测 \(y_{n+1}\)；
2. 求解上述非线性方程；
3. 用预测—校正差或等价量估计局部截断误差；
4. 误差合格时接受端点并更新历史，否则缩步重试；
5. 在允许范围内选择下一步步长和阶数；
6. 从历史多项式提供最近内部步上的稠密输出。

### 2.3 稳定性与使用特征

- 对足够光滑的问题，\(k\) 阶 BDF 的单步局部截断误差为 \(O(h^{k+1})\)，累计全局误差为
  \(O(h^k)\)。变步长、启动、降阶和非光滑点会改变实际观察到的阶数。
- BDF1、BDF2 对线性测试方程是 A-stable；BDF3–BDF5 不再 A-stable，但仍面向刚性问题使用。
  后三者的稳定域不覆盖整个左半平面，靠近虚轴的弱阻尼车辆模态可能引入额外步长限制。
- 每步只在新端点求解一个非线性系统；作为多步法，它依赖历史并需低阶启动。
- 提高阶数能减少平滑区间的截断误差，但接触切换、力元折点等非光滑行为会削弱高阶收益。
- 改变状态、参数或保持输入后，历史必须显式重建，不能继续使用改变前的多步信息。

### 2.4 ORVD 当前实现

ORVD 当前以 `CVodeCreate(CV_BDF)` 创建后端，并调用
`CVodeSetMaxOrd(memory, maximum_bdf_order)`，把每个实例的最大阶数分别设为 2 或 5；公共构造路径
最大二阶，源码树资格配方可显式选择最大五阶。这仍然是变步长、变阶 BDF。系统层以 `CV_ONE_STEP`
每次取回一个真实接受内步，在该端点更新轮轨站位提示，再继续推进。具体合同见
[continuous_state_advancer.h](../../../libs/integrators/include/orvd/integrators/continuous_state_advancer.h)
和 [ADR-0003](../../adr/0003-abstract-advancer-cvode-first.md)。

## 3. Radau5：三阶段五阶 Radau IIA

### 3.1 配点与 Butcher 表

Radau5 是三阶段 fully implicit Runge–Kutta 配点法。令：

\[
c_1=\frac{4-\sqrt 6}{10},
\qquad
c_2=\frac{4+\sqrt 6}{10},
\qquad
c_3=1.
\]

其 Butcher 表为：

\[
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
\]

最后一行权重等于 \(A\) 的最后一行，因此该方法 stiffly accurate，成功步端点就是第三阶段状态。

### 3.2 一个 Radau5 步怎样计算

三个阶段同时满足：

\[
Y_i=y_n+h\sum_{j=1}^{3}a_{ij}
f(t_n+c_jh,Y_j),
\qquad i=1,2,3.
\]

成功端点为：

\[
y_{n+1}=y_n+h\sum_{i=1}^{3}b_i f(t_n+c_i h,Y_i)=Y_3.
\]

因为 \(A\) 不是下三角矩阵，三个阶段必须耦合求解。简化 Newton 形式的原始块系统规模为
\(3N\times3N\)：

\[
\left(I_3\otimes I_N-hA\otimes J\right)\Delta=-R.
\]

成熟实现不会朴素分解整个 \(3N\) 方阵，而是利用 \(A^{-1}\) 的实特征值和一对共轭复特征值，
把线性代数化成一个实 \(N\times N\) 系统与一个复 \(N\times N\) 系统。每次 Newton 迭代仍需在
三个配点状态上计算 RHS，因此 Jacobian 和分解复用对性能十分重要。

### 3.3 误差控制与稠密输出

Radau5 主公式是五阶。实际求解器还需额外定义：

- 初始步长与下一步长控制器；
- Newton 初值和配点多项式外推；
- 局部误差估计器及安全系数；
- Jacobian、实/复分解的刷新条件；
- Newton 失败、RHS 暂时失败和误差测试失败时的重试规则。

这些属于“Radau5 求解器”而不只是一张 Butcher 表。Hairer 官方 RADAU5 提供自适应步长和配点
连续输出；SciPy 的 Radau 实现采用三阶嵌入公式控制误差，并以满足配点条件的三次多项式提供最近
成功步稠密输出。ORVD 搬运时必须选择并冻结其中一套完整控制策略，不能混用局部公式后仍声称与
某个参考实现数值同一。

### 3.4 稳定性与使用特征

三阶段 Radau IIA 的终点阶数为 5，stage order 为 3。对线性测试方程 \(y'=\lambda y\)，令
\(\zeta=h\lambda\)，其稳定函数为：

\[
\mathcal R(\zeta)=
\frac{1+\frac{2}{5}\zeta+\frac{1}{20}\zeta^2}
{1-\frac{3}{5}\zeta+\frac{3}{20}\zeta^2-\frac{1}{60}\zeta^3}.
\]

它是 A-stable，且 \(\mathcal R(\zeta)\to0\) 当左半平面刚性模态
\(|\zeta|\to\infty\)，因此也是 L-stable。

- 单步法不依赖 BDF 那样的多步状态历史，但仍会复用步长、阶段外推、Jacobian 和分解历史。
- L-stability 不保证在接触不连续处维持五阶，也不保证单步成本低于 BDF。
- 历史 `Radau3` 的阶段数、阶数和系数尚未由仓内权威证据冻结；Radau5 的性质不能据名称反推给它。

### 3.5 ORVD 适用边界

Radau5 可直接消费现有完整一阶 RHS，初版只需支持 \(M=I\)。Hairer 程序对常质量矩阵和 DAE 的
扩展不应在首轮搬运中顺带进入。ORVD 还要求每次只返回一个接受内步、保留 recoverable RHS 的
缩步机会，并在失败后保留最近公开端点；这些是移植实现必须显式承接的外围合同。

## 4. Newmark：二阶机械系统的一步法族

### 4.1 更新公式

给定位移 \(u_n\)、速度 \(v_n\)、加速度 \(a_n\)，Newmark 家族写成：

\[
u_{n+1}=u_n+h v_n
+h^2\left[\left(\frac12-\beta\right)a_n+\beta a_{n+1}\right],
\]

\[
v_{n+1}=v_n+h\left[(1-\gamma)a_n+\gamma a_{n+1}\right].
\]

隐式版本同时要求新端点满足动力学平衡：

\[
R_{n+1}=p_{n+1}-M a_{n+1}-C v_{n+1}
-f_{\mathrm{int}}(u_{n+1},v_{n+1},z_{n+1})=0.
\]

初始加速度不是独立猜测量，应由 \(t_0\) 的动力学平衡计算，例如：

\[
M a_0=p_0-Cv_0-f_{\mathrm{int}}(u_0,v_0,z_0).
\]

### 4.2 线性系统的计算步骤

对常量 \(M,C,K\)、\(f_{\mathrm{int}}=Ku\) 且 \(\beta>0\) 的模型，可把 \(u_{n+1}\) 作为未知量。
定义运动学系数：

\[
\kappa_0=\frac{1}{\beta h^2},
\qquad
\kappa_1=\frac{\gamma}{\beta h},
\]

则有效刚度为：

\[
K_{\mathrm{eff}}=K+\kappa_1C+\kappa_0M.
\]

一个步骤为：

1. 由旧端点形成预测位移、速度和有效载荷；
2. 求解 \(K_{\mathrm{eff}}u_{n+1}=p_{\mathrm{eff}}\)；
3. 用 Newmark 运动学关系回算 \(a_{n+1}\) 和 \(v_{n+1}\)；
4. 提交端点并进入下一步。

若 \(M,C\) 为常量、非线性只在内力中，则在 \(t_{n+1}\) 反复形成残差时常用切线：

\[
K_{\mathrm{eff}}^{(m)}
=K_t^{(m)}+\frac{\gamma}{\beta h}C
+\frac{1}{\beta h^2}M,
\]

直至平衡残差与增量满足收敛判据。

若 \(M=M(u)\)、\(C=C(u,v)\) 或载荷也依赖状态，上式不再是完整一致切线。此时必须形成
\(-\partial R_{n+1}/\partial u_{n+1}\)，其中还包括惯性、阻尼和状态依赖载荷的导数。另有
\(\beta=0\) 的显式 Newmark／中心差分分支，不能使用本节含 \(1/\beta\) 的有效刚度公式。

### 4.3 参数与稳定性

- 标准 Newmark 家族在 \(\gamma=1/2\) 时为二阶；\(\gamma>1/2\) 会引入算法耗散并降为一阶。
- 对线性无阻尼问题，常用无条件稳定区域为 \(2\beta\ge\gamma\ge1/2\)。
- \(\beta=1/4,\gamma=1/2\) 是平均加速度法：对线性系统无条件稳定，且没有算法高频耗散。
- \(\beta=1/6,\gamma=1/2\) 是线性加速度法，稳定性受步长限制。
- “线性无条件稳定”不等于非线性接触问题可任意取大步；Newton 未收敛、力律不光滑和漏解析的
  高频模态仍会限制可用步长。

### 4.4 ORVD 适用边界

经典公式假设 \(u,v,a\) 同维且 \(\dot u=v\)。ORVD 完整车辆含四元数、Ball-RPY 和一阶 \(z\)，
因此忠实的整车 Newmark 至少需要：

1. 用 \(v\) 的切空间增量更新 \(q\) 的 configuration retraction；
2. 定义 \(a_{n+1}\) 与 \(z_{n+1}\) 的耦合残差；
3. 在试算 Context 中求值且不污染 accepted 状态；
4. 配置 Newton、线搜索、尺度、数值或解析切线以及失败缩步；
5. 为当前稠密采样合同定义并验证插值。

不应为了套用线性教材公式而从 ABA 路径提取并长期维护完整 \(M,C,K\)。更自然的研究路线是以
`a_{n+1}` 和 `z_{n+1}` 为未知量，通过现有前向动力学形成黑箱残差。

## 5. Zhai：简单显式二步法

### 5.1 更新公式

Zhai 简单显式法使用当前和前一端点的加速度：

\[
u_{n+1}=u_n+h v_n
+\left(\frac12+\psi\right)h^2a_n
-\psi h^2a_{n-1},
\]

\[
v_{n+1}=v_n+(1+\phi)h a_n-\phi h a_{n-1}.
\]

得到 \(u_{n+1},v_{n+1}\) 后再从动力学方程显式计算：

\[
a_{n+1}=M^{-1}
\left[p_{n+1}-C v_{n+1}-f_{\mathrm{int}}(u_{n+1},v_{n+1})\right].
\]

原始高效场景使用对角或集中质量矩阵。ORVD 可用 ABA 直接计算最后一步的广义加速度，从而避免
形成和求解完整质量矩阵，但这并不自动解决位形流形和一阶内变量问题。

### 5.2 启动与单步流程

这是二步法，初始时没有 \(a_{-1}\)。常见自启动做法是在第一步使用
\(\phi=\psi=0\)：

\[
u_1=u_0+h v_0+\frac12h^2a_0,
\qquad
v_1=v_0+h a_0,
\]

其中 \(a_0\) 必须先由初始动力学平衡计算；对线性模型即
\(a_0=M^{-1}(p_0-Cv_0-Ku_0)\)。第一步成功后采用 \(\phi=\psi=1/2\)，每个正常步骤为：

1. 从两个 accepted 端点读取 \(a_n,a_{n-1}\)；
2. 显式计算 \(u_{n+1},v_{n+1}\)；
3. 在新端点计算 \(a_{n+1}\)；
4. 整步成功后才滚动加速度历史。

拒步、异常或成功端点投影失败时，\(a_n,a_{n-1}\) 都不能提前提交。外部参数或控制输入改变后，
旧加速度历史不再描述新方程，必须重新自启动。

### 5.3 精度与稳定性

- \(\phi=\psi=1/2\) 的常用简单显式法为二阶，在线性无阻尼分析中没有数值耗散，但仍有相位误差。
- 对无阻尼线性振子，该参数组合的稳定边界为 \(h\omega<2\)，与中心差分同量级；它不是
  无条件稳定方法。若 \(\omega_{\max}=2\pi/T_{\min}\)，相同条件也可写为
  \(h<2/\omega_{\max}=T_{\min}/\pi\)。
- 很高的轮轨接触频率、悬挂刚度和显式推进的一阶 Maxwell 状态都可能决定稳定步长。
- 方法本身只给出离散端点；稠密输出、非整步 stop 和变步长系数必须另行定义，不能直接套用
  等步长二步公式。

### 5.4 ORVD 适用边界

完整车辆实现应明确称为“流形 Zhai + 指定的 \(z\) 离散方法”。至少需要：

- 用切空间速度、加速度形成位形增量并 retraction 到新 \(q\)；
- 为 Maxwell \(z\) 选择并命名显式、隐式或指数型更新；
- 让固定步与控制事件网格对齐，并规定最后缩短步后的历史重建；
- 以端点状态和导数构造、验证最近一步插值；
- 在步长减半试验中同时检查相位、稳定性和轮轨接触切换。

把 AB2 直接应用于完整 \([q;v;z]\) 虽然容易实现，但那是普通一阶多步法，不应以 Zhai 方法
名义发布。

## 6. 四种方法的并列比较

| 方法 | 基本方程 | 主阶数 | 隐式性 | 步长／阶数 | 每步主要代价 | 原生稠密输出 | ORVD 状态 |
|---|---|---:|---|---|---|---|---|
| CVODE BDF | 完整一阶 \(\dot y=f(t,y)\) | 1–5 | 隐式 | 自适应步长、变阶 | 一个端点非线性系统 | 有 | 当前默认；实例最大阶为 2 或 5 |
| Radau5 | 完整一阶 \(\dot y=f(t,y)\) | 5 | 三阶段全隐式 | 通常自适应步长、固定方法阶 | 三阶段耦合 Newton，实/复线性系统 | 有配点连续输出 | 源码树纯 C++ 研究实现；待等误差性能和长窗资格 |
| Newmark | 二阶机械平衡 | 通常 2 | 常用版本隐式 | 通常固定方法参数，可变或固定步 | 端点平衡 Newton／有效刚度 | 需额外构造 | 待做；尚无整车机械适配接口 |
| Zhai 简单显式法 | 二阶机械加速度 | 2 | 显式 | 原式为固定等步长 | 一次新端点加速度求值 | 需额外构造 | 待做；需流形与 \(z\) 策略 |

不能仅按“每步 RHS 次数”判断总性能。BDF 和 Radau5 的隐式步更贵但可能取更大步长；Zhai 单步
便宜但受最高频率限制；Newmark 的成本取决于端点平衡迭代。合理比较必须在共同物理输出时钟上，
通过容差或步长加密达到相近误差后，再比较 RHS、接触、Jacobian、Newton、线性分解和墙钟。

## 7. 资格化时必须回答的问题

### 7.1 方法学问题

- 平滑解析问题是否呈现声明的收敛阶？
- 刚性衰减、振荡和非自治输入是否保持预期稳定性与相位？
- 稠密输出是否只覆盖最近成功步，并与两端状态一致？
- 失败、拒步和重初始化是否保持 accepted 状态与方法历史的原子性？
- 固定步方法遇到非整步 stop 或 100 Hz 外部事件时怎样处理历史？

### 7.2 车辆问题

- 四元数与 Ball-RPY 位形误差是否随步长或容差收敛？
- Maxwell 内变量是否在其最短时间尺度上收敛且不产生伪振荡？
- 轮对横移、摇头、轮轨法向／切向力和接触斑切换是否保持相位和事件次序？
- 接触非光滑区是否使高阶方法频繁降阶、拒步或 Newton 失败？
- 在相近物理误差下，较高单步成本是否由更少接受步抵消？

能通过简化解析系统只证明算法核正确；能与另一积分器接近只证明交叉一致。整车研究后端仍需独立
做步长或容差加密，不能把当前 CVODE 输出当作不可质疑的唯一真值。

## 8. 公开参考资料与证据边界

本文的公式和方法身份来自公开资料，不以仓库 `tmp/` 中的临时 notebook 或对话记录为权威：

1. SUNDIALS，
   [CVODE Mathematical Considerations](https://sundials.readthedocs.io/en/latest/cvode/Mathematics_link.html)：
   变步长／变阶 BDF、非线性系统、误差权重、阶数选择和稳定性说明。
2. E. Hairer 与 G. Wanner，
   [RADAU5 官方程序](https://www.unige.ch/~hairer/prog/stiff/radau5.f)及
   [官方软件目录](https://www.unige.ch/~hairer/software.html)：三阶段五阶 Radau IIA、步长控制、
   Jacobian／质量矩阵选项和配点连续输出。
3. SciPy，
   [`scipy.integrate.Radau` 官方文档](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.Radau.html)：
   五阶 Radau IIA、三阶嵌入误差估计和三次稠密输出的一种现代求解器实现。
4. N. M. Newmark，
   [A Method of Computation for Structural Dynamics](https://doi.org/10.1061/JMCEA3.0000098)，1959；
   公式与有效切线亦可对照
   [OpenSees Newmark 文档](https://opensees.github.io/OpenSeesDocumentation/user/manual/analysis/integrator/Newmark.html)，
   稳定区域可对照
   [Sandia Sierra/SM 手册](https://www.sandia.gov/files/sierra/SM_Users_5_24/main/implicit/integration.html)。
5. W. M. Zhai，
   [Two simple fast integration methods for large-scale dynamic problems in engineering](https://onlinelibrary.wiley.com/doi/abs/10.1002/%28SICI%291097-0207%2819961230%2939%3A24%3C4199%3A%3AAID-NME39%3E3.0.CO%3B2-Y)，
   *International Journal for Numerical Methods in Engineering* 39 (1996), 4199–4214；公式可由
   [作者公开全文](https://www.researchgate.net/publication/229645128_Two_simple_fast_integration_methods_for_large-scale_dynamic_problems_in_engineering)
   交叉核对。

公开参考说明算法原理，不自动成为 ORVD 产品合同。ORVD 当前已实现能力仍以源码、测试、
[ADR-0003](../../adr/0003-abstract-advancer-cvode-first.md)和
[libs/integrators/README.md](../../../libs/integrators/README.md)为准。
