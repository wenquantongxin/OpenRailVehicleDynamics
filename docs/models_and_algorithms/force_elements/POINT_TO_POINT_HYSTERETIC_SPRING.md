[English](POINT_TO_POINT_HYSTERETIC_SPRING.en.md)

# 点对点迟滞弹簧

无质量轴向元件沿两个安装点的连线传力。其弹性中线与上下包络由伸缩量确定，伸缩速度控制包络之间的连续过渡。

## 1. 几何与本构

设两个安装点在同一惯性系中的位置为 $\mathbf p_A,\mathbf p_C$，定义

$$
\mathbf d=\mathbf p_C-\mathbf p_A,\qquad
l=\|\mathbf d\|>0,\qquad
\mathbf e=\frac{\mathbf d}{l},\qquad
x=l-l_0,
$$

$$
v=\dot x=\dot l
=\mathbf e\cdot(\dot{\mathbf p}_C-\dot{\mathbf p}_A).
$$

名义长度为 $l_0>0$，故 $x>-l_0$。长度与伸缩量的单位为 m，长度率的单位为 m/s。
标量轴力 $F$（N）以拉伸为正，正力使两个安装点相互靠近。

取有限参数：中线刚度 $k\ge0$（N/m）、无量纲迟滞比 $0\le\eta\le1$、过渡速度 $\epsilon>0$（m/s）。令

$$
S(\xi)=
\begin{cases}
-1,&\xi\le-1,\\
\xi,&-1<\xi<1,\\
1,&\xi\ge1.
\end{cases}
$$

本构为

$$
F(x,v)=kx+F_h(x,v),\qquad
F_h(x,v)=\eta k|x|S(v/\epsilon).
$$

力由当前的 $x,v$ 确定，不引入独立演化的内部变量。$l=0$ 时连线方向无定义。

## 2. 包络与过渡

固定伸缩量时，力位于下、上包络之间：

$$
F_-(x)=kx-\eta k|x|,\qquad
F_+(x)=kx+\eta k|x|,\qquad
F_-(x)\le F(x,v)\le F_+(x).
$$

$v\le-\epsilon$ 时取下包络，$v\ge\epsilon$ 时取上包络。速度带内为

$$
F(x,v)=kx+\frac{\eta k|x|}{\epsilon}v,
\qquad |v|\le\epsilon.
$$

因此 $F(x,0)=kx$，$F(0,v)=0$。力在 $x=0$ 与 $v=\pm\epsilon$ 处连续，分段导数可发生跳变。
参数域保证

$$
Fx\ge(1-\eta)kx^2\ge0.
$$

在 $|v|\ge\epsilon$ 的分支上，远离名义长度的加载过程 $xv>0$ 对应 $F=(1+\eta)kx$，
返回名义长度的卸载过程 $xv<0$ 对应 $F=(1-\eta)kx$。这两个条件同时适用于拉伸与压缩。

## 3. 端点载荷、储能与耗散

作用于两安装点的力为

$$
\mathbf f_A=F\mathbf e,\qquad
\mathbf f_C=-F\mathbf e.
$$

两端无附加力偶。合力为零，关于任意共同参考点的合矩为 $-\mathbf d\times(F\mathbf e)=\mathbf0$。
元件的端点总功率（W）为

$$
\mathcal P=\mathbf f_A\cdot\dot{\mathbf p}_A
+\mathbf f_C\cdot\dot{\mathbf p}_C=-Fv.
$$

取弹性储能 $U=\tfrac12kx^2$（J），有 $\dot U=kxv$。耗散率（W）为

$$
\mathcal D=F_hv
=\eta k|x|\,|v|\min\!\left(\frac{|v|}{\epsilon},1\right)\ge0,
\qquad
\mathcal P+\dot U+\mathcal D=0.
$$

该恒等式给出两端总功率、储能变化与耗散之间的平衡。耗散非负由 $k,\eta\ge0$ 和 $\epsilon>0$ 保证；
$\eta\le1$ 进一步保证轴力的恢复方向。

## 4. 循环耗能与速率依赖

对 $x(T)=x(0)$ 的闭合伸缩历程，储能净变化为零。输入元件的净功为

$$
E_{\mathrm{cyc}}=\oint F\,dx
=\int_0^T\mathcal D\,dt
=\eta k\int_0^T|x|\,|v|
\min\!\left(\frac{|v|}{\epsilon},1\right)dt\ge0.
$$

若历程从 $a$ 单调增至 $b$ 再单调返回 $a$，其中 $-l_0<a<b$，则

$$
E_{\mathrm{cyc}}\le2\eta k\int_a^b|\xi|\,d\xi
=\eta k\bigl(b|b|-a|a|\bigr).
$$

右端为上下包络围成的面积；速度过渡带使实际回线位于包络之内。
对称往返 $a=-A,b=A$（$0<A<l_0$）的包络面积为 $2\eta kA^2$。

固定 $\epsilon$，将同一光滑闭合伸缩路径均匀放慢，耗能随之不增，并在准静态极限趋于零。
这是因为 $\sup|v|\to0$ 时，上述积分中的速度因子一致趋于零，而路径的总变差保持不变。
迟滞回线因而具有速率依赖；饱和包络面积给出该单次往返的耗能上界。
