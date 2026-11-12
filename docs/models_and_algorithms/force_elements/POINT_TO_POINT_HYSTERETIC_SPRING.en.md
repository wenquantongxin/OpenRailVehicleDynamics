[简体中文](POINT_TO_POINT_HYSTERETIC_SPRING.md)

# Point-to-point hysteretic spring

A massless axial element transmits force along the line joining two attachment points. Extension determines its elastic centreline and upper and lower envelopes; extension rate controls the continuous transition between the envelopes.

## 1. Geometry and constitutive law

Let $\mathbf p_A,\mathbf p_C$ be the attachment-point positions in the same inertial frame. Define

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

The nominal length is $l_0>0$, so $x>-l_0$. Length and extension are measured in m, and length rate in m/s.
The scalar axial force $F$ (N) is positive in tension: a positive force pulls the attachment points together.

The finite parameters are centreline stiffness $k\ge0$ (N/m), dimensionless hysteresis ratio $0\le\eta\le1$, and transition speed $\epsilon>0$ (m/s). Let

$$
S(\xi)=
\begin{cases}
-1,&\xi\le-1,\\
\xi,&-1<\xi<1,\\
1,&\xi\ge1.
\end{cases}
$$

The constitutive law is

$$
F(x,v)=kx+F_h(x,v),\qquad
F_h(x,v)=\eta k|x|S(v/\epsilon).
$$

The current $x,v$ determine the force, with no independently evolving internal variable. The attachment-line direction is undefined at $l=0$.

## 2. Envelopes and transition

At a fixed extension, the force lies between the lower and upper envelopes:

$$
F_-(x)=kx-\eta k|x|,\qquad
F_+(x)=kx+\eta k|x|,\qquad
F_-(x)\le F(x,v)\le F_+(x).
$$

The lower envelope is attained for $v\le-\epsilon$, and the upper envelope for $v\ge\epsilon$. Within the speed band,

$$
F(x,v)=kx+\frac{\eta k|x|}{\epsilon}v,
\qquad |v|\le\epsilon.
$$

Thus $F(x,0)=kx$ and $F(0,v)=0$. The force is continuous at $x=0$ and $v=\pm\epsilon$, where the piecewise derivatives may jump.
The parameter domain ensures

$$
Fx\ge(1-\eta)kx^2\ge0.
$$

On the branches with $|v|\ge\epsilon$, loading away from the nominal length, $xv>0$, gives $F=(1+\eta)kx$;
unloading towards the nominal length, $xv<0$, gives $F=(1-\eta)kx$. These conditions apply to both tension and compression.

## 3. Attachment loads, storage and dissipation

The forces applied at the two attachment points are

$$
\mathbf f_A=F\mathbf e,\qquad
\mathbf f_C=-F\mathbf e.
$$

There are no additional endpoint couples. The resultant force is zero, and the resultant moment about any common reference point is $-\mathbf d\times(F\mathbf e)=\mathbf0$.
The total endpoint power delivered by the element (W) is

$$
\mathcal P=\mathbf f_A\cdot\dot{\mathbf p}_A
+\mathbf f_C\cdot\dot{\mathbf p}_C=-Fv.
$$

With elastic storage $U=\tfrac12kx^2$ (J), its rate is $\dot U=kxv$. The dissipation rate (W) is

$$
\mathcal D=F_hv
=\eta k|x|\,|v|\min\!\left(\frac{|v|}{\epsilon},1\right)\ge0,
\qquad
\mathcal P+\dot U+\mathcal D=0.
$$

This identity balances total endpoint power, change in stored energy and dissipation. Non-negative dissipation follows from $k,\eta\ge0$ and $\epsilon>0$;
the additional bound $\eta\le1$ ensures the restoring direction of the axial force.

## 4. Cycle dissipation and rate dependence

For a closed extension history with $x(T)=x(0)$, the net change in stored energy is zero. The net work supplied to the element is

$$
E_{\mathrm{cyc}}=\oint F\,dx
=\int_0^T\mathcal D\,dt
=\eta k\int_0^T|x|\,|v|
\min\!\left(\frac{|v|}{\epsilon},1\right)dt\ge0.
$$

For one monotone excursion from $a$ to $b$ and back to $a$, with $-l_0<a<b$,

$$
E_{\mathrm{cyc}}\le2\eta k\int_a^b|\xi|\,d\xi
=\eta k\bigl(b|b|-a|a|\bigr).
$$

The right-hand side is the area enclosed by the two envelopes; the speed-transition band keeps the actual loop within them.
For a symmetric excursion $a=-A,b=A$ ($0<A<l_0$), the envelope area is $2\eta kA^2$.

At fixed $\epsilon$, uniformly slowing the same smooth closed extension path makes the dissipated energy non-increasing, with a zero quasistatic limit.
Indeed, as $\sup|v|\to0$, the speed factor in the integral above converges uniformly to zero while the path's total variation remains unchanged.
The hysteresis loop is therefore rate dependent; the saturated envelope area bounds the dissipation of that single excursion.
