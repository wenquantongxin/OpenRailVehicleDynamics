[中文](MULTIBODY_EQUATIONS_OF_MOTION.md)

# Multibody equations of motion

This chapter explains how the multibody layer of one ORVD vehicle organizes rigid bodies, joints and free bodies into a rigid-body tree, how it defines the generalized positions $q$, the generalized velocities $v$ and the map $\dot q=N(q)v$ between them, and how it obtains the generalized accelerations $\dot v$ from the body wrenches delivered by the components together with the gravity and joint damping that the multibody layer itself owns. The published chapters on line geometry, irregularity spectra, the eight wheel-rail contact chapters and the six force-element chapters are components: they produce wrenches acting on rigid bodies. This chapter combines those wrenches with gravity and joint damping into the complete right-hand side $[N(q)v;\dot v;\dot z]$; [Startup state assembly](STARTUP_STATE_ASSEMBLY.en.md) constructs the initial value $y_0$; [Contact force plan kinematics](../wheel_rail_contact/CONTACT_FORCE_PLAN_KINEMATICS.en.md) explains how the multibody state becomes the contact input of every wheel-rail interface and how the contact result becomes one equivalent wrench on the wheel body; [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md) explains how the solver advances the state and differences the complete right-hand side. The chapter first presents the objects and coordinates of the rigid-body tree, then the rate maps, poses and spatial velocities, and the power conjugacy that turns body wrenches into generalized forces; it writes $M(q)\dot v+C(q,v)v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}$, describes the articulated-body algorithm used for forward dynamics and finally the assembly of the complete right-hand side. The public interface of the multibody layer is [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h); the recursions are carried out by the external rigid-body tree, and the recursion code cited here lives in [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) and neighbouring files.

## 1. Scope and notation

### 1.1 Objects

The object is the rigid-body tree of one vehicle: finitely many rigid bodies, frames fixed to them, joints connecting pairs of bodies, and bodies declared to move freely in the world. The multibody layer admits five kinds of relation: revolute joint, prismatic joint, Ball-RPY joint, weld joint and free-body declaration. Its input is the set of body wrenches produced by the components, each naming the loaded body, the application point in the body frame, the expressed-in frame, and the moment about the application point together with the force; its output is the generalized-position derivative $N(q)v$ and the generalized acceleration $\dot v$. The chapter discusses no constitutive law, contact geometry, initial-value construction or time stepping: force-element laws are in the force-element chapters, the formation of contact inputs in the contact-force-plan chapter, initial values in the startup chapter and time stepping in the time-integration chapter.

### 1.2 Notation

| Symbol | Meaning | Source |
|---|---|---|
| W, B, P, C | World frame; rigid body B and its body frame; parent node P and child node C of B in the tree | New here |
| F, M, E | Inboard frame (fixed to the parent body) and outboard frame (fixed to the child body) of a joint; expressed-in frame of a wrench entry | New here |
| $B_o$, $\mathbf p_{PoBo}$ | Body origin of B; position vector from $P_o$ to $B_o$, expressed in W unless marked otherwise | Conventions §4.1 |
| $R_{WB}$, $R_{FM}(q_B)$ | Attitude of B in W; joint rotation of the outboard frame relative to the inboard frame | Conventions §4.1 |
| $q$, $v$, $z$ | Generalized positions, generalized velocities, internal force-element states | Conventions §5.1 |
| $q_B$, $v_B$, $n_{v,B}$ | Blocks of $q$ and $v$ belonging to B's inboard relation (joint or free-body declaration), and the dimension of the latter | New here |
| $n_q$, $n_v$ | Dimensions of $q$ and $v$ | Conventions §4.2 |
| $N(q)$, $N^{+}(q)$ | Position-derivative map and its left pseudoinverse | Conventions §4.5 |
| $(\lambda_0,\boldsymbol\lambda)$, $\hat\lambda$, $Q(\lambda)$ | Quaternion of a free body (stored order $w,x,y,z$), its normalization, and the $4\times3$ matrix built from it | New here |
| $\boldsymbol\eta$, $E(\boldsymbol\eta)$, $H(\boldsymbol\eta)$ | The three Ball-RPY angles (roll, pitch, yaw) and their angular-rate maps | Bushing chapter §2.6–2.7 |
| $\mathbf a$, $\theta$, $d$ | Unit axis in F, rotation angle and translation of a single-axis joint | New here |
| $m$, $\mathbf c$, $I_{\mathrm c}$, $G_o$ | Mass, centre-of-mass offset in the body frame, central inertia, unit inertia about the body origin | New here |
| $M_B$, $\mathbf c_W$, $G_W$ | Spatial inertia of B about $B_o$ expressed in W, with its centre-of-mass offset and unit inertia | New here |
| $\mathbf g_W$, $g$, $U$ | Gravitational acceleration vector, its magnitude, gravitational potential energy | New here |
| $V_B$, $A_B$ | Spatial velocity $(\boldsymbol\omega_{WB};\mathbf v_{WBo})$ and spatial acceleration $(\boldsymbol\alpha_{WB};\mathbf a_{WBo})$ of B, expressed in W | New here |
| $\mathcal W_Q^E=(\boldsymbol\tau_Q^E;\mathbf f^E)$ | Wrench about Q expressed in E, handled here as a six-component column | Conventions §5.2 |
| $\Phi(\mathbf p)$ | Reduction-point shift operator $\begin{bmatrix}\mathbf 1&[\mathbf p]_\times\\ \mathbf 0&\mathbf 1\end{bmatrix}$ | New here |
| $[\mathbf p]_\times$, $\mathbf 1$ | Skew-symmetric matrix (the conventions' $\operatorname{skew}(\mathbf p)$) and identity matrix, with its order as a subscript where needed | New here |
| $S_{FM}$, $S_B$ | Motion subspace of a joint in F about $M_o$; the same subspace re-expressed in W and shifted to $B_o$, a $6\times n_{v,B}$ matrix | New here |
| $J_B$, $J_Q$ | Jacobians with respect to $v$ of the spatial velocity of B and of a point Q fixed to B | New here |
| $M(q)$, $C(q,v)v$ | System mass matrix and velocity-bias generalized force | Conventions §5.3 |
| $\tau_{\mathrm g}$, $\tau_{\mathrm d}$, $\tau_{\mathrm{app}}$ | Generalized forces of gravity, joint damping and applied body wrenches | Conventions §5.3 |
| $\mathcal W_{\mathrm g,Bo}$, $\mathcal W_{\mathrm{app},Bo}$ | Equivalent wrench of gravity about $B_o$; sum of the components' body wrenches shifted to $B_o$ | New here |
| $F_{\mathrm{app},B}$, $\tau_{\mathrm{app},B}$ | Total applied spatial force accumulated at $B_o$, $\mathcal W_{\mathrm g,Bo}+\mathcal W_{\mathrm{app},Bo}$; generalized force on the coordinates of B's inboard relation (including joint damping) | New here |
| $F_{\mathrm b,B}$, $A_{\mathrm b,B}$ | Velocity-bias spatial force and bias spatial acceleration of B | New here |
| $P_B$, $P_B^{+}$, $U_B$, $D_B$, $g_B$ | Articulated-body inertia, articulated-body inertia projected across the joint, $S_B^{\mathsf T}P_B$, hinge inertia, gain | New here |
| $Z_B$, $Z_B^{+}$, $Z_{\mathrm b,B}$, $e_B$ | Residual force, residual force projected across the joint, articulated bias force, hinge residual generalized force | New here |
| $K_B$ | Composite-body inertia | New here |
| $c$ | Viscous damping coefficient of a joint | Conventions §5.3 |

### 1.3 Relation to the shared conventions

The chapter follows the rotation-matrix rules of Section 4.1 and the wrench notation of Section 5.2 of [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md). The multibody layer calls its inertial reference the world frame W; the vehicle assembly takes W to be the track inertial frame I of Section 2.1 of the conventions (the gravity vector is set along the $+z$ of I and the startup state is stated in I), so the W of this chapter is the same frame as the I of the other chapters. The following symbols are local to this chapter and are declared here: the motion subspace of the articulated-body recursion is written $S$ (source `H_PB_W`), not $H$, because $H(\boldsymbol\eta)$ and $E(\boldsymbol\eta)$ are already the angular-rate maps of Section 2.7 of the [half-angle midpoint RPY bushing](../force_elements/HALF_ANGLE_MIDPOINT_RPY_BUSHING.en.md), which Section 3.2 cites verbatim; the $J$ of this chapter is a kinematic Jacobian, unrelated to the $J=\partial f/\partial y$ of [time-integration methods](../numerical_methods/TIME_INTEGRATION_METHODS.en.md); a bare $C$ is a child node of the tree, whereas $C(q,v)$ with its arguments is the system matrix of Section 5.3 of the conventions; a bare $P$ is a parent node, whereas $P_B$ with a body subscript is an articulated-body inertia; $V_B$ is a spatial velocity, not the stored energy $\mathcal V$ of the force-element chapters; $g$ is the magnitude of gravitational acceleration, unrelated to the grade $g(s)$ of Section 2.1 of the conventions, and $g_B$ with a body subscript is a recursion gain; $\lambda$ denotes quaternion components, unrelated to the eigenvalues of the stability analysis in the time-integration chapter; $N$ denotes only the position-derivative map; lowercase $\tau$ is a generalized force and bold $\boldsymbol\tau$ a spatial moment, as in Section 5.3 of the conventions.

## 2. Rigid-body tree and generalized coordinates

### 2.1 Bodies, frames and joint types

A rigid body B carries a body frame whose origin $B_o$ is the reference point for moments, inertia and state. A fixed frame is attached to some body at a constant pose $(R_{BF},\mathbf p_{BoFo\_B})$; it is fixed to a body, never to another frame. A joint connects a frame F on the parent body to a frame M on the child body; the joint coordinates $q_B$ describe the pose $(R_{FM}(q_B),\mathbf p_{FoMo\_F}(q_B))$ of M in F, and the joint velocities $v_B$ describe the spatial velocity $V_{FM}=S_{FM}v_B$ of M relative to F. Which end is declared the parent fixes the meaning and sign of the joint coordinate; it does not decide in which direction the rigid-body tree, growing outward from the world, traverses the joint.

| Type | $n_q$ | $n_v$ | Generalized position | Generalized velocity |
|---|---|---|---|---|
| Revolute | 1 | 1 | Angle $\theta$ about the unit axis $\mathbf a$ fixed in F, right-hand rule, $R_{FM}=\exp(\theta[\mathbf a]_\times)$, $\mathbf p_{FoMo}=\mathbf 0$ | $\dot\theta$, $\boldsymbol\omega_{FM\_F}=\dot\theta\,\mathbf a$ |
| Prismatic | 1 | 1 | Displacement $d$ along $\mathbf a$, $R_{FM}=\mathbf 1$, $\mathbf p_{FoMo\_F}=d\,\mathbf a$ | $\dot d$, $\mathbf v_{FMo\_F}=\dot d\,\mathbf a$ |
| Ball-RPY | 3 | 3 | $\boldsymbol\eta=(\text{roll},\text{pitch},\text{yaw})$, $R_{FM}=R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll})$, $\mathbf p_{FoMo}=\mathbf 0$ | $\boldsymbol\omega_{FM\_F}$, not $\dot{\boldsymbol\eta}$ |
| Weld | 0 | 0 | None; $R_{FM}=\mathbf 1$, $\mathbf p_{FoMo}=\mathbf 0$, the relative placement of the two bodies is given entirely by the two fixed frames | None |
| Free body | 7 | 6 | Quaternion $(\lambda_0,\boldsymbol\lambda)$ and the position $\mathbf p_{WoBo\_W}$ of the body origin in the world | $\boldsymbol\omega_{WB\_W}$ and $\mathbf v_{WBo\_W}$, both expressed in the world frame |

The axis of a single-axis joint is given as a direction in the parent frame and normalized; only its direction is used, and since M only rotates about or translates along that axis relative to F, the axis has the same components in F and in M. The Ball-RPY composition order is that of Section 4.3 of the conventions, which is the space-XYZ angle convention of Section 2.6 of the bushing chapter; the three angles are stored as roll, pitch, yaw, and the velocity is the physical angular velocity. The quaternion of a free body is used as stored and never normalized; its two velocity blocks are the absolute angular velocity of the body and the absolute velocity of the body origin, both expressed in the world frame (Section 4.2 of the conventions).

### 2.2 Tree topology

Every relation connects two distinct bodies, and connecting two bodies that already reach each other would close a loop, so the model is a tree rooted at the world: there is at most one path between any two bodies, and no body reaches the world by two paths. A free-body declaration is a relation between a body and the world, implemented at finalization by a six-degree-of-freedom joint from the world frame to the body frame; it is not a default for "no relation". Joints may still hang outboard of a free body: declaring it free only states how it relates to the world. At finalization every body must reach the world. Afterwards every generalized coordinate belongs to exactly one joint or one free-body declaration, $q$ and $v$ are the concatenations of these blocks in the model's own order, and $n_q-n_v$ equals the number of free bodies. The rigid-body tree assigns each body a unique inboard relation and parent node, growing outward from the world; every recursion in this chapter runs along that tree.

### 2.3 Inertia parameters

The vehicle definition gives every body its mass $m$, its centre-of-mass offset $\mathbf c$ in the body frame and its inertia $I_{\mathrm c}$ about the centre of mass, expressed in the body frame. The multibody layer stores inertia as a unit inertia about the body origin, and the assembly converts once:

$$
G_o=\frac{I_{\mathrm c}}{m}+\lvert\mathbf c\rvert^2\mathbf 1-\mathbf c\mathbf c^{\mathsf T},
$$

which is the parallel-axis theorem divided by the mass. From $(m,\mathbf c,G_o)$ the spatial inertia of B about $B_o$ in the body frame is formed; re-expressed in the world frame,

$$
M_B=\begin{bmatrix}mG_W&m[\mathbf c_W]_\times\\-m[\mathbf c_W]_\times&m\mathbf 1\end{bmatrix},
\qquad
\mathbf c_W=R_{WB}\mathbf c,
\qquad
G_W=R_{WB}G_oR_{WB}^{\mathsf T},
$$

which maps a spatial acceleration $(\boldsymbol\alpha;\mathbf a_{Bo})$ to a spatial force about $B_o$: moment $mG_W\boldsymbol\alpha+m\,\mathbf c_W\times\mathbf a_{Bo}$, force $m\mathbf a_{Bo}+m\,\boldsymbol\alpha\times\mathbf c_W$. The multibody layer requires $m\ge0$ and a central inertia, obtained by shifting $G_o$ back to the centre of mass, that is positive semidefinite with principal moments satisfying the triangle inequality; the vehicle assembly additionally requires $m>0$ for every body and a strictly positive-definite $I_{\mathrm c}$ for every body declared free. The latter guarantees that the six-dimensional hinge inertia of a free body is invertible (Section 6.4).

### 2.4 Gravity vector

Gravity is a model constant, set before finalization and unchanged afterwards: $\mathbf g_W=g\,\mathbf e_3$, where $g>0$ is the magnitude of gravitational acceleration and the downward direction is fixed by the $+z$ of the track inertial frame (Section 2.1 of the conventions); the caller chooses only the magnitude. It acts at the centre of mass of every body; its wrench about the body origin and its generalized force are given in Section 4.5.

## 3. Coordinate-rate maps

$N(q)$ is block-structured by relation: the $q_B$ block of each joint or free-body declaration is related only to its own $v_B$ block, $\dot q_B=N_B(q_B)v_B$. The reverse map $v=N^{+}(q)\dot q$ has the same block structure.

### 3.1 $N(q)$ of a quaternion free body and its left pseudoinverse

The position block of a free body is the quaternion $(\lambda_0,\boldsymbol\lambda)$ together with the body-origin position, and the velocity block is $(\boldsymbol\omega;\mathbf v_{Bo})$ with $\boldsymbol\omega=\boldsymbol\omega_{WB\_W}$. The attitude of a unit quaternion satisfies $\dot R_{WB}=[\boldsymbol\omega]_\times R_{WB}$, and the corresponding quaternion differential equation is $\dot\lambda=\tfrac12\,\boldsymbol\omega\otimes\lambda$, with $\boldsymbol\omega$ multiplying from the left as a pure quaternion (a world-frame angular velocity multiplies from the left, a body-frame one from the right). Expanded,

$$
\dot\lambda_0=-\tfrac12\,\boldsymbol\lambda\cdot\boldsymbol\omega,
\qquad
\dot{\boldsymbol\lambda}=\tfrac12\left(\lambda_0\boldsymbol\omega+\boldsymbol\omega\times\boldsymbol\lambda\right),
\qquad
N_{\mathrm q}(\lambda)=\tfrac12\,Q(\lambda),
\qquad
Q(\lambda)=\begin{bmatrix}-\boldsymbol\lambda^{\mathsf T}\\ \lambda_0\mathbf 1-[\boldsymbol\lambda]_\times\end{bmatrix}.
$$

The three columns of $Q$ are orthogonal to $\lambda$ and to one another: $Q(\lambda)^{\mathsf T}\lambda=\mathbf 0$ and $Q(\lambda)^{\mathsf T}Q(\lambda)=\lVert\lambda\rVert^2\mathbf 1_3$. Hence $\lambda\cdot\dot\lambda=0$, and the exact solution preserves $\lVert\lambda\rVert$. The implementation uses the quaternion as stored, neither normalizing nor rewriting it, so $\dot q$ is linear in the stored value: scaling the quaternion by $s$ scales its four derivative entries by $s$, while the three translational derivatives are unchanged. The translational part of the position block obeys $\dot{\mathbf p}_{WoBo\_W}=\mathbf v_{WBo\_W}$, an identity map. Pose evaluation uses only the direction of the quaternion: the rotation matrix is built from the normalized quaternion.

The reverse map is the left pseudoinverse onto the quaternion tangent space. Writing $\hat\lambda=\lambda/\lVert\lambda\rVert$, the implementation takes

$$
N_{\mathrm q}^{+}(\lambda)=\frac{2}{\lVert\lambda\rVert}\,Q(\hat\lambda)^{\mathsf T}\left(\mathbf 1_4-\hat\lambda\hat\lambda^{\mathsf T}\right),
$$

where $(\mathbf 1_4-\hat\lambda\hat\lambda^{\mathsf T})/\lVert\lambda\rVert$ is the derivative of the normalization map $\lambda\mapsto\hat\lambda$: it first removes the component of $\dot\lambda$ parallel to the quaternion and then converts the remainder, treated as the derivative of a unit quaternion, into an angular velocity. By the orthogonality of $Q$, $N_{\mathrm q}^{+}N_{\mathrm q}=\mathbf 1_3$ for every nonzero stored value, so $v\to\dot q\to v$ is the identity, whereas $N_{\mathrm q}N_{\mathrm q}^{+}=\mathbf 1_4-\hat\lambda\hat\lambda^{\mathsf T}$, so an arbitrary $\dot q\to v\to\dot q$ is the orthogonal projection onto the tangent space. This is exactly the statement of Section 4.5 of the conventions: a component of $\dot q$ parallel to the quaternion does not represent a physical angular velocity.

### 3.2 Angular-rate map of the Ball-RPY joint and its singularity

The position block of a Ball-RPY joint is $\boldsymbol\eta=(\eta_1,\eta_2,\eta_3)$, that is roll, pitch and yaw, and the velocity block is $\boldsymbol\omega_{FM\_F}$. Its composition $R_{FM}=R_z(\eta_3)R_y(\eta_2)R_x(\eta_1)$ has the same form as the $R_{AC}$ of Section 2.6 of the bushing chapter, so the angular-rate maps are the two matrices of Section 2.7 of that chapter:

$$
\boldsymbol\omega_{FM\_F}=E(\boldsymbol\eta)\,\dot{\boldsymbol\eta},
\qquad
\dot{\boldsymbol\eta}=H(\boldsymbol\eta)\,\boldsymbol\omega_{FM\_F},
\qquad
E=\begin{bmatrix}\mathrm c_3\mathrm c_2&-\mathrm s_3&0\\ \mathrm s_3\mathrm c_2&\mathrm c_3&0\\ -\mathrm s_2&0&1\end{bmatrix},
\qquad
H=E^{-1}=\begin{bmatrix}\frac{\mathrm c_3}{\mathrm c_2}&\frac{\mathrm s_3}{\mathrm c_2}&0\\ -\mathrm s_3&\mathrm c_3&0\\ \frac{\mathrm c_3\mathrm s_2}{\mathrm c_2}&\frac{\mathrm s_3\mathrm s_2}{\mathrm c_2}&1\end{bmatrix},
$$

with $\mathrm c_i=\cos\eta_i$ and $\mathrm s_i=\sin\eta_i$. Thus $N_{\mathrm r}(\boldsymbol\eta)=H(\boldsymbol\eta)$ and $N_{\mathrm r}^{+}(\boldsymbol\eta)=E(\boldsymbol\eta)$; the two are inverses of each other and neither contains the roll angle. The implementation of the rate map does not form $H$ but solves the three scalar equations of $v=E\dot{\boldsymbol\eta}$ in turn: $\dot\eta_1=(\mathrm c_3\omega_1+\mathrm s_3\omega_2)/\mathrm c_2$, $\dot\eta_2=-\mathrm s_3\omega_1+\mathrm c_3\omega_2$, $\dot\eta_3=\mathrm s_2\dot\eta_1+\omega_3$, which is equivalent to multiplying by $H$. Since $\det E=\cos\eta_2$, $\cos\eta_2=0$ is the singularity of these angular coordinates: there $E$ loses rank, $H$ is unbounded and the map $v\to\dot q$ is undefined, whereas $\dot q\to v$ and pose evaluation are defined at every angle. The local invertibility condition is $\cos\eta_2\ne0$; the principal branch $\lvert\eta_2\rvert<\tfrac\pi2$ selected by the angle extraction is only one connected component of it, not the whole invertible domain; the singularity belongs to the angular coordinates themselves and has nothing to do with the motion of the bodies.

### 3.3 Single-axis joints and welds

For revolute and prismatic joints $\dot q_B=v_B$ and $N_B=1$; a weld has no coordinates. $N(q)$ is therefore block diagonal, with identity blocks everywhere except the quaternion blocks $N_{\mathrm q}$ and the Ball-RPY blocks $N_{\mathrm r}$; $N^{+}(q)$ has the same block structure, $N^{+}N=\mathbf 1_{n_v}$ holds wherever $N$ is defined, and $NN^{+}$ is the identity only when there is no free body.

## 4. Poses, spatial velocities and loads

### 4.1 Poses and body-fixed points

Poses are composed joint by joint outward from the world. For a joint whose inboard frame F is on the parent body P and whose outboard frame M is on body B,

$$
R_{WB}=R_{WP}R_{PF}R_{FM}(q_B)R_{MB},
\qquad
\mathbf p_{WoBo}=\mathbf p_{WoPo}+R_{WP}\left(\mathbf p_{PoFo\_P}+R_{PF}\,\mathbf p_{FoMo\_F}(q_B)+R_{PF}R_{FM}\,\mathbf p_{MoBo\_M}\right),
$$

while a free body has directly $R_{WB}=R(\hat\lambda)$ and $\mathbf p_{WoBo}=\mathbf p_{WoBo\_W}$. A fixed frame has the pose $R_{WF}=R_{WB}R_{BF}$, $\mathbf p_{WoFo}=\mathbf p_{WoBo}+R_{WB}\mathbf p_{BoFo\_B}$. A point fixed to body B with body coordinates $\mathbf r_Q^B$ sits at $\mathbf p_{WoBo}+R_{WB}\mathbf r_Q^B$ in the world. Every frame origin resolves to a pair (body, body coordinates), which is the identity of a wrench application point: a frame fixed to a welded body still resolves to that body itself and to coordinates in its own frame; welding does not change which body an endpoint belongs to (Section 3.1 of the shared force-element chapter). If the rigid-body tree traverses a revolute, prismatic or weld joint in reverse, this section and those that follow take the joint's declared child frame as F and its declared parent frame as M, with the axis of a single-axis joint taken as $-\mathbf a$; the value and sign of the coordinate $q_B$ are unchanged.

### 4.2 Spatial velocities and their shifts

The spatial velocity of body B, $V_B=(\boldsymbol\omega_{WB\_W};\mathbf v_{WBo\_W})$, is referred to the body origin. The velocity of another point Q of the same body is shifted by $\mathbf v_{WQ}=\mathbf v_{WBo}+\boldsymbol\omega_{WB}\times\mathbf p_{BoQ}$, written with the shift operator as

$$
V_Q=\Phi(\mathbf p_{BoQ})^{\mathsf T}V_B,
\qquad
\Phi(\mathbf p)=\begin{bmatrix}\mathbf 1&[\mathbf p]_\times\\ \mathbf 0&\mathbf 1\end{bmatrix},
\qquad
\Phi(\mathbf p)\Phi(\mathbf p')=\Phi(\mathbf p+\mathbf p').
$$

$\Phi$ itself shifts wrenches: $\mathcal W_O=\Phi(\mathbf p_{OQ})\mathcal W_Q$ is the $\boldsymbol\tau_O=\boldsymbol\tau_Q+\mathbf p_{OQ}\times\mathbf f$ of Section 5.2 of the conventions; $\Phi^{\mathsf T}$ shifts velocities; shifting an acceleration between two points of the same body carries the additional centripetal term $\boldsymbol\omega\times(\boldsymbol\omega\times\mathbf p_{OQ})$ and cannot be done by $\Phi^{\mathsf T}$ alone, see the corresponding term of the bias acceleration in Section 6.2; a spatial inertia is shifted by $M_O=\Phi(\mathbf p_{OQ})M_Q\Phi(\mathbf p_{OQ})^{\mathsf T}$. The power $\mathcal W_Q\cdot V_Q=\mathcal W_Q^{\mathsf T}\Phi(\mathbf p_{OQ})^{\mathsf T}V_O=\mathcal W_O\cdot V_O$ does not depend on the reference point.

Velocities are recursed outward from the world. The joint gives the spatial velocity of M relative to F, $V_{FM}=S_{FM}v_B$, where the columns of $S_{FM}$ are $(\mathbf a;\mathbf 0)$ for a revolute joint, $(\mathbf 0;\mathbf a)$ for a prismatic joint, the three columns of $(\mathbf 1_3;\mathbf 0)$ for a Ball-RPY joint and $\mathbf 1_6$ for a free body, none of which depends on $q_B$. Shifting it to $B_o$ and re-expressing it in the world frame gives the motion subspace of B, and from it the velocity recursion and the Jacobian:

$$
S_B=\begin{bmatrix}R_{WF}&\mathbf 0\\ \mathbf 0&R_{WF}\end{bmatrix}\Phi(\mathbf p_{MoBo\_F})^{\mathsf T}S_{FM},
\qquad
V_B=\Phi(\mathbf p_{PoBo})^{\mathsf T}V_P+S_Bv_B,
\qquad
V_B=J_Bv,
\qquad
J_B=\sum_{K\preceq B}\Phi(\mathbf p_{KoBo})^{\mathsf T}S_K\,\Pi_K,
$$

where $K\preceq B$ runs over B itself and every body on its path to the world, and $\Pi_K$ selects the block $v_K$ from $v$; coordinates off that path contribute nothing to $V_B$. $S_B$ is a $6\times n_{v,B}$ matrix that depends on the configuration only. A fixed frame F has the spatial velocity $V_F=\Phi(\mathbf p_{BoFo})^{\mathsf T}V_B$: a frame offset from the body origin has a different translational velocity because of $\boldsymbol\omega\times\mathbf p$.

The multibody layer offers a query for the spatial velocity of a frame F relative to a frame M, expressed in E, whose contract is

$$
V_{MF}^{W}=V_F-\Phi(\mathbf p_{MoFo\_W})^{\mathsf T}V_M,
\qquad
V_{MF}^{E}=\begin{bmatrix}R_{WE}^{\mathsf T}&\mathbf 0\\ \mathbf 0&R_{WE}^{\mathsf T}\end{bmatrix}V_{MF}^{W}.
$$

The angular component is $\boldsymbol\omega_{WF}-\boldsymbol\omega_{WM}$; the translational component is the velocity of $F_o$ measured in M, obtained by first shifting the velocity of M to $F_o$ and then taking the difference, not the simple difference of the world velocities of the two origins. This is precisely the multibody source of the relative velocity $\mathbf u$ with its transport term in Section 2.3 of [Force-element kinematics and spatial wrenches](../force_elements/FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.en.md).

### 4.3 Body-wrench entries and their shift to the body origin

A body-wrench entry `AppliedBodyWrench` is the five-tuple (body B, application point $\mathbf r_Q^B$ in the body frame, expressed-in frame E, moment $\boldsymbol\tau_Q^E$ about Q, force $\mathbf f^E$). The multibody layer processes each entry in three steps: it first computes the position of the application point relative to the body origin in E, $\mathbf p_{BoQ}^E=R_{WE}^{\mathsf T}R_{WB}\,\mathbf r_Q^B$; the external rigid-body tree then re-expresses the wrench in the world frame and shifts it to the body origin,

$$
\mathcal W_{Bo}^{W}=\Phi\left(\mathbf p_{BoQ}^W\right)\begin{bmatrix}R_{WE}&\mathbf 0\\ \mathbf 0&R_{WE}\end{bmatrix}\mathcal W_Q^E,
\qquad
\mathbf p_{BoQ}^W=R_{WE}\,\mathbf p_{BoQ}^E=R_{WB}\,\mathbf r_Q^B,
\qquad
\boldsymbol\tau_{Bo}^W=R_{WE}\boldsymbol\tau_Q^E+\mathbf p_{BoQ}^W\times R_{WE}\mathbf f^E,
\qquad
\mathbf f^W=R_{WE}\mathbf f^E;
$$

finally the result is accumulated into the applied spatial force of that body, giving $\mathcal W_{\mathrm{app},Bo}$. Changing the expressed-in frame and moving the reduction point are two distinct operations, and the force itself is unchanged by the shift. No generalized-force projection takes place at this step: the entry is merely shifted to the body origin and accumulated, and the projection $S^{\mathsf T}(\cdot)$ happens in the tip-to-base sweep of Section 6.2. Section 3.5 of the shared force-element chapter describes this same step from the force-element side.

### 4.4 Power conjugacy and the generalized force $\tau_{\mathrm{app}}=J^{\mathsf T}\mathcal W$

A wrench $\mathcal W_Q$ acting at a point Q fixed to body B delivers the power $\mathcal P=\mathcal W_Q\cdot V_Q$ to the system (Section 4.1 of the shared force-element chapter). The spatial velocity of Q is $V_Q=J_Qv$ with $J_Q=\Phi(\mathbf p_{BoQ})^{\mathsf T}J_B$, so $\mathcal P=\left(J_Q^{\mathsf T}\mathcal W_Q\right)\cdot v$ and the generalized force conjugate to $v$ is

$$
\tau_{\mathrm{app}}=J_Q^{\mathsf T}\mathcal W_Q=J_B^{\mathsf T}\Phi(\mathbf p_{BoQ})\mathcal W_Q=J_B^{\mathsf T}\mathcal W_{Bo},
$$

that is, shifting the wrench to the body origin and projecting with the body-origin Jacobian gives the same generalized force as projecting with the Jacobian of the application point. By the structure of $J_B$ in Section 4.2, the block of $\tau_{\mathrm{app}}$ belonging to a body K on the path is $S_K^{\mathsf T}\Phi(\mathbf p_{KoBo})\mathcal W_{Bo}$: shift the wrench to $K_o$, then project onto the motion subspace of K. Summed over all bodies, each K receives the projection of the sum of the applied wrenches of every body in its subtree shifted to $K_o$, which one tip-to-base sweep computes: each node projects the wrench accumulated at its own $B_o$ with $S_B^{\mathsf T}$ to obtain its block, then shifts that wrench to the parent origin and adds it to the parent. The implementation never forms $J$ explicitly; the same projection is carried out node by node either by this sweep (the gravity generalized-force query) or by the $S_B^{\mathsf T}$ inside the articulated-body recursion of Section 6 (forward dynamics). The multibody layer also offers a query for the $J_Q$ of a body-fixed point Q, returning the angular-velocity and point-velocity blocks, each $3\times n_v$.

### 4.5 Generalized forces of gravity and joint damping

Gravity acts at the centre of mass. Its equivalent wrench about the body origin is the force $m\mathbf g_W$ with the moment $\mathbf c_W\times m\mathbf g_W$, $\mathbf c_W=R_{WB}\mathbf c$:

$$
\mathcal W_{\mathrm g,Bo}=\begin{bmatrix}\mathbf c_W\times m\mathbf g_W\\ m\mathbf g_W\end{bmatrix},
\qquad
\tau_{\mathrm g}=\sum_BJ_B^{\mathsf T}\mathcal W_{\mathrm g,Bo},
\qquad
\tau_{\mathrm g}\cdot v=-\frac{dU}{dt},
\qquad
U=-\sum_Bm_B\,\mathbf g_W\cdot\mathbf p_{Wo\,\mathrm{cm},B},
$$

where $\mathbf p_{Wo\,\mathrm{cm},B}=\mathbf p_{WoBo}+\mathbf c_W$ is the centre-of-mass position. With a nonzero offset $\mathbf c$ one cannot write $m\mathbf g_W$ alone: the moment term varies with the attitude and is what lets gravity do work on the rotational coordinates. $U$ is the gravitational potential energy and $\tau_{\mathrm g}$ is a conservative force. The external rigid-body tree adds $\mathcal W_{\mathrm g,Bo}$ of every body into the body spatial forces and obtains $\tau_{\mathrm g}$ with the tip-to-base sweep of Section 4.4.

Joint damping is the only velocity-dependent applied generalized force the multibody layer itself provides (the inertial bias is not an applied force). For a revolute joint $\tau_{\mathrm d}=-c\,\dot\theta$, with $c\ge0$ declared joint by joint in the vehicle definition (zero allowed) and passed in as a parameter when the joint is created during assembly; a prismatic joint has the same form, $\tau_{\mathrm d}=-c\,\dot d$. It is written directly on that joint's own coordinate and is a generalized force, not a body wrench; the Ball-RPY joint and the free body carry no joint damping, and any rotational restraint a ball joint needs is provided by a force element. The external rigid-body tree first accumulates the gravity body forces in `CalcForceElementsContribution` and adds the damping generalized forces at its end through `AddJointDampingForces`; together these are the loads the multibody layer carries before any component load.

## 5. Equations of motion

### 5.1 $M(q)\dot v+C(q,v)v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}$

Differentiating $V_B=J_Bv$ of Section 4.2 gives $A_B=J_B\dot v+\dot J_Bv$, where $A_B=(\boldsymbol\alpha_{WB};\mathbf a_{WBo})$ and $\mathbf a_{WBo}$ is the acceleration of the material point $B_o$. The Newton–Euler equation of body B about $B_o$ reads

$$
M_BA_B+F_{\mathrm b,B}=F_{\mathrm{tot},B},
\qquad
F_{\mathrm b,B}=m\begin{bmatrix}\boldsymbol\omega\times G_W\boldsymbol\omega\\ \boldsymbol\omega\times(\boldsymbol\omega\times\mathbf c_W)\end{bmatrix},
\qquad
\boldsymbol\omega=\boldsymbol\omega_{WB\_W},
$$

where $F_{\mathrm{tot},B}$ is the sum of all spatial forces acting on B shifted to $B_o$, including component wrenches, gravity and the forces transmitted by the joints. $F_{\mathrm b,B}$ is the velocity bias: its rotational part is the gyroscopic moment $\boldsymbol\omega\times I_{Bo}\boldsymbol\omega$, which in general remains nonzero when the centre-of-mass offset vanishes; its translational part is the centripetal acceleration of the centre of mass relative to the body origin times the mass, and only this term vanishes with the offset. The constraint forces of ideal joints come in pairs and may transmit power between two adjacent bodies; what vanishes is the total power of each constraint pair, $\sum_BJ_B^{\mathsf T}(\text{constraint forces})=\mathbf 0$; multiplying by $J_B^{\mathsf T}$ on the left and summing over all bodies gives

$$
M(q)\dot v+C(q,v)\,v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}},
\qquad
M(q)=\sum_BJ_B^{\mathsf T}M_BJ_B,
\qquad
C(q,v)\,v=\sum_BJ_B^{\mathsf T}\left(M_B\dot J_Bv+F_{\mathrm b,B}\right),
$$

where $\tau_{\mathrm{app}}=\sum_BJ_B^{\mathsf T}\mathcal W_{\mathrm{app},Bo}$ is the projection of all body wrenches of Section 4.3 shifted to the body origins, plus any applied generalized force written directly on joint coordinates (empty in the system, Section 7.1). $M(q)$ is symmetric, positive definite under the conditions of Section 8, and depends on $q$ only; $C(q,v)v$ is a quadratic form in the velocities, and the multibody layer computes only this vector without claiming to construct any particular matrix $C(q,v)$. Here $\dot J_Bv$ is the spatial acceleration of B when $\dot v=0$ for the whole tree; it accumulates along the tree from the node-local bias accelerations $A_{\mathrm b,B}$ of Section 6.2, $\dot J_Bv=\Phi(\mathbf p_{PoBo})^{\mathsf T}\dot J_Pv+A_{\mathrm b,B}$ with $\dot J_Wv=\mathbf 0$; the two must not be identified, or the bias already produced at the parent would be counted twice.

### 5.2 Inverse-dynamics form and the bias term

For a prescribed $\dot v$ the same equations are written in the inverse-dynamics form of Section 5.3 of the conventions,

$$
\tau_{\mathrm{required}}=M(q)\dot v+C(q,v)v-\tau_{\mathrm g}-\tau_{\mathrm d},
$$

the generalized force that must still be applied on the joint coordinates to realize $\dot v$. The external rigid-body tree obtains it with two sweeps: base to tip, from $\dot v$, the spatial acceleration of every body, $A_B=\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+A_{\mathrm b,B}+S_B\dot v_B$; tip to base, $F_B=M_BA_B+F_{\mathrm b,B}-F_{\mathrm{app},B}+\sum_C\Phi(\mathbf p_{BoCo})F_C$, the spatial force the joint must transmit to B, which is then shifted to the joint's outboard origin $M_o$, re-expressed in F, projected with $S_{FM}^{\mathsf T}$, and reduced by the applied generalized force on that joint. Projecting with $S_{FM}^{\mathsf T}$ in F about $M_o$ gives the same number as projecting with $S_B^{\mathsf T}$ in W about $B_o$, because $S_B$ is exactly $S_{FM}$ after the same re-expression and shift (Section 4.2) and power does not depend on the reference point or the expressed-in frame. Of the four queries of the multibody layer, `CalcVelocityBiasGeneralizedForces` and `CalcRequiredGeneralizedForces` run these two sweeps: the former takes $\dot v=0$ with no load and yields $C(q,v)v$; the latter feeds gravity and damping into inverse dynamics as known loads and yields $\tau_{\mathrm{required}}$. The other two do not pass through inverse dynamics: `CalcGravityAppliedGeneralizedForces` applies the tip-to-base projection of Section 4.4 to the gravity body forces and yields $\tau_{\mathrm g}$; `CalcJointDampingAppliedGeneralizedForces` reads out directly the $\tau_{\mathrm d}$ of Section 4.5 written on the joint coordinates.

### 5.3 On-demand assembly of the mass matrix

$M(q)$ is assembled only when `CalcGeneralizedMassMatrix` is queried, by the composite-body algorithm in the world frame in two steps: the composite-body inertias are formed first, tip to base, and the blocks of the mass matrix are then formed from them. Welding B to every body of its subtree gives a composite body whose inertia about $B_o$ is recursed tip to base,

$$
K_B=M_B+\sum_C\Phi(\mathbf p_{BoCo})K_C\Phi(\mathbf p_{BoCo})^{\mathsf T},
$$

and the diagonal and off-diagonal blocks of $M$ are

$$
M_{BB}=S_B^{\mathsf T}K_BS_B,
\qquad
M_{KB}=M_{BK}^{\mathsf T}=S_K^{\mathsf T}\Phi(\mathbf p_{KoBo})K_BS_B\quad(K\prec B),
\qquad
M_{KB}=\mathbf 0\quad(K\text{ and }B\text{ not on one path to the world}),
$$

which is exactly $M=\sum_BJ_B^{\mathsf T}M_BJ_B$ expanded with the structure of $J_B$ from Section 4.2: $K_BS_B$ is the spatial force produced by the subtree of B under a unit $\dot v_B$, shifted level by level to the ancestor origins and projected. A weld has no block of its own; its body is merely merged into the composite. Rows and columns follow the model's generalized-velocity order, and the entries of $M\dot v$ are forces for translational coordinates and moments for rotational ones. The forward-dynamics path (Section 6) neither assembles nor factorizes $M$, and the right-hand-side evaluation of Section 7 never passes through this section.

## 6. Forward dynamics: the articulated-body algorithm

Forward dynamics solves $M(q)\dot v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}-C(q,v)v$, and the implementation uses the articulated-body algorithm: $M$ is not assembled, and $\dot v$ is obtained by three sweeps along the tree. All quantities are expressed in the world frame and reduced about the body origins; every node B has a parent node P, child nodes C and a motion subspace $S_B$. The starting point is that the equation of every body can be written $F_B=P_BA_B+Z_B$, where $F_B$ is the spatial force (about $B_o$) transmitted to B by its inboard relation and $P_B$, $Z_B$ depend only on the subtree of B; the condition of an ideal joint is $S_B^{\mathsf T}F_B=\tau_{\mathrm{app},B}$, where $\tau_{\mathrm{app},B}$ is the generalized force written on the coordinates of B's inboard relation (joint damping and applied joint forces; zero for a free body).

### 6.1 Tip-to-base recursion of the articulated-body inertia

A leaf has $P_B=M_B$. A general node first shifts the joint-projected inertias of its children to $B_o$ and adds its own inertia, then projects across its own joint:

$$
P_B=M_B+\sum_C\Phi(\mathbf p_{BoCo})P_C^{+}\Phi(\mathbf p_{BoCo})^{\mathsf T},
\qquad
U_B=S_B^{\mathsf T}P_B,
\qquad
D_B=U_BS_B=S_B^{\mathsf T}P_BS_B,
$$

$$
g_B=P_BS_BD_B^{-1}=\left(D_B^{-1}U_B\right)^{\mathsf T},
\qquad
P_B^{+}=P_B-g_BU_B=P_B-P_BS_BD_B^{-1}S_B^{\mathsf T}P_B.
$$

$D_B$ is the $n_{v,B}\times n_{v,B}$ hinge inertia; when symmetric positive definite it is Cholesky-factorized (source `llt_D_B`), and both $g_B$ and $P_B^{+}$ are obtained from that factorization. $P_B^{+}$ is the inertia of the subtree of B as felt by the parent through the joint: the response along the free directions of the joint has been solved out and removed from $P_B$, so $P_B^{+}S_B=\mathbf 0$. This sweep depends on $q$ only.

### 6.2 Tip-to-base recursion of the residual force

The second sweep carries the loads. $F_{\mathrm{app},B}=\mathcal W_{\mathrm g,Bo}+\mathcal W_{\mathrm{app},Bo}$ is the total applied spatial force accumulated at $B_o$, that is, the gravity wrench of Section 4.5 plus all body wrenches of Section 4.3 shifted to $B_o$; $F_{\mathrm b,B}$ is the velocity bias of Section 5.1; $A_{\mathrm b,B}$ is the node-local bias acceleration, the spatial acceleration of B when the parent acceleration is $A_P=\mathbf 0$ and $\dot v_B=0$; it does not contain the parent's own acceleration, and the acceleration $\dot J_Bv$ of B when $\dot v=0$ for the whole tree accumulates from it along the tree (Section 5.1). A counterexample shows the difference: a body welded to its parent at the same origin has zero local bias, yet the centripetal acceleration it inherits may be nonzero. In closed form,

$$
A_{\mathrm b,B}=\begin{bmatrix}\boldsymbol\omega_{WP}\times\boldsymbol\omega_{PB}\\ \boldsymbol\omega_{WP}\times(\boldsymbol\omega_{WP}\times\mathbf p_{PoBo})+2\,\boldsymbol\omega_{WP}\times\mathbf v_{PBo}\end{bmatrix}+\begin{bmatrix}R_{WF}&\mathbf 0\\ \mathbf 0&R_{WF}\end{bmatrix}\begin{bmatrix}\mathbf 0\\ \boldsymbol\omega_{FM}\times(\boldsymbol\omega_{FM}\times\mathbf p_{MoBo\_F})\end{bmatrix},
$$

where $(\boldsymbol\omega_{PB};\mathbf v_{PBo})=S_Bv_B$ is the spatial velocity of B relative to its parent (in W); the first term collects the centripetal and Coriolis contributions of the parent's motion, the second is the centripetal term of the offset from the joint's outboard origin $M_o$ to the body origin under the joint angular velocity. Since $S_{FM}$ of all five relations is independent of $q_B$, the joint itself contributes no other bias term. The recursion is

$$
Z_B=F_{\mathrm b,B}-F_{\mathrm{app},B}+\sum_C\Phi(\mathbf p_{BoCo})Z_C^{+},
\qquad
e_B=\tau_{\mathrm{app},B}-S_B^{\mathsf T}Z_B,
\qquad
Z_{\mathrm b,B}=P_B^{+}A_{\mathrm b,B},
\qquad
Z_B^{+}=Z_B+Z_{\mathrm b,B}+g_Be_B.
$$

The signs and ownership of the terms follow the source: the applied spatial force enters $Z_B$ with a minus sign and the bias force with a plus sign; $e_B$ is the applied generalized force on the joint minus the projection of the residual force; $Z_{\mathrm b,B}$ depends on $(q,v)$ only, while $Z_B$, $e_B$ and $Z_B^{+}$ depend on the loads. The derivation is as follows: substituting $A_B=A_B^{+}+S_B\dot v_B$ (with $A_B^{+}$ as in Section 6.3) into $F_B=P_BA_B+Z_B$ and the joint condition $S_B^{\mathsf T}F_B=\tau_{\mathrm{app},B}$ gives $D_B\dot v_B=e_B-U_BA_B^{+}$; substituting back gives $F_B=P_B^{+}A_B^{+}+Z_B+g_Be_B$, and expanding with $A_B^{+}=\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+A_{\mathrm b,B}$ gives $F_B=P_B^{+}\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+Z_B^{+}$. Shifting $F_B$ to $P_o$ yields its contribution to the parent's equation, $\Phi(\mathbf p_{PoBo})P_B^{+}\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+\Phi(\mathbf p_{PoBo})Z_B^{+}$, which are the summands of Section 6.1 and of this section.

### 6.3 Base-to-tip recovery of the accelerations

The third sweep proceeds outward from the world ($A_W=\mathbf 0$):

$$
A_B^{+}=\Phi(\mathbf p_{PoBo})^{\mathsf T}A_P+A_{\mathrm b,B},
\qquad
\dot v_B=D_B^{-1}\left(e_B-U_BA_B^{+}\right),
\qquad
A_B=A_B^{+}+S_B\dot v_B.
$$

$A_B^{+}$ is the spatial acceleration of B when the current configuration and velocities are kept and this node's generalized acceleration is set to $\dot v_B=0$, not the acceleration with the joint locked; $D_B^{-1}$ is applied through the Cholesky factorization of Section 6.1, the source first forming $\nu_B=D_B^{-1}e_B$ and then subtracting $g_B^{\mathsf T}A_B^{+}=D_B^{-1}U_BA_B^{+}$. After the three sweeps $\dot v$ equals $M(q)^{-1}\left(\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}-C(q,v)v\right)$, and the $A_B$ of every body are obtained at the same time.

### 6.4 Degenerate cases: zero-mobility joints and free bodies

Weld, $n_v=0$: $S_B$ is empty, $U_B$, $D_B$, $g_B$ and $e_B$ do not appear, $P_B^{+}=P_B$, $Z_B^{+}=Z_B+P_BA_{\mathrm b,B}$ and $A_B=A_B^{+}$. A welded body merely merges its inertia and loads into its parent and moves with the parent body; a body welded to the world has $A_B=\mathbf 0$.

Free body, $S_B=\mathbf 1_6$: $U_B=P_B$, $D_B=P_B$, $g_B=\mathbf 1_6$ and $P_B^{+}=\mathbf 0$; its inboard relation carries no generalized force, $\tau_{\mathrm{app},B}=\mathbf 0$, so $e_B=-Z_B$ and $Z_B^{+}=\mathbf 0$, and neither inertia nor load is transmitted to the world. With the world as parent, $A_P=\mathbf 0$ and $\boldsymbol\omega_{WP}=\mathbf 0$, and the outboard frame is the body frame itself, so $A_{\mathrm b,B}=\mathbf 0$ and $A_B^{+}=\mathbf 0$,

$$
P_B\,\dot v_B=F_{\mathrm{app},B}-F_{\mathrm b,B}-\sum_C\Phi(\mathbf p_{BoCo})Z_C^{+},
$$

the six equations of the free body: on the left the articulated-body inertia of the body together with the subtree hanging from it, on the right the applied loads, the velocity bias and the residual forces transmitted by the subtree. Here the Cholesky factorization of $D_B=P_B$ requires $P_B$ to be positive definite; the subtree terms in $P_B$ are positive semidefinite, and $M_B$ is positive definite if and only if $m>0$ and the central inertia is positive definite, which is the requirement the vehicle assembly places on free bodies in Section 2.3.

### 6.5 Equivalence with the mass-matrix method and complexity

In exact arithmetic the articulated-body algorithm and assembling $M(q)$ followed by a linear solve give the same $\dot v$: the derivation of Section 6.2 is nothing but a block elimination of $M\dot v=\tau_{\mathrm g}+\tau_{\mathrm d}+\tau_{\mathrm{app}}-C(q,v)v$ along the tree structure, and the Cholesky factorization of $D_B$ corresponds to the pivot of each joint block in that elimination. The work per node in the three sweeps depends only on $n_{v,B}\le6$, so the total cost grows linearly with the number of bodies; assembling $M$ grows at most quadratically with the number of bodies and factorizing $M$ cubically with $n_v$. The two algorithms differ in rounding but not in modelling; the forward path neither assembles nor factorizes the global mass matrix.

## 7. Assembly of the complete right-hand side

### 7.1 The three load sources

The continuous state of the system is the $[q;v;z]$ of Section 5.1 of the conventions, where $z$ holds only the internal force states of series spring-viscous dampers. The loads delivered to the multibody layer in one right-hand-side evaluation come from three plans, each writing a number of body-wrench entries of Section 4.3:

- The vehicle force plan: the five force-element families each write two load entries as in Section 5.2 of the shared force-element chapter, and $\dot z$ at the same time (Section 3.4 of the [series spring-viscous-damper element](../force_elements/SERIES_SPRING_VISCOUS_DAMPER.en.md)).
- The wheel-rail contact force plan: exactly one wrench per wheel-rail interface, acting on the wheel body, applied at the wheel body origin and expressed in the world frame, a zero wrench when there is no contact; the path from the multibody state to the contact inputs and on to this wrench is Section 7 of the contact-force-plan chapter.
- The drive-moment couples of independently rotating wheels: each channel supplies a held scalar $\tau$ (not part of the continuous state) and writes one pair of pure couples, $(+\tau\,\mathbf a;\mathbf 0)$ on the wheel body and $(-\tau\,\mathbf a;\mathbf 0)$ on the named reaction body, both applied at their own body origins and expressed in the world frame, where $\mathbf a=R_{WA}\mathbf e_2$ is the world direction of the body-frame $+y$ axis of the axle body A that supplies the axis. This plan applies no wrench to the axle body itself; the joint constraint forces between the axle body and the wheel are accounted for by the recursion of Section 6.

The entries of the three plans are merged into one list handed to the multibody layer; the multibody layer receives body wrenches only, the revolute-joint moment and prismatic-joint force inputs are empty, and the system has no call-time external force input at all. Any of the plans may be absent.

### 7.2 Order of one evaluation and the state derivative $[\dot q;\dot v;\dot z]$

Given $(t,q,v,z)$ and all non-state data, one evaluation proceeds in the following order: first the kinematic inputs of every plan are computed from the same $(q,v)$ and all body wrenches are written, the vehicle force plan writing $\dot z$ at the same time; then the multibody facade applies the gravity and joint damping of Section 4.5, the wrench shift of Section 4.3 and the three sweeps of Section 6 to obtain $\dot v$, and forms $N(q)v$ from $v$ as in Section 3; finally the assembly layer concatenates the three blocks:

$$
\frac{d}{dt}\begin{bmatrix}q\\v\\z\end{bmatrix}
=\begin{bmatrix}N(q)\,v\\ \dot v(q,v,z)\\ \dot z(q,v,z)\end{bmatrix}.
$$

$\dot v$ depends on $z$ only through the force states of the series family entering the body wrenches; $\dot z$ depends on $(q,v)$ through the relative velocities of the force elements; no load depends on $\dot v$ or $\dot z$, so there is no algebraic loop among the three blocks. The multibody layer itself contains no time; $t$ enters the right-hand side only through the inputs of the plans.

### 7.3 Interface to the integrator

The integrator supplies $(t,y)$ with $y=[q;v;z]$; the right-hand-side bridge writes it into a trial context and the assembly layer evaluates $f(t,y)$ as in Section 7.2, see Section 1.1 of the time-integration chapter. Because the derivative of the $q$ block is $N(q)v$ rather than $v$, $q$ and $v$ have different dimensions whenever there is a free body, and the integrator advances $q$, $v$ and $z$ alike as state components; the quaternion is advanced as stored, and its norm is not forced to one by the equations of this chapter, see Section 8. The construction of the initial value $y_0$ is the subject of the startup chapter.

## 8. Mathematical properties and conditions of applicability

- **Tree topology and ideal joints.** The model expresses only loop-free rigid-body trees; joints are ideal, without friction, clearance or compliance, and their constraint forces do no work, so no constraint force and no constraint equation appears in the equations of motion. Structures that need closed loops, additional constraints or flexibility are outside the scope of this chapter.
- **World-frame expression and body-origin reduction.** Every spatial quantity of the recursions is expressed in the world frame and reduced about the body origin; the wrenches delivered by the components may be expressed in any frame and reduced about any body-fixed point, and are shifted to the body origin on entry. Under this convention the centre-of-mass offset enters in three places, $M_B$, $F_{\mathrm b,B}$ and the gravity moment $\mathbf c_W\times m\mathbf g_W$; dropping any one of them breaks the equivalence with the centre-of-mass formulation.
- **Positive definiteness of the mass matrix and the hinge inertias.** Every $M_B$ is positive semidefinite and so is $M(q)=\sum_BJ_B^{\mathsf T}M_BJ_B$; the articulated-body recursion requires every $D_B=S_B^{\mathsf T}P_BS_B$ to be positive definite, which is equivalent to $M(q)$ being positive definite. The assembly's $m>0$ and the positive-definite central inertia of free bodies guarantee positive-definite $D_B$ for free bodies; for a jointed body, a positive-definite $D_B$ requires its subtree to have nonzero inertia about the free directions of the joint, so, for instance, a revolute joint cannot carry only a point mass whose centre of mass lies on the axis.
- **Coordinate singularities.** The quaternion has no singularity, at the price of $n_q>n_v$ and of a norm the equations do not enforce; the Ball-RPY joint has an undefined $N_{\mathrm r}$ at $\cos(\text{pitch})=0$, a singularity of the angular coordinates and not of the dynamics, where the pose and $\dot q\to v$ remain defined.
- **Quaternion norm and linearity.** $\dot q=N(q)v$ gives $\lambda\cdot\dot\lambda=0$, so the exact solution preserves the norm; numerical stepping lets the norm drift, and since pose evaluation takes only the direction, the drift changes the scale of the quaternion block but not the attitude, while the derivative of that block scales linearly with it. When the complete right-hand side is differenced, the quaternion components are perturbed as stored and not projected back onto the unit sphere, see Section 2.3 of the time-integration chapter.
- **Energy.** Gravity is conservative, $\tau_{\mathrm g}\cdot v=-\dot U$; joint damping dissipates, each revolute joint contributing $-c\,\dot\theta^2\le0$ and each prismatic joint $-c\,\dot d^2\le0$; the $C(q,v)v$ obtained by projecting the Newton–Euler equations satisfies $v^{\mathsf T}C(q,v)v=\tfrac12v^{\mathsf T}\dot M(q)v$, so without component loads and damping the sum of the kinetic energy $T=\tfrac12v^{\mathsf T}M(q)v$ and $U$ is conserved. The power of the component wrenches is governed by the properties of each component (Section 4 of the force-element chapters or the corresponding sections).
- **Ownership of gravity and joint damping.** Both are owned by the multibody layer; a component must not deliver a gravity wrench or joint damping of its own, or they would be counted twice.
- **Inboard direction of the ball joint.** A Ball-RPY joint can be traversed by the rigid-body tree only from its declared parent to its declared child: the declared parent must lie on the side nearer the world, and a tree that would traverse a ball joint in reverse is outside the supported range. Revolute, prismatic and weld joints may be traversed in either direction with unchanged coordinate meaning (Section 4.1).
- **Rigid bodies and uniform gravity.** Bodies do not deform; gravity is a constant uniform field; the world frame is inertial and carries no acceleration effects beyond those of the line.

## 9. Source mapping

| Theoretical object | Principal implementation |
|---|---|
| Elements of the rigid-body tree: bodies, fixed frames, the four joint types and the free-body declaration; coordinate ranges | `MultibodyModel::AddRigidBody`, `AddFixedFrame`, `AddRevoluteJoint`, `AddPrismaticJoint`, `AddBallRpyJoint`, `AddWeldJoint`, `DeclareFreeBody`, `GetJointPositionRange`, `GetFreeBodyPositionRange`, in [`multibody_model.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_model.h) |
| Reachability, the six-degree-of-freedom joint of a free body and finalization | `MultibodyModel::Finalize`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Inertia parameters $(m,\mathbf c,G_o)$ and their realisability conditions | `RigidBodyInertiaParameters`, in [`multibody_physical_parameters.h`](../../../libs/multibody_runtime/include/orvd/multibody_runtime/multibody_physical_parameters.h); `ThrowIfNotRealisableInertia`, in [`multibody_physical_parameter_validation.cc`](../../../libs/multibody_runtime/src/multibody_physical_parameter_validation.cc) |
| Conversion from central inertia to unit inertia about the body origin; $m>0$ and the positive-definiteness requirement for free bodies | `UnitInertiaAboutBodyOrigin`, in [`assemble_vehicle_multibody_model.cc`](../../../libs/configuration/src/assemble_vehicle_multibody_model.cc); `ThrowIfSingularFreeBodyCenterOfMassInertia`, in [`vehicle_definition_inertia.cc`](../../../libs/configuration/src/vehicle_definition_inertia.cc) |
| Gravity vector $\mathbf g_W=g\,\mathbf e_3$ | `GravitationalAccelerationInInertial`, in [`track_inertial_frame.cc`](../../../libs/track_geometry/src/track_inertial_frame.cc); `MultibodyModel::SetGravityVector`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Quaternion blocks $N_{\mathrm q}$ and $N_{\mathrm q}^{+}$ | `CalcQMatrix`, `QuaternionRateToAngularVelocityMatrix`, `DoMapVelocityToQDot`, `DoMapQDotToVelocity`, in [`quaternion_floating_mobilizer.cc`](../../../external/drake_mbtree/drake/multibody/tree/quaternion_floating_mobilizer.cc) |
| Ball-RPY blocks $N_{\mathrm r}=H$ and $N_{\mathrm r}^{+}=E$ | `DoMapVelocityToQDot`, `DoMapQDotToVelocity`, in [`rpy_ball_mobilizer.cc`](../../../external/drake_mbtree/drake/multibody/tree/rpy_ball_mobilizer.cc) |
| The full maps $\dot q=N(q)v$ and $v=N^{+}(q)\dot q$ | `MultibodyModel::MapGeneralizedVelocitiesToPositionDerivatives`, `MapGeneralizedPositionDerivativesToVelocities`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Poses and the body-fixed-point identity of frame origins | `MultibodyModel::CalcPoseInWorld`, `CalcFrameOriginAsBodyFixedPoint`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Spatial velocities, shifted frame velocities and the relative-velocity contract | `MultibodyModel::CalcBodyFrameSpatialVelocityRelativeToWorldExpressedInWorld`, `CalcFrameSpatialVelocityRelativeToFrameExpressedInFrame`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc); `Frame::CalcSpatialVelocityInWorld`, `Frame::CalcSpatialVelocity`, in [`frame.cc`](../../../external/drake_mbtree/drake/multibody/tree/frame.cc) |
| Motion subspace $S_B$ and the velocity recursion | `BodyNodeImpl::CalcAcrossNodeJacobianWrtVExpressedInWorld`, `CalcVelocityKinematicsCache_BaseToTip`, in [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) |
| Body-wrench entries, their re-expression, shift to the body origin and accumulation | `AppliedBodyWrench`, in [`multibody_applied_forces.h`](../../../libs/multibody_model/include/orvd/multibody_model/multibody_applied_forces.h); `MultibodyModel::EvaluateForwardDynamics`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc); `RigidBody::AddInForce`, in [`rigid_body.cc`](../../../external/drake_mbtree/drake/multibody/tree/rigid_body.cc) |
| Body-fixed-point Jacobian $J_Q$ | `MultibodyModel::CalcRigidBodyPointSpatialVelocityJacobianRelativeToWorldExpressedInWorld`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Gravity wrench $(\mathbf c_W\times m\mathbf g_W;m\mathbf g_W)$ and the tip-to-base sweep for $\tau=J^{\mathsf T}\mathcal W$ | `UniformGravityFieldElement::AccumulateGravitySpatialForces`, in [`uniform_gravity_field_element.cc`](../../../external/drake_mbtree/drake/multibody/tree/uniform_gravity_field_element.cc); `BodyNodeImpl::CalcSystemJacobianTransposeTimesF_TipToBase`, in [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) |
| Joint damping $\tau_{\mathrm d}=-c\,\dot\theta$ and its inclusion | `RevoluteJoint::DoAddInDamping`, in [`revolute_joint.h`](../../../external/drake_mbtree/drake/multibody/tree/revolute_joint.h); `MultibodyTree::CalcForceElementsContribution`, `AddJointDampingForces`, in [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc) |
| Velocity bias $F_{\mathrm b,B}$, bias acceleration $A_{\mathrm b,B}$ and the two inverse-dynamics sweeps | `MultibodyTree::CalcDynamicBiasForces`, `CalcInverseDynamics`, in [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc); `BodyNodeImpl::CalcSpatialAccelerationBias`, `CalcInverseDynamics_TipToBase`, in [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc) |
| Dynamics queries: $M(q)$, $C(q,v)v$, $\tau_{\mathrm g}$, $\tau_{\mathrm d}$, $\tau_{\mathrm{required}}$ | `MultibodyModel::CalcGeneralizedMassMatrix`, `CalcVelocityBiasGeneralizedForces`, `CalcGravityAppliedGeneralizedForces`, `CalcJointDampingAppliedGeneralizedForces`, `CalcRequiredGeneralizedForces`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Composite-body inertia $K_B$ and the mass-matrix blocks | `MultibodyTree::CalcMassMatrix`, in [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc); `BodyNodeImpl::CalcMassMatrixContributionViaWorld_TipToBase`, in [`body_node_impl_mass_matrix.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl_mass_matrix.cc) |
| The three sweeps of the articulated-body algorithm: $P_B$, $D_B$, $g_B$, $P_B^{+}$; $Z_B$, $e_B$, $Z_B^{+}$; $\dot v_B$, $A_B$ | `BodyNodeImpl::CalcArticulatedBodyInertiaCache_TipToBase`, `CalcArticulatedBodyForceCache_TipToBase`, `CalcArticulatedBodyAccelerations_BaseToTip`, in [`body_node_impl.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node_impl.cc); `BodyNode::CalcArticulatedBodyHingeInertiaMatrixFactorization`, in [`body_node.cc`](../../../external/drake_mbtree/drake/multibody/tree/body_node.cc); `MultibodyTree::CalcArticulatedBodyForceCache`, `CalcArticulatedBodyAccelerations`, in [`multibody_tree.cc`](../../../external/drake_mbtree/drake/multibody/tree/multibody_tree.cc) |
| The facade's $[N(q)v;\dot v]$ | `MultibodyModel::CalcGeneralizedVelocityDerivatives`, `CalcStateTimeDerivatives`, in [`multibody_model.cc`](../../../libs/multibody_model/src/multibody_model.cc) |
| Drive-moment couples $(\pm\tau\,\mathbf a;\mathbf 0)$ | `IndependentWheelActiveTorquePlan::CalcAppliedForces`, in [`independent_wheel_active_torque_plan.cc`](../../../libs/forces/src/independent_wheel_active_torque_plan.cc); `EmitCoupleWrenchPair`, in [`body_wrench_pair.h`](../../../libs/forces/src/body_wrench_pair.h) |
| The three load sources and the concatenation into $[N(q)v;\dot v;\dot z]$ | `CompiledSystemPlan::CalcStateTimeDerivatives`, in [`compiled_system_plan.cc`](../../../libs/system_assembly/src/compiled_system_plan.cc) |
| The integrator's $(t,y)$ into the trial context | `SystemRhsBridge::CalcTimeDerivatives`, in [`system_rhs_bridge.cc`](../../../libs/integrators/src/system_rhs_bridge.cc) |
