[中文](TRACK_IRREGULARITY_SPECTRA.md)

# Track-irregularity spectra and their spatial random realization

This chapter explains how a one-sided spatial power spectral density (PSD) becomes a finite-length track-irregularity field, and how that field is placed on the two rails. It proceeds from physical quantities and units, through the spectral families, the finite band and the random realization, to the spatial envelope and the geometric placement; the final section maps the theoretical objects to the source.

## 1. Irregularity components and coordinate conventions

Track irregularity is the stationary random geometric deviation superimposed on the ideal line. This chapter describes it with four components, all expressed in the track frame, with the planar-projected station $s$ of [Coordinates and notation](../CONVENTIONS_AND_NOTATION.en.md) as the independent variable and the signs of its §2.9.

| Component | Symbol | Unit | Meaning and sign |
|---|---|---|---|
| Lateral | $y(s)$ | m | Displacement of both rails together along the lateral track axis, positive to the right |
| Vertical | $z(s)$ | m | Displacement of both rails together along the vertical track axis, positive downward |
| Gauge | $g(s)$ | m | Narrowing of the gauge relative to its nominal value, positive when narrower; each rail moves $g/2$ toward the centreline |
| Roll | $\phi(s)$ | rad | Rotation of the plane formed by the two rails about the track $+x$ axis, positive when the right rail drops and the left rail rises |

Cross-level is the other expression of roll: let $u$ be the height difference between the reference points of the two rails, positive when the right rail sits lower; at small angles $u$ and $\phi$ are related through the base length of section 2.3. Gauge and cross-level are lengths, as are the lateral and vertical components; roll is an angle. Each of the four components is defined independently over station.

Horizontal curvature and superelevation belong to [Track geometry and track frames](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md), grades and vertical curves to [Track vertical profile modelling and its three-dimensional coupling](../track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.en.md); localized deterministic defects such as welds, corrugation and wheel burns are described by dedicated deterministic models. This chapter treats the stationary random part.

A random irregularity case is defined by three independent choices: the spectral formula fixes the second-order energy at each spatial frequency, the finite band fixes the longest and shortest retained wavelengths, and the random realization fixes the phase at each discrete frequency. One spectrum and band admit infinitely many realizations; changing the seed changes only the phases, while changing a cutoff changes the variance and the dynamic meaning.

Each component has two possible sources: a realization generated from a spectrum (section 5), or a supplied station series. A supplied series carries its own phases, gating and interpolation history and is used as it is. Both sources end up in the same field object (section 6). AAR5, AAR6 and ERRI Low/High name spectral parameter sets in this chapter; ERRI Low/High distinguishes amplitude levels.

## 2. Spectral variables, units and conversions

### 2.1 Cyclic spatial frequency, angular wavenumber and wavelength

Let $f$ be the cyclic spatial frequency in $\mathrm{cycles/m}$, $\Omega$ the spatial angular wavenumber in $\mathrm{rad/m}$ and $\lambda$ the spatial wavelength. They satisfy

$$
\Omega=2\pi f,
\qquad
\lambda=\frac{1}{f}=\frac{2\pi}{\Omega}.
$$

### 2.2 One-sided spectra and the variance-preserving change of variable

Let $S_f(f)$ and $S_\Omega(\Omega)$ be the one-sided PSDs in $f$ and in $\Omega$. The change of variable preserves variance:

$$
\sigma^2
=\int_{f_{\min}}^{f_{\max}}S_f(f)\,df
=\int_{\Omega_{\min}}^{\Omega_{\max}}S_\Omega(\Omega)\,d\Omega.
$$

Because $d\Omega=2\pi\,df$, the two representations are related by

$$
S_f(f)=2\pi S_\Omega(2\pi f),
\qquad
S_\Omega(\Omega)=\frac{1}{2\pi}S_f\!\left(\frac{\Omega}{2\pi}\right).
$$

The change of variable acts on the frequency axis and on the PSD axis together, and polynomial coefficients expanded in $f$ convert to coefficients in $\Omega$ through it. The one-sided spectra of this chapter integrate positive spatial frequency only; a two-sided spectrum is first converted to the consistent one-sided form under its own normalization.

A vehicle travelling at speed $v_s$ over a spatial wavelength $\lambda$ experiences the temporal frequency

$$
f_t=v_s f=\frac{v_s}{\lambda}.
$$

Speed fixes the map from spatial excitation to the vehicle's temporal response; the spatial PSD of the line is independent of speed.

### 2.3 Converting a height-difference spectrum into a roll-angle spectrum

Cross-level is described by a height difference $u$, roll by an angle $\phi$. With the two measuring points of the height difference separated laterally by $b_{\mathrm{ref}}$, at small angles

$$
\phi=\frac{u}{b_{\mathrm{ref}}},
\qquad
S_\phi(\Omega)=\frac{S_u(\Omega)}{b_{\mathrm{ref}}^2}.
$$

$b_{\mathrm{ref}}$ is the reference base of the height-difference measurement and is a different quantity from the gauge. On standard-gauge track, the gauge $1.435\,\mathrm m$ is measured between the inner rail faces at a prescribed depth below the rail crowns, whereas cross-level and superelevation use the nominal spacing of the two running-surface centres, $1.5\,\mathrm m$, as their reference base; the same base turns superelevation into the track roll angle. Once a height-difference spectrum has been converted with $b_{\mathrm{ref}}$, generation, interpolation and placement all act directly on the angle.

A spectrum defined directly for the angle carries its normalizing length inside its own definition. The ERRI cross-level angle spectrum of section 3.2.2 is written this way, with half the rail-centre spacing, $b=0.75\,\mathrm m$.

## 3. Spectral models

Every rational spectrum in this chapter is written in one polynomial-quotient template,

$$
S_\Omega(\Omega)=\frac{b_0+b_2\Omega^2}{a_0+a_2\Omega^2+a_4\Omega^4+a_6\Omega^6},
$$

with $S_\Omega$ in SI units: $\mathrm{m^2/(rad/m)}$ for a length spectrum and $\mathrm{rad^2/(rad/m)}$ for an angle spectrum. The displacement spectra use $b_0$ with $a_0$, $a_2$, $a_4$ only; the ERRI cross-level angle spectrum uses $b_2$ and $a_6$ as well. Both families are given as products of cutoff factors, so the template coefficients follow by expansion. Each family is presented in the same order: the quantity and its unit, the closed form, the parameter table, the template coefficients.

### 3.1 AAR spectra

The AAR family describes six track classes with the roughness coefficients $A_a$, $A_v$, the cutoff wavenumbers $\Omega_c$, $\Omega_s$ and the constant $k$; the closed forms and parameter table are given in chapter 5 of Lei, [High Speed Railway Track Dynamics](https://doi.org/10.1007/978-981-10-2039-1). The closed forms are in $\mathrm{cm^2/(rad/m)}$, with $A_a$ and $A_v$ in $\mathrm{cm^2\,rad/m}$; the SI spectrum follows by multiplying by $10^{-4}$. The two classes implemented here have the following parameters.

| Class | $A_a$ | $A_v$ | $\Omega_c$ / $\mathrm{rad\,m^{-1}}$ | $\Omega_s$ / $\mathrm{rad\,m^{-1}}$ | $k$ |
|---|---:|---:|---:|---:|---:|
| AAR5 | 0.0762 | 0.2095 | 0.8245 | 0.8209 | 0.25 |
| AAR6 | 0.0339 | 0.0339 | 0.8245 | 0.4380 | 0.25 |

#### 3.1.1 Lateral and vertical

The lateral and vertical displacement spectra are single-cutoff spectra:

$$
S_{y}(\Omega)
=\frac{kA_a\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)},
\qquad
S_{z}(\Omega)
=\frac{kA_v\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)}.
$$

With $A$ standing for $A_a$ or $A_v$ of the respective direction, the SI template coefficients are

$$
a_0=0,
\qquad
a_2=\Omega_c^2,
\qquad
a_4=1,
\qquad
b_0=kA\cdot10^{-4}\,\Omega_c^2.
$$

$a_0=0$ is the origin of the $\Omega^{-2}$ growth of this spectrum at low angular wavenumber.

#### 3.1.2 Gauge and cross-level

The gauge narrowing $g$ and the cross-level height difference $u$ share one double-cutoff spectrum whose second cutoff $\Omega_s$ depends on the class:

$$
S_{g}(\Omega)=S_{u}(\Omega)
=\frac{4kA_v\Omega_c^2}
{(\Omega^2+\Omega_c^2)(\Omega^2+\Omega_s^2)}.
$$

The same form appears in Xin et al., [Research on characterization methods for track irregularities](https://doi.org/10.1007/s40534-024-00367-z). The SI template coefficients are

$$
a_0=\Omega_c^2\Omega_s^2,
\qquad
a_2=\Omega_c^2+\Omega_s^2,
\qquad
a_4=1,
\qquad
b_0=4kA_v\cdot10^{-4}\,\Omega_c^2.
$$

The roll-angle spectrum follows from the base-length conversion of section 2.3; only the numerator changes, $b_{0,\phi}=b_0/b_{\mathrm{ref}}^2$.

### 3.2 ERRI B176 spectra

The ERRI B176 family describes the two amplitude levels Low and High with $A_a$, $A_v$ and three cutoff wavenumbers $\Omega_r$, $\Omega_c$, $\Omega_s$; the parameter table is given in chapter 5 of Lei, [High Speed Railway Track Dynamics](https://doi.org/10.1007/978-981-10-2039-1). The closed forms are written directly in SI units.

| Level | $A_a$ / $\mathrm{m\,rad}$ | $A_v$ / $\mathrm{m\,rad}$ | $\Omega_r$ / $\mathrm{rad\,m^{-1}}$ | $\Omega_c$ / $\mathrm{rad\,m^{-1}}$ | $\Omega_s$ / $\mathrm{rad\,m^{-1}}$ |
|---|---:|---:|---:|---:|---:|
| ERRI Low | $2.119\times10^{-7}$ | $4.032\times10^{-7}$ | 0.0206 | 0.8246 | 0.4380 |
| ERRI High | $6.125\times10^{-7}$ | $10.80\times10^{-7}$ | 0.0206 | 0.8246 | 0.4380 |

#### 3.2.1 Lateral and vertical displacement spectra

$$
S_{y}(\Omega)
=\frac{A_a\Omega_c^2}
{(\Omega^2+\Omega_r^2)(\Omega^2+\Omega_c^2)},
\qquad
S_{z}(\Omega)
=\frac{A_v\Omega_c^2}
{(\Omega^2+\Omega_r^2)(\Omega^2+\Omega_c^2)}.
$$

With $A$ standing for $A_a$ or $A_v$ of the respective direction, the template coefficients are

$$
a_0=\Omega_r^2\Omega_c^2,
\qquad
a_2=\Omega_r^2+\Omega_c^2,
\qquad
a_4=1,
\qquad
b_0=A\,\Omega_c^2.
$$

Substituting the parameter table gives the coefficients below; the four spectra share one denominator.

| Parameter set | Direction | $b_0$ | $a_0$ | $a_2$ | $a_4$ |
|---|---|---:|---:|---:|---:|
| ERRI Low | Lateral | $1.440846\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |
| ERRI Low | Vertical | $2.741619\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |
| ERRI High | Lateral | $4.164787\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |
| ERRI High | Vertical | $7.343623\times10^{-7}$ | $2.8855\times10^{-4}$ | 0.6803895 | 1 |

Low and High differ only in $b_0$ within one direction and have the same spectral shape; with identical frequency grids and phases, the High realization equals the Low realization multiplied by $\sqrt{b_{0,\mathrm{High}}/b_{0,\mathrm{Low}}}$.

#### 3.2.2 Cross-level angle spectrum

ERRI B176 defines the spectrum of the roll angle $\phi$ directly, with half the rail-centre spacing $b=0.75\,\mathrm m$ as the normalizing length and a third cutoff wavenumber $\Omega_s$; the same form is equation 9 of Xin et al., [Research on characterization methods for track irregularities](https://doi.org/10.1007/s40534-024-00367-z), which defines $b$ as half the distance between the left and right wheel centres and writes the unit of the angle spectrum as $\mathrm{1/(rad/m)}$, the same quantity as $\mathrm{rad^2/(rad/m)}$ here:

$$
S_{\phi}(\Omega)
=\frac{A_v\Omega_c^2\,\Omega^2}
{b^2(\Omega^2+\Omega_r^2)(\Omega^2+\Omega_c^2)(\Omega^2+\Omega_s^2)},
\qquad
[S_\phi]=\mathrm{rad^2/(rad/m)}.
$$

The template coefficients are

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

Substituting the parameter table, $b_2$ is $4.87399\times10^{-7}$ for Low and $1.305533\times10^{-6}$ for High; $a_0=5.535659\times10^{-5}$, $a_2=0.1308172$, $a_4=0.8722335$. This chapter records the angle spectrum to give the family's angle convention in full; the current random realization generates the lateral and vertical displacement channels, and the roll channel is supplied as a series.

### 3.3 Spectral shape and derivative moments

The low-wavenumber end is governed by the constant term of the denominator. A single-cutoff spectrum has $a_0=0$ and grows as $\Omega^{-2}$; its displacement variance diverges as the lower cutoff tends to zero, so a positive $f_{\min}$ is part of this spectrum. The double-cutoff spectra and the ERRI displacement spectra have $a_0>0$ and tend to a finite plateau,

$$
S_\Omega(0)=\frac{b_0}{a_0}.
$$

A finite plateau means bounded long-wave energy; the actual lower cutoff is still set by the case, and the value of the continuous PSD at zero is a different thing from a constant component in a realization, which contains strictly positive frequencies only.

The high-wavenumber end is governed by the leading term of the denominator. All displacement spectra of this chapter and the ERRI angle spectrum decay as $\Omega^{-4}$. The variance moment of the $n$th spatial derivative contains $\int\Omega^{2n}S_\Omega(\Omega)\,d\Omega$: the high-frequency tails of displacement and first slope converge, and the tail of the second derivative diverges. A finite $f_{\max}$ is therefore part of the geometric model; it fixes the highest spatial frequency received by the track, the wheel-rail contact and the vehicle model.

## 4. Finite band, sampling spacing and variance

For $0<\Omega_{\min}<\Omega_{\max}$ with $\Omega_{\min,\max}=2\pi f_{\min,\max}$, the continuous finite-band variance of a single-cutoff spectrum follows from

$$
\mathcal F(\Omega)=-\frac{1}{\Omega}
-\frac{1}{\Omega_c}\arctan\!\left(\frac{\Omega}{\Omega_c}\right)
$$

as

$$
\sigma_{\mathrm{continuous}}^2
=kA\cdot10^{-4}
\left[\mathcal F(\Omega_{\max})-\mathcal F(\Omega_{\min})\right].
$$

The denominators of the double-cutoff spectra and of the ERRI displacement spectra are products of two cutoff factors. Let $\Omega_1$ be the smaller and $\Omega_2$ the larger of the two cutoffs ($\Omega_s$ and $\Omega_c$ for the AAR gauge spectrum, $\Omega_r$ and $\Omega_c$ for the ERRI displacement spectra). Then

$$
\frac{b_0}{(\Omega^2+\Omega_1^2)(\Omega^2+\Omega_2^2)}
=\frac{b_0}{\Omega_2^2-\Omega_1^2}
\left[\frac{1}{\Omega^2+\Omega_1^2}-\frac{1}{\Omega^2+\Omega_2^2}\right],
$$

and the continuous finite-band variance is

$$
\sigma_{\mathrm{continuous}}^2
=\frac{b_0}{\Omega_2^2-\Omega_1^2}
\left[
\frac{1}{\Omega_1}\arctan\frac{\Omega}{\Omega_1}
-\frac{1}{\Omega_2}\arctan\frac{\Omega}{\Omega_2}
\right]_{\Omega_{\min}}^{\Omega_{\max}}.
$$

The denominator of the ERRI angle spectrum is a product of three cutoff factors, and the same partial-fraction decomposition yields three arctangent terms.

Lowering $f_{\min}$ adds long-wave energy and raising $f_{\max}$ adds short-wave energy; the spectral formula, the finite band and the realization together define one random case. The spacing $\Delta s$ of the spatial output grid satisfies the Nyquist condition

$$
\Delta s\,f_{\max}<\frac12,
$$

and the implementation uses the strict inequality: a harmonic exactly at the Nyquist frequency degenerates on the grid into an alternating sequence whose amplitude depends on its phase, and the strict inequality excludes that degeneracy. The sample mean square of a finite-length realization fluctuates about the discrete spectral-sum variance of section 5.1, which approaches the continuous integral as the frequency grid is refined; the end envelopes further change the whole-interval statistics.

## 5. Random-phase harmonic realization

### 5.1 Discrete frequency grid and harmonic sum

On an equidistant frequency grid that includes both endpoints and has $N\ge2$ points,

$$
f_j=f_{\min}+(j-1)\Delta f,
\qquad
\Delta f=\frac{f_{\max}-f_{\min}}{N-1},
\qquad
j=1,\ldots,N,
$$

each component uses the random-phase harmonic sum

$$
r_{\mathrm{raw}}(s)=\sum_{j=1}^{N}
\sqrt{2S_f(f_j)\Delta f}
\cos\!\left(2\pi f_j\xi+\theta_j\right),
\qquad
\xi=s-s_0,
$$

where $s_0$ is the placement start and $\theta_j$ is uniform on $[0,2\pi)$. Taking the expectation over the phases, each harmonic has zero mean and variance $S_f(f_j)\Delta f$; with mutually independent phases the sum has zero mean and variance

$$
\sigma_{\mathrm{discrete}}^2
=\sum_{j=1}^{N}S_f(f_j)\Delta f.
$$

Endpoint and interior frequencies carry the same rectangular weight, and $\sigma_{\mathrm{discrete}}^2$ approaches the continuous integral of section 4 as the grid is refined. The phase coordinate is the local station $\xi$, so moving the placement interval with the spectrum, grid and seed unchanged translates the same realization with its start.

The equidistant grid also yields an identity. Write the complex form $Z(\xi)=\sum_{j=1}^{N}\sqrt{2S_f(f_j)\Delta f}\,e^{\mathrm i(2\pi f_j\xi+\theta_j)}$ and let $T=1/\Delta f$; from $f_jT=f_{\min}/\Delta f+(j-1)$ follows $Z(\xi+T)=e^{\mathrm i2\pi f_{\min}/\Delta f}Z(\xi)$, so a shift by $T$ multiplies the complex sum by a constant phase factor and $|Z|$ is periodic with period $T$. The period of the real signal $r_{\mathrm{raw}}=\operatorname{Re}Z$ is set by $f_{\min}/\Delta f$: for a reduced fraction $p/q$ the period is $qT$, and for an irrational ratio the sum is quasi-periodic.

### 5.2 From realization seed to phases

One unsigned 64-bit realization seed yields the channel seeds through a fixed domain separation: each channel combines the seed with its own domain constant and passes the result through one bijective mixing map, which gives distinct channel seeds. Each channel drives its own deterministic pseudo-random sequence from its channel seed, producing uniform numbers $u_j$ on $[0,1)$ in order of increasing frequency, mapped to

$$
\theta_j=2\pi u_j.
$$

The construction depends only on the generation specification and the realization seed, so the same specification and seed determine the same phases. The mixing map, the domain constants and the pseudo-random engine are given in section 8.

### 5.3 Oscillator recurrence

Let the phase increment between adjacent stations be $\Delta\alpha_j=2\pi f_j\Delta s$. The implementation advances $c_n=\cos\alpha_{j,n}$ and $s_n=\sin\alpha_{j,n}$ by

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

and so avoids a trigonometric call at every station. In real arithmetic the recurrence equals the direct harmonic form; in floating point it is re-anchored to the analytic phase every fixed number of stations, which keeps the drift at rounding level.

### 5.4 Multi-channel correlation

The phase streams of the channels are driven by different channel seeds. This chapter takes mutually independent channel phases as a modelling assumption; under it the sample correlation over any finite interval fluctuates about zero. A spectral family prescribes only the auto-spectrum of each component, and the cross-spectrum between channels is a separate modelling choice: with one shared set of phases, the AAR6 lateral and vertical channels would coincide pointwise and the AAR5 channels would be in the fixed ratio $\sqrt{A_v/A_a}$, which corresponds to a fully coherent cross-spectrum; this chapter chooses a zero cross-spectrum. That gauge and cross-level share one spectral shape is likewise a modelling choice of the family; the two are different geometric quantities. Where directional coherence is required, a positive semi-definite cross-spectral matrix with a joint spectral factorization generates a multi-channel field within the same framework.

## 6. Spatial envelope, interpolation and placement on the rails

### 6.1 Quintic envelope

Let the placement interval be $[s_0,s_1]$ with fade-in and fade-out lengths $L_{\mathrm{in}}$ and $L_{\mathrm{out}}$. The quintic smoothstep is

$$
q(u)=6u^5-15u^4+10u^3,
\qquad
0\le u\le1,
$$

which satisfies

$$
q(0)=0,
\quad q(1)=1,
\quad q'(0)=q'(1)=q''(0)=q''(1)=0.
$$

The placement envelope is

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

and the final realization is

$$
r(s)=w(s)\,r_{\mathrm{raw}}(s).
$$

The construction requires $s_1>s_0$, $L_{\mathrm{in}}>0$, $L_{\mathrm{out}}>0$ and $L_{\mathrm{in}}+L_{\mathrm{out}}\le s_1-s_0$; equality corresponds to the two fades meeting at a single full-amplitude point. The finite harmonic sum is smooth, so the analytic product $r(s)$ has continuous displacement, first slope and second derivative across the joints between the zero, fade and full-amplitude segments.

### 6.2 Interpolation and domain

The generated samples are multiplied by the envelope and cropped to $[s_0,s_1]$. Each component is represented by a natural cubic spline on its own station grid, with the grids of different components independent of one another; the value and first derivative of the spline are the component's displacement (or angle) and slope. The first and last knots of each component are its activation boundaries, and outside them both value and slope are zero. A supplied series is interpolated the same way, and the gating it carries is its envelope. The natural boundary condition sets the spline's second derivative to zero at the first and last knots; the first derivative at those knots comes from the discrete interpolation and differs from the analytic $q'=0$ of the envelope by an interpolation error that decreases with the grid spacing.

### 6.3 Placement and orientation of the two rails

The four components are sampled at each rail's own station and together fix the position and orientation of that rail's profile reference point. Let $\boldsymbol p_0$ be the nominal profile origin in the track frame, determined by the gauge datum and the rail cant as in [Coordinates and notation](../CONVENTIONS_AND_NOTATION.en.md) §2.4. The placement convention rotates about the track $x$ axis by the roll angle first and then translates laterally and vertically:

$$
\boldsymbol p=R_x(\phi)\,\boldsymbol p_0+\left(0,\;y\pm\frac{g}{2},\;z\right)^{\mathsf T},
$$

with the positive sign of the gauge term on the left rail and the negative sign on the right, so that a positive $g$ draws both rails $g/2$ toward the centreline, and with a positive $\phi$ lowering the right rail and raising the left. The profile orientation consists of the roll given by the laying cant plus $\phi$, the yaw formed from the lateral slope $y'\pm g'/2$, and the pitch formed from the vertical slope $z'$. Distributing the gauge term along the rolled rail plane differs from the expression above by a term of size $\phi\,g/2$; placing the lateral and vertical translations before the rotation as well adds differences of size $\phi\,y$ and $\phi\,z$. Each of these differences is a product of two small quantities and is of second order in the perturbations taken jointly. How these positions and orientations enter the wheel-rail pose is given in [Wheel-rail pose reduction and irregularity inputs](../wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.en.md).

## 7. Theoretical assumptions and applicability

In the infinite-station idealization the raw random-phase sum has the second-order statistics prescribed by the input PSD; the finite band, finite number of frequencies and finite station interval turn it into a discrete approximation. After multiplication by the placement envelope the field is non-stationary in the fade segments, and the stationary reading of spectrum and variance applies to the unwindowed harmonic sum and to the full-amplitude plateau.

A PSD prescribes second-order statistics; random phases, channel independence, the chosen spectral family and the natural-spline reconstruction form the model assumptions of this chapter. Deterministic defects, non-stationary evolution and directional coherence each have a corresponding extension: localized defects are superimposed as deterministic geometry, non-stationarity is described by station-dependent spectral parameters, and directional coherence is introduced through a cross-spectral matrix.

## 8. Numerical implementation and source mapping

The mixing map for the channel seeds is `SplitMix64`:

```text
x = x + 0x9E3779B97F4A7C15
x = (x XOR (x >> 30)) * 0xBF58476D1CE4E5B9
x = (x XOR (x >> 27)) * 0x94D049BB133111EB
x = x XOR (x >> 31)
```

All operations are unsigned 64-bit modulo $2^{64}$. The domain constants of the lateral and vertical channels encode `LATERAL` followed by a null byte and `VERTICAL`:

```text
lateral_seed  = SplitMix64(realization_seed XOR 0x4C41544552414C00)
vertical_seed = SplitMix64(realization_seed XOR 0x564552544943414C)
```

The addition, the odd multiplications and the xor-shifts are all invertible, so the whole map is a bijection. Each channel uses its own `std::mt19937_64` engine; from the $j$th 64-bit output $x_j$ the top 53 bits form the uniform number $u_j=(x_j\mathbin{\texttt{>>}}11)\,2^{-53}$. The oscillator recurrence is re-anchored to the analytic phase every 256 stations. `Smoothstep5` evaluates the symmetric form about $u=0.5$ for $u>0.5$, which reduces cancellation in the polynomial near 1.

| Theoretical object | Main implementation |
|---|---|
| AAR single-cutoff parameters, $S_\Omega\to S_f$ and continuous band variance | `AarSingleCutoffPsdParametersFor`, `EvaluateAarOneSidedSpatialPsd`, `ContinuousBandVariance` in [`aar_track_irregularity_generator.cc`](../../../libs/track_irregularity/src/aar_track_irregularity_generator.cc) |
| ERRI B176 displacement parameters, angular-wavenumber spectrum, $S_\Omega\to S_f$ and continuous band variance | `ErriB176DisplacementPsdParametersFor`, `EvaluateErriB176AngularWavenumberPsd`, `EvaluateErriB176OneSidedSpatialPsd`, `ContinuousBandVariance` in [`erri_b176_track_irregularity_generator.cc`](../../../libs/track_irregularity/src/erri_b176_track_irregularity_generator.cc) |
| Domain separation, seed-to-phase map, harmonic sum and periodic re-anchoring | `DeriveTrackIrregularityChannelSeeds`, `SplitMix64`, `ReproduciblePhaseGenerator`, `AccumulateHarmonicChannel`, `GenerateTrackIrregularityCore` in [`track_irregularity_generation.cc`](../../../libs/track_irregularity/src/track_irregularity_generation.cc) |
| Quintic envelope and lateral/vertical channel generation | `Smoothstep5`, `TrackIrregularityPlacementWeight`, `GenerateAarTrackIrregularity`, `GenerateErriB176TrackIrregularity`; common types in [`track_irregularity_generation.h`](../../../libs/track_irregularity/include/orvd/track_irregularity/track_irregularity_generation.h) |
| Generated or supplied series to field | `ResolveTrackIrregularityField` in [`resolve_track_irregularity_field.cc`](../../../libs/configuration/src/resolve_track_irregularity_field.cc); `LoadTrackIrregularityFieldFromDataRoot` in [`load_track_irregularity_field.cc`](../../../libs/configuration/src/load_track_irregularity_field.cc) |
| Spline evaluation of the four components and zero outside the domain | `TrackIrregularityField::LateralDisplacementMeters`, `VerticalDisplacementMeters`, `GaugeNarrowingMeters`, `RollRadians` and the corresponding slope functions in [`track_irregularity_field.cc`](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) |
| Placement and orientation of the two rails | `ApplyGaugeAndRollIrregularity` in [`wheel_rail_pose.cc`](../../../libs/wheel_rail_contact/src/wheel_rail_pose.cc); station-wise sampling and slope composition in [`wheel_rail_contact_force_plan.cc`](../../../libs/forces/src/wheel_rail_contact_force_plan.cc) |
