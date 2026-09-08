[中文](TRACK_IRREGULARITY_SPECTRA.md)

# Track-irregularity spectra and their spatial random realization

This chapter explains how ORVD turns a one-sided spatial power spectral density (PSD) into a finite-length lateral and vertical track-irregularity field. It focuses on spectral variables and units, the finite band, the random-phase harmonic sum, the deterministic seed-to-phase map, inter-channel correlation and the two-ended `smoothstep5` envelope; a compact table at the end maps these theoretical objects to the source.

## 1. Model objects and scope

Track-irregularity modelling contains three independent choices: the spectral formula specifies second-order statistical energy across spatial frequencies, the finite band specifies the longest and shortest retained wavelengths, and the random realization specifies the phase at each discrete frequency. One spectrum and band admit infinitely many realizations; changing the random seed does not change the theoretical PSD, whereas changing either cutoff changes the variance and dynamical meaning.

This chapter treats only stationary random geometric irregularity superposed on an ideal line. Plan curvature and superelevation belong to [Line geometry and track frames](../track_geometry/TRACK_GEOMETRY_AND_FRAMES.en.md), and the piecewise closed-form models of grade and vertical curves belong to [Track vertical profile modelling and its three-dimensional coupling](../track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.en.md), while local deterministic defects such as welds, corrugation and scuffing cannot be specified uniquely by a stationary PSD. Lateral displacement is positive to the right in the track frame, vertical displacement is positive downward, and station is the planar-projected mileage defined in [Conventions and notation](../CONVENTIONS_AND_NOTATION.en.md).

The current implementation has two source families: generating one two-channel realization from an AAR5/AAR6 or ERRI B176 spectrum, or resolving a frozen field. A frozen field is a point-series asset that preserves its existing phase, gating and interpolation history; even when its spectral family is known, a PSD alone cannot recover it pointwise. Both paths ultimately form the same `TrackIrregularityField`, and a frozen point series is neither regenerated nor given a second envelope.

AAR5, AAR6 and ERRI Low/High denote track-irregularity spectral parameter sets here, with ERRI Low/High explicitly distinguishing irregularity-amplitude levels. They identify neither vehicle classes nor speed classes and are not universal descriptions valid for every line and vehicle frequency range.

## 2. Spatial frequency and one-sided PSD

### 2.1 Cyclic spatial frequency, angular wavenumber and wavelength

Let $f$ be cyclic spatial frequency in $\mathrm{cycles/m}$, $\Omega$ angular spatial wavenumber in $\mathrm{rad/m}$ and $\lambda$ spatial wavelength. They satisfy

$$
\Omega=2\pi f,
\qquad
\lambda=\frac{1}{f}=\frac{2\pi}{\Omega}.
$$

Let $S_f(f)$ and $S_\Omega(\Omega)$ be one-sided PSDs expressed against $f$ and $\Omega$, respectively. A change of variable must preserve variance:

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

It is therefore incorrect to multiply only the frequency axis by $2\pi$ while leaving the PSD ordinate unchanged, or to reuse polynomial coefficients expanded in $f$ directly with $\Omega$. The one-sided spectra in this chapter are integrated only over positive spatial frequencies; a two-sided spectrum must first be converted according to its own normalization convention.

### 2.2 From spatial frequency to temporal frequency

If a vehicle traverses spatial wavelength $\lambda$ at speed magnitude $v_s$, the corresponding temporal frequency is

$$
f_t=v_s f=\frac{v_s}{\lambda}.
$$

Speed changes the mapping from spatial excitation to the vehicle's temporal response; it does not change the spatial PSD of the line itself.

## 3. Spectral models

### 3.1 Lateral and vertical single-cutoff spectra

The AAR5/AAR6 model implemented by ORVD uses a single-cutoff angular-wavenumber spectrum. This family of stationary random track representations in terms of a PSD, roughness parameters and cutoffs is described in the [FRA report on statistical representations of track geometry](https://rosap.ntl.bts.gov/view/dot/9617). The lateral and vertical spectra are

$$
S_{\mathrm{lat}}(\Omega)
=\frac{kA_a\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)},
\qquad
S_{\mathrm{ver}}(\Omega)
=\frac{kA_v\Omega_c^2}
{\Omega^2(\Omega^2+\Omega_c^2)}.
$$

The implementation uses the following parameters. $A_a$ and $A_v$ are traditional tabulated values in $\mathrm{cm^2\,rad/m}$; forming an SI length PSD additionally multiplies them by $10^{-4}$ to convert $\mathrm{cm^2\to m^2}$.

| Class | $A_a$ | $A_v$ | $\Omega_c$ / $\mathrm{rad\,m^{-1}}$ | $k$ |
|---|---:|---:|---:|---:|
| AAR5 | `0.0762` | `0.2095` | `0.8245` | `0.25` |
| AAR6 | `0.0339` | `0.0339` | `0.8245` | `0.25` |

Writing $A$ for the corresponding direction's $A_a$ or $A_v$, every rational spectrum in this chapter uses one polynomial-ratio template

$$
S_\Omega(\Omega)=\frac{b_0}{a_0+a_2\Omega^2+a_4\Omega^4},
$$

in which $a_0$, $a_2$ and $a_4$ are the coefficients of the denominator expanded in even powers of $\Omega$ and $b_0$ is the constant numerator. The $S_\Omega$ of the template is in SI form, that is, the $\mathrm{cm^2\to m^2}$ conversion above has already been applied, so it equals the closed form written earlier in this section from the traditional tabulated values multiplied by $10^{-4}$. The single-cutoff spectrum is the special case

$$
a_0=0,
\qquad
a_2=\Omega_c^2,
\qquad
a_4=1,
\qquad
b_0=kA\cdot10^{-4}\,\Omega_c^2,
$$

in which $a_0=0$ is what makes the spectrum grow as $\Omega^{-2}$ at low angular wavenumber.

`AarTrackClass` selects only the shape and amplitude determined by $A_a$, $A_v$, $k$ and $\Omega_c$; it does not implicitly prescribe unique values of $f_{\min}$, $f_{\max}$ or the number of discrete frequencies.

### 3.2 ERRI B176 lateral and vertical displacement spectra

The SIMPACK 2021x QCH page `Power Spectral Densities: Library | 3: Predefined` gives predefined lateral, vertical and crosslevel spectra based on ORE (ERRI) B176. This section adopts its implementation definition for the lateral and vertical displacement spectra:

$$
S_\Omega(\Omega)=
\frac{b_0}{a_0+a_2\Omega^2+a_4\Omega^4}.
$$

The lateral and vertical Low/High spectra share one denominator and differ in amplitude only through $b_0$:

| Parameter set | Direction | $b_0$ | $a_0$ | $a_2$ | $a_4$ |
|---|---|---:|---:|---:|---:|
| ERRI Low | Lateral | `1.440846e-7` | `0.00028855` | `0.6803895` | `1` |
| ERRI Low | Vertical | `2.741619e-7` | `0.00028855` | `0.6803895` | `1` |
| ERRI High | Lateral | `4.164787e-7` | `0.00028855` | `0.6803895` | `1` |
| ERRI High | Vertical | `7.343623e-7` | `0.00028855` | `0.6803895` | `1` |

These numbers are directly the coefficients of the angular-wavenumber spectrum above, with $S_\Omega$ in $\mathrm{m^2/(rad/m)}$. They do not pass through the AAR-specific $k=0.25$, $\mathrm{cm^2\to m^2}$ or $\Omega_c=0.8245\,\mathrm{rad/m}$ parameter conversions. Conversion to cyclic spatial frequency uses only the variance-preserving change of variable from section 2.1, $S_f(f)=2\pi S_\Omega(2\pi f)$.

The ERRI denominator has a positive constant term, so its low-angular-wavenumber limit is finite:

$$
S_\Omega(0)=\frac{b_0}{a_0}.
$$

This differs from the $\Omega^{-2}$ growth of the current AAR single-cutoff spectrum, but it does not prescribe an engineering lower cutoff. A finite value of the continuous PSD at zero also does not add a random constant component to a realization; the current spatial generator still uses a strictly positive $f_{\min}$.

Let

$$
p=\frac{a_2}{a_4},
\qquad
q=\frac{a_0}{a_4},
$$

and define $0<\Omega_1<\Omega_2$ by

$$
\Omega_2^2=\frac{p+\sqrt{p^2-4q}}{2},
\qquad
\Omega_1^2=\frac{q}{\Omega_2^2}.
$$

Then

$$
a_0+a_2\Omega^2+a_4\Omega^4
=a_4(\Omega^2+\Omega_1^2)(\Omega^2+\Omega_2^2).
$$

All four spectra in this section share the same pair $\Omega_1$ and $\Omega_2$ determined by the denominator coefficients above. Their continuous finite-band variance is

$$
\sigma_{\mathrm{continuous}}^2
=\frac{b_0}{a_4(\Omega_2^2-\Omega_1^2)}
\left[
\frac{1}{\Omega_1}\arctan\frac{\Omega}{\Omega_1}
-\frac{1}{\Omega_2}\arctan\frac{\Omega}{\Omega_2}
\right]_{\Omega_{\min}}^{\Omega_{\max}}.
$$

Low and High differ only in $b_0$ within one direction and therefore have the same spectral shape. If their frequency grids and phases are also identical, the High realization is the Low realization multiplied by $\sqrt{b_{0,\mathrm{High}}/b_{0,\mathrm{Low}}}$.

The ERRI displacement spectrum decays as $\Omega^{-4}$ at high angular wavenumber. The formal variance moment of its $n$th spatial derivative contains $\int\Omega^{2n}S_\Omega(\Omega)\,d\Omega$: the high-frequency tails of displacement and first slope converge, whereas that of the second derivative diverges. A finite $f_{\max}$ therefore remains part of the geometric model; integrability of the displacement spectrum does not imply infinite usable bandwidth.

The 200 km/h straight line in the IRW SIMPACK reference model uses the ERRI Low lateral and vertical spectra from this section. The repository's frozen field of the same name retains the independent asset identity of that existing realization; support for its spectral family does not claim that the new generator can recover its phases and boundary-processing history pointwise.

### 3.3 Gauge and cross-level (theory only)

One common simplified reference form writes gauge or cross-level as a double-cutoff spectrum, where $\Omega_s$ is the second cutoff angular wavenumber:

$$
S_{gcl}(\Omega)=
\frac{4kA_v\Omega_c^2}
{(\Omega^2+\Omega_c^2)(\Omega^2+\Omega_s^2)}.
$$

In the template of section 3.1, likewise in the converted SI form, its coefficients are

$$
a_0=\Omega_c^2\Omega_s^2,
\qquad
a_2=\Omega_c^2+\Omega_s^2,
\qquad
a_4=1,
\qquad
b_0=4kA_v10^{-4}\Omega_c^2.
$$

If cross-level is expressed as a height difference $u$ between two reference points while the required quantity is a small roll angle $\phi$, the reference base length $b_{\mathrm{ref}}$ must also be supplied:

$$
\phi\simeq\frac{u}{b_{\mathrm{ref}}},
\qquad
S_\phi=\frac{S_u}{b_{\mathrm{ref}}^2}.
$$

Gauge and cross-level are different geometric quantities; sharing a rational-function shape does not make them physically equivalent. The QCH also gives ERRI crosslevel Low/High spectra, but their source quantity, angular conversion and application to the two rails are outside this lateral-vertical displacement implementation. The current generator implements only those two displacement channels; crosslevel spectra and correlated multichannel generation discussed here are theory only and cannot be recovered from the existing two-channel output.

## 4. Finite band and variance

The AAR single-cutoff spectrum grows as $\Omega^{-2}$ at low angular wavenumber, so its displacement variance diverges if the lower cutoff tends to zero; the ERRI displacement spectrum instead approaches the finite plateau in section 3.2. Both generation specifications require a positive $f_{\min}$: it bounds the longest retained wavelength and excludes a zero-frequency random constant not separately defined by the PSD density. The value of $f_{\max}$ sets the shortest wavelength and the highest spatial frequency delivered to the track, wheel-rail contact and vehicle models.

For $0<\Omega_{\min}<\Omega_{\max}$, define

$$
\mathcal F(\Omega)=-\frac{1}{\Omega}
-\frac{1}{\Omega_c}\arctan\!\left(\frac{\Omega}{\Omega_c}\right).
$$

The continuous finite-band variance of the single-cutoff spectrum is

$$
\sigma_{\mathrm{continuous}}^2
=kA10^{-4}
\left[\mathcal F(\Omega_{\max})-\mathcal F(\Omega_{\min})\right],
\qquad
\Omega_{\min,\max}=2\pi f_{\min,\max}.
$$

Lowering $f_{\min}$ adds long-wave energy, while raising $f_{\max}$ adds short-wave energy, so the spectral formula, finite band and realization jointly define a random operating case. The spacing $\Delta s$ of the spatial output grid must also satisfy the necessary Nyquist condition

$$
\Delta s\,f_{\max}<\frac12.
$$

The implementation uses a strict inequality to avoid degeneration of a random-phase harmonic exactly at the Nyquist frequency. The sample mean square of a finite realization generally does not equal the continuous integral variance exactly, and the end envelope further changes statistics over the complete interval.

## 5. Random-phase harmonic realization

### 5.1 Discrete frequency grid and harmonic sum

For an equally spaced frequency grid with $N\ge2$ that includes both endpoints,

$$
f_j=f_{\min}+(j-1)\Delta f,
\qquad
\Delta f=\frac{f_{\max}-f_{\min}}{N-1},
\qquad
j=1,\ldots,N,
$$

ORVD uses the random-phase harmonic sum

$$
r_{\mathrm{raw}}(s)=\sum_{j=1}^{N}
\sqrt{2S_f(f_j)\Delta f}
\cos\!\left(2\pi f_j\xi+\theta_j\right),
\qquad
\xi=s-s_0,
$$

where $s_0$ is the placement start and $\theta_j$ is uniform over $[0,2\pi)$. Taking expectation over the phases, each harmonic term has zero mean and variance $S_f(f_j)\Delta f$. If the phases are mutually independent, the harmonic sum has zero mean and variance

$$
\sigma_{\mathrm{discrete}}^2
=\sum_{j=1}^{N}S_f(f_j)\Delta f.
$$

The generator gives endpoints and interior frequencies the same rectangular weight instead of trapezoidal half-weights at the endpoints. Consequently, $\sigma_{\mathrm{discrete}}^2$ and the preceding continuous integral are distinct quantities that approach each other as the frequency grid is refined. The phase coordinate is local station $\xi$, so moving the placement while holding the spectrum, frequency grid and seed fixed translates the same realization with its start.

The equally spaced frequency grid carries one further identity. Write the harmonic sum in complex form as $Z(\xi)=\sum_{j=1}^{N}\sqrt{2S_f(f_j)\Delta f}\,e^{\mathrm i(2\pi f_j\xi+\theta_j)}$, so that $r_{\mathrm{raw}}=\operatorname{Re}Z$. With $T=1/\Delta f$, the relations $f_jT=f_{\min}/\Delta f+(j-1)$ and $e^{\mathrm i2\pi(j-1)}=1$ give

$$
Z(\xi+T)=e^{\mathrm i2\pi f_{\min}/\Delta f}Z(\xi).
$$

A shift by $T$ multiplies the complex harmonic sum by a single constant phase factor independent of both $\xi$ and $j$. Thus $|Z|$ has $T$ as a period, but this does not make $T$ a general period or an “effective random length” of the real-valued realization. The real signal $r_{\mathrm{raw}}=\operatorname{Re}Z$ usually does not repeat pointwise after a shift by $T$: if $f_{\min}/\Delta f=p/q$ in lowest terms, then $qT$ is one of its periods; if that ratio is irrational, the equally spaced grid forms a quasiperiodic sum with no finite common period.

### 5.2 From realization seed to phase

One unsigned 64-bit realization seed first undergoes the fixed domain separation shared by AAR and ERRI to produce lateral and vertical channel seeds. The `SplitMix64` map is

```text
x = x + 0x9E3779B97F4A7C15
x = (x XOR (x >> 30)) * 0xBF58476D1CE4E5B9
x = (x XOR (x >> 27)) * 0x94D049BB133111EB
x = x XOR (x >> 31)
```

All operations are unsigned 64-bit arithmetic modulo $2^{64}$, and the two channels are defined by

```text
lateral_seed  = SplitMix64(realization_seed XOR 0x4C41544552414C00)
vertical_seed = SplitMix64(realization_seed XOR 0x564552544943414C)
```

The two domain constants encode `LATERAL` followed by a null byte and `VERTICAL`, respectively. The additions, odd multiplications and xor-shifts above are invertible, so the complete `SplitMix64` map is a bijection; the two different domain inputs therefore produce two different channel seeds.

Each channel uses an independent `std::mt19937_64` engine and consumes random words in ascending frequency order. For its $j$th 64-bit output $x_j$, the phase map is defined explicitly as

$$
u_j=(x_j\mathbin{\texttt{>>}}11)2^{-53},
\qquad
\theta_j=2\pi u_j.
$$

This definition directly takes the high 53 bits, so it does not depend on the particular algorithm of a standard-library distribution object and uses neither a clock nor process-global random state. The same generation specification and realization seed therefore determine the same pseudorandom phases.

### 5.3 Oscillator recurrence

Let the phase increment between adjacent stations be $\Delta\alpha_j=2\pi f_j\Delta s$. Rather than invoking trigonometric functions at every station, the source advances $c_n=\cos\alpha_{j,n}$ and $s_n=\sin\alpha_{j,n}$ by

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

In real arithmetic this is equivalent to direct evaluation of the harmonic formula. Floating-point recurrence accumulates drift, so the implementation reanchors from the analytic phase every 256 station intervals.

### 5.4 Lateral-vertical correlation

If both channels reused exactly the same phases, the AAR6 lateral and vertical sequences would be pointwise identical because their PSDs are equal. The two AAR5 spectra differ only in amplitude, so their sequences would have the fixed ratio

$$
\sqrt{\frac{A_v}{A_a}}.
$$

Such phase locking is not a physical correlation prescribed by an AAR spectrum. Domain separation gives the current two channels different pseudorandom phase streams, but different seeds do not force the sample correlation coefficient over every finite interval to equal zero. A multidirectional field with a known cross-spectrum should instead specify a positive-semidefinite cross-spectral matrix and use joint spectral factorization and generation; the current implementation does not contain that algorithm, so this is a theoretical extension only.

## 6. Placement and two-ended `smoothstep5`

Let the placement be $[s_0,s_1]$, with fade-in and fade-out lengths $L_{\mathrm{in}}$ and $L_{\mathrm{out}}$. The quintic smooth step is

$$
q(u)=6u^5-15u^4+10u^3,
\qquad
0\le u\le1.
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
0, & s\ge s_1.
\end{cases}
$$

The final realization is

$$
r(s)=w(s)r_{\mathrm{raw}}(s).
$$

This construction requires $s_1>s_0$, $L_{\mathrm{in}}>0$, $L_{\mathrm{out}}>0$ and $L_{\mathrm{in}}+L_{\mathrm{out}}\le s_1-s_0$. Equality means that the two fades meet at one full-amplitude point without a negative-length plateau.

The quintic satisfies

$$
q(0)=0,
\quad q(1)=1,
\quad q'(0)=q'(1)=q''(0)=q''(1)=0.
$$

Because the finite harmonic sum is itself smooth, the analytic product $r(s)$ has continuous displacement, first slope and second derivative where the zero, fade and full-amplitude regions join. For $u>0.5$, `Smoothstep5` evaluates the form symmetric about $u=0.5$ to reduce cancellation from direct polynomial evaluation near 1.

The generated samples are multiplied by the envelope first, then cropped to $[s_0,s_1]$ and used to construct the natural-cubic-spline `TrackIrregularityField`. This field returns zero displacement and zero slope strictly outside its definition interval, but the spline endpoint derivatives arise from discrete interpolation and are not necessarily bitwise zero merely because the analytic window has $q'(0)=q'(1)=0$.

## 7. Theoretical assumptions and applicability

In the infinite-station idealization, the raw random-phase sum has the second-order statistical structure prescribed by the input PSD. A finite band, finite frequency count and finite station interval turn it into a discrete approximation. Multiplication by the placement envelope makes the field non-stationary in the transition regions, so the stationary interpretation of spectrum and variance applies directly only to the ungated harmonic sum or the full-amplitude plateau.

A PSD specifies only second-order statistics; it preserves neither the deterministic phase of a real line nor isolated defects. Random phases, no prescribed cross-spectrum between lateral and vertical channels, the selected AAR or ERRI spectral family and natural-spline reconstruction are model assumptions. A study that depends on deterministic defects, non-stationary evolution or directional coherence needs an extended model.

## 8. Source mapping

| Theoretical object | Primary implementation |
|---|---|
| AAR parameters, $S_\Omega\to S_f$ and continuous finite-band variance | `AarSingleCutoffPsdParametersFor`, `EvaluateAarOneSidedSpatialPsd`, `ContinuousBandVariance`; see [`aar_track_irregularity_generator.cc`](../../../libs/track_irregularity/src/aar_track_irregularity_generator.cc) |
| ERRI B176 parameters, angular-wavenumber PSD, $S_\Omega\to S_f$ and continuous finite-band variance | `ErriB176DisplacementPsdParametersFor`, `EvaluateErriB176AngularWavenumberPsd`, `EvaluateErriB176OneSidedSpatialPsd`, `ContinuousBandVariance`; see [`erri_b176_track_irregularity_generator.cc`](../../../libs/track_irregularity/src/erri_b176_track_irregularity_generator.cc) |
| Domain separation, seed-to-phase mapping, harmonic sum and periodic reanchoring | `DeriveTrackIrregularityChannelSeeds`, `SplitMix64`, `ReproduciblePhaseGenerator`, `AccumulateHarmonicChannel`, and `GenerateTrackIrregularityCore`; see [`track_irregularity_generation.cc`](../../../libs/track_irregularity/src/track_irregularity_generation.cc) |
| Quintic envelope and two-channel generation | `Smoothstep5`, `TrackIrregularityPlacementWeight`, `GenerateAarTrackIrregularity`, `GenerateErriB176TrackIrregularity`; common types are in [`track_irregularity_generation.h`](../../../libs/track_irregularity/include/orvd/track_irregularity/track_irregularity_generation.h) |
| Generated samples to vehicle field | `ResolveTrackIrregularityField`; see [`resolve_track_irregularity_field.cc`](../../../libs/configuration/src/resolve_track_irregularity_field.cc) |
| Zero displacement and zero slope outside the definition interval | `TrackIrregularityField::LateralDisplacementMeters`, `VerticalDisplacementMeters`, `LateralSlopeMetersPerMeter`, `VerticalSlopeMetersPerMeter`; see [`track_irregularity_field.cc`](../../../libs/wheel_rail_contact/src/track_irregularity_field.cc) |
