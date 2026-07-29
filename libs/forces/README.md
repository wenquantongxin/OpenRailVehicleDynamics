# libs/forces

**职责**：力元。GZ18 live 路径的弹性/耗散力。

- **重力**：live 路径唯一的 Drake 力元是 `UniformGravityFieldElement`（9.81 常数）。
- **specialized suspension**：`SpecializedSuspensionForceElement` 自带整套 bushing 闭式（半角坐标系），
  数学已在仓库内，解钩即可。
- **Maxwell 阻尼器**（连续/离散横向、纵向）、**anti-hunting**：物理在仓库内，搬迁时状态并入自有积分状态。
- `LinearBushingRollPitchYaw`：**仅 Legacy 装配路径**用（`AddGz18ForceElementsLegacy` 无非测试调用方），
  live 路径不用；仅作 oracle 参照。

**里程碑**：M2（3–5 pw；若复用 core 空间数学核则偏下限）。

**必须复现的陷阱**：Mitiguy 半角 RPY、`RotationMatrix::ToQuaternion` 的 w≥0 规范化、
万向锁 THROW（`kGimbalLockToleranceCosPitchAngle=0.008`，force 计算内 `CalcRpyDtFromAngularVelocityInParent` 调用 6 次）、
SpatialForce/SpatialVelocity 的 Shift 符号相反。详见设计基线 §3.6、§8。
