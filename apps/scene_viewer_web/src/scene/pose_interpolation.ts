import * as THREE from 'three';

import type { SceneSampleView } from '../record/scene_sample.ts';

// Display interpolation between two recorded frames. Positions are linear.
// An orientation is the shortest-path slerp, except for a wheel whose record
// carries an unwrapped spin angle. Align the endpoint spin phases before
// slerping the carrier motion, then apply the fractional spin, including
// whole turns. At either frame the result is the sampled pose; spin is not
// added a second time to a sampled orientation.

const spinAxis = new THREE.Vector3();
const spinStep = new THREE.Quaternion();
const spinFull = new THREE.Quaternion();
const scratch = new THREE.Quaternion();

export interface WheelSpinBinding {
  /** Index into the record's wheel placements. */
  wheelPlacementIndex: number;
  spinAxisInBody: [number, number, number];
}

export function interpolatePosition(
  firstSample: SceneSampleView,
  secondSample: SceneSampleView,
  alpha: number,
  bodyIndex: number,
  out: THREE.Vector3,
): void {
  const first = firstSample.bodyPose(bodyIndex);
  out.set(...first.position);
  if (secondSample.timeSeconds !== firstSample.timeSeconds && alpha > 0) {
    const second = secondSample.bodyPose(bodyIndex);
    if (alpha >= 1) out.set(...second.position);
    else out.lerp(new THREE.Vector3(...second.position), alpha);
  }
}

export function interpolateOrientation(
  firstSample: SceneSampleView,
  secondSample: SceneSampleView,
  alpha: number,
  bodyIndex: number,
  spin: WheelSpinBinding | undefined,
  out: THREE.Quaternion,
): void {
  const first = firstSample.bodyPose(bodyIndex);
  const [wA, xA, yA, zA] = first.orientationWxyz;
  out.set(xA, yA, zA, wA);
  if (secondSample.timeSeconds === firstSample.timeSeconds || alpha <= 0) {
    return;
  }
  const second = secondSample.bodyPose(bodyIndex);
  const [wB, xB, yB, zB] = second.orientationWxyz;
  if (alpha >= 1) { out.set(xB, yB, zB, wB); return; }
  scratch.set(xB, yB, zB, wB);
  const angleA = spin === undefined ? null : firstSample.wheelSpinAngle(spin.wheelPlacementIndex);
  const angleB = spin === undefined ? null : secondSample.wheelSpinAngle(spin.wheelPlacementIndex);
  if (spin === undefined || angleA === null || angleB === null) {
    out.slerp(scratch, alpha);
    return;
  }
  // q_B * spin(-delta) has the same spin phase as q_A. Interpolate these
  // aligned orientations first: carrier rotation and wheel spin do not commute.
  const delta = angleB - angleA;
  spinAxis.set(...spin.spinAxisInBody).normalize();
  spinFull.setFromAxisAngle(spinAxis, -delta);
  scratch.multiply(spinFull);
  spinStep.setFromAxisAngle(spinAxis, delta * alpha);
  out.slerp(scratch, alpha).multiply(spinStep);
}
