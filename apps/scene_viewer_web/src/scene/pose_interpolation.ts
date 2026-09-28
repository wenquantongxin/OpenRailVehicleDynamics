import * as THREE from 'three';

import { bodyAngularVelocity, bodyPose, frameTimeSeconds, wheelSpinAngle, type SceneRecord } from '../record/scene_record.ts';

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
  record: SceneRecord,
  firstFrameIndex: number,
  secondFrameIndex: number,
  alpha: number,
  bodyIndex: number,
  out: THREE.Vector3,
): void {
  const first = bodyPose(record, firstFrameIndex, bodyIndex);
  out.set(...first.position);
  if (secondFrameIndex !== firstFrameIndex && alpha > 0) {
    const second = bodyPose(record, secondFrameIndex, bodyIndex);
    out.lerp(new THREE.Vector3(...second.position), alpha);
  }
}

export function interpolateOrientation(
  record: SceneRecord,
  firstFrameIndex: number,
  secondFrameIndex: number,
  alpha: number,
  bodyIndex: number,
  spin: WheelSpinBinding | undefined,
  out: THREE.Quaternion,
): void {
  const first = bodyPose(record, firstFrameIndex, bodyIndex);
  const [wA, xA, yA, zA] = first.orientationWxyz;
  out.set(xA, yA, zA, wA);
  if (secondFrameIndex === firstFrameIndex || alpha <= 0) {
    return;
  }
  const second = bodyPose(record, secondFrameIndex, bodyIndex);
  const [wB, xB, yB, zB] = second.orientationWxyz;
  scratch.set(xB, yB, zB, wB);
  const angleA = spin === undefined ? null : wheelSpinAngle(record, firstFrameIndex, spin.wheelPlacementIndex);
  const angleB = spin === undefined ? null : wheelSpinAngle(record, secondFrameIndex, spin.wheelPlacementIndex);
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

/**
 * The largest rotation, in radians, that the recorded angular velocities imply
 * for any wheel between two consecutive frames. Above a half turn a display
 * cannot choose the spin branch from the orientations alone.
 */
export function maximumWheelRotationBetweenFrames(record: SceneRecord, wheelBodyIndices: number[]): number {
  let maximum = 0;
  for (let frameIndex = 1; frameIndex < record.frameCount; ++frameIndex) {
    const interval = frameTimeSeconds(record, frameIndex) - frameTimeSeconds(record, frameIndex - 1);
    for (const bodyIndex of wheelBodyIndices) {
      const [x, y, z] = bodyAngularVelocity(record, frameIndex, bodyIndex);
      maximum = Math.max(maximum, Math.hypot(x, y, z) * interval);
    }
  }
  return maximum;
}
