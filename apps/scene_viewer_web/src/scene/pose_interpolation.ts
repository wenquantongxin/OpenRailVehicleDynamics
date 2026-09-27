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
  wheel: number;
  spinAxisInBody: [number, number, number];
}

export function interpolatePosition(
  record: SceneRecord,
  frameA: number,
  frameB: number,
  alpha: number,
  body: number,
  out: THREE.Vector3,
): void {
  const poseA = bodyPose(record, frameA, body);
  out.set(...poseA.position);
  if (frameB !== frameA && alpha > 0) {
    const poseB = bodyPose(record, frameB, body);
    out.lerp(new THREE.Vector3(...poseB.position), alpha);
  }
}

export function interpolateOrientation(
  record: SceneRecord,
  frameA: number,
  frameB: number,
  alpha: number,
  body: number,
  spin: WheelSpinBinding | undefined,
  out: THREE.Quaternion,
): void {
  const poseA = bodyPose(record, frameA, body);
  const [wA, xA, yA, zA] = poseA.orientationWxyz;
  out.set(xA, yA, zA, wA);
  if (frameB === frameA || alpha <= 0) {
    return;
  }
  const poseB = bodyPose(record, frameB, body);
  const [wB, xB, yB, zB] = poseB.orientationWxyz;
  scratch.set(xB, yB, zB, wB);
  const angleA = spin === undefined ? null : wheelSpinAngle(record, frameA, spin.wheel);
  const angleB = spin === undefined ? null : wheelSpinAngle(record, frameB, spin.wheel);
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
  for (let frame = 1; frame < record.frameCount; ++frame) {
    const interval = frameTimeSeconds(record, frame) - frameTimeSeconds(record, frame - 1);
    for (const body of wheelBodyIndices) {
      const [x, y, z] = bodyAngularVelocity(record, frame, body);
      maximum = Math.max(maximum, Math.hypot(x, y, z) * interval);
    }
  }
  return maximum;
}
