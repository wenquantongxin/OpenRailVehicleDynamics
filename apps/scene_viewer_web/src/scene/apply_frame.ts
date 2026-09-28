import * as THREE from 'three';

import type { SceneRecord } from '../record/scene_record.ts';
import { interpolateOrientation, interpolatePosition, type WheelSpinBinding } from './pose_interpolation.ts';
import type { BuiltScene } from './scene_builder.ts';

// Writes every body's pose for a display time between two recorded frames.

const position = new THREE.Vector3();
const orientation = new THREE.Quaternion();

/** Which body slots are wheels with a recorded spin angle, by body index. */
export function wheelSpinBindings(record: SceneRecord): Map<number, WheelSpinBinding> {
  const bindings = new Map<number, WheelSpinBinding>();
  if (record.columns.wheelSpinAngleCount === 0) {
    return bindings;
  }
  record.wheelPlacements.forEach((placement, wheelPlacementIndex) => {
    const bodyIndex = record.bodies.findIndex((candidate) => candidate.name === placement.wheelBodyName);
    if (bodyIndex >= 0) {
      bindings.set(bodyIndex, { wheelPlacementIndex, spinAxisInBody: placement.spinAxisInWheelBodyFrame });
    }
  });
  return bindings;
}

export function applyFrames(
  record: SceneRecord,
  scene: BuiltScene,
  bindings: Map<number, WheelSpinBinding>,
  firstFrameIndex: number,
  secondFrameIndex: number,
  alpha: number,
): void {
  record.bodies.forEach((body, bodyIndex) => {
    const object = scene.bodyObjects.get(body.name);
    if (object === undefined) {
      return;
    }
    interpolatePosition(record, firstFrameIndex, secondFrameIndex, alpha, bodyIndex, position);
    interpolateOrientation(record, firstFrameIndex, secondFrameIndex, alpha, bodyIndex, bindings.get(bodyIndex), orientation);
    object.position.copy(position);
    object.quaternion.copy(orientation);
  });
}
