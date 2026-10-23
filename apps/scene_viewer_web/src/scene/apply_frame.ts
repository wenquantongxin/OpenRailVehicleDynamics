import * as THREE from 'three';

import type { SceneDefinition } from '../record/scene_definition.ts';
import type { SceneSampleView } from '../record/scene_sample.ts';
import { interpolateOrientation, interpolatePosition, type WheelSpinBinding } from './pose_interpolation.ts';
import type { BuiltScene } from './scene_builder.ts';

// Writes every body's pose for a display time between two recorded frames.

const position = new THREE.Vector3();
const orientation = new THREE.Quaternion();

/** Wheel axes and angle slots by body index; each sample determines angle availability. */
export function wheelSpinBindings(definition: SceneDefinition): Map<number, WheelSpinBinding> {
  const bindings = new Map<number, WheelSpinBinding>();
  definition.wheelPlacements.forEach((placement, wheelPlacementIndex) => {
    const bodyIndex = definition.bodies.findIndex((candidate) => candidate.name === placement.wheelBodyName);
    if (bodyIndex >= 0) {
      bindings.set(bodyIndex, { wheelPlacementIndex, spinAxisInBody: placement.spinAxisInWheelBodyFrame });
    }
  });
  return bindings;
}

export function applyFrames(
  definition: SceneDefinition,
  scene: BuiltScene,
  bindings: Map<number, WheelSpinBinding>,
  firstSample: SceneSampleView,
  secondSample: SceneSampleView,
  alpha: number,
): void {
  definition.bodies.forEach((body, bodyIndex) => {
    const object = scene.bodyObjects.get(body.name);
    if (object === undefined) {
      return;
    }
    interpolatePosition(firstSample, secondSample, alpha, bodyIndex, position);
    interpolateOrientation(firstSample, secondSample, alpha, bodyIndex, bindings.get(bodyIndex), orientation);
    object.position.copy(position);
    object.quaternion.copy(orientation);
  });
}
