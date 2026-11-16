import * as THREE from 'three';

import type { Preview } from '../service/protocol.ts';
import { drawsAnything } from './preview_scene.ts';

// Camera presets that frame the drawn vehicle. A preset is a viewing
// direction, a field of view and a margin; the target is the centre of the
// drawing's bounding box and the distance is the smallest at which all eight
// corners of that box fall inside the picture, less the margin (the fit of
// apps/scene_viewer_web/src/scene/camera_framing.ts, here for a picture no
// card covers). Directions are stated in the vehicle's own terms, forward,
// left and up, which in three's world are +x, -z and +y.

export type ViewPreset = 'overview' | 'side' | 'plan' | 'front';

export const viewPresets: ViewPreset[] = ['overview', 'side', 'plan', 'front'];

interface PresetSpec {
  /** From the target towards the camera: forward, left, up. */
  direction: [number, number, number];
  fov: number;
  margin: number;
}

const presets: Record<ViewPreset, PresetSpec> = {
  overview: { direction: [0.62, 0.66, 0.42], fov: 30, margin: 0.06 },
  side: { direction: [0, 1, 0.1], fov: 30, margin: 0.05 },
  // Leaning slightly to the right puts forward to the right of the picture.
  plan: { direction: [0, -0.001, 1], fov: 30, margin: 0.05 },
  front: { direction: [1, 0, 0.06], fov: 22, margin: 0.08 },
};

const localUp = new THREE.Vector3(0, 0, 1);

/** The camera basis for a viewing direction in forward-left-up terms, as Object3D.lookAt builds it with the vertical up. */
function viewBasis(direction: [number, number, number]): { toCamera: THREE.Vector3; right: THREE.Vector3; up: THREE.Vector3 } {
  const toCamera = new THREE.Vector3(...direction).normalize();
  const right = new THREE.Vector3().crossVectors(localUp, toCamera).normalize();
  const up = new THREE.Vector3().crossVectors(toCamera, right).normalize();
  return { toCamera, right, up };
}

/**
 * Distance from the target, along the direction, at which every corner falls
 * inside the picture shrunk by the margin on each side, with the camera at
 * least one metre in front of the nearest corner.
 */
export function fitDistanceMeters(
  corners: readonly THREE.Vector3[],
  target: THREE.Vector3,
  direction: [number, number, number],
  fovDegrees: number,
  aspect: number,
  marginRatio: number,
): number {
  const { toCamera, right, up } = viewBasis(direction);
  const tanVertical = Math.tan(THREE.MathUtils.degToRad(fovDegrees / 2));
  const limitHorizontal = (tanVertical * aspect) / (1 + marginRatio);
  const limitVertical = tanVertical / (1 + marginRatio);
  const offset = new THREE.Vector3();
  let distance = 0;
  let nearestDepth = Infinity;
  for (const corner of corners) {
    offset.subVectors(corner, target);
    // Depth of the corner beyond the target, away from the camera.
    const depth = -offset.dot(toCamera);
    distance = Math.max(distance, Math.abs(offset.dot(right)) / limitHorizontal - depth, Math.abs(offset.dot(up)) / limitVertical - depth);
    nearestDepth = Math.min(nearestDepth, depth);
  }
  return Math.max(distance, 1.0 - nearestDepth);
}

function localFromWorld(world: THREE.Vector3): THREE.Vector3 {
  return new THREE.Vector3(world.x, -world.z, world.y);
}

function worldFromLocal(local: THREE.Vector3): THREE.Vector3 {
  return new THREE.Vector3(local.x, local.z, -local.y);
}

/** Camera position, look-at target and field of view in three's world for a preset framing the given world box. */
export function presetPlacement(preset: ViewPreset, bounds: THREE.Box3, aspect: number): { position: THREE.Vector3; target: THREE.Vector3; fov: number } {
  const spec = presets[preset];
  const corners = [0, 1, 2, 3, 4, 5, 6, 7].map((corner) =>
    localFromWorld(new THREE.Vector3(corner & 1 ? bounds.max.x : bounds.min.x, corner & 2 ? bounds.max.y : bounds.min.y, corner & 4 ? bounds.max.z : bounds.min.z)),
  );
  const target = localFromWorld(bounds.getCenter(new THREE.Vector3()));
  const distance = fitDistanceMeters(corners, target, spec.direction, spec.fov, aspect, spec.margin);
  const position = target.clone().addScaledVector(new THREE.Vector3(...spec.direction).normalize(), distance);
  return { position: worldFromLocal(position), target: worldFromLocal(target), fov: spec.fov };
}

/** Whether showing a preview frames the camera: the first preview of an opened vehicle that draws anything does. */
export function framesCamera(preview: Preview | null, framedAlready: boolean): boolean {
  return preview !== null && !framedAlready && drawsAnything(preview);
}
