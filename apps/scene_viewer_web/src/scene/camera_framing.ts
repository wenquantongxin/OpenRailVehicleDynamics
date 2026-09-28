import * as THREE from 'three';

// The fit constraint of a camera preset, separated from its taste. Given the
// corners of what must be seen, the look-at target, the viewing direction and
// the rectangle of the picture left free by the cards, the distance is the
// smallest one at which every corner projects inside that rectangle, less a
// named margin. Everything is stated in the anchor's local frame: forward,
// left, up.

/** Pixels within the viewport, y downwards. */
export interface FreeRect {
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

export interface ViewportArea {
  width: number;
  height: number;
  /** The part of the picture not covered by cards; the target is centred in it. */
  free: FreeRect;
}

const localUp = new THREE.Vector3(0, 0, 1);

/**
 * The camera basis for a viewing direction, built the way Object3D.lookAt
 * builds it with the vertical as the camera's up: `toCamera` from the target
 * to the camera, `right` to the right of the picture and `up` up the picture.
 */
export function viewBasis(direction: [number, number, number]): { toCamera: THREE.Vector3; right: THREE.Vector3; up: THREE.Vector3 } {
  const toCamera = new THREE.Vector3(...direction).normalize();
  const right = new THREE.Vector3().crossVectors(localUp, toCamera);
  if (right.lengthSq() < 1e-12) {
    // Looking straight up or down: any horizontal right will do; use forward.
    right.set(1, 0, 0);
  }
  right.normalize();
  const up = new THREE.Vector3().crossVectors(toCamera, right).normalize();
  return { toCamera, right, up };
}

/** View offset that puts the camera axis at the free rectangle's centre: `setViewOffset(width, height, offsetX, offsetY, width, height)`. */
export function viewOffsetPixels(viewport: ViewportArea): { offsetX: number; offsetY: number } {
  const centreX = 0.5 * (viewport.free.x0 + viewport.free.x1);
  const centreY = 0.5 * (viewport.free.y0 + viewport.free.y1);
  return { offsetX: 0.5 * viewport.width - centreX, offsetY: 0.5 * viewport.height - centreY };
}

/**
 * Distance from the target, along `direction`, at which every corner falls
 * inside the free rectangle shrunk by `marginRatio` on each side, and the
 * camera stays at least one metre in front of the nearest corner. Horizontal
 * extent, vertical extent and perspective depth all take part.
 */
export function fitDistanceMeters(
  corners: readonly [number, number, number][],
  target: [number, number, number],
  direction: [number, number, number],
  fovDegrees: number,
  viewport: ViewportArea,
  marginRatio: number,
): number {
  const { toCamera, right, up } = viewBasis(direction);
  const tanVertical = Math.tan(THREE.MathUtils.degToRad(fovDegrees / 2));
  const aspect = viewport.width / viewport.height;
  const horizontalFraction = (viewport.free.x1 - viewport.free.x0) / viewport.width;
  const verticalFraction = (viewport.free.y1 - viewport.free.y0) / viewport.height;
  const limitHorizontal = (tanVertical * aspect * horizontalFraction) / (1 + marginRatio);
  const limitVertical = (tanVertical * verticalFraction) / (1 + marginRatio);
  const offset = new THREE.Vector3();
  let distance = 0;
  let nearestDepth = Infinity;
  for (const corner of corners) {
    offset.set(corner[0] - target[0], corner[1] - target[1], corner[2] - target[2]);
    // Depth of the corner beyond the target, away from the camera.
    const depth = -offset.dot(toCamera);
    const x = offset.dot(right);
    const y = offset.dot(up);
    distance = Math.max(distance, Math.abs(x) / limitHorizontal - depth, Math.abs(y) / limitVertical - depth);
    nearestDepth = Math.min(nearestDepth, depth);
  }
  return Math.max(distance, 1.0 - nearestDepth);
}
