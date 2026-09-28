import * as THREE from 'three';

import type { PartGroup, ScenePart } from './scene_builder.ts';
import type { TrackModel } from './track_model.ts';

// The display extent of a vehicle, or of one bogie's running gear, in a local
// frame at its anchor: forward along the line, left, up. It is measured once
// at the first displayed pose from the parts' own geometry boxes taken through
// their world matrices into the frame, so a vehicle turned along a curve is
// measured as itself and not as its world-axis-aligned box. Visibility is
// ignored on purpose: the extent describes the vehicle definition, not the
// display switches, so hiding the carbody does not re-frame the picture.

export interface LocalFrame {
  origin: THREE.Vector3;
  forward: THREE.Vector3;
  left: THREE.Vector3;
  up: THREE.Vector3;
}

/** Bounds in a local frame, metres: forward, left and up components. */
export interface DisplayExtent {
  minForward: number;
  maxForward: number;
  minLeft: number;
  maxLeft: number;
  minUp: number;
  maxUp: number;
}

/** Three.js world up: the root group turns ORVD's z-down inertial frame into Y-up. */
export const worldUp = new THREE.Vector3(0, 1, 0);

/** Part groups that belong to the vehicle; line, ground, coordinate axes and labels are not framed. */
export const framedPartGroups: ReadonlySet<PartGroup> = new Set<PartGroup>(['carbody', 'running_gear', 'wheels', 'body_axes']);

export function extentLengthMeters(extent: DisplayExtent): number {
  return extent.maxForward - extent.minForward;
}

export function extentWidthMeters(extent: DisplayExtent): number {
  return extent.maxLeft - extent.minLeft;
}

export function extentHeightMeters(extent: DisplayExtent): number {
  return extent.maxUp - extent.minUp;
}

export function extentCentre(extent: DisplayExtent): [number, number, number] {
  return [0.5 * (extent.minForward + extent.maxForward), 0.5 * (extent.minLeft + extent.maxLeft), 0.5 * (extent.minUp + extent.maxUp)];
}

/** The eight corners of an extent, in its local frame. */
export function extentCorners(extent: DisplayExtent): [number, number, number][] {
  const corners: [number, number, number][] = [];
  for (let k = 0; k < 8; ++k) {
    corners.push([k & 1 ? extent.maxForward : extent.minForward, k & 2 ? extent.maxLeft : extent.minLeft, k & 4 ? extent.maxUp : extent.minUp]);
  }
  return corners;
}

/**
 * The local frame at an anchor: forward is the horizontal heading of the
 * recorded line at the nearest station, so the frame does not sway with the
 * body; without a line it is the horizontal part of `fallbackForwardWorld`.
 * `root` is the scene root that maps the inertial frame to Three.js world.
 */
export function anchorLocalFrame(originWorld: THREE.Vector3, track: TrackModel | null, root: THREE.Object3D, fallbackForwardWorld: THREE.Vector3): LocalFrame {
  const forward = new THREE.Vector3();
  if (track !== null) {
    const inertial = root.worldToLocal(originWorld.clone());
    const stationMeters = track.stationNearestToInertialPosition(inertial);
    const tangent = track.trackFrameAtStation(stationMeters).tangent;
    // Inertial (x, y, z) maps to Three.js (x, -z, y); keep the horizontal part.
    forward.set(tangent.x, 0, tangent.y);
  } else {
    forward.copy(fallbackForwardWorld);
    forward.y = 0;
  }
  if (forward.lengthSq() < 1e-12) {
    forward.set(1, 0, 0);
  }
  forward.normalize();
  const left = new THREE.Vector3().crossVectors(worldUp, forward).normalize();
  return { origin: originWorld.clone(), forward, left, up: worldUp.clone() };
}

const localFromWorld = new THREE.Matrix4();
const geometryToLocal = new THREE.Matrix4();
const scratchBox = new THREE.Box3();

/**
 * Bounds of the framed parts in a local frame, or null when no part
 * contributes. With `bodyNames`, only parts on those bodies count. Every mesh,
 * line and point cloud under a part contributes its geometry's bounding box
 * through its current world matrix, so the matrices must be up to date.
 */
export function measureDisplayExtent(parts: readonly ScenePart[], frame: LocalFrame, bodyNames: ReadonlySet<string> | null): DisplayExtent | null {
  localFromWorld.makeBasis(frame.forward, frame.left, frame.up).setPosition(frame.origin).invert();
  let extent: DisplayExtent | null = null;
  for (const part of parts) {
    if (!framedPartGroups.has(part.group) || (bodyNames !== null && !bodyNames.has(part.bodyName))) {
      continue;
    }
    part.object.traverse((object) => {
      if (!(object instanceof THREE.Mesh || object instanceof THREE.Line || object instanceof THREE.Points)) {
        return;
      }
      const geometry = object.geometry as THREE.BufferGeometry;
      if (geometry.boundingBox === null) {
        geometry.computeBoundingBox();
      }
      const box = geometry.boundingBox;
      if (box === null || box.isEmpty()) {
        return;
      }
      scratchBox.copy(box).applyMatrix4(geometryToLocal.multiplyMatrices(localFromWorld, object.matrixWorld));
      extent =
        extent === null
          ? { minForward: scratchBox.min.x, maxForward: scratchBox.max.x, minLeft: scratchBox.min.y, maxLeft: scratchBox.max.y, minUp: scratchBox.min.z, maxUp: scratchBox.max.z }
          : {
              minForward: Math.min(extent.minForward, scratchBox.min.x),
              maxForward: Math.max(extent.maxForward, scratchBox.max.x),
              minLeft: Math.min(extent.minLeft, scratchBox.min.y),
              maxLeft: Math.max(extent.maxLeft, scratchBox.max.y),
              minUp: Math.min(extent.minUp, scratchBox.min.z),
              maxUp: Math.max(extent.maxUp, scratchBox.max.z),
            };
    });
  }
  return extent;
}
