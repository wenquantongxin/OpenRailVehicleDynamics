import * as THREE from 'three';

import type { TrackTable } from '../src/record/scene_record.ts';

/** A straight line along inertial x with a planar curvature profile; rails at ±halfGauge. */
export function straightTable(stations: number[], curvature: (s: number) => number, halfGauge = 0.75315): TrackTable {
  return {
    stationsMeters: stations,
    curvatureRadiansPerMeter: stations.map(curvature),
    superelevationMeters: stations.map(() => 0),
    centerlineInInertialMeters: stations.map((s) => [s, 0, 0]),
    rotationInertialFromTrackWxyz: stations.map(() => [1, 0, 0, 0]),
    leftRailDatumInInertialMeters: stations.map((s) => [s, -halfGauge, -0.00018]),
    rightRailDatumInInertialMeters: stations.map((s) => [s, halfGauge, -0.00018]),
  };
}

export function range(first: number, last: number, step: number): number[] {
  const values: number[] = [];
  for (let k = 0; first + k * step <= last + 1e-9; ++k) {
    values.push(first + k * step);
  }
  return values;
}

/**
 * A straight line turned by `headingRadians` about the inertial +z axis (which
 * points down, so a positive heading turns towards +y) and canted by
 * `cantRollRadians` about its own tangent, with rails at ±halfGauge in the
 * rolled Track-T frame. Used to check that headings are relative to the local
 * track frame, not to the world axes.
 */
export function turnedTable(stations: number[], headingRadians: number, cantRollRadians: number, halfGauge = 0.75315): TrackTable {
  const rotation = new THREE.Quaternion()
    .setFromAxisAngle(new THREE.Vector3(0, 0, 1), headingRadians)
    .multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), cantRollRadians));
  const centre = (s: number): THREE.Vector3 => new THREE.Vector3(s, 0, 0).applyQuaternion(rotation);
  const rail = (s: number, lateral: number): number[] => centre(s).add(new THREE.Vector3(0, lateral, -0.00018).applyQuaternion(rotation)).toArray();
  return {
    stationsMeters: stations,
    curvatureRadiansPerMeter: stations.map(() => 0),
    superelevationMeters: stations.map(() => 0),
    centerlineInInertialMeters: stations.map((s) => centre(s).toArray()),
    rotationInertialFromTrackWxyz: stations.map(() => [rotation.w, rotation.x, rotation.y, rotation.z]),
    leftRailDatumInInertialMeters: stations.map((s) => rail(s, -halfGauge)),
    rightRailDatumInInertialMeters: stations.map((s) => rail(s, halfGauge)),
  };
}

/** The rotation of `turnedTable`, for posing bodies in the same turned and canted frame. */
export function turnedRotation(headingRadians: number, cantRollRadians: number): THREE.Quaternion {
  return new THREE.Quaternion()
    .setFromAxisAngle(new THREE.Vector3(0, 0, 1), headingRadians)
    .multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), cantRollRadians));
}
