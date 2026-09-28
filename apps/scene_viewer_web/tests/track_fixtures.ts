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
