import * as THREE from 'three';

import { ScalarStatus, bodyLinearVelocity, frameTimeSeconds, type SceneRecord, type WheelSide } from '../record/scene_record.ts';
import type { BilingualName } from '../record/visual_definition.ts';
import type { TrackModel } from '../scene/track_model.ts';
import type { BoundBody, VehicleDisplayBindings } from '../scene/vehicle_display_bindings.ts';
import { niceCeiling } from './format.ts';

// Maps the record's scalars onto the cards through the resolved display
// bindings. Nothing here discovers the vehicle: which body is the carbody,
// which are bogie frames and which carrier owns which wheels all come from the
// bindings. Nothing here computes a new physical quantity either: values are
// recorded samples, converted to display units, and the only derived numbers
// are the carbody speed (the norm of its recorded velocity), record-wide scale
// ceilings, initial longitudinal offsets along the carbody axis and the plan
// heading used to draw a body in the carrier plan. A scalar that is not ready
// stays unknown; a scalar the record never exported is a null reference and
// its readout shows as not recorded.
//
// Scalar names follow the exporter's convention: a body's track coordinates
// are `<body>.track_station_meters`, `<body>.lateral_meters` and
// `<body>.yaw_radians`; a wheel's contact results are
// `<interface>.contact_patch_count`,
// `<interface>.vertical_support_force_on_wheel_newtons` and
// `<interface>.normal_force_newtons`.

export interface ScalarRef {
  index: number;
}

export interface WheelReadout {
  placementIndex: number;
  /** Carrier number and side, such as `1L`. */
  code: string;
  side: WheelSide;
  verticalSupportForceScalar: ScalarRef | null;
  normalForceScalar: ScalarRef | null;
  contactPatchCountScalar: ScalarRef | null;
}

export interface TrackBodyReadout {
  bodyName: string;
  bodyIndex: number;
  displayName: BilingualName;
  /** Along the carbody +x axis from the carbody origin at the first frame; null without a carbody binding. */
  initialLongitudinalOffsetInCarbodyFrameMeters: number | null;
  lateralScalar: ScalarRef | null;
  yawScalar: ScalarRef | null;
  stationScalar: ScalarRef | null;
}

export interface CarrierReadoutBinding extends TrackBodyReadout {
  /** 1-based position in the bindings' carrier order. */
  number: number;
  bogieIndex: number | null;
  leftWheel: WheelReadout | null;
  rightWheel: WheelReadout | null;
}

export interface ContactSpan {
  startFrameIndex: number;
  endFrameIndex: number;
  patches: number;
  code: string;
}

/**
 * single: every wheel recorded, all with one contact point. incomplete: some
 * wheels not ready and none of the recorded ones in two-point contact or loss.
 * unknown: no wheel recorded at this sample.
 */
export type ContactState = 'single' | 'two' | 'loss' | 'incomplete' | 'unknown';

export interface ContactSummary {
  state: ContactState;
  /** Wheel codes in loss of contact or two-point contact, by state. */
  codes: string[];
  /** Wheels whose contact patch count is valid at this sample. */
  known: number;
  total: number;
}

export interface Reading {
  value: number;
  valid: boolean;
  status: number;
}

export interface VehicleReadoutModel {
  carriers: CarrierReadoutBinding[];
  wheels: WheelReadout[];
  carbody: TrackBodyReadout | null;
  bogies: TrackBodyReadout[];
  scales: { forceKn: number; bodyLateralMm: number; bodyYawMrad: number };
  contactSpans: ContactSpan[];
  /** Carbody chainage at every frame; NaN throughout without a carbody binding. */
  carbodyStation: Float64Array;
  carbodyBodyIndex: number | null;
  initialSpeedKmh: number | null;
  sampleIntervalSeconds: number;
  read: (frameIndex: number, ref: ScalarRef | null) => Reading | null;
  speedKmhAt: (frameIndex: number) => number | null;
  contactAt: (frameIndex: number) => ContactSummary;
  /** Loaded contact patch count per wheel placement; NaN where it is not ready. */
  contactPatchesAt: (frameIndex: number, out: Float64Array) => void;
  /**
   * Plan heading of a body relative to the track at its station, radians,
   * positive when its +x axis turns towards the right-hand rail. Computed from
   * the recorded pose and the recorded track frame; null without a track.
   */
  headingAt: (frameIndex: number, body: TrackBodyReadout) => number | null;
}

function bodyPosition(record: SceneRecord, frameIndex: number, bodyIndex: number): THREE.Vector3 {
  const base = frameIndex * record.columns.rowValueCount + record.columns.bodyStatesColumnOffset + bodyIndex * record.columns.valuesPerBody;
  const v = record.values;
  return new THREE.Vector3(v[base] as number, v[base + 1] as number, v[base + 2] as number);
}

function bodyQuaternion(record: SceneRecord, frameIndex: number, bodyIndex: number): THREE.Quaternion {
  const base = frameIndex * record.columns.rowValueCount + record.columns.bodyStatesColumnOffset + bodyIndex * record.columns.valuesPerBody + 3;
  const v = record.values;
  return new THREE.Quaternion(v[base + 1] as number, v[base + 2] as number, v[base + 3] as number, v[base] as number);
}

export function buildVehicleReadoutModel(record: SceneRecord, track: TrackModel | null, bindings: VehicleDisplayBindings): VehicleReadoutModel {
  const scalarIndex = new Map(record.scalars.map((definition, index) => [definition.name, index]));
  const ref = (name: string): ScalarRef | null => {
    const index = scalarIndex.get(name);
    return index === undefined ? null : { index };
  };
  const carbodyBodyIndex = bindings.carbody?.bodyIndex ?? null;

  // Longitudinal offsets along the carbody x axis at the first frame; without
  // a carbody there is no such reference and the offsets stay null.
  const origin = carbodyBodyIndex === null ? null : bodyPosition(record, 0, carbodyBodyIndex);
  const axis = carbodyBodyIndex === null ? null : new THREE.Vector3(1, 0, 0).applyQuaternion(bodyQuaternion(record, 0, carbodyBodyIndex));
  const offsetOf = (bodyIndex: number): number | null =>
    origin === null || axis === null ? null : bodyPosition(record, 0, bodyIndex).sub(origin).dot(axis);
  const trackBody = (bound: BoundBody): TrackBodyReadout => ({
    bodyName: bound.bodyName,
    bodyIndex: bound.bodyIndex,
    displayName: bound.displayName,
    initialLongitudinalOffsetInCarbodyFrameMeters: offsetOf(bound.bodyIndex),
    lateralScalar: ref(`${bound.bodyName}.lateral_meters`),
    yawScalar: ref(`${bound.bodyName}.yaw_radians`),
    stationScalar: ref(`${bound.bodyName}.track_station_meters`),
  });

  const wheels: WheelReadout[] = [];
  const wheelReadout = (placementIndex: number | null, number: number, side: WheelSide): WheelReadout | null => {
    if (placementIndex === null) {
      return null;
    }
    const placement = record.wheelPlacements[placementIndex];
    if (placement === undefined) {
      throw new Error(`display bindings refer to wheel placement ${placementIndex}, which the record does not have`);
    }
    const name = placement.interfaceName;
    const wheel: WheelReadout = {
      placementIndex,
      code: `${number}${side === 'left' ? 'L' : 'R'}`,
      side,
      verticalSupportForceScalar: ref(`${name}.vertical_support_force_on_wheel_newtons`),
      normalForceScalar: ref(`${name}.normal_force_newtons`),
      contactPatchCountScalar: ref(`${name}.contact_patch_count`),
    };
    wheels.push(wheel);
    return wheel;
  };
  const carriers: CarrierReadoutBinding[] = bindings.carriers.map((carrier, position) => {
    const number = position + 1;
    return {
      ...trackBody(carrier),
      number,
      bogieIndex: carrier.bogieIndex,
      leftWheel: wheelReadout(carrier.leftWheelPlacementIndex, number, 'left'),
      rightWheel: wheelReadout(carrier.rightWheelPlacementIndex, number, 'right'),
    };
  });
  const carbody = bindings.carbody === null ? null : trackBody(bindings.carbody);
  const bogies = bindings.bogies.map((bogie) => trackBody(bogie));

  const columns = record.columns;
  const read = (frameIndex: number, scalar: ScalarRef | null): Reading | null => {
    if (scalar === null) {
      return null;
    }
    const value = record.values[frameIndex * columns.rowValueCount + columns.scalarValuesColumnOffset + scalar.index] as number;
    const status = record.statuses[frameIndex * columns.scalarCount + scalar.index] as number;
    return { value, status, valid: status === ScalarStatus.valid && Number.isFinite(value) };
  };

  // Record-wide ceilings for bars; only valid samples count.
  let forceMax = 0;
  let lateralMax = 0;
  let yawMax = 0;
  for (let frameIndex = 0; frameIndex < record.frameCount; ++frameIndex) {
    for (const wheel of wheels) {
      const force = read(frameIndex, wheel.verticalSupportForceScalar);
      if (force?.valid === true) {
        forceMax = Math.max(forceMax, Math.abs(force.value));
      }
    }
    for (const body of [carbody, ...bogies]) {
      if (body === null) {
        continue;
      }
      const lateral = read(frameIndex, body.lateralScalar);
      const yaw = read(frameIndex, body.yawScalar);
      if (lateral?.valid === true) {
        lateralMax = Math.max(lateralMax, Math.abs(lateral.value));
      }
      if (yaw?.valid === true) {
        yawMax = Math.max(yawMax, Math.abs(yaw.value));
      }
    }
  }

  const contactAt = (frameIndex: number): ContactSummary => {
    const loss: string[] = [];
    const two: string[] = [];
    let known = 0;
    const total = wheels.length;
    for (const wheel of wheels) {
      const patches = read(frameIndex, wheel.contactPatchCountScalar);
      if (patches?.valid !== true) {
        continue;
      }
      known += 1;
      if (patches.value === 0) {
        loss.push(wheel.code);
      } else if (patches.value >= 2) {
        two.push(wheel.code);
      }
    }
    if (known === 0) {
      return { state: 'unknown', codes: [], known, total };
    }
    if (loss.length > 0) {
      return { state: 'loss', codes: loss, known, total };
    }
    if (two.length > 0) {
      return { state: 'two', codes: two, known, total };
    }
    return { state: known === total ? 'single' : 'incomplete', codes: [], known, total };
  };

  const contactSpans: ContactSpan[] = [];
  for (const wheel of wheels) {
    let open: ContactSpan | null = null;
    for (let frameIndex = 0; frameIndex < record.frameCount; ++frameIndex) {
      const patches = read(frameIndex, wheel.contactPatchCountScalar);
      // A sample that is not ready is no event and ends any open span.
      if (patches?.valid !== true) {
        open = null;
        continue;
      }
      const count = patches.value;
      if (count !== 1) {
        if (open !== null && open.patches === count && open.endFrameIndex === frameIndex - 1) {
          open.endFrameIndex = frameIndex;
        } else {
          open = { startFrameIndex: frameIndex, endFrameIndex: frameIndex, patches: count, code: wheel.code };
          contactSpans.push(open);
        }
      }
    }
  }
  contactSpans.sort((a, b) => a.startFrameIndex - b.startFrameIndex);

  const contactPatchesAt = (frameIndex: number, out: Float64Array): void => {
    out.fill(Number.NaN);
    for (const wheel of wheels) {
      const patches = read(frameIndex, wheel.contactPatchCountScalar);
      if (patches?.valid === true) {
        out[wheel.placementIndex] = patches.value;
      }
    }
  };

  const forward = new THREE.Vector3();
  const headingPosition = new THREE.Vector3();
  const headingAt = (frameIndex: number, body: TrackBodyReadout): number | null => {
    if (track === null) {
      return null;
    }
    const recordedStation = read(frameIndex, body.stationScalar);
    headingPosition.copy(bodyPosition(record, frameIndex, body.bodyIndex));
    const stationMeters = recordedStation?.valid === true ? recordedStation.value : track.stationNearestToInertialPosition(headingPosition);
    const trackFrame = track.trackFrameAtStation(stationMeters);
    // Every ORVD body frame has +x forward, whatever its y and z basis.
    forward.set(1, 0, 0).applyQuaternion(bodyQuaternion(record, frameIndex, body.bodyIndex));
    return Math.atan2(forward.dot(trackFrame.right), forward.dot(trackFrame.tangent));
  };

  const carbodyStation = new Float64Array(record.frameCount);
  const stationRef = carbody?.stationScalar ?? null;
  const scratch = new THREE.Vector3();
  for (let frameIndex = 0; frameIndex < record.frameCount; ++frameIndex) {
    const recorded = read(frameIndex, stationRef);
    if (recorded?.valid === true) {
      carbodyStation[frameIndex] = recorded.value;
    } else if (track !== null && carbodyBodyIndex !== null) {
      scratch.copy(bodyPosition(record, frameIndex, carbodyBodyIndex));
      carbodyStation[frameIndex] = track.stationNearestToInertialPosition(scratch);
    } else {
      carbodyStation[frameIndex] = Number.NaN;
    }
  }

  const speedKmhAt = (frameIndex: number): number | null => {
    if (carbodyBodyIndex === null) {
      return null;
    }
    const [x, y, z] = bodyLinearVelocity(record, frameIndex, carbodyBodyIndex);
    return 3.6 * Math.hypot(x, y, z);
  };

  const intervals: number[] = [];
  for (let frameIndex = 1; frameIndex < Math.min(record.frameCount, 200); ++frameIndex) {
    intervals.push(frameTimeSeconds(record, frameIndex) - frameTimeSeconds(record, frameIndex - 1));
  }
  intervals.sort((a, b) => a - b);

  return {
    carriers,
    wheels,
    carbody,
    bogies,
    // Display floors keep numerical noise on a quiet record from filling a bar.
    scales: {
      forceKn: niceCeiling(Math.max(10, forceMax / 1000)),
      bodyLateralMm: niceCeiling(Math.max(1, lateralMax * 1000)),
      bodyYawMrad: niceCeiling(Math.max(1, yawMax * 1000)),
    },
    contactSpans,
    carbodyStation,
    carbodyBodyIndex,
    initialSpeedKmh: speedKmhAt(0),
    sampleIntervalSeconds: intervals[intervals.length >> 1] ?? 0,
    read,
    speedKmhAt,
    contactAt,
    contactPatchesAt,
    headingAt,
  };
}
