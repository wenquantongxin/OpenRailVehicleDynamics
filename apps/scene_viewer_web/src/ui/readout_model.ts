import * as THREE from 'three';

import { ScalarStatus, frameTimeSeconds, type SceneRecord } from '../record/scene_record.ts';
import type { TrackModel } from '../scene/track_model.ts';
import { niceCeiling } from './format.ts';

// Maps the record's scalars onto the cards. Nothing here computes a new
// physical quantity: values are recorded samples, converted to display units,
// and the only derived numbers are the carbody speed (the norm of its recorded
// velocity), record-wide scale ceilings, axle numbering by position and the
// plan heading used to draw a body in the axle plan (from its recorded pose and
// the recorded track frame). A scalar that is not ready stays unknown: it is
// never filled in with a nominal value.

export interface ScalarRef {
  index: number;
}

export interface WheelRef {
  placement: number;
  code: string;
  side: 'left' | 'right';
  force: ScalarRef | null;
  normal: ScalarRef | null;
  patches: ScalarRef | null;
}

export interface AxleRef {
  number: number;
  /** Longitudinal position relative to the carbody at the first frame (m). */
  offset: number;
  bridge: TrackBodyRef | null;
  left: WheelRef | null;
  right: WheelRef | null;
}

export interface TrackBodyRef {
  bodyName: string;
  en: string;
  zh: string;
  offset: number;
  lateral: ScalarRef | null;
  yaw: ScalarRef | null;
  station: ScalarRef | null;
}

export interface ContactSpan {
  startFrame: number;
  endFrame: number;
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

export interface ReadoutModel {
  axles: AxleRef[];
  wheels: WheelRef[];
  carbody: TrackBodyRef | null;
  bogies: TrackBodyRef[];
  scales: { forceKn: number; bodyLateralMm: number; bodyYawMrad: number };
  contactSpans: ContactSpan[];
  /** Carbody chainage at every frame. */
  carbodyStation: Float64Array;
  carbodyBodyIndex: number | null;
  initialSpeedKmh: number | null;
  sampleIntervalSeconds: number;
  read: (frame: number, ref: ScalarRef | null) => Reading | null;
  speedKmhAt: (frame: number) => number | null;
  contactAt: (frame: number) => ContactSummary;
  /** Loaded contact patch count per wheel placement; NaN where it is not ready. */
  contactPatchesAt: (frame: number, out: Float64Array) => void;
  /**
   * Plan heading of a body relative to the track at its station, radians,
   * positive when its +x axis turns towards the right-hand rail. Computed from
   * the recorded pose and the recorded track frame; null without a track.
   */
  headingAt: (frame: number, body: TrackBodyRef) => number | null;
}

/** 1 -> 一, 2 -> 二 ...; vehicle ends and bogies are numbered from the +x end. */
function chineseOrdinal(value: number): string {
  return ['一', '二', '三', '四', '五', '六'][value - 1] ?? String(value);
}

function bodyPosition(record: SceneRecord, frame: number, body: number): THREE.Vector3 {
  const base = frame * record.columns.rowValueCount + record.columns.bodyOffset + body * record.columns.valuesPerBody;
  const v = record.values;
  return new THREE.Vector3(v[base] as number, v[base + 1] as number, v[base + 2] as number);
}

function bodyQuaternion(record: SceneRecord, frame: number, body: number): THREE.Quaternion {
  const base = frame * record.columns.rowValueCount + record.columns.bodyOffset + body * record.columns.valuesPerBody + 3;
  const v = record.values;
  return new THREE.Quaternion(v[base + 1] as number, v[base + 2] as number, v[base + 3] as number, v[base] as number);
}

export function buildReadoutModel(record: SceneRecord, track: TrackModel | null): ReadoutModel {
  const scalarIndex = new Map(record.scalars.map((definition, index) => [definition.name, index]));
  const ref = (name: string): ScalarRef | null => {
    const index = scalarIndex.get(name);
    return index === undefined ? null : { index };
  };
  const bodyIndex = new Map(record.bodies.map((body, index) => [body.name, index]));
  const carbodyName = record.bodies.find((body) => body.name === 'carbody' || body.name.startsWith('carbody'))?.name ?? null;
  const carbodyBodyIndex = carbodyName === null ? null : (bodyIndex.get(carbodyName) ?? null);

  // Longitudinal offsets along the carbody x axis at the first frame.
  const origin = carbodyBodyIndex === null ? new THREE.Vector3() : bodyPosition(record, 0, carbodyBodyIndex);
  const axis =
    carbodyBodyIndex === null
      ? new THREE.Vector3(1, 0, 0)
      : new THREE.Vector3(1, 0, 0).applyQuaternion(bodyQuaternion(record, 0, carbodyBodyIndex));
  const offsetOf = (name: string): number => {
    const index = bodyIndex.get(name);
    return index === undefined ? 0 : bodyPosition(record, 0, index).sub(origin).dot(axis);
  };
  const trackBody = (bodyName: string, en: string, zh: string): TrackBodyRef => ({
    bodyName,
    en,
    zh,
    offset: offsetOf(bodyName),
    lateral: ref(`${bodyName}.lateral_meters`),
    yaw: ref(`${bodyName}.yaw_radians`),
    station: ref(`${bodyName}.track_station_meters`),
  });

  // Axles: wheels grouped by longitudinal position, numbered from the front.
  const placements = record.wheelPlacements.map((placement, index) => ({
    placement,
    index,
    offset: offsetOf(placement.wheelBodyName),
  }));
  placements.sort((a, b) => b.offset - a.offset);
  const groups: (typeof placements)[] = [];
  for (const entry of placements) {
    const group = groups[groups.length - 1];
    if (group !== undefined && Math.abs((group[0]?.offset ?? 0) - entry.offset) < 0.3) {
      group.push(entry);
    } else {
      groups.push([entry]);
    }
  }
  const bridgeNames = record.bodies
    .map((body) => body.name)
    .filter((name) => name.startsWith('axlebridge') || name.startsWith('wheelset'));
  const wheels: WheelRef[] = [];
  const axles: AxleRef[] = groups.map((group, position) => {
    const number = position + 1;
    const offset = group.reduce((sum, entry) => sum + entry.offset, 0) / group.length;
    const wheelFor = (side: 'left' | 'right'): WheelRef | null => {
      const entry = group.find((candidate) => candidate.placement.side === side);
      if (entry === undefined) {
        return null;
      }
      const name = entry.placement.interfaceName;
      const wheel: WheelRef = {
        placement: entry.index,
        code: `${number}${side === 'left' ? 'L' : 'R'}`,
        side,
        force: ref(`${name}.vertical_support_force_on_wheel_newtons`),
        normal: ref(`${name}.normal_force_newtons`),
        patches: ref(`${name}.contact_patch_count`),
      };
      wheels.push(wheel);
      return wheel;
    };
    const bridgeName = bridgeNames.find((name) => Math.abs(offsetOf(name) - offset) < 0.3) ?? null;
    return {
      number,
      offset,
      bridge: bridgeName === null ? null : trackBody(bridgeName, `Axle bridge ${number}`, `${number}位轴桥`),
      left: wheelFor('left'),
      right: wheelFor('right'),
    };
  });

  const bogies = record.bodies
    .map((body) => body.name)
    .filter((name) => name.startsWith('frame') || name.startsWith('bogie'))
    .sort((a, b) => offsetOf(b) - offsetOf(a))
    .map((name, index) => trackBody(name, `Bogie ${index + 1} frame`, `${chineseOrdinal(index + 1)}位端构架`));
  const carbody = carbodyName === null ? null : trackBody(carbodyName, 'Carbody', '车体');

  const columns = record.columns;
  const read = (frame: number, scalar: ScalarRef | null): Reading | null => {
    if (scalar === null) {
      return null;
    }
    const value = record.values[frame * columns.rowValueCount + columns.scalarOffset + scalar.index] as number;
    const status = record.statuses[frame * columns.scalarCount + scalar.index] as number;
    return { value, status, valid: status === ScalarStatus.valid && Number.isFinite(value) };
  };

  // Record-wide ceilings for bars; only valid samples count.
  let forceMax = 0;
  let lateralMax = 0;
  let yawMax = 0;
  for (let frame = 0; frame < record.frameCount; ++frame) {
    for (const wheel of wheels) {
      const force = read(frame, wheel.force);
      if (force?.valid === true) {
        forceMax = Math.max(forceMax, Math.abs(force.value));
      }
    }
    for (const body of [carbody, ...bogies]) {
      if (body === null) {
        continue;
      }
      const lateral = read(frame, body.lateral);
      const yaw = read(frame, body.yaw);
      if (lateral?.valid === true) {
        lateralMax = Math.max(lateralMax, Math.abs(lateral.value));
      }
      if (yaw?.valid === true) {
        yawMax = Math.max(yawMax, Math.abs(yaw.value));
      }
    }
  }

  const contactAt = (frame: number): ContactSummary => {
    const loss: string[] = [];
    const two: string[] = [];
    let known = 0;
    const total = wheels.length;
    for (const wheel of wheels) {
      const patches = read(frame, wheel.patches);
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
    for (let frame = 0; frame < record.frameCount; ++frame) {
      const patches = read(frame, wheel.patches);
      // A sample that is not ready is no event and ends any open span.
      if (patches?.valid !== true) {
        open = null;
        continue;
      }
      const count = patches.value;
      if (count !== 1) {
        if (open !== null && open.patches === count && open.endFrame === frame - 1) {
          open.endFrame = frame;
        } else {
          open = { startFrame: frame, endFrame: frame, patches: count, code: wheel.code };
          contactSpans.push(open);
        }
      }
    }
  }
  contactSpans.sort((a, b) => a.startFrame - b.startFrame);

  const contactPatchesAt = (frame: number, out: Float64Array): void => {
    out.fill(Number.NaN);
    for (const wheel of wheels) {
      const patches = read(frame, wheel.patches);
      if (patches?.valid === true) {
        out[wheel.placement] = patches.value;
      }
    }
  };

  const forward = new THREE.Vector3();
  const headingPosition = new THREE.Vector3();
  const headingAt = (frame: number, body: TrackBodyRef): number | null => {
    const index = bodyIndex.get(body.bodyName);
    if (track === null || index === undefined) {
      return null;
    }
    const recordedStation = read(frame, body.station);
    headingPosition.copy(bodyPosition(record, frame, index));
    const station = recordedStation?.valid === true ? recordedStation.value : track.stationNearest(headingPosition);
    const trackFrame = track.frameAt(station);
    // Every ORVD body frame has +x forward, whatever its y and z basis.
    forward.set(1, 0, 0).applyQuaternion(bodyQuaternion(record, frame, index));
    return Math.atan2(forward.dot(trackFrame.right), forward.dot(trackFrame.tangent));
  };

  const carbodyStation = new Float64Array(record.frameCount);
  const stationRef = carbody?.station ?? null;
  const scratch = new THREE.Vector3();
  for (let frame = 0; frame < record.frameCount; ++frame) {
    const recorded = read(frame, stationRef);
    if (recorded?.valid === true) {
      carbodyStation[frame] = recorded.value;
    } else if (track !== null && carbodyBodyIndex !== null) {
      scratch.copy(bodyPosition(record, frame, carbodyBodyIndex));
      carbodyStation[frame] = track.stationNearest(scratch);
    } else {
      carbodyStation[frame] = Number.NaN;
    }
  }

  const speedKmhAt = (frame: number): number | null => {
    if (carbodyBodyIndex === null) {
      return null;
    }
    const base = frame * columns.rowValueCount + columns.bodyOffset + carbodyBodyIndex * columns.valuesPerBody + 7;
    const v = record.values;
    return 3.6 * Math.hypot(v[base] as number, v[base + 1] as number, v[base + 2] as number);
  };

  const intervals: number[] = [];
  for (let frame = 1; frame < Math.min(record.frameCount, 200); ++frame) {
    intervals.push(frameTimeSeconds(record, frame) - frameTimeSeconds(record, frame - 1));
  }
  intervals.sort((a, b) => a - b);

  return {
    axles,
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
