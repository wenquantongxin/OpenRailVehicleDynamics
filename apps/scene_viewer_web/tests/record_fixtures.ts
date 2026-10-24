import { sceneOrientation, scalarStatusCodes, phaseCodes } from '../src/record/scene_definition.ts';
import type { SceneRecord } from '../src/record/scene_record.ts';
import type { TrackTable, WheelPlacement } from '../src/record/scene_definition.ts';
import { ScalarStatus } from '../src/record/scene_sample.ts';
import { RecordedSamples } from '../src/record/recorded_samples.ts';

// Small programmatic records for the tests: stated bodies, placements, scalars
// and frames laid out exactly as the viewer's parser would lay them out. They
// are display fixtures, not ORVD results.

export interface FixturePose {
  position?: [number, number, number];
  orientationWxyz?: [number, number, number, number];
  linearVelocity?: [number, number, number];
  angularVelocity?: [number, number, number];
}

export interface FixtureFrame {
  timeSeconds: number;
  /** The run program's sample identity; defaults to -1, "absent". */
  sampleIndex?: number;
  phase?: number;
  poses?: Record<string, FixturePose>;
  scalarValues?: number[];
  wheelSpinAngles?: number[];
  scalarStatuses?: number[];
}

export interface RecordFixtureSpec {
  bodies: string[];
  placements?: WheelPlacement[];
  scalars?: string[];
  scalarUnits?: string[];
  frames: FixtureFrame[];
  track?: TrackTable | null;
  visualDefinitionText?: string | null;
}

export function placement(interfaceName: string, wheelBodyName: string, carrierBodyName: string, side: 'left' | 'right', spinAxisY = -1): WheelPlacement {
  return {
    interfaceName,
    wheelBodyName,
    carrierBodyName,
    side,
    datumInWheelBodyFrameMeters: [0, side === 'left' ? 0.7465 : -0.7465, 0],
    spinAxisInWheelBodyFrame: [0, spinAxisY, 0],
    nominalRollingRadiusMeters: 0.43,
    spinJointName: null,
    spinAngleScalar: null,
  };
}

export function makeRecord(spec: RecordFixtureSpec): SceneRecord {
  const bodies = spec.bodies;
  const placements = spec.placements ?? [];
  const scalars = spec.scalars ?? [];
  const bodyStatesColumnOffset = 4;
  const valuesPerBody = 13;
  const wheelSpinAnglesColumnOffset = bodyStatesColumnOffset + valuesPerBody * bodies.length;
  const wheelSpinAngleCount = spec.frames.some((frame) => frame.wheelSpinAngles !== undefined) ? placements.length : 0;
  const scalarValuesColumnOffset = wheelSpinAnglesColumnOffset + wheelSpinAngleCount;
  const rowValueCount = scalarValuesColumnOffset + scalars.length;
  const frameCount = spec.frames.length;
  const values = new Float64Array(frameCount * rowValueCount);
  const statuses = new Uint8Array(frameCount * scalars.length).fill(ScalarStatus.valid);
  spec.frames.forEach((frame, frameIndex) => {
    const row = frameIndex * rowValueCount;
    values[row] = frame.timeSeconds;
    values[row + 1] = Math.round(frame.timeSeconds * 1e9);
    values[row + 2] = frame.sampleIndex ?? -1;
    values[row + 3] = frame.phase ?? 1;
    bodies.forEach((name, bodyIndex) => {
      const base = row + bodyStatesColumnOffset + valuesPerBody * bodyIndex;
      const pose = frame.poses?.[name] ?? {};
      values.set(pose.position ?? [0, 0, 0], base);
      values.set(pose.orientationWxyz ?? [1, 0, 0, 0], base + 3);
      values.set(pose.linearVelocity ?? [0, 0, 0], base + 7);
      values.set(pose.angularVelocity ?? [0, 0, 0], base + 10);
    });
    if (frame.wheelSpinAngles !== undefined) values.set(frame.wheelSpinAngles, row + wheelSpinAnglesColumnOffset);
    if (frame.scalarValues !== undefined) {
      values.set(frame.scalarValues, row + scalarValuesColumnOffset);
    }
    if (frame.scalarStatuses !== undefined) {
      statuses.set(frame.scalarStatuses, frameIndex * scalars.length);
    }
  });
  return {
    definition: {
      worldFrame: 'test', lengthUnit: 'meter', timeUnit: 'second', angleUnit: 'radian',
      orientation: sceneOrientation, nominalSamplePeriodSeconds: null, scalarStatusCodes, phaseCodes,
      bodies: bodies.map((name) => ({ name, movesFreelyInWorld: true })),
      wheelPlacements: placements,
      scalars: scalars.map((name, index) => ({ name, unit: spec.scalarUnits?.[index] ?? '1', referenceFrame: 'test', quantity: 'test', method: 'test', sampleSemantics: 'instantaneous' })),
      visualDefinitionText: spec.visualDefinitionText ?? null,
      track: spec.track ?? null,
    },
    samples: new RecordedSamples({
      timeSecondsColumnOffset: 0,
      timeNanosecondsColumnOffset: 1,
      sampleIndexColumnOffset: 2,
      phaseColumnOffset: 3,
      bodyStatesColumnOffset,
      valuesPerBody,
      wheelSpinAnglesColumnOffset,
      wheelSpinAngleCount,
      scalarValuesColumnOffset,
      scalarCount: scalars.length,
      rowValueCount,
    }, values, statuses, { scalars: scalars.map((name, index) => ({ name, unit: spec.scalarUnits?.[index] ?? '1', referenceFrame: 'test', quantity: 'test', method: 'test', sampleSemantics: 'instantaneous' })), wheelPlacements: placements }),
  };
}

/** A visual definition text with no parts and the given display bindings, for binding tests. */
export function visualDefinitionWithBindings(bindings: unknown): string {
  return JSON.stringify({
    vehicle_name: 'FIXTURE',
    length_unit: 'meter',
    display_name: { en: 'Fixture vehicle', zh: '夹具车辆' },
    wheel_visual: {
      width_meters: 0.135,
      back_face_offset_meters: 0.07,
      flange_height_meters: 0.028,
      flange_thickness_meters: 0.032,
      rim_depth_meters: 0.045,
      web_thickness_meters: 0.03,
      hub_radius_meters: 0.11,
      hub_length_meters: 0.13,
      rim_marker: true,
    },
    parts: [],
    display_bindings: bindings,
  });
}
