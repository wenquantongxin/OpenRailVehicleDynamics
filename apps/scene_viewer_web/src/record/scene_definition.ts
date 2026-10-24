import * as field from './json_fields.ts';

export type WheelSide = 'left' | 'right';

export interface SceneBody {
  name: string;
  movesFreelyInWorld: boolean;
}

export interface WheelPlacement {
  interfaceName: string;
  wheelBodyName: string;
  /**
   * The body carrying this wheel's non-spinning profile frame: the axle bridge
   * of an independently rotating wheel, or the wheelset body itself for a
   * rigid wheelset, in which case it equals `wheelBodyName`. The left and
   * right wheels of one carrier are paired through it.
   */
  carrierBodyName: string;
  side: WheelSide;
  datumInWheelBodyFrameMeters: [number, number, number];
  spinAxisInWheelBodyFrame: [number, number, number];
  nominalRollingRadiusMeters: number;
  spinJointName: string | null;
  spinAngleScalar: { name: string; sign: 1 | -1 } | null;
}

export interface ScalarDefinition {
  name: string;
  unit: string;
  referenceFrame: string;
  quantity: string;
  method: string;
  sampleSemantics: string;
}

export interface RailDatum { lateralMeters: number; verticalMeters: number; }

export interface TrackTable {
  railDatums: { left: RailDatum; right: RailDatum };
  stationsMeters: number[];
  centerlineInInertialMeters: number[][];
  rotationInertialFromTrackWxyz: number[][];
  curvatureRadiansPerMeter: number[];
  superelevationMeters: number[];
  leftRailDatumInInertialMeters: number[][];
  rightRailDatumInInertialMeters: number[][];
}

/** Public topology and conventions, independent of records, streams and resources. */
export interface SceneDescription {
  worldFrame: string;
  lengthUnit: 'meter';
  timeUnit: 'second';
  angleUnit: 'radian';
  orientation: string;
  nominalSamplePeriodSeconds: number | null;
  scalarStatusCodes: Readonly<Record<string, string>>;
  phaseCodes: Readonly<Record<string, string>>;
  bodies: SceneBody[];
  wheelPlacements: WheelPlacement[];
  scalars: ScalarDefinition[];
}

/** A public description combined with resolved rendering resources. */
export interface SceneDefinition extends SceneDescription {
  visualDefinitionText: string | null;
  track: TrackTable | null;
}

export const sceneOrientation = "unit quaternion [w, x, y, z] mapping body coordinates to world coordinates; a wheel body's orientation includes its spin";
export const scalarStatusCodes = { '0': 'not_ready', '1': 'valid', '2': 'placeholder' } as const;
export const phaseCodes = { '0': 'initial_accepted_state', '1': 'dense_intermediate_sample', '2': 'accepted_endpoint' } as const;

function codes(value: unknown, expected: Readonly<Record<string, string>>, what: string): Readonly<Record<string, string>> {
  const entries = field.object(value, what);
  if (Object.keys(entries).length !== Object.keys(expected).length || Object.entries(expected).some(([key, label]) => entries[key] !== label)) {
    throw new Error(`${what} disagrees with the public scene codes`);
  }
  return { ...expected };
}

/** Parse only the common description object, without inspecting an envelope or loading assets. */
export function parseSceneDefinition(value: unknown): SceneDescription {
  const scene = field.object(value, 'description');
  const worldFrame = field.string(scene['world_frame'], 'description.world_frame');
  field.exact(scene['length_unit'], 'meter', 'description.length_unit');
  field.exact(scene['time_unit'], 'second', 'description.time_unit');
  field.exact(scene['angle_unit'], 'radian', 'description.angle_unit');
  const orientation = field.exact(scene['orientation'], sceneOrientation, 'description.orientation');
  const nominal = scene['nominal_sample_period_seconds'];
  const nominalSamplePeriodSeconds = nominal === null ? null : field.number(nominal, 'nominal_sample_period_seconds');
  if (nominalSamplePeriodSeconds !== null && nominalSamplePeriodSeconds <= 0) throw new Error('nominal_sample_period_seconds must be positive or null');
  const bodies = field.array(scene['bodies'], 'bodies').map((entry, index): SceneBody => {
    const body = field.object(entry, `bodies[${index}]`);
    if (typeof body['moves_freely_in_world'] !== 'boolean') throw new Error(`bodies[${index}].moves_freely_in_world must be boolean`);
    return { name: field.string(body['name'], `bodies[${index}].name`), movesFreelyInWorld: body['moves_freely_in_world'] };
  });
  if (bodies.length === 0) throw new Error('description names no rigid body');
  field.unique(bodies.map((body) => body.name), 'bodies');
  const bodyNames = new Set(bodies.map((body) => body.name));
  const scalars = field.array(scene['scalars'], 'scalars').map((entry, index): ScalarDefinition => {
    const scalar = field.object(entry, `scalars[${index}]`);
    return {
      name: field.string(scalar['name'], 'scalar.name'), unit: field.text(scalar['unit'], 'scalar.unit'),
      referenceFrame: field.text(scalar['reference_frame'], 'scalar.reference_frame'),
      quantity: field.text(scalar['quantity'], 'scalar.quantity'), method: field.text(scalar['method'], 'scalar.method'),
      sampleSemantics: field.text(scalar['sample_semantics'], 'scalar.sample_semantics'),
    };
  });
  field.unique(scalars.map((scalar) => scalar.name), 'scalars');
  const scalarByName = new Map(scalars.map((scalar) => [scalar.name, scalar]));
  const sidesByCarrier = new Set<string>();
  const wheelPlacements = field.array(scene['wheel_placements'], 'wheel_placements').map((entry, index): WheelPlacement => {
    const wheel = field.object(entry, `wheel_placements[${index}]`);
    const side = field.string(wheel['side'], 'wheel.side');
    if (side !== 'left' && side !== 'right') throw new Error(`unknown wheel side '${side}'`);
    const wheelBodyName = field.string(wheel['wheel_body_name'], 'wheel.wheel_body_name');
    const carrierBodyName = field.string(wheel['carrier_body_name'], 'wheel.carrier_body_name');
    if (!bodyNames.has(wheelBodyName) || !bodyNames.has(carrierBodyName)) throw new Error('wheel placement names an unknown body');
    const carrierSide = JSON.stringify([carrierBodyName, side]);
    if (sidesByCarrier.has(carrierSide)) throw new Error(`carrier '${carrierBodyName}' has two ${side} wheels`);
    sidesByCarrier.add(carrierSide);
    const axis = field.triple(wheel['spin_axis_in_wheel_body_frame'], 'wheel.spin_axis_in_wheel_body_frame');
    if (Math.abs(axis.reduce((sum, value) => sum + value * value, 0) - 1) > 1e-8) throw new Error('wheel spin axis must be a unit vector');
    const radius = field.number(wheel['nominal_rolling_radius_meters'], 'wheel.nominal_rolling_radius_meters');
    if (radius <= 0) throw new Error('wheel nominal rolling radius must be positive');
    let spinAngleScalar: WheelPlacement['spinAngleScalar'] = null;
    if (wheel['spin_angle_scalar'] !== null) {
      const binding = field.object(wheel['spin_angle_scalar'], 'wheel.spin_angle_scalar');
      const name = field.string(binding['name'], 'wheel.spin_angle_scalar.name');
      const sign = binding['sign'];
      if (sign !== 1 && sign !== -1) throw new Error('wheel.spin_angle_scalar.sign must be +1 or -1');
      if (scalarByName.get(name)?.unit !== 'rad') throw new Error(`wheel spin scalar '${name}' must name a scalar with unit rad`);
      spinAngleScalar = { name, sign };
    }
    return {
      interfaceName: field.string(wheel['interface_name'], 'wheel.interface_name'), wheelBodyName, carrierBodyName, side,
      datumInWheelBodyFrameMeters: field.triple(wheel['datum_in_wheel_body_frame_meters'], 'wheel.datum'),
      spinAxisInWheelBodyFrame: axis, nominalRollingRadiusMeters: radius,
      spinJointName: field.nullableString(wheel['spin_joint_name'], 'wheel.spin_joint_name'), spinAngleScalar,
    };
  });
  field.unique(wheelPlacements.map((wheel) => wheel.interfaceName), 'wheel interfaces');
  return { worldFrame, lengthUnit: 'meter', timeUnit: 'second', angleUnit: 'radian', orientation,
    nominalSamplePeriodSeconds, bodies, wheelPlacements, scalars,
    scalarStatusCodes: codes(scene['scalar_status_codes'], scalarStatusCodes, 'scalar_status_codes'),
    phaseCodes: codes(scene['phase_codes'], phaseCodes, 'phase_codes') };
}

/** Stream envelope decoding shares exactly the record's public description parser. */
export function parseSceneDescription(value: unknown): { description: SceneDescription; resourceBundle: string | null } {
  const envelope = field.object(value, 'scene description');
  field.exact(envelope['schema'], 'orvd.scene_description', 'schema');
  return { description: parseSceneDefinition(envelope['description']),
    resourceBundle: field.nullableString(envelope['resource_bundle'], 'resource_bundle') };
}
