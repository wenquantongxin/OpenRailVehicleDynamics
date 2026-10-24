import { makeRecord, placement, visualDefinitionWithBindings, type RecordFixtureSpec } from './record_fixtures.ts';
import type { SceneRecord } from '../src/record/scene_record.ts';
import type { TrackTable } from '../src/record/scene_definition.ts';

export function trackJson(track: TrackTable | null) {
  return track === null ? null : {
    stations_meters: track.stationsMeters, centerline_in_inertial_meters: track.centerlineInInertialMeters,
    rotation_inertial_from_track_wxyz: track.rotationInertialFromTrackWxyz,
    curvature_radians_per_meter: track.curvatureRadiansPerMeter, superelevation_meters: track.superelevationMeters,
    left_rail_datum_in_inertial_meters: track.leftRailDatumInInertialMeters,
    right_rail_datum_in_inertial_meters: track.rightRailDatumInInertialMeters,
    rail_datums: { left: { lateral_meters: track.railDatums.left.lateralMeters, vertical_meters: track.railDatums.left.verticalMeters },
      right: { lateral_meters: track.railDatums.right.lateralMeters, vertical_meters: track.railDatums.right.verticalMeters } },
  };
}

export function serializeRecord(record: SceneRecord) {
  const { definition: d, samples } = record;
  const c = samples.columns;
  return {
    schema: 'orvd.scene_record',
    description: {
      world_frame: d.worldFrame, length_unit: d.lengthUnit, time_unit: d.timeUnit, angle_unit: d.angleUnit,
      orientation: d.orientation, nominal_sample_period_seconds: d.nominalSamplePeriodSeconds,
      bodies: d.bodies.map((body) => ({ name: body.name, moves_freely_in_world: body.movesFreelyInWorld })),
      wheel_placements: d.wheelPlacements.map((wheel) => ({
        interface_name: wheel.interfaceName, wheel_body_name: wheel.wheelBodyName, carrier_body_name: wheel.carrierBodyName,
        side: wheel.side, datum_in_wheel_body_frame_meters: wheel.datumInWheelBodyFrameMeters,
        spin_axis_in_wheel_body_frame: wheel.spinAxisInWheelBodyFrame, nominal_rolling_radius_meters: wheel.nominalRollingRadiusMeters,
        spin_joint_name: wheel.spinJointName, spin_angle_scalar: wheel.spinAngleScalar,
      })),
      scalars: d.scalars.map((scalar) => ({ name: scalar.name, unit: scalar.unit, reference_frame: scalar.referenceFrame,
        quantity: scalar.quantity, method: scalar.method, sample_semantics: scalar.sampleSemantics })),
      scalar_status_codes: d.scalarStatusCodes, phase_codes: d.phaseCodes,
    },
    assets: { world_frame: d.worldFrame, length_unit: d.lengthUnit, angle_unit: d.angleUnit,
      track: trackJson(d.track), visual_definition_file: d.visualDefinitionText === null ? null : 'visual_definition.json' },
    frame_table: { file: 'frames.f64le', encoding: 'little-endian binary64 rows', frame_count: samples.frameCount, row_value_count: c.rowValueCount,
      columns: { time_seconds: c.timeSecondsColumnOffset, time_nanoseconds: c.timeNanosecondsColumnOffset,
        sample_index: c.sampleIndexColumnOffset, phase: c.phaseColumnOffset,
        body_states: { offset: c.bodyStatesColumnOffset, body_count: d.bodies.length, values_per_body: c.valuesPerBody },
        wheel_spin_angles: { offset: c.wheelSpinAnglesColumnOffset, count: c.wheelSpinAngleCount },
        scalar_values: { offset: c.scalarValuesColumnOffset, count: c.scalarCount } } },
    scalar_status_table: { file: 'scalar_statuses.u8', frame_count: samples.frameCount, scalar_count: c.scalarCount },
  };
}

export function contractFixture(overrides: Partial<RecordFixtureSpec> = {}) {
  const wheel = placement('contact', 'wheel', 'wheel', 'left');
  wheel.spinAngleScalar = { name: 'angle', sign: -1 };
  wheel.spinJointName = 'axle';
  const record = makeRecord({ bodies: ['wheel'], placements: [wheel], scalars: ['angle'], scalarUnits: ['rad'],
    frames: [{ timeSeconds: 0, scalarValues: [0] }, { timeSeconds: 1, scalarValues: [12] }], ...overrides });
  return { record, json: serializeRecord(record), frames: record.samples.values.slice().buffer as ArrayBuffer,
    statuses: record.samples.statuses.slice().buffer as ArrayBuffer };
}

export function fixtureVisual(name = 'Fixture') {
  const json = JSON.parse(visualDefinitionWithBindings({ carbody: null, bogies: [], carriers: [
    { carrier_body_name: 'wheel', display_name: { en: 'wheel', zh: '轮' } },
  ] })) as Record<string, unknown>;
  json['vehicle_name'] = name;
  return JSON.stringify(json);
}
