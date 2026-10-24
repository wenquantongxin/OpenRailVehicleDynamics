import assert from 'node:assert/strict';
import { test } from 'node:test';
import { parseSceneDefinition, parseSceneDescription } from '../src/record/scene_definition.ts';
import { parseSceneRecord } from '../src/record/scene_record.ts';
import { parseSceneAssets, composeSceneDefinition } from '../src/record/scene_assets.ts';
import { parseSceneBundle } from '../src/record/load_bundle.ts';
import { contractFixture, fixtureVisual, trackJson } from './scene_contract_fixtures.ts';
import { straightTable } from './track_fixtures.ts';

test('record and stream envelopes parse exactly the same public description without resource I/O', () => {
  const { json, frames, statuses } = contractFixture();
  json.description.nominal_sample_period_seconds = 0.001;
  const standalone = parseSceneDefinition(json.description);
  const record = parseSceneRecord(json, frames, statuses, null);
  const stream = parseSceneDescription({ schema: 'orvd.scene_description', description: json.description, resource_bundle: '../bundle/' });
  const { track, visualDefinitionText, ...recordDescription } = record.definition;
  assert.equal(track, null);
  assert.equal(visualDefinitionText, null);
  assert.deepEqual(recordDescription, standalone);
  assert.deepEqual(stream.description, standalone);
  assert.equal(stream.resourceBundle, '../bundle/');
  assert.equal(standalone.wheelPlacements[0]?.spinJointName, 'axle');
  assert.equal(record.samples.viewAt(1).wheelSpinAngle(0), -12);
  assert.throws(() => parseSceneDefinition(json), /description.world_frame/);
});

test('legacy record envelopes and missing shared fields are rejected', () => {
  const { json, frames, statuses } = contractFixture();
  assert.throws(() => parseSceneRecord({ record: 'orvd.scene_record', ...json.description }, frames, statuses, null), /schema/);
  assert.throws(() => parseSceneRecord({ ...json, record: 'orvd.scene_record' }, frames, statuses, null), /legacy/);
  assert.throws(() => parseSceneDefinition({ ...json.description, nominal_sample_period_seconds: undefined }), /nominal_sample/);
  assert.throws(() => parseSceneDefinition({ ...json.description, nominal_sample_period_seconds: 0 }), /positive/);
  assert.throws(() => parseSceneDescription({ schema: 'orvd.scene_description', description: json.description }), /resource_bundle/);
});

test('description validates conventions, topology and spin scalar contracts', () => {
  const { json } = contractFixture();
  const d = json.description;
  assert.throws(() => parseSceneDefinition({ ...d, length_unit: 'mm' }), /length_unit/);
  assert.throws(() => parseSceneDefinition({ ...d, orientation: 'xyz' }), /orientation/);
  assert.throws(() => parseSceneDefinition({ ...d, bodies: [d.bodies[0], d.bodies[0]] }), /duplicate/);
  assert.throws(() => parseSceneDefinition({ ...d, scalar_status_codes: { 0: 'valid' } }), /codes/);
  for (const wheel of [
    { ...d.wheel_placements[0], carrier_body_name: 'missing' },
    { ...d.wheel_placements[0], spin_angle_scalar: { name: 'missing', sign: 1 } },
    { ...d.wheel_placements[0], spin_angle_scalar: { name: 'angle', sign: 0 } },
  ]) assert.throws(() => parseSceneDefinition({ ...d, wheel_placements: [wheel] }), /unknown body|scalar/);
  assert.throws(() => parseSceneDefinition({ ...d, scalars: [{ ...d.scalars[0], unit: 'deg' }] }), /unit rad/);
});

test('pure bundle composition replaces provided resources and retains absent ones without changing topology', () => {
  const { json } = contractFixture();
  const description = parseSceneDefinition(json.description);
  const oldTrack = trackJson(straightTable([0, 1], () => 0));
  const newTrack = trackJson(straightTable([10, 20], () => 0));
  const recordAssets = parseSceneAssets({ ...json.assets, track: oldTrack, visual_definition_file: 'visual_definition.json' }, fixtureVisual('record'));
  const bundle = (track: unknown, visual = false) => parseSceneBundle({ schema: 'orvd.scene_bundle',
    assets: { ...json.assets, track, visual_definition_file: visual ? 'visual_definition.json' : null } }, visual ? fixtureVisual('bundle') : null);
  const empty = composeSceneDefinition(description, recordAssets, bundle(null));
  assert.equal(empty.track, recordAssets.track);
  assert.equal(empty.visualDefinitionText, recordAssets.visualDefinitionText);
  const trackOnly = composeSceneDefinition(description, recordAssets, bundle(newTrack));
  assert.deepEqual(trackOnly.track?.stationsMeters, [10, 20]);
  assert.equal(trackOnly.visualDefinitionText, recordAssets.visualDefinitionText);
  const visualOnly = composeSceneDefinition(description, recordAssets, bundle(null, true));
  assert.equal(visualOnly.track, recordAssets.track);
  assert.equal(JSON.parse(visualOnly.visualDefinitionText!).vehicle_name, 'bundle');
  const both = composeSceneDefinition(description, recordAssets, bundle(newTrack, true));
  assert.deepEqual(both.track?.stationsMeters, [10, 20]);
  assert.equal(JSON.parse(both.visualDefinitionText!).vehicle_name, 'bundle');
  assert.equal(both.bodies, description.bodies);
  assert.equal(both.wheelPlacements, description.wheelPlacements);
});

test('bundle coordinates and visual body, carrier and wheel references must match public topology', () => {
  const { json } = contractFixture();
  const d = parseSceneDefinition(json.description);
  const assets = parseSceneAssets(json.assets);
  assert.throws(() => composeSceneDefinition(d, assets, { ...assets, worldFrame: 'other' }), /coordinate/);
  assert.throws(() => parseSceneAssets({ ...json.assets, length_unit: 'mm' }), /length_unit/);
  assert.throws(() => parseSceneAssets({ ...json.assets, visual_definition_file: 'arbitrary.json' }), /visual_definition/);
  const badVisuals = [
    { ...JSON.parse(fixtureVisual()), parts: [{ name: 'bad', body_name: 'other' }] },
    { ...JSON.parse(fixtureVisual()), display_bindings: { carbody: { body_name: 'other', display_name: { en: 'x', zh: 'x' } }, bogies: [], carriers: [] } },
    { ...JSON.parse(fixtureVisual()), wheel_visual: { ...JSON.parse(fixtureVisual()).wheel_visual, labels: [{ interface_name: 'other', en: 'x', zh: 'x' }] } },
  ];
  for (const visual of badVisuals) assert.throws(() => composeSceneDefinition(d, assets,
    { ...assets, visualDefinitionFile: 'visual_definition.json', visualDefinitionText: JSON.stringify(visual) }), /unknown body|no display binding|unknown wheel interface/);
  assert.throws(() => composeSceneDefinition(d, assets, { ...assets, visualDefinitionFile: 'visual_definition.json' }), /lack/);
  const misleading = { ...JSON.parse(fixtureVisual()), mechanical_definition_identifier: 'different metadata' };
  assert.doesNotThrow(() => composeSceneDefinition(d, assets, { ...assets, visualDefinitionFile: 'visual_definition.json', visualDefinitionText: JSON.stringify(misleading) }));
});

test('track assets require full columns and rail datum metadata', () => {
  const { json } = contractFixture();
  const track = trackJson(straightTable([0, 1], () => 0))!;
  assert.throws(() => parseSceneAssets({ ...json.assets, track: { ...track, rail_datums: undefined } }), /rail_datums/);
  assert.throws(() => parseSceneAssets({ ...json.assets, track: { ...track, curvature_radians_per_meter: [0] } }), /length/);
  const parsed = parseSceneAssets({ ...json.assets, track });
  assert.equal(parsed.track?.railDatums.left.lateralMeters, -0.75315);
});


test('wheel axes and track quaternions share the C++ squared-norm tolerance', () => {
  const { json } = contractFixture();
  const description = json.description;
  // These finite perturbations straddle |squaredNorm - 1| <= 1e-8.
  for (const [component, accepted] of [[1 + 4e-9, true], [1 + 6e-9, false]] as const) {
    const parseAxis = () => parseSceneDefinition({ ...description, wheel_placements: [
      { ...description.wheel_placements[0], spin_axis_in_wheel_body_frame: [component, 0, 0] },
    ] });
    const track = trackJson(straightTable([0, 1], () => 0))!;
    track.rotation_inertial_from_track_wxyz = [[component, 0, 0, 0], [1, 0, 0, 0]];
    const parseTrack = () => parseSceneAssets({ ...json.assets, track });
    if (accepted) {
      assert.doesNotThrow(parseAxis);
      assert.doesNotThrow(parseTrack);
    } else {
      assert.throws(parseAxis, /unit vector/);
      assert.throws(parseTrack, /unit quaternion/);
    }
  }
});
