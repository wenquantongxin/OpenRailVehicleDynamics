import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';
import { ScalarStatus } from '../src/record/scene_sample.ts';
import { parseSceneRecord } from '../src/record/scene_record.ts';
import { wheelSpinBindings } from '../src/scene/apply_frame.ts';
import { interpolateOrientation, selectWheelSpin } from '../src/scene/pose_interpolation.ts';
import { summarizeRecord } from '../src/ui/record_summary.ts';
import { buildVehicleReadoutModel } from '../src/ui/vehicle_readout_model.ts';
import { resolveVehicleDisplayBindings } from '../src/scene/vehicle_display_bindings.ts';
import { contractFixture } from './scene_contract_fixtures.ts';
import { makeRecord, placement } from './record_fixtures.ts';

for (const sign of [1, -1] as const) test(`wheel scalar binding applies sign ${sign} and preserves multiple turns`, () => {
  const wheel = placement('contact', 'wheel', 'wheel', 'left');
  wheel.spinAngleScalar = { name: 'angle', sign };
  const { json, frames, statuses } = contractFixture({ placements: [wheel], frames: [
    { timeSeconds: 0, scalarValues: [3 * Math.PI] }, { timeSeconds: 1, scalarValues: [23 * Math.PI] },
  ] });
  const record = parseSceneRecord(json, frames, statuses, null);
  assert.equal(record.samples.hasWheelSpinAngle(0), true);
  assert.equal(record.samples.viewAt(0).wheelSpinAngle(0), sign * 3 * Math.PI);
  assert.equal(record.samples.viewAt(1).wheelSpinAngle(0), sign * 23 * Math.PI);
});

test('non-valid and non-finite scalar samples yield no wheel angle', () => {
  const { record } = contractFixture({ frames: [
    { timeSeconds: 0, scalarValues: [4], scalarStatuses: [ScalarStatus.notReady] },
    { timeSeconds: 1, scalarValues: [4], scalarStatuses: [ScalarStatus.placeholder] },
    { timeSeconds: 2, scalarValues: [Number.NaN] },
    { timeSeconds: 3, scalarValues: [4] },
  ] });
  assert.deepEqual([0, 1, 2, 3].map((row) => record.samples.viewAt(row).wheelSpinAngle(0)), [null, null, null, -4]);
  assert.equal(record.samples.viewAt(0).wheelSpinAngle(1), null);
});

test('dedicated wheel columns take priority over valid or unavailable scalar bindings', () => {
  const { record } = contractFixture({ frames: [
    { timeSeconds: 0, wheelSpinAngles: [100], scalarValues: [4], scalarStatuses: [ScalarStatus.placeholder] },
    { timeSeconds: 1, wheelSpinAngles: [200], scalarValues: [5] },
  ] });
  assert.equal(record.samples.viewAt(0).wheelSpinAngle(0), 100);
  assert.equal(record.samples.viewAt(1).wheelSpinAngle(0), 200);
});

test('a scalar binding is resolved once when the sample source initializes', () => {
  const { record } = contractFixture();
  record.definition.scalars[0]!.name = 'renamed-after-initialization';
  record.definition.wheelPlacements[0]!.spinAngleScalar!.name = 'also-renamed';
  assert.equal(record.samples.viewAt(1).wheelSpinAngle(0), -12);
});

test('scalar wheel turns select the interpolation branch without adding spin twice to pose', () => {
  const angleA = 0.7;
  const angleB = angleA + 6 * Math.PI + 0.6;
  const quaternion = (angle: number): [number, number, number, number] => [Math.cos(angle / 2), 0, -Math.sin(angle / 2), 0];
  const { record } = contractFixture({ frames: [
    { timeSeconds: 0, scalarValues: [-angleA], poses: { wheel: { orientationWxyz: quaternion(angleA) } } },
    { timeSeconds: 1, scalarValues: [-angleB], poses: { wheel: { orientationWxyz: quaternion(angleB) } } },
  ] });
  const first = record.samples.viewAt(0);
  const second = record.samples.viewAt(1);
  const spin = wheelSpinBindings(record.definition).get(0);
  const actual = new THREE.Quaternion();
  for (const alpha of [0, 0.25, 0.5, 1]) {
    interpolateOrientation(first, second, alpha, 0, spin, actual);
    const [w, x, y, z] = quaternion(angleA + alpha * (angleB - angleA));
    assert.ok(actual.angleTo(new THREE.Quaternion(x, y, z, w)) < 1e-7);
  }
  interpolateOrientation(first, second, 0, 0, spin, actual);
  assert.deepEqual([actual.w, actual.x, actual.y, actual.z], first.bodyPose(0).orientationWxyz);
});

test('invalid dedicated values remain unavailable instead of changing angle sources within a column', () => {
  const { record } = contractFixture({ frames: [
    { timeSeconds: 0, wheelSpinAngles: [Number.NaN], scalarValues: [4] },
    { timeSeconds: 1, wheelSpinAngles: [Number.POSITIVE_INFINITY], scalarValues: [5] },
  ] });
  assert.equal(record.samples.viewAt(0).wheelSpinAngle(0), null);
  assert.equal(record.samples.viewAt(1).wheelSpinAngle(0), null);
});


test('missing-angle summaries inspect scalar validity rather than dedicated column counts', () => {
  const summary = (missing: boolean) => {
    const { record } = contractFixture({ frames: [
      { timeSeconds: 0, scalarValues: [4] },
      { timeSeconds: 1, scalarValues: [5], scalarStatuses: [missing ? ScalarStatus.placeholder : ScalarStatus.valid] },
    ] });
    const model = buildVehicleReadoutModel(record.definition, null, resolveVehicleDisplayBindings(record.definition, null), record.samples.viewAt(0));
    return summarizeRecord(record.samples, model);
  };
  assert.equal(summary(false).hasMissingWheelSpinAngles, false);
  assert.equal(summary(true).hasMissingWheelSpinAngles, true);
});


test('two wheels sharing a body retain a valid scalar candidate when the other side has no binding', () => {
  for (const reverse of [false, true]) {
    const left = placement('left', 'wheel', 'wheel', 'left');
    const right = placement('right', 'wheel', 'wheel', 'right');
    left.spinAngleScalar = { name: 'angle', sign: 1 };
    const { record } = contractFixture({ placements: reverse ? [right, left] : [left, right], frames: [
      { timeSeconds: 0, scalarValues: [0] }, { timeSeconds: 1, scalarValues: [4 * Math.PI] },
    ] });
    const candidates = wheelSpinBindings(record.definition).get(0);
    assert.equal(candidates?.length, 2);
    const actual = new THREE.Quaternion();
    interpolateOrientation(record.samples.viewAt(0), record.samples.viewAt(1), 0.25, 0, candidates, actual);
    assert.ok(actual.angleTo(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, -1, 0), Math.PI)) < 1e-7);
    const model = buildVehicleReadoutModel(record.definition, null, resolveVehicleDisplayBindings(record.definition, null), record.samples.viewAt(0));
    assert.equal(summarizeRecord(record.samples, model).hasMissingWheelSpinAngles, false);
  }
});

test('a shared wheel body chooses candidates using both endpoints and can switch sides between intervals', () => {
  const left = placement('left', 'wheel', 'wheel', 'left');
  const right = placement('right', 'wheel', 'wheel', 'right');
  left.spinAngleScalar = { name: 'left_angle', sign: 1 };
  right.spinAngleScalar = { name: 'right_angle', sign: 1 };
  const angles = [0.7, 0.7 + 4 * Math.PI, 0.7 + 8 * Math.PI, 0.7 + 12 * Math.PI];
  const statuses = [
    [ScalarStatus.valid, ScalarStatus.valid], [ScalarStatus.placeholder, ScalarStatus.valid],
    [ScalarStatus.valid, ScalarStatus.valid], [ScalarStatus.valid, ScalarStatus.notReady],
  ];
  const record = makeRecord({ bodies: ['wheel'], placements: [left, right], scalars: ['left_angle', 'right_angle'], scalarUnits: ['rad', 'rad'],
    frames: angles.map((angle, index) => ({ timeSeconds: index, scalarValues: [angle, angle], scalarStatuses: statuses[index],
      poses: { wheel: { orientationWxyz: [Math.cos(angle / 2), 0, -Math.sin(angle / 2), 0] } } })),
  });
  const candidates = wheelSpinBindings(record.definition).get(0);
  const selected: number[] = [];
  for (let index = 0; index < 3; ++index) {
    const first = record.samples.viewAt(index);
    const second = record.samples.viewAt(index + 1);
    selected.push(selectWheelSpin(first, second, candidates)!.binding.wheelPlacementIndex);
    const actual = new THREE.Quaternion();
    interpolateOrientation(first, second, 0.25, 0, candidates, actual);
    const expected = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, -1, 0), angles[index]! + Math.PI);
    assert.ok(actual.angleTo(expected) < 1e-7);
  }
  assert.deepEqual(selected, [1, 1, 0]);
});

test('shared wheel candidates preserve opposite spin axes and scalar signs', () => {
  for (const reverse of [false, true]) {
    const left = placement('left', 'wheel', 'wheel', 'left', 1);
    const right = placement('right', 'wheel', 'wheel', 'right', -1);
    left.spinAngleScalar = { name: 'angle', sign: 1 };
    right.spinAngleScalar = { name: 'angle', sign: -1 };
    const firstAngle = 0.7;
    const delta = 4 * Math.PI + 0.6;
    const { record } = contractFixture({ placements: reverse ? [right, left] : [left, right], frames: [0, 1].map((index) => {
      const angle = firstAngle + index * delta;
      return { timeSeconds: index, scalarValues: [angle], poses: { wheel: { orientationWxyz: [Math.cos(angle / 2), 0, Math.sin(angle / 2), 0] } } };
    }) });
    const candidates = wheelSpinBindings(record.definition).get(0);
    const first = record.samples.viewAt(0);
    const second = record.samples.viewAt(1);
    for (const alpha of [0, 0.25, 1]) {
      const actual = new THREE.Quaternion();
      interpolateOrientation(first, second, alpha, 0, candidates, actual);
      const expected = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), firstAngle + alpha * delta);
      assert.ok(actual.angleTo(expected) < 1e-7);
      if (alpha === 0 || alpha === 1) assert.deepEqual([actual.w, actual.x, actual.y, actual.z], (alpha === 0 ? first : second).bodyPose(0).orientationWxyz);
    }
  }
});

test('different valid sides at opposite endpoints cannot be mixed into one spin interval', () => {
  const left = placement('left', 'wheel', 'wheel', 'left');
  const right = placement('right', 'wheel', 'wheel', 'right');
  left.spinAngleScalar = { name: 'left_angle', sign: 1 };
  right.spinAngleScalar = { name: 'right_angle', sign: 1 };
  const record = makeRecord({ bodies: ['wheel'], placements: [left, right], scalars: ['left_angle', 'right_angle'], scalarUnits: ['rad', 'rad'], frames: [
    { timeSeconds: 0, scalarValues: [0, 100], scalarStatuses: [ScalarStatus.valid, ScalarStatus.notReady] },
    { timeSeconds: 1, scalarValues: [4 * Math.PI, 100 + 4 * Math.PI], scalarStatuses: [ScalarStatus.notReady, ScalarStatus.valid] },
  ] });
  const candidates = wheelSpinBindings(record.definition).get(0);
  const first = record.samples.viewAt(0);
  const second = record.samples.viewAt(1);
  assert.equal(selectWheelSpin(first, second, candidates), null);
  const actual = new THREE.Quaternion();
  interpolateOrientation(first, second, 0.25, 0, candidates, actual);
  assert.deepEqual(actual.toArray(), [0, 0, 0, 1]);
  const model = buildVehicleReadoutModel(record.definition, null, resolveVehicleDisplayBindings(record.definition, null), first);
  assert.equal(summarizeRecord(record.samples, model).hasMissingWheelSpinAngles, true);
});
