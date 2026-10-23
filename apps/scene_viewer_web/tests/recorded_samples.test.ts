import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { ScalarStatus, type SceneSampleView } from '../src/record/scene_sample.ts';
import { interpolateOrientation, interpolatePosition } from '../src/scene/pose_interpolation.ts';
import { buildVehicleReadoutModel } from '../src/ui/vehicle_readout_model.ts';
import { resolveVehicleDisplayBindings } from '../src/scene/vehicle_display_bindings.ts';
import { makeRecord, placement } from './record_fixtures.ts';

test('borrowed views keep separate row bindings and all recorded identities', () => {
  const record = makeRecord({ bodies: ['wheel'], placements: [placement('if', 'wheel', 'wheel', 'left')], scalars: ['force'],
    frames: [
      { timeSeconds: 0.013, sampleIndex: 83, phase: 2, poses: { wheel: { position: [1, 2, 3], linearVelocity: [4, 5, 6], angularVelocity: [7, 8, 9] } }, wheelSpinAngles: [17], scalarValues: [42], scalarStatuses: [ScalarStatus.placeholder] },
      { timeSeconds: 0.014, sampleIndex: -1, phase: 1, poses: { wheel: { position: [10, 20, 30] } }, wheelSpinAngles: [18], scalarValues: [43] },
    ] });
  const first = record.samples.viewAt(0);
  const second = record.samples.viewAt(1);
  assert.notEqual(first, second);
  assert.deepEqual([first.timeSeconds, first.timeNanoseconds, first.sampleIndex, first.phase], [0.013, 13000000, 83, 2]);
  assert.equal(second.sampleIndex, -1);
  assert.equal(second.phase, 1);
  assert.deepEqual(first.bodyPose(0).position, [1, 2, 3]);
  assert.deepEqual(first.bodyLinearVelocity(0), [4, 5, 6]);
  assert.deepEqual(first.bodyAngularVelocity(0), [7, 8, 9]);
  assert.equal(first.wheelSpinAngle(0), 17);
  assert.deepEqual(first.scalar(0), { value: 42, status: ScalarStatus.placeholder });
  assert.deepEqual(second.bodyPose(0).position, [10, 20, 30]);
  assert.throws(() => record.samples.viewAt(-1), RangeError);
  assert.throws(() => record.samples.viewAt(2), RangeError);
});

test('rendering and current readouts consume a sample without recorded history', () => {
  const definition = makeRecord({ bodies: ['wheel'], placements: [placement('if', 'wheel', 'wheel', 'left')],
    scalars: ['if.contact_patch_count'], frames: [] }).definition;
  const sample: SceneSampleView = {
    timeSeconds: 2, timeNanoseconds: 2000000000, sampleIndex: -1, phase: 2,
    bodyPose: () => ({ position: [0.1, 0.2, 0.3], orientationWxyz: [0.5, 0.5, 0.5, 0.5] }),
    bodyLinearVelocity: () => [1, 2, 3], bodyAngularVelocity: () => [0, 0, 0],
    wheelSpinAngle: () => null, scalar: () => ({ value: 2, status: ScalarStatus.valid }),
  };
  const model = buildVehicleReadoutModel(definition, null, resolveVehicleDisplayBindings(definition, null), sample);
  assert.equal(model.contactAt(sample).state, 'two');
  const position = new THREE.Vector3();
  const orientation = new THREE.Quaternion();
  interpolatePosition(sample, sample, 0.5, 0, position);
  interpolateOrientation(sample, sample, 0.5, 0, undefined, orientation);
  assert.deepEqual(position.toArray(), sample.bodyPose(0).position);
  assert.deepEqual(orientation.toArray(), [0.5, 0.5, 0.5, 0.5]);
});
