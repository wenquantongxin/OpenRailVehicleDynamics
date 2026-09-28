import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { ScalarStatus, type SceneRecord } from '../src/record/scene_record.ts';
import { TrackModel } from '../src/scene/track_model.ts';
import { buildReadoutModel } from '../src/ui/readout_model.ts';
import { range, straightTable } from './track_fixtures.ts';

const bodies = ['carbody', 'frame_front', 'axlebridge_ff', 'wheel_ff_l', 'wheel_ff_r'];
const scalars = [
  'axlebridge_ff.track_station_meters',
  'axlebridge_ff.lateral_meters',
  'axlebridge_ff.yaw_radians',
  'wheel_ff_l.contact_patch_count',
  'wheel_ff_r.contact_patch_count',
];
const positions: Record<string, [number, number, number]> = {
  carbody: [0, 0, -1],
  frame_front: [8.75, 0, -0.43],
  axlebridge_ff: [10, 0, -0.43],
  wheel_ff_l: [10, 0, -0.43],
  wheel_ff_r: [10, 0, -0.43],
};

/** A two-frame record; `set` fills per-frame body orientations, scalar values and statuses. */
function makeRecord(
  set: (frame: number) => { axleQuaternion?: THREE.Quaternion; patches: [number, number]; statuses: [number, number] },
): SceneRecord {
  const frameCount = 2;
  const bodyOffset = 4;
  const scalarOffset = bodyOffset + 13 * bodies.length;
  const rowValueCount = scalarOffset + scalars.length;
  const values = new Float64Array(frameCount * rowValueCount);
  const statuses = new Uint8Array(frameCount * scalars.length).fill(ScalarStatus.valid);
  for (let frame = 0; frame < frameCount; ++frame) {
    const row = frame * rowValueCount;
    values[row] = 0.01 * frame;
    const spec = set(frame);
    bodies.forEach((name, index) => {
      const base = row + bodyOffset + 13 * index;
      values.set(positions[name] ?? [0, 0, 0], base);
      const q = name === 'axlebridge_ff' && spec.axleQuaternion !== undefined ? spec.axleQuaternion : new THREE.Quaternion();
      values.set([q.w, q.x, q.y, q.z], base + 3);
    });
    values.set([10, 0, 0, spec.patches[0], spec.patches[1]], row + scalarOffset);
    statuses[frame * scalars.length + 3] = spec.statuses[0];
    statuses[frame * scalars.length + 4] = spec.statuses[1];
  }
  return {
    worldFrame: 'test',
    bodies: bodies.map((name) => ({ name, movesFreelyInWorld: true })),
    wheelPlacements: [
      { interfaceName: 'wheel_ff_l', wheelBodyName: 'wheel_ff_l', side: 'left', datumInWheelBodyFrameMeters: [0, 0.7465, 0], spinAxisInWheelBodyFrame: [0, -1, 0], nominalRollingRadiusMeters: 0.43 },
      { interfaceName: 'wheel_ff_r', wheelBodyName: 'wheel_ff_r', side: 'right', datumInWheelBodyFrameMeters: [0, -0.7465, 0], spinAxisInWheelBodyFrame: [0, -1, 0], nominalRollingRadiusMeters: 0.43 },
    ],
    scalars: scalars.map((name) => ({ name, unit: '1', referenceFrame: '', quantity: '', method: '', sampleSemantics: '' })),
    visualDefinitionText: null,
    track: null,
    frameCount,
    columns: {
      timeSeconds: 0,
      timeNanoseconds: 1,
      sampleIndex: 2,
      phase: 3,
      bodyOffset,
      valuesPerBody: 13,
      wheelSpinOffset: scalarOffset,
      wheelSpinCount: 0,
      scalarOffset,
      scalarCount: scalars.length,
      rowValueCount,
    },
    values,
    statuses,
    timesSeconds: Float64Array.from([0, 0.01]),
  };
}

test('a wheel whose contact is not ready stays unknown', () => {
  const record = makeRecord((frame) =>
    frame === 0
      ? { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.notReady] }
      : { patches: [1, 1], statuses: [ScalarStatus.notReady, ScalarStatus.notReady] },
  );
  const model = buildReadoutModel(record, null);
  assert.deepEqual(model.contactAt(0), { state: 'incomplete', codes: [], known: 1, total: 2 });
  assert.deepEqual(model.contactAt(1), { state: 'unknown', codes: [], known: 0, total: 2 });
  const patches = new Float64Array(2);
  model.contactPatchesAt(0, patches);
  assert.equal(patches[0], 1);
  assert.ok(Number.isNaN(patches[1]), 'a not-ready count is NaN, not 1');
  assert.equal(model.contactSpans.length, 0, 'not ready is no contact event');
});

test('recorded loss and two-point contact are reported with the coverage', () => {
  const record = makeRecord((frame) =>
    frame === 0
      ? { patches: [0, 1], statuses: [ScalarStatus.valid, ScalarStatus.notReady] }
      : { patches: [2, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] },
  );
  const model = buildReadoutModel(record, null);
  assert.deepEqual(model.contactAt(0), { state: 'loss', codes: ['1L'], known: 1, total: 2 });
  assert.deepEqual(model.contactAt(1), { state: 'two', codes: ['1L'], known: 2, total: 2 });
  assert.deepEqual(
    model.contactSpans.map((span) => [span.startFrame, span.patches, span.code]),
    [
      [0, 0, '1L'],
      [1, 2, '1L'],
    ],
  );
});

test('plan heading comes from the pose, whatever the body basis', () => {
  const track = new TrackModel(straightTable(range(-20, 40, 0.5), () => 0));
  const theta = 0.004;
  // Nose to the right: a turn about the inertial +z axis, which points down.
  const turn = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), theta);
  const halfTurn = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
  for (const [basis, orientation] of [
    ['identity basis (y right, z down)', turn.clone()],
    ['half-turn basis (y left, z up)', turn.clone().multiply(halfTurn)],
  ] as const) {
    const record = makeRecord(() => ({ axleQuaternion: orientation, patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] }));
    const model = buildReadoutModel(record, track);
    const bridge = model.axles[0]?.bridge;
    assert.ok(bridge !== null && bridge !== undefined);
    const heading = model.headingAt(0, bridge);
    assert.ok(heading !== null && Math.abs(heading - theta) < 1e-12, `${basis}: heading ${heading}`);
  }
});
