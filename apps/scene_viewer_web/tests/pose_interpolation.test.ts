import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { makeRecord, placement } from './record_fixtures.ts';
import { interpolateOrientation, interpolatePosition } from '../src/scene/pose_interpolation.ts';

test('sample endpoints preserve stored coordinates and quaternion signs exactly', () => {
  const record = makeRecord({ bodies: ['wheel'], placements: [placement('contact', 'wheel', 'wheel', 'left')],
    frames: [
      { timeSeconds: 0, poses: { wheel: { position: [1e16, 0, 0], orientationWxyz: [0.5, 0.5, 0.5, 0.5] } }, wheelSpinAngles: [0] },
      { timeSeconds: 1, poses: { wheel: { position: [1, 2, 3], orientationWxyz: [-0.5, -0.5, -0.5, -0.5] } }, wheelSpinAngles: [4 * Math.PI] },
    ] });
  const first = record.samples.viewAt(0);
  const second = record.samples.viewAt(1);
  const binding = { wheelPlacementIndex: 0, spinAxisInBody: [0, 1, 0] as [number, number, number] };
  const position = new THREE.Vector3();
  const orientation = new THREE.Quaternion();
  for (const alpha of [0, 1]) {
    const expected = alpha === 0 ? first : second;
    interpolatePosition(first, second, alpha, 0, position);
    interpolateOrientation(first, second, alpha, 0, [binding], orientation);
    assert.deepEqual(position.toArray(), expected.bodyPose(0).position);
    const [w, x, y, z] = expected.bodyPose(0).orientationWxyz;
    assert.deepEqual(orientation.toArray(), [x, y, z, w]);
  }
  // Separate views of the same row must not perform another interpolation.
  interpolateOrientation(second, record.samples.viewAt(1), 0.5, 0, [binding], orientation);
  assert.deepEqual(orientation.toArray(), [-0.5, -0.5, -0.5, -0.5]);
});

for (const flipped of [false, true]) {
  test(`wheel spin follows a turning carrier (${flipped ? 'half-turn' : 'identity'} body basis)`, () => {
    const basis = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), flipped ? Math.PI : 0);
    const axis = new THREE.Vector3(0, flipped ? -1 : 1, 0);
    const binding = { wheelPlacementIndex: 0, spinAxisInBody: axis.toArray() as [number, number, number] };
    const carrier = (alpha: number): THREE.Quaternion =>
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), 0.2 + 0.1 * alpha).multiply(basis);

    for (const delta of [0, Math.PI, 2 * Math.PI, -2 * Math.PI, 13]) {
      const angleA = 0.7;
      const pose = (alpha: number): THREE.Quaternion =>
        carrier(alpha).multiply(new THREE.Quaternion().setFromAxisAngle(axis, angleA + delta * alpha));
      const record = makeRecord({ bodies: ['wheel'], placements: [placement('contact', 'wheel', 'wheel', 'left')],
        frames: [0, 1].map((index) => {
          const q = pose(index);
          return { timeSeconds: index, poses: { wheel: { orientationWxyz: [q.w, q.x, q.y, q.z] } },
            wheelSpinAngles: [angleA + delta * index] };
        }) });

      for (const alpha of [0, 0.25, 0.5, 0.75, 1]) {
        const actual = new THREE.Quaternion();
        interpolateOrientation(record.samples.viewAt(0), record.samples.viewAt(1), alpha, 0, [binding], actual);
        const relative = pose(alpha).invert().multiply(actual);
        const error = 2 * Math.atan2(Math.hypot(relative.x, relative.y, relative.z), Math.abs(relative.w));
        assert.ok(error < 1e-12, `orientation error ${error} at delta=${delta}, alpha=${alpha}`);
        const wheelAxis = axis.clone().applyQuaternion(actual);
        const carrierAxis = axis.clone().applyQuaternion(carrier(alpha));
        assert.ok(wheelAxis.distanceTo(carrierAxis) < 1e-12, 'wheel axle must follow the carrier');
      }
    }
  });
}
