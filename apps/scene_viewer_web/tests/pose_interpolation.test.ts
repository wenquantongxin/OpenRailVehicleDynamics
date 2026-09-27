import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import type { SceneRecord } from '../src/record/scene_record.ts';
import { interpolateOrientation } from '../src/scene/pose_interpolation.ts';

for (const flipped of [false, true]) {
  test(`wheel spin follows a turning carrier (${flipped ? 'half-turn' : 'identity'} body basis)`, () => {
    const basis = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), flipped ? Math.PI : 0);
    const axis = new THREE.Vector3(0, flipped ? -1 : 1, 0);
    const binding = { wheel: 0, spinAxisInBody: axis.toArray() as [number, number, number] };
    const carrier = (alpha: number): THREE.Quaternion =>
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), 0.2 + 0.1 * alpha).multiply(basis);

    for (const delta of [0, Math.PI, 2 * Math.PI, -2 * Math.PI, 13]) {
      const angleA = 0.7;
      const pose = (alpha: number): THREE.Quaternion =>
        carrier(alpha).multiply(new THREE.Quaternion().setFromAxisAngle(axis, angleA + delta * alpha));
      // Two one-body rows: position, wxyz, linear/angular velocity, spin angle.
      const values = new Float64Array(28);
      for (const frame of [0, 1]) {
        const q = pose(frame);
        values.set([q.w, q.x, q.y, q.z], 14 * frame + 3);
        values[14 * frame + 13] = angleA + delta * frame;
      }
      const record = {
        values,
        columns: { rowValueCount: 14, bodyOffset: 0, valuesPerBody: 13, wheelSpinOffset: 13, wheelSpinCount: 1 },
      } as unknown as SceneRecord;

      for (const alpha of [0, 0.25, 0.5, 0.75, 1]) {
        const actual = new THREE.Quaternion();
        interpolateOrientation(record, 0, 1, alpha, 0, binding, actual);
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
