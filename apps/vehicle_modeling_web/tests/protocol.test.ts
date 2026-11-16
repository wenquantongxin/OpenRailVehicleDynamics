import assert from 'node:assert/strict';
import { test } from 'node:test';

import { readAnswer, readPreview, readProperties } from '../src/service/protocol.ts';
import { fakeVehicle } from './fake_modeling_service.ts';

// Answers are read leniently, but what the interface uses must be there.

test('an accepted answer carries the session envelope and a refused one the service’s reason', () => {
  const accepted = readAnswer({ id: 3, ok: true, revision: 4, can_undo: true, can_redo: false, result: {}, extra: 'ignored' });
  assert.deepEqual(accepted, { ok: true, revision: 4, canUndo: true, canRedo: false, result: {} });
  const refused = readAnswer({ id: 3, ok: false, error: { element: { kind: 'kind_a', name: 'a' }, property: 'p', message: 'no' } });
  assert.deepEqual(refused, { ok: false, refusal: { element: { kind: 'kind_a', name: 'a' }, property: 'p', message: 'no' } });
  assert.throws(() => readAnswer({ ok: true, revision: 4, can_undo: true, result: {} }), /can_redo is missing/);
});

test('every value kind is read with its shape, and an absent value is null', () => {
  const properties = readProperties({
    properties: [
      { name: 'n', value_kind: 'number', editable: true, value: 1.5e-12 },
      { name: 'absent', value_kind: 'number', editable: true, value: null },
      { name: 'v', value_kind: 'vector3', editable: true, value: [1, 2, 3] },
      { name: 't', value_kind: 'text', editable: false, value: 'frame_a' },
      { name: 'b', value_kind: 'boolean', editable: false, value: false },
      { name: 'r', value_kind: 'rotation', editable: false, value: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] },
      { name: 's', value_kind: 'wheel_side', editable: false, value: 'right' },
      { name: 'c', value_kind: 'saturated_piecewise_linear_damper_curve', editable: false, value: [{ speed: 0.1, push: 30 }] },
    ],
  });
  assert.deepEqual(
    properties.map((property) => [property.valueKind, property.value]),
    [
      ['number', 1.5e-12],
      ['number', null],
      ['vector3', [1, 2, 3]],
      ['text', 'frame_a'],
      ['boolean', false],
      ['rotation', [[1, 0, 0], [0, 1, 0], [0, 0, 1]]],
      ['wheel_side', 'right'],
      ['saturated_piecewise_linear_damper_curve', [{ speed: 0.1, push: 30 }]],
    ],
  );
});

test('a value kind the interface cannot show, or a value of the wrong shape, fails loudly', () => {
  assert.throws(() => readProperties({ properties: [{ name: 'q', value_kind: 'quaternion', editable: false, value: [1, 0, 0, 0] }] }), /quaternion/);
  assert.throws(() => readProperties({ properties: [{ name: 'v', value_kind: 'vector3', editable: true, value: [1, 2] }] }), /three entries/);
  assert.throws(() => readProperties({ properties: [{ name: 'n', value_kind: 'number', value: 1 }] }), /editable is missing/);
});

test('every drawing of a preview names its element', () => {
  const vehicle = fakeVehicle();
  assert.equal(readPreview({ preview: null }), null);
  const preview = readPreview({ preview: { project_revision: 7, ...vehicle.drawings } })!;
  assert.equal(preview.projectRevision, 7);
  assert.deepEqual(
    preview.rigidBodies.map((body) => body.element),
    [
      { kind: 'kind_body', name: 'body_a' },
      { kind: 'kind_body', name: 'body_b' },
    ],
  );
  assert.deepEqual(preview.forceElements[0]?.element, { kind: 'kind_spring', name: 'spring_a' });
  assert.deepEqual(
    preview.wheels.map((wheel) => [wheel.element.name, wheel.wheelBody.name, wheel.outwardSpinAxisInDrawingFrame[1]]),
    [
      ['body_b.left', 'body_b', -1],
      ['body_b.right', 'body_b', 1],
    ],
  );
  const { nominal_rolling_radius_meters: _, ...withoutRadius } = (vehicle.drawings['wheels'] as Record<string, unknown>[])[0]!;
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, wheels: [withoutRadius] } }), /nominal_rolling_radius_meters is missing/);
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, wheels: undefined } }), /wheels/);
  const unnamed = { body_name: 'body_a', rotation_drawing_from_body: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], origin_in_drawing_frame_meters: [0, 0, 0] };
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, rigid_bodies: [unnamed] } }), /element is missing/);
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, weld_joints: undefined } }), /weld_joints/);
});
