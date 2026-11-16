import assert from 'node:assert/strict';
import { test } from 'node:test';

import { readAnswer, readEditedElement, readElementKinds, readPreview, readProperties, readRemovedElements } from '../src/service/vehicle_modeling_service_protocol.ts';
import { fakeVehicle } from './fake_vehicle_modeling_service.ts';

// Answers are read leniently, but what the interface uses must be there.

test('an accepted answer carries the session envelope and a refused one the service’s reason', () => {
  const accepted = readAnswer({ id: 3, ok: true, revision: 4, can_undo: true, can_redo: false, result: {}, extra: 'ignored' });
  assert.deepEqual(accepted, { ok: true, revision: 4, canUndo: true, canRedo: false, result: {} });
  const refused = readAnswer({ id: 3, ok: false, error: { element: { kind: 'kind_a', name: 'a' }, property: 'p', message: 'no' } });
  assert.deepEqual(refused, { ok: false, refusal: { element: { kind: 'kind_a', name: 'a' }, property: 'p', message: 'no' } });
  assert.throws(() => readAnswer({ ok: true, revision: 4, can_undo: true, result: {} }), /can_redo is missing/);
});

test('every value kind is read with its shape and what its description says, and an absent value is null', () => {
  const properties = readProperties({
    properties: [
      { name: 'n', value_kind: 'number', editable: true, value: 1.5e-12 },
      { name: 'absent', value_kind: 'number', editable: false, value: null },
      { name: 'v', value_kind: 'vector3', editable: true, value: [1, 2, 3] },
      { name: 'b', value_kind: 'boolean', editable: true, value: false },
      { name: 'r', value_kind: 'rotation', editable: false, value: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] },
      { name: 'own', value_kind: 'element_name', editable: true, value: 'frame_a' },
      { name: 'to', value_kind: 'element_reference', editable: true, referenced_kinds: ['kind_a', 'kind_b'], optional: false, value: 'body_a' },
      { name: 'maybe', value_kind: 'element_reference', editable: true, referenced_kinds: ['kind_c'], optional: true, value: null },
      { name: 't', value_kind: 'token', editable: true, choices: ['left', 'right'], value: 'right' },
      { name: 'c', value_kind: 'saturated_piecewise_linear_damper_curve', editable: true, row_fields: ['speed', 'push'], value: [{ push: 30, speed: 0.1, extra: 1 }] },
      { name: 'empty', value_kind: 'saturated_piecewise_linear_damper_curve', editable: true, row_fields: ['speed', 'push'], value: [] },
    ],
  });
  assert.deepEqual(
    properties.map(({ name: _, editable: __, ...rest }) => rest),
    [
      { valueKind: 'number', value: 1.5e-12 },
      { valueKind: 'number', value: null },
      { valueKind: 'vector3', value: [1, 2, 3] },
      { valueKind: 'boolean', value: false },
      { valueKind: 'rotation', value: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] },
      { valueKind: 'element_name', value: 'frame_a' },
      { valueKind: 'element_reference', referencedKinds: ['kind_a', 'kind_b'], optional: false, value: 'body_a' },
      { valueKind: 'element_reference', referencedKinds: ['kind_c'], optional: true, value: null },
      { valueKind: 'token', choices: ['left', 'right'], value: 'right' },
      { valueKind: 'saturated_piecewise_linear_damper_curve', rowFields: ['speed', 'push'], value: [{ speed: 0.1, push: 30 }] },
      { valueKind: 'saturated_piecewise_linear_damper_curve', rowFields: ['speed', 'push'], value: [] },
    ],
  );
});

test('a value kind the interface cannot show, a missing description field or a value of the wrong shape fails loudly', () => {
  for (const gone of ['text', 'wheel_side', 'quaternion']) {
    assert.throws(() => readProperties({ properties: [{ name: 'q', value_kind: gone, editable: false, value: 'a' }] }), new RegExp(gone));
  }
  assert.throws(() => readProperties({ properties: [{ name: 'v', value_kind: 'vector3', editable: true, value: [1, 2] }] }), /three entries/);
  assert.throws(() => readProperties({ properties: [{ name: 'n', value_kind: 'number', value: 1 }] }), /editable is missing/);
  assert.throws(() => readProperties({ properties: [{ name: 'to', value_kind: 'element_reference', editable: true, optional: false, value: 'a' }] }), /referenced_kinds is missing/);
  assert.throws(() => readProperties({ properties: [{ name: 't', value_kind: 'token', editable: true, value: 'left' }] }), /choices is missing/);
  const curve = { name: 'c', value_kind: 'saturated_piecewise_linear_damper_curve', editable: true, row_fields: ['speed', 'push'] };
  assert.throws(() => readProperties({ properties: [{ ...curve, value: [{ speed: 1 }] }] }), /push is missing/);
  assert.throws(() => readProperties({ properties: [{ name: 'c', value_kind: curve.value_kind, editable: true, value: [] }] }), /row_fields is missing/);
});

test('the element kinds are read with their property descriptions, in the service’s order', () => {
  const vehicle = fakeVehicle();
  const kinds = readElementKinds({ element_kinds: vehicle.kinds });
  assert.deepEqual(
    kinds.map((kind) => kind.kind),
    vehicle.kinds.map((kind) => kind.kind),
  );
  const interfaceKind = kinds.find((kind) => kind.kind === 'kind_interface')!;
  assert.deepEqual(interfaceKind.properties.find((property) => property.name === 'hinge_ref'), {
    name: 'hinge_ref',
    editable: true,
    valueKind: 'element_reference',
    referencedKinds: ['kind_hinge'],
    optional: true,
  });
  assert.equal(
    kinds.flatMap((kind) => kind.properties).some((property) => 'value' in property),
    false,
  );
  assert.throws(() => readElementKinds({ element_kinds: [{ kind: 'kind_a' }] }), /properties is missing/);
});

test('an edit names the element it leaves, and a removal everything it takes', () => {
  assert.deepEqual(readEditedElement({ element: { kind: 'kind_a', name: 'renamed' } }), { kind: 'kind_a', name: 'renamed' });
  assert.throws(() => readEditedElement({}), /element is missing/);
  assert.deepEqual(
    readRemovedElements({
      removed_elements: [
        { kind: 'kind_a', name: 'a' },
        { kind: 'kind_b', name: 'b' },
      ],
    }),
    [
      { kind: 'kind_a', name: 'a' },
      { kind: 'kind_b', name: 'b' },
    ],
  );
  assert.throws(() => readRemovedElements({}), /removed_elements is missing/);
});

test('every drawing of a preview names its element, and a rigid body the name its shapes are stated under', () => {
  const vehicle = fakeVehicle();
  assert.equal(readPreview({ preview: null }), null);
  const preview = readPreview({ preview: { project_revision: 7, ...vehicle.drawings } })!;
  assert.equal(preview.projectRevision, 7);
  assert.deepEqual(
    preview.rigidBodies.map((body) => [body.element, body.visualDefinitionBodyName]),
    [
      [{ kind: 'kind_body', name: 'body_a' }, 'body_a'],
      [{ kind: 'kind_body', name: 'body_b' }, 'body_b'],
    ],
  );
  const [first] = vehicle.drawings['rigid_bodies'] as Record<string, unknown>[];
  const created = readPreview({ preview: { ...vehicle.drawings, project_revision: 1, rigid_bodies: [{ ...first, visual_definition_body_name: null }] } })!;
  assert.equal(created.rigidBodies[0]?.visualDefinitionBodyName, null);
  const { visual_definition_body_name: _, ...withoutShapeName } = first!;
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, rigid_bodies: [withoutShapeName] } }), /visual_definition_body_name is missing/);
  assert.deepEqual(preview.forceElements[0]?.element, { kind: 'kind_spring', name: 'spring_a' });
  assert.deepEqual(
    preview.wheels.map((wheel) => [wheel.element.name, wheel.wheelBody.name, wheel.outwardSpinAxisInDrawingFrame[1]]),
    [
      ['body_b.left', 'body_b', -1],
      ['body_b.right', 'body_b', 1],
    ],
  );
  const { nominal_rolling_radius_meters: __, ...withoutRadius } = (vehicle.drawings['wheels'] as Record<string, unknown>[])[0]!;
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, wheels: [withoutRadius] } }), /nominal_rolling_radius_meters is missing/);
  const unnamed = { body_name: 'body_a', rotation_drawing_from_body: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], origin_in_drawing_frame_meters: [0, 0, 0] };
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, rigid_bodies: [unnamed] } }), /element is missing/);
  assert.throws(() => readPreview({ preview: { ...vehicle.drawings, project_revision: 1, weld_joints: undefined } }), /weld_joints/);
});
