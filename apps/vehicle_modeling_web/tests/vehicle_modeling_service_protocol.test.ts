import assert from 'node:assert/strict';
import { test } from 'node:test';

import {
  readAnswer,
  readCreatedElement,
  readEditedSubject,
  readElementKinds,
  readModelTree,
  readPreview,
  readProperties,
  readRemovedElements,
  readServiceSessionDescription,
  readVehicleProjectNames,
  readVisualDefinitionDocument,
} from '../src/service/vehicle_modeling_service_protocol.ts';
import { fakeVehicle } from './fake_vehicle_modeling_service.ts';

// Answers are read leniently, but what the interface uses must be there.

/** The fake vehicle's kinds as describe_element_kinds states them: every description with its section, null included. */
function describedKinds() {
  return fakeVehicle().kinds.map((kind) => ({ ...kind, properties: kind.properties.map((property) => ({ ...property, section: property.section ?? null })) }));
}

test('an accepted answer carries the session envelope and a refused one the service’s reason', () => {
  const accepted = readAnswer({ id: 3, ok: true, revision: 4, can_undo: true, can_redo: false, has_unsaved_changes: true, result: {}, extra: 'ignored' });
  assert.deepEqual(accepted, { ok: true, session: { revision: 4, canUndo: true, canRedo: false, hasUnsavedChanges: true }, result: {} });
  assert.throws(() => readAnswer({ ok: true, revision: 4, can_undo: true, can_redo: false, result: {} }), /has_unsaved_changes is missing/);
  const refused = readAnswer({ id: 3, ok: false, error: { subject: { element: { kind: 'kind_a', name: 'a' } }, property: 'p', message: 'no' } });
  assert.deepEqual(refused, { ok: false, refusal: { subject: { element: { kind: 'kind_a', name: 'a' } }, property: 'p', message: 'no' } });
  const aboutSettings = readAnswer({ id: 4, ok: false, error: { subject: { settings: 'made_up_word' }, property: 'p', message: 'read-only' } });
  assert.deepEqual(aboutSettings, { ok: false, refusal: { subject: { settings: 'made_up_word' }, property: 'p', message: 'read-only' } });
  assert.deepEqual(readAnswer({ id: 5, ok: false, error: { subject: null, property: null, message: 'no' } }), { ok: false, refusal: { subject: null, property: null, message: 'no' } });
  assert.throws(() => readAnswer({ id: 6, ok: false, error: { element: null, property: null, message: 'no' } }), /subject is missing/);
  assert.throws(() => readAnswer({ ok: true, revision: 4, can_undo: true, result: {} }), /can_redo is missing/);
});

test('an answer while nothing is open has no session: its four session fields are null', () => {
  const answer = readAnswer({ id: 1, ok: true, revision: null, can_undo: null, can_redo: null, has_unsaved_changes: null, result: { open: false } });
  assert.deepEqual(answer, { ok: true, session: null, result: { open: false } });
});

test('get_session says whether a vehicle is open and with which project file, if any', () => {
  assert.deepEqual(readServiceSessionDescription({ open: false }), { open: false });
  assert.deepEqual(readServiceSessionDescription({ open: true, vehicle_project_file: null }), { open: true, vehicleProjectFile: null });
  assert.deepEqual(readServiceSessionDescription({ open: true, vehicle_project_file: { directory: '/somewhere/projects', name: 'mine' } }), {
    open: true,
    vehicleProjectFile: { directory: '/somewhere/projects', name: 'mine' },
  });
  assert.throws(() => readServiceSessionDescription({ open: true }), /vehicle_project_file is missing/);
  assert.throws(() => readServiceSessionDescription({ open: true, vehicle_project_file: { directory: '/somewhere/projects' } }), /name is missing/);
});

test('a folder lists its project names as given, and a visual definition is passed on unread, or null', () => {
  assert.deepEqual(readVehicleProjectNames({ vehicle_projects: ['a', 'b.c', 'd-e'] }), ['a', 'b.c', 'd-e']);
  assert.throws(() => readVehicleProjectNames({ vehicle_projects: [1] }), /not a string/);
  const document = { anything: ['the service', { does: 'not read' }] };
  assert.equal(readVisualDefinitionDocument({ visual_definition: document }), document);
  assert.equal(readVisualDefinitionDocument({ visual_definition: null }), null);
  assert.throws(() => readVisualDefinitionDocument({}), /visual_definition is missing/);
});

test('every value kind is read with its shape and what its description says, and an absent value is null', () => {
  const sectioned = { section: { en: 'A section', zh: '一节' } };
  const properties = readProperties({
    properties: [
      { name: 'n', value_kind: 'number', editable: true, section: null, value: 1.5e-12 },
      { name: 'absent', value_kind: 'number', editable: false, section: null, value: null },
      { name: 'v', value_kind: 'vector3', editable: true, section: null, value: [1, 2, 3] },
      { name: 'b', value_kind: 'boolean', editable: true, section: null, value: false },
      { name: 'r', value_kind: 'rotation', editable: false, section: null, value: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] },
      { name: 'own', value_kind: 'element_name', editable: true, section: null, value: 'frame_a' },
      { name: 'to', value_kind: 'element_reference', editable: true, section: null, referenced_kinds: ['kind_a', 'kind_b'], optional: false, value: 'body_a' },
      { name: 'maybe', value_kind: 'element_reference', editable: true, section: null, referenced_kinds: ['kind_c'], optional: true, value: null },
      { name: 't', value_kind: 'token', editable: true, section: null, choices: ['left', 'right'], value: 'right' },
      { name: 'c', value_kind: 'saturated_piecewise_linear_damper_curve', editable: true, section: null, row_fields: ['speed', 'push'], value: [{ push: 30, speed: 0.1, extra: 1 }] },
      { name: 'empty', value_kind: 'saturated_piecewise_linear_damper_curve', editable: true, section: null, row_fields: ['speed', 'push'], value: [] },
      { name: 'words', value_kind: 'text', editable: false, ...sectioned, value: 'shown as it is: /a/path.txt' },
      { name: 'no_words', value_kind: 'text', editable: false, ...sectioned, value: null },
    ],
  });
  assert.deepEqual(
    properties.slice(-2).map(({ name, section }) => [name, section]),
    [
      ['words', { en: 'A section', zh: '一节' }],
      ['no_words', { en: 'A section', zh: '一节' }],
    ],
  );
  assert.deepEqual(
    properties.map(({ name: _, editable: __, section: ___, ...rest }) => rest),
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
      { valueKind: 'text', value: 'shown as it is: /a/path.txt' },
      { valueKind: 'text', value: null },
    ],
  );
});

test('a value kind the interface cannot show, a missing description field or a value of the wrong shape fails loudly', () => {
  for (const gone of ['wheel_side', 'quaternion']) {
    assert.throws(() => readProperties({ properties: [{ name: 'q', value_kind: gone, editable: false, section: null, value: 'a' }] }), new RegExp(gone));
  }
  assert.throws(() => readProperties({ properties: [{ name: 'n', value_kind: 'number', editable: true, value: 1 }] }), /section is missing/);
  assert.throws(() => readProperties({ properties: [{ name: 'v', value_kind: 'vector3', editable: true, section: null, value: [1, 2] }] }), /three entries/);
  assert.throws(() => readProperties({ properties: [{ name: 'n', value_kind: 'number', section: null, value: 1 }] }), /editable is missing/);
  assert.throws(() => readProperties({ properties: [{ name: 'to', value_kind: 'element_reference', editable: true, section: null, optional: false, value: 'a' }] }), /referenced_kinds is missing/);
  assert.throws(() => readProperties({ properties: [{ name: 't', value_kind: 'token', editable: true, section: null, value: 'left' }] }), /choices is missing/);
  const curve = { name: 'c', value_kind: 'saturated_piecewise_linear_damper_curve', editable: true, section: null, row_fields: ['speed', 'push'] };
  assert.throws(() => readProperties({ properties: [{ ...curve, value: [{ speed: 1 }] }] }), /push is missing/);
  assert.throws(() => readProperties({ properties: [{ name: 'c', value_kind: curve.value_kind, editable: true, section: null, value: [] }] }), /row_fields is missing/);
});

test('the element kinds are read with their titles and property descriptions, in the service’s order', () => {
  const described = describedKinds();
  const kinds = readElementKinds({ element_kinds: described });
  assert.deepEqual(
    kinds.map((kind) => kind.kind),
    described.map((kind) => kind.kind),
  );
  const interfaceKind = kinds.find((kind) => kind.kind === 'kind_interface')!;
  assert.deepEqual(interfaceKind.title, { en: 'Made-up interface', zh: '虚构接口' });
  assert.deepEqual(interfaceKind.properties.find((property) => property.name === 'hinge_ref'), {
    name: 'hinge_ref',
    editable: true,
    section: null,
    valueKind: 'element_reference',
    referencedKinds: ['kind_hinge'],
    optional: true,
  });
  assert.deepEqual(kinds.find((kind) => kind.kind === 'kind_body')!.properties[2]!.section, { en: 'Size', zh: '大小' });
  assert.equal(
    kinds.flatMap((kind) => kind.properties).some((property) => 'value' in property),
    false,
  );
  assert.throws(() => readElementKinds({ element_kinds: [{ kind: 'kind_a', title: { en: 'A', zh: '甲' } }] }), /properties is missing/);
  assert.throws(() => readElementKinds({ element_kinds: [{ kind: 'kind_a', properties: [] }] }), /title is missing/);
  assert.throws(() => readElementKinds({ element_kinds: [{ kind: 'kind_a', title: { en: 'A' }, properties: [] }] }), /zh is missing/);
});

test('the model tree is read row by row: titles, subjects, notes, what each row creates, and the rows under it', () => {
  const [domain] = readModelTree({
    nodes: [
      {
        key: 'a domain',
        title: { en: 'A domain', zh: '一个域' },
        subject: null,
        provided: true,
        note: { en: 'A note', zh: '注' },
        creates: [{ kind: 'kind_a', fixed_properties: {} }],
        children: [
          { key: 'an element', title: null, subject: { element: { kind: 'kind_a', name: 'a' } }, provided: true, note: null, creates: [{ kind: 'kind_b', fixed_properties: { owner_ref: 'a' } }], children: [] },
          { key: 'settings', title: { en: 'Settings', zh: '设置' }, subject: { settings: 'made_up_word' }, provided: true, note: null, creates: [], children: [] },
          { key: 'later', title: { en: 'Later', zh: '以后' }, subject: null, provided: false, note: { en: 'Comes later.', zh: '以后提供。' }, creates: [], children: [] },
        ],
      },
    ],
  });
  assert.deepEqual(domain, {
    key: 'a domain',
    title: { en: 'A domain', zh: '一个域' },
    subject: null,
    provided: true,
    note: { en: 'A note', zh: '注' },
    creates: [{ kind: 'kind_a', fixedProperties: {} }],
    children: [
      { key: 'an element', title: null, subject: { element: { kind: 'kind_a', name: 'a' } }, provided: true, note: null, creates: [{ kind: 'kind_b', fixedProperties: { owner_ref: 'a' } }], children: [] },
      { key: 'settings', title: { en: 'Settings', zh: '设置' }, subject: { settings: 'made_up_word' }, provided: true, note: null, creates: [], children: [] },
      { key: 'later', title: { en: 'Later', zh: '以后' }, subject: null, provided: false, note: { en: 'Comes later.', zh: '以后提供。' }, creates: [], children: [] },
    ],
  });
  assert.throws(() => readModelTree({ nodes: [{ key: 'k', title: null, subject: null, provided: true, note: null, creates: [] }] }), /k.children is missing/);
  assert.throws(() => readModelTree({ nodes: [{ key: 'k', title: null, subject: null, note: null, creates: [], children: [] }] }), /k.provided is missing/);
  assert.throws(() => readModelTree({ nodes: [{ key: 'k', title: null, subject: { neither: 1 }, provided: true, note: null, creates: [], children: [] }] }), /settings is missing/);
  assert.throws(() => readModelTree({ nodes: [{ key: 'k', title: null, subject: null, provided: true, note: null, creates: [{ kind: 'kind_a' }], children: [] }] }), /fixed_properties is missing/);
});

test('a creation names the element it made, an edit the subject it leaves, and a removal everything it takes', () => {
  assert.deepEqual(readCreatedElement({ element: { kind: 'kind_a', name: 'made' } }), { kind: 'kind_a', name: 'made' });
  assert.throws(() => readCreatedElement({}), /element is missing/);
  assert.deepEqual(readEditedSubject({ subject: { element: { kind: 'kind_a', name: 'renamed' } } }), { element: { kind: 'kind_a', name: 'renamed' } });
  assert.deepEqual(readEditedSubject({ subject: { settings: 'made_up_word' } }), { settings: 'made_up_word' });
  assert.throws(() => readEditedSubject({ element: { kind: 'kind_a', name: 'renamed' } }), /subject is missing/);
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
