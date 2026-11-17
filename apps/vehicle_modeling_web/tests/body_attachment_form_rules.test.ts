import assert from 'node:assert/strict';
import { test } from 'node:test';

import { readElementKinds, readProperties } from '../src/service/vehicle_modeling_service_protocol.ts';
import {
  attachmentChoices,
  bodyKindAsked,
  entriesCarriedOver,
  entriesStartingFrom,
  jointKindAsked,
  refusalAboutJoint,
  sameAttachmentChoice,
  withReferenceChosen,
} from '../src/ui/body_attachment_form_rules.ts';
import { blankEntries, formProperties, readCreationForm } from '../src/ui/element_creation_form_rules.ts';
import { FakeVehicleModelingService, fakeVehicle } from './fake_vehicle_modeling_service.ts';

// What the fields stating how a body is attached ask for, and where they
// start: free or a joint kind; the joint's fields without its end on a body
// not created yet; the body's properties only a free body has only while free
// is chosen; a joint taking another's place starting from the other's values
// of the same name and value kind.

const kinds = readElementKinds({
  element_kinds: fakeVehicle().kinds.map((kind) => ({
    ...kind,
    properties: kind.properties.map((property) => ({ ...property, section: property.section ?? null })),
    attachment: kind.attachment ?? null,
  })),
});
const bodyKind = kinds.find((kind) => kind.kind === 'kind_body')!;
const description = bodyKind.attachment!;
const hingeKind = kinds.find((kind) => kind.kind === 'kind_hinge')!;
const weldKind = kinds.find((kind) => kind.kind === 'kind_weld')!;
const names = (kind: typeof hingeKind) => formProperties(kind).map((property) => property.name);

test('the choices are free, where it is offered, then each joint kind, where a joint is', () => {
  assert.deepEqual(attachmentChoices(description, true, true), [{ free: true }, { jointKind: 'kind_hinge' }, { jointKind: 'kind_weld' }]);
  assert.deepEqual(attachmentChoices(description, false, true), [{ jointKind: 'kind_hinge' }, { jointKind: 'kind_weld' }]);
  assert.deepEqual(attachmentChoices(description, true, false), [{ free: true }]);
  assert.equal(sameAttachmentChoice({ jointKind: 'kind_hinge' }, { jointKind: 'kind_hinge' }), true);
  assert.equal(sameAttachmentChoice({ jointKind: 'kind_hinge' }, { jointKind: 'kind_weld' }), false);
  assert.equal(sameAttachmentChoice({ free: true }, { jointKind: 'kind_weld' }), false);
  assert.equal(sameAttachmentChoice(null, { free: true }), false);
});

test('a new body is asked for the properties only a free body has only while free is chosen', () => {
  assert.deepEqual(names(bodyKindAsked(bodyKind, description, { free: true })), ['name', 'size_quantity', 'offset_quantity']);
  assert.deepEqual(names(bodyKindAsked(bodyKind, description, { jointKind: 'kind_hinge' })), ['name', 'size_quantity']);
  const entries = { ...blankEntries(bodyKind, {}), offset_quantity: { valueKind: 'vector3' as const, texts: ['1', '2', '3'] } };
  assert.deepEqual(readCreationForm(bodyKindAsked(bodyKind, description, { jointKind: 'kind_hinge' }), {}, entries), { outcome: 'ready', name: '', properties: {} }, 'nor sent');
});

test('a joint’s fields leave out its end on the body for a body not created yet, which is that end', () => {
  assert.deepEqual(names(jointKindAsked(hingeKind, description, false)), ['name', 'end_parent', 'axis_quantity']);
  assert.deepEqual(names(jointKindAsked(hingeKind, description, true)), ['name', 'end_parent', 'end_child', 'axis_quantity']);
});

test('a joint taking another’s place starts from its values of the same name and value kind, its name included', async () => {
  const service = new FakeVehicleModelingService({ '/fake/vehicle.json': fakeVehicle() });
  await service.send('open_vehicle', { vehicle_definition_path: '/fake/vehicle.json', visual_definition_path: null });
  const answer = await service.send('get_properties', { subject: { element: { kind: 'kind_hinge', name: 'hinge_b' } } });
  assert.ok(answer.ok);
  const hingeValues = readProperties(answer.result);
  const sameKind = entriesStartingFrom(hingeKind, hingeValues);
  assert.deepEqual(readCreationForm(hingeKind, {}, sameKind), {
    outcome: 'ready',
    name: 'hinge_b',
    properties: { end_parent: 'frame_a', end_child: 'body_b', axis_quantity: [0, 1, 0] },
  });
  const otherKind = entriesStartingFrom(weldKind, hingeValues);
  assert.deepEqual(
    readCreationForm(weldKind, {}, otherKind),
    { outcome: 'ready', name: 'hinge_b', properties: { end_parent: 'frame_a', end_child: 'body_b' } },
    'the weld’s axis is a number, the hinge’s three: it starts empty, and so does what the hinge has not',
  );
  assert.deepEqual(entriesStartingFrom(weldKind, []), blankEntries(weldKind, {}));
});

test('a form changing from one joint kind to another keeps what was entered of the same name and value kind', () => {
  const entered = {
    ...blankEntries(hingeKind, {}),
    name: { valueKind: 'element_name' as const, text: 'held' },
    axis_quantity: { valueKind: 'vector3' as const, texts: ['1', '0', '0'] },
  };
  const chosen = withReferenceChosen(entered, 'end_parent', 'frame_a');
  assert.deepEqual(readCreationForm(weldKind, {}, entriesCarriedOver(weldKind, chosen)), { outcome: 'ready', name: 'held', properties: { end_parent: 'frame_a' } });
  assert.deepEqual(withReferenceChosen(chosen, 'end_parent', 'body_a'), chosen, 'a choice made is kept');
});

test('a refusal is about the joint when it names an element of the joint kind chosen, and about the body otherwise', () => {
  const aboutJoint = { subject: { element: { kind: 'kind_hinge', name: 'hinge_c' } }, property: 'end_parent', message: 'a loop' };
  assert.equal(refusalAboutJoint(aboutJoint, 'kind_hinge'), true);
  assert.equal(refusalAboutJoint(aboutJoint, 'kind_weld'), false);
  assert.equal(refusalAboutJoint(aboutJoint, null), false);
  assert.equal(refusalAboutJoint({ subject: { element: { kind: 'kind_body', name: 'body_b' } }, property: null, message: 'named' }, 'kind_hinge'), false);
});
