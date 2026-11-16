import assert from 'node:assert/strict';
import { test } from 'node:test';

import { choicesShowKind, referenceChoices, type ReferenceDescription } from '../src/ui/element_reference_picker_rules.ts';

// What a reference field offers: the outline's elements of the kinds it may
// name, narrowed by what is typed, and none where the property is optional.

const bodyA = { kind: 'kind_body', name: 'body_a' };
const frameA = { kind: 'kind_frame', name: 'front_frame' };
const frameB = { kind: 'kind_frame', name: 'rear_frame' };
const carrier = { kind: 'kind_carrier', name: 'body_a' };
const elements = [bodyA, frameA, frameB, carrier];

function description(referencedKinds: string[], optional: boolean): ReferenceDescription {
  return { name: 'to', editable: true, valueKind: 'element_reference', referencedKinds, optional };
}

test('only the elements of the kinds the property may name are offered, in outline order', () => {
  assert.deepEqual(referenceChoices(elements, description(['kind_frame', 'kind_body'], false), ''), [bodyA, frameA, frameB]);
  assert.deepEqual(referenceChoices(elements, description(['kind_carrier'], false), ''), [carrier], 'an element of another kind with the same name is not offered');
});

test('typing narrows the choices by name, whatever the case', () => {
  assert.deepEqual(referenceChoices(elements, description(['kind_frame', 'kind_body'], false), ' REAR '), [frameB]);
  assert.deepEqual(referenceChoices(elements, description(['kind_frame'], false), 'body'), []);
});

test('none is offered first where the property is optional, and only while nothing is typed', () => {
  assert.deepEqual(referenceChoices(elements, description(['kind_frame'], true), ''), [null, frameA, frameB]);
  assert.deepEqual(referenceChoices(elements, description(['kind_frame'], true), 'front'), [frameA]);
  assert.deepEqual(referenceChoices(elements, description(['kind_frame'], false), ''), [frameA, frameB]);
});

test('a choice shows its kind where the property may name more than one kind', () => {
  assert.equal(choicesShowKind(description(['kind_frame', 'kind_body'], false)), true);
  assert.equal(choicesShowKind(description(['kind_frame'], true)), false);
});
