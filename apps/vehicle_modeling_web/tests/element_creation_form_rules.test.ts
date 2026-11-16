import assert from 'node:assert/strict';
import { test } from 'node:test';

import { readElementKinds } from '../src/service/vehicle_modeling_service_protocol.ts';
import { blankEntries, formProperties, readCreationForm, refusedField, type FormEntries } from '../src/ui/element_creation_form_rules.ts';
import { fakeVehicle } from './fake_vehicle_modeling_service.ts';

// The creation form asks for the name and every editable property, starts
// empty, and sends only what is filled in.

const kinds = readElementKinds({ element_kinds: fakeVehicle().kinds });
const bodyKind = kinds.find((kind) => kind.kind === 'kind_body')!;
const springKind = kinds.find((kind) => kind.kind === 'kind_spring')!;

test('the form asks for every editable property, the name included, in the kind’s order, and no rotation', () => {
  assert.deepEqual(
    formProperties(bodyKind).map((property) => property.name),
    ['name', 'free_flag', 'size_quantity', 'offset_quantity'],
  );
});

test('an empty form sends the name alone, so that the service’s defaults stand for everything else', () => {
  assert.deepEqual(readCreationForm(springKind, blankEntries(springKind)), { outcome: 'ready', name: '', properties: {} });
});

test('what is filled in is sent by property, the name on its own', () => {
  const entries: FormEntries = {
    ...blankEntries(springKind),
    name: { valueKind: 'element_name', text: 'spring_b' },
    end_a: { ...(blankEntries(springKind)['end_a'] as Extract<FormEntries[string], { valueKind: 'element_reference' }>), chosen: 'frame_a' },
    rate_quantity: { valueKind: 'vector3', texts: ['1', '2.5e3', '-3'] },
    axis_choice: { valueKind: 'token', choices: ['axis_x', 'axis_y', 'axis_z'], chosen: 'axis_y' },
    curve_quantity: { valueKind: 'saturated_piecewise_linear_damper_curve', rowFields: ['speed', 'push'], rows: [['0.1', '30']] },
  };
  assert.deepEqual(readCreationForm(springKind, entries), {
    outcome: 'ready',
    name: 'spring_b',
    properties: { end_a: 'frame_a', rate_quantity: [1, 2500, -3], axis_choice: 'axis_y', curve_quantity: [{ speed: 0.1, push: 30 }] },
  });
  const unticked = readCreationForm(bodyKind, { ...blankEntries(bodyKind), free_flag: { valueKind: 'boolean', chosen: false } });
  assert.deepEqual(unticked, { outcome: 'ready', name: '', properties: { free_flag: false } }, 'a box once ticked and cleared is filled in');
});

test('a field filled in only in part, or with a text that is no number, is pointed at and nothing is sent', () => {
  const partVector = readCreationForm(springKind, { ...blankEntries(springKind), rate_quantity: { valueKind: 'vector3', texts: ['1', '', '3'] } });
  assert.deepEqual(partVector, { outcome: 'unreadable', property: 'rate_quantity', at: 1 });
  const badRow = readCreationForm(springKind, {
    ...blankEntries(springKind),
    curve_quantity: { valueKind: 'saturated_piecewise_linear_damper_curve', rowFields: ['speed', 'push'], rows: [['0.1', '30'], ['fast', '40']] },
  });
  assert.deepEqual(badRow, { outcome: 'unreadable', property: 'curve_quantity', at: 2 });
});

test('a refusal is shown at the property it names when the form asks for it, and at the form otherwise', () => {
  assert.equal(refusedField(springKind, { element: null, property: 'end_a', message: 'no' }), 'end_a');
  assert.equal(refusedField(springKind, { element: null, property: 'name', message: 'taken' }), 'name');
  assert.equal(refusedField(springKind, { element: null, property: null, message: 'no contact record' }), null);
  assert.equal(refusedField(bodyKind, { element: null, property: 'turn_quantity', message: 'not editable' }), null);
});
