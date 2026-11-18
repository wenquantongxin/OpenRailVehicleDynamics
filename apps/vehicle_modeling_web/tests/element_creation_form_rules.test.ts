import assert from 'node:assert/strict';
import { test } from 'node:test';

import { readElementKinds } from '../src/service/vehicle_modeling_service_protocol.ts';
import { blankEntries, fixedPropertyReading, formProperties, readCreationForm, refusedField, type FormEntries } from '../src/ui/element_creation_form_rules.ts';
import { fakeVehicle } from './fake_vehicle_modeling_service.ts';

// The creation form asks for the name and every editable property, starts
// empty, and sends only what is filled in, beside what the row's offer fixes.

const kinds = readElementKinds({
  element_kinds: fakeVehicle().kinds.map((kind) => ({
    ...kind,
    properties: kind.properties.map((property) => ({ ...property, section: property.section ?? null })),
    attachment: kind.attachment ?? null,
  })),
});
const bodyKind = kinds.find((kind) => kind.kind === 'kind_body')!;
const frameKind = kinds.find((kind) => kind.kind === 'kind_frame')!;
const springKind = kinds.find((kind) => kind.kind === 'kind_spring')!;

test('the form asks for every editable property, the name and a rotation included, in the kind’s order', () => {
  assert.deepEqual(
    formProperties(bodyKind).map((property) => property.name),
    ['name', 'size_quantity', 'offset_quantity', 'turn_quantity'],
  );
});

test('an empty form sends the name alone, so that the service’s defaults stand for everything else', () => {
  assert.deepEqual(readCreationForm(springKind, {}, blankEntries(springKind, {})), { outcome: 'ready', name: '', properties: {} });
});

test('what is filled in is sent by property, the name on its own', () => {
  const blank = blankEntries(springKind, {});
  const entries: FormEntries = {
    ...blank,
    name: { valueKind: 'element_name', text: 'spring_b' },
    end_a: { ...(blank['end_a'] as Extract<FormEntries[string], { valueKind: 'element_reference' }>), chosen: 'frame_a' },
    rate_quantity: { valueKind: 'vector3', texts: ['1', '2.5e3', '-3'] },
    axis_choice: { valueKind: 'token', choices: ['axis_x', 'axis_y', 'axis_z'], chosen: 'axis_y' },
    curve_quantity: { valueKind: 'saturated_piecewise_linear_damper_curve', rowFields: ['speed', 'push'], rows: [['0.1', '30']] },
  };
  assert.deepEqual(readCreationForm(springKind, {}, entries), {
    outcome: 'ready',
    name: 'spring_b',
    properties: { end_a: 'frame_a', rate_quantity: [1, 2500, -3], axis_choice: 'axis_y', curve_quantity: [{ speed: 0.1, push: 30 }] },
  });
});

test('a property the row fixes is shown with its value, takes no entry, and is always sent', () => {
  const fixed = { body_ref: 'body_b' };
  const entries = blankEntries(frameKind, fixed);
  assert.deepEqual(Object.keys(entries), ['name', 'frame_turn'], 'only what is not fixed is entered');
  assert.deepEqual(
    formProperties(frameKind).map((property) => property.name),
    ['name', 'body_ref', 'frame_turn'],
    'the fixed property keeps its place in the form',
  );
  const shown = fixedPropertyReading(frameKind.properties.find((property) => property.name === 'body_ref')!, fixed);
  assert.deepEqual([shown.editable, shown.value], [false, 'body_b']);
  assert.deepEqual(readCreationForm(frameKind, fixed, { ...entries, name: { valueKind: 'element_name', text: 'frame_b' } }), {
    outcome: 'ready',
    name: 'frame_b',
    properties: { body_ref: 'body_b' },
  });
});

test('a field filled in only in part, or with a text that is no number, is pointed at and nothing is sent', () => {
  const partVector = readCreationForm(springKind, {}, { ...blankEntries(springKind, {}), rate_quantity: { valueKind: 'vector3', texts: ['1', '', '3'] } });
  assert.deepEqual(partVector, { outcome: 'unreadable', property: 'rate_quantity', at: 1 });
  const badRow = readCreationForm(springKind, {}, {
    ...blankEntries(springKind, {}),
    curve_quantity: { valueKind: 'saturated_piecewise_linear_damper_curve', rowFields: ['speed', 'push'], rows: [['0.1', '30'], ['fast', '40']] },
  });
  assert.deepEqual(badRow, { outcome: 'unreadable', property: 'curve_quantity', at: 2 });
});

test('a refusal is shown at the property it names when the form shows it, and at the form otherwise', () => {
  assert.equal(refusedField(springKind, { subject: null, property: 'end_a', message: 'no' }), 'end_a');
  assert.equal(refusedField(springKind, { subject: null, property: 'name', message: 'taken' }), 'name');
  assert.equal(refusedField(springKind, { subject: null, property: null, message: 'no contact record' }), null);
  assert.equal(refusedField(bodyKind, { subject: null, property: 'carrier_body', message: 'not of this kind' }), null);
});

test('a rotation is entered as its three angles in radians: sent together when all three are filled in, not at all when none is', () => {
  const fixed = { body_ref: 'body_b' };
  const named = { ...blankEntries(frameKind, fixed), name: { valueKind: 'element_name' as const, text: 'frame_b' } };
  assert.deepEqual(readCreationForm(frameKind, fixed, named), { outcome: 'ready', name: 'frame_b', properties: { body_ref: 'body_b' } }, 'left empty, it is not sent');
  const turned = { ...named, frame_turn: { valueKind: 'rotation' as const, texts: ['0', '-0.25', '1.5707963267948966'] } };
  assert.deepEqual(readCreationForm(frameKind, fixed, turned), {
    outcome: 'ready',
    name: 'frame_b',
    properties: { body_ref: 'body_b', frame_turn: { roll_pitch_yaw_radians: [0, -0.25, 1.5707963267948966] } },
  });
});

test('a rotation filled in only in part, or with a text that is no number, is pointed at and nothing is sent', () => {
  const fixed = { body_ref: 'body_b' };
  const entries = blankEntries(frameKind, fixed);
  assert.deepEqual(readCreationForm(frameKind, fixed, { ...entries, frame_turn: { valueKind: 'rotation', texts: ['', '', '1.5'] } }), {
    outcome: 'unreadable',
    property: 'frame_turn',
    at: 0,
  });
  assert.deepEqual(readCreationForm(frameKind, fixed, { ...entries, frame_turn: { valueKind: 'rotation', texts: ['0', 'pi/2', '0'] } }), {
    outcome: 'unreadable',
    property: 'frame_turn',
    at: 1,
  });
});
