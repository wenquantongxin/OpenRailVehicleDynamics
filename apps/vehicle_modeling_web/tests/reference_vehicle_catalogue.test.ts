import assert from 'node:assert/strict';
import { test } from 'node:test';

import { parseReferenceVehicleCatalogue } from '../server/reference_vehicle_catalogue.ts';

// People write the catalogue, so it is read strictly.

const entry = {
  name: 'A',
  title: { en: 'Vehicle A', zh: '车辆 A' },
  vehicle_definition_path: '../library/a/vehicle_definition.json',
  startup_state_path: '../library/a/startup.json',
  contact_record_path: '../library/a/contact_record.json',
  visual_definition_path: '../library/a/visual_definition.json',
};

function parse(entries: unknown): ReturnType<typeof parseReferenceVehicleCatalogue> {
  return parseReferenceVehicleCatalogue(JSON.stringify(entries), '/catalogues/apps');
}

test('entries are read in order, with titles and with paths relative to the catalogue’s folder', () => {
  const { visual_definition_path: _, ...withoutVisual } = entry;
  const [a, b] = parse([entry, { ...withoutVisual, name: 'B' }]);
  assert.deepEqual(a, {
    name: 'A',
    title: { en: 'Vehicle A', zh: '车辆 A' },
    openArguments: {
      vehicle_definition_path: '/catalogues/library/a/vehicle_definition.json',
      startup_state_path: '/catalogues/library/a/startup.json',
      contact_record_path: '/catalogues/library/a/contact_record.json',
    },
    visualDefinitionPath: '/catalogues/library/a/visual_definition.json',
  });
  assert.equal(b?.name, 'B');
  assert.equal(b?.visualDefinitionPath, null);
});

test('anything but exactly the keys, as non-empty strings, under unique names is refused', () => {
  const { startup_state_path: _, ...withoutStartupState } = entry;
  for (const [entries, message] of [
    [{ ...entry }, /list/],
    [[{ ...entry, colour: 'red' }], /unknown key 'colour'/],
    [[withoutStartupState], /lacks 'startup_state_path'/],
    [[{ ...entry, name: ' ' }], /name/],
    [[{ ...entry, contact_record_path: 3 }], /contact_record_path/],
    [[{ ...entry, title: { en: 'Vehicle A' } }], /lacks 'zh'/],
    [[{ ...entry, title: { en: 'Vehicle A', zh: '车辆 A', de: 'Fahrzeug A' } }], /unknown key 'de'/],
    [[entry, { ...entry }], /listed twice/],
  ] as const) {
    assert.throws(() => parse(entries), message);
  }
});
