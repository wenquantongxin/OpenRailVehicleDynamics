import assert from 'node:assert/strict';
import { test } from 'node:test';

import type { BilingualTitle, PropertyReading } from '../src/service/vehicle_modeling_service_protocol.ts';
import { propertySections } from '../src/ui/property_section_rules.ts';

// Properties fall into the sections their descriptions state, kept in the
// service's order; section titles are made up.

function property(name: string, section: BilingualTitle | null): PropertyReading {
  return { name, editable: true, section, valueKind: 'number', value: 1 };
}

const first = { en: 'First', zh: '一' };
const second = { en: 'Second', zh: '二' };

test('consecutive properties with the same section form one, in the service’s order; a property without one stands untitled', () => {
  const sections = propertySections([property('own', null), property('a', first), property('b', first), property('c', second), property('d', first)]);
  assert.deepEqual(
    sections.map((section) => [section.title?.en ?? null, section.properties.map((each) => each.name)]),
    [
      [null, ['own']],
      ['First', ['a', 'b']],
      ['Second', ['c']],
      ['First', ['d']],
    ],
  );
  assert.deepEqual(propertySections([]), []);
  assert.equal(propertySections([property('a', first), property('b', { en: 'First', zh: '另' })]).length, 2, 'a section is both its titles');
});
