import assert from 'node:assert/strict';
import { test } from 'node:test';

import type { ProjectStartingPoint } from '../src/service/fetch_project_starting_points.ts';
import { readStartLinkFromPageAddress } from '../src/session/read_start_link_from_page_address.ts';

const startingPoints: ProjectStartingPoint[] = [
  {
    name: 'A',
    title: { en: 'Starting point A', zh: '起点 A' },
    description: { en: 'A made-up one.', zh: '虚构的。' },
    category: { en: 'Made-up', zh: '虚构' },
    openCommand: 'made_up_command',
    openArguments: { any: 'argument' },
  },
];

function link(search: string) {
  return readStartLinkFromPageAddress(new URLSearchParams(search), startingPoints);
}

test('a starting point link starts a new project from it and may select an element, whose name runs past the first colon', () => {
  assert.deepEqual(link('?starting_point=A&select=kind_a:name.with:colon'), {
    open: { source: 'starting_point', startingPoint: startingPoints[0] },
    select: { kind: 'kind_a', name: 'name.with:colon' },
  });
  assert.throws(() => link('?starting_point=B'), /no starting point is named 'B'/);
  assert.throws(() => link('?starting_point=A&select=no_colon'), /is not <kind>:<name>/);
});

test('a project link opens the project by its name as given; without a link there is nothing to follow', () => {
  assert.deepEqual(link('?project=any%20name'), { open: { source: 'vehicle_project', name: 'any name' }, select: null });
  assert.deepEqual(link('?starting_point=A&project=p')?.open, { source: 'starting_point', startingPoint: startingPoints[0] }, 'a starting point link comes first');
  assert.equal(link(''), null);
  assert.equal(link('?select=kind_a:x'), null);
  assert.equal(link('?vehicle=A'), null, 'a link names a starting point, not a vehicle');
});
