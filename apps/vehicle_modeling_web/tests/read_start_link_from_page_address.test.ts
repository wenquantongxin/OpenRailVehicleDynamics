import assert from 'node:assert/strict';
import { test } from 'node:test';

import type { ReferenceVehicle } from '../src/service/fetch_reference_vehicles.ts';
import { readStartLinkFromPageAddress } from '../src/session/read_start_link_from_page_address.ts';

const catalogue: ReferenceVehicle[] = [{ name: 'A', title: { en: 'Vehicle A', zh: '车辆 A' }, openArguments: { any: 'argument' } }];

function link(search: string) {
  return readStartLinkFromPageAddress(new URLSearchParams(search), catalogue);
}

test('a vehicle link opens that reference vehicle and may select an element, whose name runs past the first colon', () => {
  assert.deepEqual(link('?vehicle=A&select=kind_a:name.with:colon'), {
    open: { source: 'reference_vehicle', vehicle: catalogue[0] },
    select: { kind: 'kind_a', name: 'name.with:colon' },
  });
  assert.throws(() => link('?vehicle=B'), /no reference vehicle is named 'B'/);
  assert.throws(() => link('?vehicle=A&select=no_colon'), /is not <kind>:<name>/);
});

test('a project link opens the project by its name as given; without a link there is nothing to follow', () => {
  assert.deepEqual(link('?project=any%20name'), { open: { source: 'vehicle_project', name: 'any name' }, select: null });
  assert.deepEqual(link('?vehicle=A&project=p')?.open, { source: 'reference_vehicle', vehicle: catalogue[0] }, 'a vehicle link comes first');
  assert.equal(link(''), null);
  assert.equal(link('?select=kind_a:x'), null);
});
