import assert from 'node:assert/strict';
import { test } from 'node:test';

import { treeKinds } from '../src/ui/vehicle_element_tree_rules.ts';

const frontBody = { kind: 'kind_body', name: 'front_body' };
const rearBody = { kind: 'kind_body', name: 'rear_body' };
const frontLink = { kind: 'kind_link', name: 'front_link' };
const middleLink = { kind: 'kind_link', name: 'middle_link' };
const elements = [frontBody, rearBody, frontLink, middleLink];
// The kinds as the service describes them, one of them without elements.
const kinds = ['kind_body', 'kind_hinge', 'kind_link'];

test('without a filter every described kind is listed in the service’s order, also one with no elements, open as it was left', () => {
  const listed = treeKinds(kinds, elements, '', new Set(['kind_link']), null);
  assert.deepEqual(
    listed.map(({ kind, rows, total, open }) => [kind, rows.length, total, open]),
    [
      ['kind_body', 2, 2, false],
      ['kind_hinge', 0, 0, false],
      ['kind_link', 2, 2, true],
    ],
  );
});

test('a filter lists the matching rows and opens their kinds, leaving out kinds it finds nothing in', () => {
  const listed = treeKinds(kinds, elements, ' REAR ', new Set(), null);
  assert.deepEqual(listed, [{ kind: 'kind_body', rows: [rearBody], total: 2, matched: 1, open: true }]);
});

test('the selected element keeps its row and its kind open, whatever the filter says', () => {
  const link = treeKinds(kinds, elements, 'rear', new Set(), frontLink).find((kind) => kind.kind === 'kind_link');
  assert.ok(link !== undefined, 'the kind of the selected element is listed');
  assert.deepEqual(link.rows, [frontLink]);
  assert.equal(link.open, true);
  assert.equal(link.matched, 0);
  const body = treeKinds(kinds, elements, 'rear', new Set(), frontBody).find((kind) => kind.kind === 'kind_body');
  assert.deepEqual(body?.rows, [frontBody, rearBody]);
});
