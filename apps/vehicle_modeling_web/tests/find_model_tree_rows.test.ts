import assert from 'node:assert/strict';
import { test } from 'node:test';

import { rowPathOfSubject, rowWithKey, treeHasSubject } from '../src/service/find_model_tree_rows.ts';
import type { ModelTreeNode } from '../src/service/vehicle_modeling_service_protocol.ts';

// Rows are found by what they select or by their key, at any depth; titles
// are made up and never looked at.

function node(key: string, subject: ModelTreeNode['subject'], children: ModelTreeNode[] = []): ModelTreeNode {
  return { key, title: subject === null ? { en: `Title of ${key}`, zh: '标题' } : null, subject, provided: true, note: null, creates: [], attaches: [], children };
}

const owner = { element: { kind: 'kind_owner', name: 'owner' } };
const owned = { element: { kind: 'kind_owned', name: 'owned' } };
const sameNameOtherKind = { element: { kind: 'kind_other', name: 'owned' } };
const settings = { settings: 'made_up_word' };
const tree = [
  node('first', null, [node('settings row', settings)]),
  node('second', null, [node('owner row', owner, [node('group', null, [node('owned row', owned)])]), node('other row', sameNameOtherKind)]),
];

test('a subject’s row is found with the rows above it, from the first level down', () => {
  assert.deepEqual(
    rowPathOfSubject(tree, owned)?.map((row) => row.key),
    ['second', 'owner row', 'group', 'owned row'],
  );
  assert.deepEqual(
    rowPathOfSubject(tree, settings)?.map((row) => row.key),
    ['first', 'settings row'],
  );
  assert.deepEqual(
    rowPathOfSubject(tree, sameNameOtherKind)?.map((row) => row.key),
    ['second', 'other row'],
    'an element is its kind and its name',
  );
  assert.equal(rowPathOfSubject(tree, { element: { kind: 'kind_owned', name: 'gone' } }), null);
  assert.equal(rowPathOfSubject(tree, { settings: 'other_word' }), null);
});

test('whether the tree has a subject, and a row by its key at any depth', () => {
  assert.equal(treeHasSubject(tree, owned), true);
  assert.equal(treeHasSubject(tree, { settings: 'other_word' }), false);
  assert.equal(treeHasSubject(null, owned), false);
  assert.equal(rowWithKey(tree, 'group')?.children[0]?.key, 'owned row');
  assert.equal(rowWithKey(tree, 'no such row'), null);
});
