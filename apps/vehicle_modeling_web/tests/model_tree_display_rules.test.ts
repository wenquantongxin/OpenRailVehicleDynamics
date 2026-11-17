import assert from 'node:assert/strict';
import { test } from 'node:test';

import { rowWithKey } from '../src/service/find_model_tree_rows.ts';
import { readModelTree, type ModelTreeNode } from '../src/service/vehicle_modeling_service_protocol.ts';
import {
  elementRowsDirectlyUnder,
  matchingElementRowsDirectlyUnder,
  openRowsCarriedOver,
  rowsAboveSubject,
  rowsOpenWhenAVehicleOpens,
  rowsShownByFilter,
  showsElementCount,
  showsKindTag,
} from '../src/ui/model_tree_display_rules.ts';
import { FakeVehicleModelingService, fakeVehicle } from './fake_vehicle_modeling_service.ts';

// How the model tree is drawn, on the tree the fake service describes of its
// own records: which rows are open, and stay open when the tree is read again,
// what a row counts, which rows show a kind tag, what a filter shows, and which
// rows open to reveal a selection.

async function openedFakeService(): Promise<FakeVehicleModelingService> {
  const service = new FakeVehicleModelingService({ '/fake/vehicle.json': fakeVehicle() });
  await service.send('open_vehicle', { vehicle_definition_path: '/fake/vehicle.json', visual_definition_path: null });
  return service;
}

async function treeOf(service: FakeVehicleModelingService): Promise<ModelTreeNode[]> {
  const answer = await service.send('get_model_tree');
  assert.ok(answer.ok);
  return readModelTree(answer.result);
}

async function fakeTree(): Promise<ModelTreeNode[]> {
  return treeOf(await openedFakeService());
}

function row(tree: readonly ModelTreeNode[], key: string): ModelTreeNode {
  return rowWithKey(tree, key)!;
}

test('when a vehicle opens only the first first-level row that lists elements is open, and none of its rows', async () => {
  const tree = await fakeTree();
  assert.deepEqual([...rowsOpenWhenAVehicleOpens(tree)], ['domain/bodies']);
  assert.deepEqual([...rowsOpenWhenAVehicleOpens([])], []);
});

test('the open rows stay open when the tree is read again: a renamed element’s row and the rows under it under their new keys, and nothing else renamed by an element added', async () => {
  const service = await openedFakeService();
  const before = await treeOf(service);
  const open = new Set(['domain/bodies', 'element/kind_body/body_a', 'frames/body_a', 'domain/contact']);
  assert.ok((await service.send('set_property', { subject: { element: { kind: 'kind_body', name: 'body_a' } }, property: 'name', value: 'body_z' })).ok);
  const renamed = await treeOf(service);
  const carried = openRowsCarriedOver(before, renamed, open);
  assert.deepEqual([...carried].sort(), ['domain/bodies', 'domain/contact', 'element/kind_body/body_z', 'frames/body_z']);

  assert.ok((await service.send('add_element', { kind: 'kind_body', name: 'body_c', properties: {} })).ok);
  const grown = await treeOf(service);
  assert.deepEqual([...openRowsCarriedOver(renamed, grown, carried)].sort(), [...carried].sort());
  // Each undo is followed by a reading of its own, as every edit is.
  assert.ok((await service.send('undo')).ok);
  const shrunk = await treeOf(service);
  assert.deepEqual([...openRowsCarriedOver(grown, shrunk, carried)].sort(), [...carried].sort());
  assert.ok((await service.send('undo')).ok);
  assert.deepEqual([...openRowsCarriedOver(shrunk, await treeOf(service), carried)].sort(), [...open].sort(), 'undoing the rename brings the old keys back');
});

/** Every row of the tree, the first level first and each row before the rows under it. */
function everyRow(rows: readonly ModelTreeNode[]): ModelTreeNode[] {
  return rows.flatMap((each) => [each, ...everyRow(each.children)]);
}

test('a titled row at any level counts the element rows directly under it and says so when it has any; an element row never counts', async () => {
  const tree = await fakeTree();
  assert.deepEqual(
    everyRow(tree)
      .filter(showsElementCount)
      .map((each) => [each.key, elementRowsDirectlyUnder(each)]),
    [
      ['domain/bodies', 2],
      ['frames/body_a', 1],
      ['domain/links', 1],
      ['carriers', 1],
    ],
    'the domain of settings and carriers has none directly under it, nor has a row that only creates',
  );
  assert.equal(elementRowsDirectlyUnder(row(tree, 'element/kind_carrier/body_b')), 2);
  assert.equal(showsElementCount(row(tree, 'element/kind_carrier/body_b')), false);
});

test('an element row shows its kind as a tag only in a list that offers several kinds', async () => {
  const tree = await fakeTree();
  assert.equal(showsKindTag(row(tree, 'domain/links')), true);
  assert.equal(showsKindTag(row(tree, 'domain/bodies')), false);
  assert.equal(showsKindTag(row(tree, 'frames/body_a')), false);
  assert.equal(showsKindTag(row(tree, 'element/kind_carrier/body_b')), false);
  assert.equal(showsKindTag(null), false);
});

test('revealing a subject opens every row above its own, from the first level down; a subject the tree does not have opens nothing', async () => {
  const tree = await fakeTree();
  assert.deepEqual(rowsAboveSubject(tree, { element: { kind: 'kind_frame', name: 'frame_a' } }), ['domain/bodies', 'element/kind_body/body_a', 'frames/body_a']);
  assert.deepEqual(rowsAboveSubject(tree, { element: { kind: 'kind_interface', name: 'body_b.left' } }), ['domain/contact', 'carriers', 'element/kind_carrier/body_b']);
  assert.deepEqual(rowsAboveSubject(tree, { settings: 'setting_wheel' }), ['domain/contact']);
  assert.deepEqual(rowsAboveSubject(tree, { element: { kind: 'kind_body', name: 'body_a' } }), ['domain/bodies']);
  assert.deepEqual(rowsAboveSubject(tree, { element: { kind: 'kind_frame', name: 'gone' } }), []);
});

test('a filter shows the element rows whose name holds it, whatever the case, with the rows above them opened, and the selected row', async () => {
  const tree = await fakeTree();
  const filtered = rowsShownByFilter(tree, ' LEFT ', { settings: 'setting_constant' });
  assert.deepEqual([...filtered.matching], ['element/kind_interface/body_b.left']);
  assert.deepEqual([...filtered.open].sort(), ['carriers', 'domain/contact', 'domain/settings', 'element/kind_carrier/body_b']);
  assert.deepEqual(
    [...filtered.shown].sort(),
    ['carriers', 'domain/contact', 'domain/settings', 'element/kind_carrier/body_b', 'element/kind_interface/body_b.left', 'settings/setting_constant'],
  );
  const bodyB = rowsShownByFilter(tree, 'body_b', null);
  assert.deepEqual(
    everyRow(tree)
      .filter(showsElementCount)
      .map((each) => [each.key, matchingElementRowsDirectlyUnder(each, bodyB.matching), elementRowsDirectlyUnder(each)]),
    [
      ['domain/bodies', 1, 2],
      ['frames/body_a', 0, 1],
      ['domain/links', 0, 1],
      ['carriers', 1, 1],
    ],
    'a body and a carrier hold the name, each counted by the row directly above it; a titled row is never matched',
  );
});
