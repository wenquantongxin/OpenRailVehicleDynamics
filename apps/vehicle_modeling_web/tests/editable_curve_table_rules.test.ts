import assert from 'node:assert/strict';
import { test } from 'node:test';

import { curveEntryOf, readCurveEntry, withCellText, withRowAdded, withRowRemoved } from '../src/ui/editable_curve_table_rules.ts';

// A curve table: rows of texts by the row fields, rows added and removed, the
// whole curve read at once.

const rowFields = ['speed', 'push'];
const curve = [
  { speed: 0, push: 0 },
  { speed: 0.1, push: 2.5e3 },
];

test('a curve shows a text per row field in each row, with full precision', () => {
  assert.deepEqual(curveEntryOf(curve, rowFields), [
    ['0', '0'],
    ['0.1', '2500'],
  ]);
  assert.deepEqual(curveEntryOf([], rowFields), []);
});

test('the first row of an empty curve can be added, filled in and read', () => {
  const added = withRowAdded(curveEntryOf([], rowFields), rowFields);
  assert.deepEqual(added, [['', '']]);
  const filled = withCellText(withCellText(added, 0, 0, '0.2'), 0, 1, '1e4');
  assert.deepEqual(readCurveEntry(filled, rowFields, []), { outcome: 'changed', value: [{ speed: 0.2, push: 1e4 }] });
});

test('a row is removed anywhere, and an empty table is a change to an empty curve', () => {
  const entry = curveEntryOf(curve, rowFields);
  assert.deepEqual(readCurveEntry(withRowRemoved(entry, 0), rowFields, curve), { outcome: 'changed', value: [{ speed: 0.1, push: 2500 }] });
  assert.deepEqual(readCurveEntry(withRowRemoved(withRowRemoved(entry, 1), 0), rowFields, curve), { outcome: 'changed', value: [] });
});

test('a table that states the curve shown is unchanged, and a text that is no number is pointed at row by row', () => {
  assert.deepEqual(readCurveEntry(curveEntryOf(curve, rowFields), rowFields, curve), { outcome: 'unchanged' });
  assert.deepEqual(readCurveEntry(withCellText(curveEntryOf(curve, rowFields), 0, 0, '0e0'), rowFields, curve), { outcome: 'unchanged' });
  assert.deepEqual(readCurveEntry(withCellText(curveEntryOf(curve, rowFields), 1, 1, 'fast'), rowFields, curve), { outcome: 'unreadable', at: 3 });
  assert.deepEqual(readCurveEntry(withRowAdded(curveEntryOf(curve, rowFields), rowFields), rowFields, curve), { outcome: 'unreadable', at: 4 });
});
