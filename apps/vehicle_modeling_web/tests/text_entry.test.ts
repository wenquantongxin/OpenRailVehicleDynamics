import assert from 'node:assert/strict';
import { test } from 'node:test';

import { identifierLabel } from '../src/ui/identifier_label.ts';
import { numberFromText, readDraft } from '../src/ui/number_entry.ts';

test('a number field reads decimal and scientific notation, nothing else', () => {
  for (const [text, value] of [
    ['185000', 185000],
    [' -2.5e-3 ', -0.0025],
    ['+.5', 0.5],
    ['7.', 7],
    ['1E6', 1e6],
  ] as const) {
    assert.equal(numberFromText(text), value, text);
  }
  for (const text of ['', ' ', '0x10', 'Infinity', '1e999', '1,5', '2 3', 'NaN', '--1']) {
    assert.equal(numberFromText(text), null, text);
  }
});

test('a number is shown with its full precision and read back unchanged', () => {
  for (const value of [0.1 + 0.2, -1.0076393671195088e-15, 30549, 1e21]) {
    assert.equal(numberFromText(String(value)), value);
  }
});

test('a draft is unchanged, not a number at one position, or a change of every component', () => {
  assert.deepEqual(readDraft(['10'], [10]), { outcome: 'unchanged' });
  assert.deepEqual(readDraft(['1e1'], [10]), { outcome: 'unchanged' });
  assert.deepEqual(readDraft(['1', 'x', '3'], [1, 2, 3]), { outcome: 'not_a_number', position: 1 });
  assert.deepEqual(readDraft([''], [4]), { outcome: 'not_a_number', position: 0 });
  assert.deepEqual(readDraft(['1', '2', '-3'], [1, 2, 3]), { outcome: 'changed', values: [1, 2, -3] });
});

test('identifiers read as words, with a dot as a separator', () => {
  assert.equal(identifierLabel('point_to_point_linear_spring_damper'), 'point to point linear spring damper');
  assert.equal(identifierLabel('placement.lateral_offset_in_local_track_frame_meters'), 'placement · lateral offset in local track frame meters');
});
