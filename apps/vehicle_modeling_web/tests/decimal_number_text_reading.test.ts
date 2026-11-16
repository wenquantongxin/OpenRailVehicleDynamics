import assert from 'node:assert/strict';
import { test } from 'node:test';

import { numberFromText, readNumberEntry } from '../src/ui/decimal_number_text_reading.ts';

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

test('an entry is unchanged, unreadable at one input, or a change of every component', () => {
  assert.deepEqual(readNumberEntry(['10'], [10]), { outcome: 'unchanged' });
  assert.deepEqual(readNumberEntry(['1e1'], [10]), { outcome: 'unchanged' });
  assert.deepEqual(readNumberEntry(['1', 'x', '3'], [1, 2, 3]), { outcome: 'unreadable', at: 1 });
  assert.deepEqual(readNumberEntry([''], [4]), { outcome: 'unreadable', at: 0 });
  assert.deepEqual(readNumberEntry(['1', '2', '-3'], [1, 2, 3]), { outcome: 'changed', value: [1, 2, -3] });
});
