import assert from 'node:assert/strict';
import { test } from 'node:test';

import { readNumberEntry } from '../src/ui/decimal_number_text_reading.ts';
import { afterAnswer, afterCommit, afterTyping, restingField, takesTyping, type FieldState } from '../src/ui/typed_property_field_commit_rules.ts';

// A committing field from typing to the service's answer, here a number field.

const readAgainst = (current: number[]) => (texts: readonly string[]) => readNumberEntry(texts, current);
const resting: FieldState<readonly string[]> = restingField;

test('nothing typed while a commit is on its way is dropped', () => {
  const sending = afterCommit(afterTyping(resting, ['5']), readAgainst([1])).state;
  const typedMeanwhile = afterTyping(sending, ['6']);
  const answered = afterAnswer(typedMeanwhile, true);
  // Either the inputs take no typing while the commit is on its way, or what was typed outlives the answer.
  if (takesTyping(sending)) {
    assert.deepEqual(answered.entry, ['6'], 'the text typed during the commit was dropped');
  } else {
    assert.deepEqual(typedMeanwhile, sending);
  }
});

test('while its commit is on its way a field takes no typing and sends nothing more', () => {
  const commit = afterCommit(afterTyping(resting, ['5']), readAgainst([1]));
  assert.deepEqual(commit.send, { value: [5] });
  assert.equal(takesTyping(commit.state), false);
  assert.equal(afterCommit(commit.state, readAgainst([1])).send, null);
  const accepted = afterAnswer(commit.state, true);
  assert.equal(accepted.entry, null);
  assert.equal(takesTyping(accepted), true);
});

test('a refused commit keeps the text entered, for correction', () => {
  const refused = afterAnswer(afterCommit(afterTyping(resting, ['-5']), readAgainst([1])).state, false);
  assert.deepEqual(refused.entry, ['-5']);
  assert.equal(takesTyping(refused), true);
});

test('an unchanged value is not sent, and a text that is no number is pointed at', () => {
  const unchanged = afterCommit(afterTyping(resting, ['1e1']), readAgainst([10]));
  assert.equal(unchanged.send, null);
  assert.equal(unchanged.reverted, true);
  const unreadable = afterCommit(afterTyping(resting, ['1', 'x', '3']), readAgainst([1, 2, 3]));
  assert.equal(unreadable.send, null);
  assert.equal(unreadable.state.unreadableAt, 1);
});
