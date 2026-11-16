import assert from 'node:assert/strict';
import { test } from 'node:test';

import { afterAnswer, afterCommit, afterTyping, restingField, takesTyping } from '../src/ui/number_field.ts';

// A number field from typing to the service's answer.

test('nothing typed while a commit is on its way is dropped', () => {
  const sending = afterCommit(afterTyping(restingField, ['5']), [1]).state;
  const typedMeanwhile = afterTyping(sending, ['6']);
  const answered = afterAnswer(typedMeanwhile, true);
  // Either the inputs take no typing while the commit is on its way, or what was typed outlives the answer.
  if (takesTyping(sending)) {
    assert.deepEqual(answered.draft, ['6'], 'the text typed during the commit was dropped');
  } else {
    assert.deepEqual(typedMeanwhile, sending);
  }
});

test('while its commit is on its way a field takes no typing and sends nothing more', () => {
  const commit = afterCommit(afterTyping(restingField, ['5']), [1]);
  assert.deepEqual(commit.send, [5]);
  assert.equal(takesTyping(commit.state), false);
  assert.equal(afterCommit(commit.state, [1]).send, null);
  const accepted = afterAnswer(commit.state, true);
  assert.equal(accepted.draft, null);
  assert.equal(takesTyping(accepted), true);
});

test('a refused commit keeps the text entered, for correction', () => {
  const refused = afterAnswer(afterCommit(afterTyping(restingField, ['-5']), [1]).state, false);
  assert.deepEqual(refused.draft, ['-5']);
  assert.equal(takesTyping(refused), true);
});

test('an unchanged value is not sent, and a text that is no number is pointed at', () => {
  const unchanged = afterCommit(afterTyping(restingField, ['1e1']), [10]);
  assert.equal(unchanged.send, null);
  assert.equal(unchanged.reverted, true);
  const notANumber = afterCommit(afterTyping(restingField, ['1', 'x', '3']), [1, 2, 3]);
  assert.equal(notANumber.send, null);
  assert.equal(notANumber.state.notANumberAt, 1);
});
