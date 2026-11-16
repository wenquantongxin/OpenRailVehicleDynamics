import assert from 'node:assert/strict';
import { test } from 'node:test';

import { shownChoice } from '../src/ui/token_choice_select_rules.ts';

// A token field shows its value only when it is one of the choices; otherwise
// it shows no choice, and every choice, the first included, can be chosen.

const choices = ['first_choice', 'second_choice', 'third_choice'];

test('a value among the choices is the choice shown', () => {
  assert.equal(shownChoice('second_choice', choices), 'second_choice');
  assert.equal(shownChoice('first_choice', choices), 'first_choice');
});

test('a value that is none of the choices shows no choice, so that the first one is not taken for it', () => {
  assert.equal(shownChoice('', choices), undefined);
  assert.equal(shownChoice('FIRST_CHOICE', choices), undefined);
  assert.equal(shownChoice(null, choices), undefined);
  assert.equal(shownChoice(undefined, choices), undefined);
});
