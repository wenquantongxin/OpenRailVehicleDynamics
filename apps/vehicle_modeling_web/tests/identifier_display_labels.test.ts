import assert from 'node:assert/strict';
import { test } from 'node:test';

import { identifierLabel } from '../src/ui/identifier_display_labels.ts';

// The identifiers here are made up: the interface knows no kind or property of the vehicle formats.

test('identifiers read as words, with a dot as a separator', () => {
  assert.equal(identifierLabel('made_up_kind_of_element'), 'made up kind of element');
  assert.equal(identifierLabel('made_up_record.made_up_offset_meters'), 'made up record · made up offset meters');
});
