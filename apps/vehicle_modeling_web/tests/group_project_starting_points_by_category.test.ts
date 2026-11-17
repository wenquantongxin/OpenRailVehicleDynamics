import assert from 'node:assert/strict';
import { test } from 'node:test';

import type { ProjectStartingPoint } from '../src/service/fetch_project_starting_points.ts';
import { startingPointsByCategory } from '../src/ui/group_project_starting_points_by_category.ts';

// The start page lists the starting points under their categories: the
// categories in the order they first appear, each with its starting points in
// the server's order, a category known by its title alone.

function point(name: string, category: { en: string; zh: string }): ProjectStartingPoint {
  return { name, title: { en: name, zh: name }, description: { en: '-', zh: '-' }, category, openCommand: 'made_up', openArguments: {} };
}

test('the categories come in the order they first appear, each with its starting points in the server’s order', () => {
  const first = { en: 'First', zh: '甲' };
  const second = { en: 'Second', zh: '乙' };
  const grouped = startingPointsByCategory([point('a', first), point('b', second), point('c', first), point('d', second), point('e', first)]);
  assert.deepEqual(
    grouped.map(({ category, startingPoints }) => [category.en, startingPoints.map((each) => each.name)]),
    [
      ['First', ['a', 'c', 'e']],
      ['Second', ['b', 'd']],
    ],
  );
});

test('a category is the same only when both of its titles are; with no starting point there is no category', () => {
  const grouped = startingPointsByCategory([point('a', { en: 'Same', zh: '甲' }), point('b', { en: 'Same', zh: '乙' })]);
  assert.deepEqual(
    grouped.map(({ category }) => category.zh),
    ['甲', '乙'],
  );
  assert.deepEqual(startingPointsByCategory([]), []);
});
