import assert from 'node:assert/strict';
import { test } from 'node:test';

import { stripStationsMeters } from '../src/scene/track_builder.ts';

// The ballast and formation strips must reach both ends of the sampled line:
// a tail shorter than one step is sampled, a span shorter than one step still
// gives two stations, and an exact multiple gives no duplicate end.

test('a tail shorter than one step is sampled at the exact end', () => {
  assert.deepEqual(stripStationsMeters(0, 10.5, 1), [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10.5]);
  assert.deepEqual(stripStationsMeters(-30.25, -27.75, 1), [-30.25, -29.25, -28.25, -27.75]);
});

test('a span shorter than one step still has a start and an end', () => {
  assert.deepEqual(stripStationsMeters(0, 0.4, 1), [0, 0.4]);
  assert.deepEqual(stripStationsMeters(0, 5e-10, 1), [0, 5e-10], 'the endpoint tolerance must not discard the start');
});

test('an exact multiple of the step ends once, on the exact end', () => {
  assert.deepEqual(stripStationsMeters(0, 10, 1), [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
  // A final regular sample within tolerance of the end is replaced by the exact end.
  const stations = stripStationsMeters(0, 10 + 1e-12, 1);
  assert.equal(stations.length, 11);
  assert.equal(stations[10], 10 + 1e-12);
});

test('the result is increasing and covers the whole span', () => {
  const stations = stripStationsMeters(-3.3, 86.5, 0.7);
  assert.equal(stations[0], -3.3);
  assert.equal(stations[stations.length - 1], 86.5);
  for (let index = 1; index < stations.length; ++index) {
    assert.ok((stations[index] as number) > (stations[index - 1] as number));
  }
  assert.throws(() => stripStationsMeters(0, 1, 0), /positive/);
  assert.throws(() => stripStationsMeters(1, 0, 1), /precede/);
});
