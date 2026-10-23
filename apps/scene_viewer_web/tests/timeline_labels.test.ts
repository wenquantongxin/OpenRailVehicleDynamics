import assert from 'node:assert/strict';
import { test } from 'node:test';

import { elementLabelWidthPixels, layoutElementLabels } from '../src/ui/timeline_labels.ts';

test('dense line elements retain non-overlapping labels as the timeline resizes', () => {
  const points = Array.from({ length: 300 }, (_, index) => ({ at: index / 299, stationMeters: index * 60 }));
  for (const width of [120, 480, 900]) {
    const labels = layoutElementLabels(points, width);
    assert.ok(labels.length > 1 && labels.length < points.length);
    let previousRight = -1;
    for (const { point, leftPixels } of labels) {
      assert.ok(points.includes(point), 'labels keep the original element identity');
      assert.ok(leftPixels >= 0 && leftPixels + elementLabelWidthPixels <= width);
      assert.ok(leftPixels > previousRight, 'text boxes have a gap');
      previousRight = leftPixels + elementLabelWidthPixels;
    }
  }
  assert.ok(layoutElementLabels(points, 900).length > layoutElementLabels(points, 120).length);
  assert.equal(points.length, 300, 'all element ticks remain available');
});

test('sparse labels and both endpoints fit without suppressing a short record', () => {
  const points = [{ at: 0 }, { at: 0.25 }, { at: 0.75 }, { at: 1 }];
  const labels = layoutElementLabels(points, 480);
  assert.deepEqual(labels.map(({ point }) => point), points);
  assert.equal(labels[0]?.leftPixels, 0);
  assert.equal(labels[3]?.leftPixels, 480 - elementLabelWidthPixels);
  assert.deepEqual(layoutElementLabels(points, 0), []);
});
