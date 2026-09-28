import assert from 'node:assert/strict';
import { test } from 'node:test';

import { measureCardLayout, viewportAreaFromLayout } from '../src/viewer/card_layout.ts';

test('the free picture keeps the same clearance from a card that the card keeps from the edge', () => {
  const stage = { left: 0, right: 1400, top: 0, bottom: 900 };
  const layout = measureCardLayout(
    stage,
    { left: 16, right: 278, top: 86, bottom: 700 },
    { left: 1122, right: 1384, top: 86, bottom: 700 },
    { left: 310, right: 1090, top: 754, bottom: 870 },
  );
  assert.deepEqual(layout, { left: 294, right: 294, top: 86, bottom: 176 });
});

test('a narrower column or a lower top moves the free picture with it', () => {
  const stage = { left: 0, right: 1200, top: 0, bottom: 800 };
  const layout = measureCardLayout(stage, { left: 16, right: 260, top: 82, bottom: 700 }, { left: 940, right: 1184, top: 82, bottom: 700 }, null);
  assert.deepEqual(layout, { left: 276, right: 276, top: 82, bottom: 0 });
});

test('absent cards cover nothing and a stage offset is subtracted', () => {
  assert.deepEqual(measureCardLayout({ left: 100, right: 900, top: 50, bottom: 650 }, null, null, null), { left: 0, right: 0, top: 0, bottom: 0 });
  const layout = measureCardLayout({ left: 100, right: 900, top: 50, bottom: 650 }, { left: 116, right: 378, top: 136, bottom: 600 }, null, { left: 300, right: 700, top: 560, bottom: 620 });
  assert.deepEqual(layout, { left: 294, right: 0, top: 86, bottom: 120 });
});

test('a narrow but measured opening is not replaced by the full viewport', () => {
  assert.deepEqual(viewportAreaFromLayout(800, 600, { left: 276, right: 276, top: 220, bottom: 200 }), {
    width: 800,
    height: 600,
    free: { x0: 276, y0: 220, x1: 524, y1: 400 },
  });
});

test('absent margins use the full viewport and only a closed axis falls back', () => {
  assert.deepEqual(viewportAreaFromLayout(800, 600, { left: 0, right: 0, top: 0, bottom: 0 }).free, {
    x0: 0, y0: 0, x1: 800, y1: 600,
  });
  assert.deepEqual(viewportAreaFromLayout(500, 600, { left: 276, right: 276, top: 82, bottom: 176 }).free, {
    x0: 0, y0: 82, x1: 500, y1: 424,
  });
});
