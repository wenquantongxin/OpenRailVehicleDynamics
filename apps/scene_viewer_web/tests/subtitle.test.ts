import assert from 'node:assert/strict';
import { test } from 'node:test';

import { recordRangeSubtitle } from '../src/ui/subtitle.ts';

test('the title line handles a record of 300 000 frames', () => {
  const frames = 300_000;
  const stations = new Float64Array(frames);
  for (let frame = 0; frame < frames; ++frame) {
    stations[frame] = (250 * frame) / (frames - 1);
  }
  stations[17] = Number.NaN;
  const subtitle = recordRangeSubtitle(stations);
  assert.ok(subtitle !== null);
  assert.equal(subtitle.en, 'RECORDED RANGE · K0+000.0 – K0+250.0');
});

test('route coverage describes a long or reversing record without choosing one representative curve', () => {
  assert.deepEqual(recordRangeSubtitle(new Float64Array([20, 9360, 18656.364, 9000])), {
    en: 'RECORDED RANGE · K0+020.0 – K18+656.4', zh: '回放里程范围',
  });
  assert.equal(recordRangeSubtitle(new Float64Array([Number.NaN])), null);
});
