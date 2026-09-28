import assert from 'node:assert/strict';
import { test } from 'node:test';

import { TrackModel } from '../src/scene/track_model.ts';
import { scenarioSubtitle } from '../src/ui/subtitle.ts';
import { range, straightTable } from './track_fixtures.ts';

test('the title line handles a record of 300 000 frames', () => {
  const k = (s: number): number => (s < 50 ? 0 : s < 100 ? (s - 50) / 50 / 300 : 1 / 300);
  const track = new TrackModel(straightTable(range(0, 300, 0.5), k));
  const frames = 300_000;
  const stations = new Float64Array(frames);
  for (let frame = 0; frame < frames; ++frame) {
    stations[frame] = (250 * frame) / (frames - 1);
  }
  stations[17] = Number.NaN;
  const subtitle = scenarioSubtitle(stations, 60, track);
  assert.ok(subtitle !== null);
  assert.equal(subtitle.en, 'CURVE NEGOTIATION · R 300 m · CANT 0 mm · INITIAL SPEED 60.0 km/h');
});
