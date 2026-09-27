import assert from 'node:assert/strict';
import { test } from 'node:test';

import { PlaybackController } from '../src/playback/playback.ts';
import type { SceneRecord } from '../src/record/scene_record.ts';

// The controller owns the playing state. Reaching the end stops it; one
// toggle after seeking back resumes; one toggle at the end restarts.

function recordWithTimes(times: number[]): SceneRecord {
  return {
    frameCount: times.length,
    timesSeconds: Float64Array.from(times),
  } as unknown as SceneRecord;
}

test('reaching the end stops playback and one toggle resumes after a seek back', () => {
  const playback = new PlaybackController(recordWithTimes([0, 0.5, 1.0]));
  playback.togglePlay();
  assert.equal(playback.playing, true);
  playback.advance(5);
  assert.equal(playback.timeSeconds, 1.0);
  assert.equal(playback.playing, false);

  playback.seek(0.25);
  assert.equal(playback.playing, false);
  playback.togglePlay();
  assert.equal(playback.playing, true);
  playback.advance(0.1);
  assert.ok(Math.abs(playback.timeSeconds - 0.35) < 1e-12);
  assert.deepEqual(playback.bracket(), { frameA: 0, frameB: 1, alpha: 0.35 / 0.5 });
});

test('one toggle at the end restarts from the beginning', () => {
  const playback = new PlaybackController(recordWithTimes([0, 0.5, 1.0]));
  playback.seek(1.0);
  playback.togglePlay();
  assert.equal(playback.playing, true);
  assert.equal(playback.timeSeconds, 0);
  playback.speed = 2;
  playback.advance(0.1);
  assert.ok(Math.abs(playback.timeSeconds - 0.2) < 1e-12);
  playback.togglePlay();
  assert.equal(playback.playing, false);
});
