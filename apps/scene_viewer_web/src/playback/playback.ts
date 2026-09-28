import { frameIndexAtOrBefore, frameTimeSeconds, type SceneRecord } from '../record/scene_record.ts';

// The display clock. It advances with wall time scaled by the playback rate,
// clamps to the record's time span, and resolves a display time to the two
// bracketing frame rows. It never touches the record's own sample times.

export interface FrameBracket {
  firstFrameIndex: number;
  secondFrameIndex: number;
  alpha: number;
}

export class PlaybackController {
  timeSeconds: number;
  playing = false;
  speed = 1;
  readonly startSeconds: number;
  readonly endSeconds: number;
  private readonly record: SceneRecord;

  constructor(record: SceneRecord) {
    this.record = record;
    this.startSeconds = frameTimeSeconds(record, 0);
    this.endSeconds = frameTimeSeconds(record, record.frameCount - 1);
    this.timeSeconds = this.startSeconds;
  }

  advance(wallDeltaSeconds: number): void {
    if (!this.playing) {
      return;
    }
    this.timeSeconds += wallDeltaSeconds * this.speed;
    if (this.timeSeconds >= this.endSeconds) {
      this.timeSeconds = this.endSeconds;
      this.playing = false;
    }
    if (this.timeSeconds < this.startSeconds) {
      this.timeSeconds = this.startSeconds;
      this.playing = false;
    }
  }

  seek(timeSeconds: number): void {
    this.timeSeconds = Math.min(this.endSeconds, Math.max(this.startSeconds, timeSeconds));
  }

  /** The one play/pause rule: pausing stops; playing at the end restarts from the beginning. */
  togglePlay(): void {
    if (this.playing) {
      this.playing = false;
      return;
    }
    if (this.timeSeconds >= this.endSeconds) {
      this.timeSeconds = this.startSeconds;
    }
    this.playing = true;
  }

  bracket(): FrameBracket {
    const firstFrameIndex = Math.max(0, frameIndexAtOrBefore(this.record, this.timeSeconds));
    const secondFrameIndex = Math.min(this.record.frameCount - 1, firstFrameIndex + 1);
    if (secondFrameIndex === firstFrameIndex) {
      return { firstFrameIndex, secondFrameIndex, alpha: 0 };
    }
    const first = frameTimeSeconds(this.record, firstFrameIndex);
    const second = frameTimeSeconds(this.record, secondFrameIndex);
    const alpha = Math.min(1, Math.max(0, (this.timeSeconds - first) / (second - first)));
    return { firstFrameIndex, secondFrameIndex, alpha };
  }
}
