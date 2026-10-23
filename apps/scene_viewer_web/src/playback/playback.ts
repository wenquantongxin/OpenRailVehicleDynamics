import type { RecordedSamples } from '../record/recorded_samples.ts';

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
  private readonly samples: RecordedSamples;

  constructor(samples: RecordedSamples) {
    this.samples = samples;
    this.startSeconds = samples.timeAt(0);
    this.endSeconds = samples.timeAt(samples.frameCount - 1);
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
    const firstFrameIndex = Math.max(0, this.samples.frameIndexAtOrBefore(this.timeSeconds));
    const secondFrameIndex = Math.min(this.samples.frameCount - 1, firstFrameIndex + 1);
    if (secondFrameIndex === firstFrameIndex) {
      return { firstFrameIndex, secondFrameIndex, alpha: 0 };
    }
    const first = this.samples.timeAt(firstFrameIndex);
    const second = this.samples.timeAt(secondFrameIndex);
    const alpha = Math.min(1, Math.max(0, (this.timeSeconds - first) / (second - first)));
    return { firstFrameIndex, secondFrameIndex, alpha };
  }
}
