import type { SceneSampleView } from '../record/scene_sample.ts';

export interface DisplayInterval {
  readonly first: SceneSampleView;
  readonly second: SceneSampleView;
  readonly alpha: number;
  readonly displayTimeSeconds: number;
}

/** Supplies display samples without exposing storage or transport to the renderer. */
export interface DisplayDriver {
  readonly playing: boolean;
  advance(wallDeltaSeconds: number): void;
  interval(): DisplayInterval;
}
