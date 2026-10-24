import type { DisplayDriver, DisplayInterval } from '../playback/display_driver.ts';
import type { LiveSample } from './live_samples.ts';

const maximumSamples = 512;
const maximumBytes = 8 * 1024 * 1024;
const delaySeconds = 0.1;
export const staleAfterMilliseconds = 2000;

export class LiveController implements DisplayDriver {
  private samples: LiveSample[] = [];
  private bytes = 0;
  private displayed: DisplayInterval;
  private lastArrival: number;
  private paused = false;
  private disconnected = false;
  private failed = false;
  private readonly runId: bigint;
  private readonly now: () => number;

  constructor(first: LiveSample, now: () => number = () => performance.now()) {
    this.runId = first.identity.runId;
    this.now = now;
    this.lastArrival = now();
    this.displayed = { first, second: first, alpha: 0, displayTimeSeconds: first.timeSeconds };
    this.append(first);
  }
  get playing() { return !this.paused && !this.disconnected && !this.failed; }
  get isPaused() { return this.paused; }
  get isStale() { return this.now() - this.lastArrival >= staleAfterMilliseconds; }
  get sampleCount() { return this.samples.length; }
  get bufferedBytes() { return this.bytes; }
  get hasError() { return this.failed; }

  append(sample: LiveSample): boolean {
    if (this.failed) return false;
    if (sample.identity.runId !== this.runId) throw new Error('a live buffer belongs to one run');
    const latest = this.samples.at(-1);
    if (latest !== undefined) {
      if (sample.identity.sequence <= latest.identity.sequence) return false;
      if (sample.timeSeconds < latest.timeSeconds) {
        this.failed = true;
        throw new Error('Simulation time moved backwards; waiting for a new run. 仿真时间倒退，等待新运行。');
      }
      if (sample.timeSeconds === latest.timeSeconds) {
        this.bytes -= this.samples.pop()!.byteLength;
      }
    }
    if (sample.byteLength > maximumBytes) throw new Error('a state exceeds the live buffer limit');
    this.samples.push(sample);
    this.bytes += sample.byteLength;
    while (this.samples.length > maximumSamples || this.bytes > maximumBytes) this.bytes -= this.samples.shift()!.byteLength;
    this.lastArrival = this.now();
    return true;
  }
  disconnect(): void {
    this.disconnected = true;
    this.samples = [];
    this.bytes = 0;
  }
  /** Seed a new connection after its persistent sequence and time watermark checks. */
  reconnect(first: LiveSample): boolean {
    if (this.failed) return false;
    if (first.identity.runId !== this.runId) throw new Error('a live buffer belongs to one run');
    this.disconnect();
    this.append(first);
    this.disconnected = false;
    if (!this.paused) this.displayed = this.resolve(first.timeSeconds);
    return true;
  }
  pause(): void { this.paused = true; }
  resume(): void {
    this.paused = false;
    if (this.samples.length > 0) {
      this.displayed = this.resolve(Math.max(this.displayed.displayTimeSeconds, this.samples.at(-1)!.timeSeconds - delaySeconds));
    }
  }
  advance(_wallDeltaSeconds: number): void {
    if (!this.playing || this.samples.length === 0) return;
    const latest = this.samples.at(-1)!;
    const target = Math.min(latest.timeSeconds, latest.timeSeconds + (this.now() - this.lastArrival) / 1000 - delaySeconds);
    this.displayed = this.resolve(Math.max(this.displayed.displayTimeSeconds, target));
  }
  interval(): DisplayInterval { return this.displayed; }
  private resolve(time: number): DisplayInterval {
    const firstAvailable = this.samples[0]!;
    const latest = this.samples.at(-1)!;
    const displayTimeSeconds = Math.max(firstAvailable.timeSeconds, Math.min(latest.timeSeconds, time));
    let left = 0;
    let right = this.samples.length - 1;
    while (left < right) {
      const middle = (left + right + 1) >> 1;
      if (this.samples[middle]!.timeSeconds <= displayTimeSeconds) left = middle;
      else right = middle - 1;
    }
    const first = this.samples[left]!;
    const second = this.samples[Math.min(left + 1, this.samples.length - 1)]!;
    const alpha = second === first ? 0 : (displayTimeSeconds - first.timeSeconds) / (second.timeSeconds - first.timeSeconds);
    return { first, second, alpha, displayTimeSeconds };
  }
}
