import type { SceneDescription } from './scene_definition.ts';
import { ScalarStatus, type BodyPose, type SceneSampleView, type ScalarSample } from './scene_sample.ts';

/** Column offsets within one row of the frame table. */
export interface FrameColumns {
  timeSecondsColumnOffset: number;
  timeNanosecondsColumnOffset: number;
  sampleIndexColumnOffset: number;
  phaseColumnOffset: number;
  bodyStatesColumnOffset: number;
  valuesPerBody: number;
  /** Unwrapped spin angle per wheel placement; the count is 0 when the record has none. */
  wheelSpinAnglesColumnOffset: number;
  wheelSpinAngleCount: number;
  scalarValuesColumnOffset: number;
  scalarCount: number;
  rowValueCount: number;
}

/** Packed offline history. Row indices and recorded sample identities are distinct. */
export class RecordedSamples {
  readonly frameCount: number;
  readonly timesSeconds: Float64Array;
  readonly columns: FrameColumns;
  readonly values: Float64Array;
  readonly statuses: Uint8Array;

  readonly wheelSpinScalarBindings: ReadonlyArray<{ scalarIndex: number; sign: 1 | -1 } | null>;

  constructor(columns: FrameColumns, values: Float64Array, statuses: Uint8Array, description?: Pick<SceneDescription, 'scalars' | 'wheelPlacements'>) {
    const scalarIndices = new Map(description?.scalars.map((scalar, index) => [scalar.name, index]));
    this.wheelSpinScalarBindings = description?.wheelPlacements.map((wheel) => {
      if (wheel.spinAngleScalar === null) return null;
      const scalarIndex = scalarIndices.get(wheel.spinAngleScalar.name);
      if (scalarIndex === undefined) throw new Error(`unknown wheel spin scalar '${wheel.spinAngleScalar.name}'`);
      return { scalarIndex, sign: wheel.spinAngleScalar.sign };
    }) ?? [];
    this.columns = columns;
    this.values = values;
    this.statuses = statuses;
    this.frameCount = values.length / columns.rowValueCount;
    this.timesSeconds = Float64Array.from({ length: this.frameCount }, (_, index) =>
      values[index * columns.rowValueCount + columns.timeSecondsColumnOffset] as number);
  }

  /** Whether this wheel has a dedicated column or a resolved scalar binding. */
  hasWheelSpinAngle(index: number): boolean {
    return (index >= 0 && index < this.columns.wheelSpinAngleCount) || this.wheelSpinScalarBindings[index] != null;
  }

  timeAt(frameIndex: number): number {
    return this.timesSeconds[frameIndex] as number;
  }

  viewAt(frameIndex: number): RecordedSampleView {
    if (!Number.isInteger(frameIndex) || frameIndex < 0 || frameIndex >= this.frameCount) {
      throw new RangeError(`sample row ${frameIndex} is outside the recording`);
    }
    return new RecordedSampleView(this, frameIndex);
  }

  /** Last row not after the requested time; -1 before the first row. */
  frameIndexAtOrBefore(timeSeconds: number): number {
    const times = this.timesSeconds;
    let low = 0;
    let high = times.length - 1;
    if (high < 0 || timeSeconds < (times[0] as number)) return -1;
    while (low < high) {
      const middle = (low + high + 1) >> 1;
      if ((times[middle] as number) <= timeSeconds) low = middle;
      else high = middle - 1;
    }
    return low;
  }
}

class RecordedSampleView implements SceneSampleView {
  private readonly samples: RecordedSamples;
  private readonly row: number;
  private readonly statusRow: number;

  constructor(samples: RecordedSamples, frameIndex: number) {
    this.samples = samples;
    this.row = frameIndex * samples.columns.rowValueCount;
    this.statusRow = frameIndex * samples.columns.scalarCount;
  }

  private at(offset: number): number { return this.samples.values[this.row + offset] as number; }
  get timeSeconds(): number { return this.at(this.samples.columns.timeSecondsColumnOffset); }
  get identity() {
    return { kind: 'record' as const,
      timeNanoseconds: this.at(this.samples.columns.timeNanosecondsColumnOffset),
      sampleIndex: this.at(this.samples.columns.sampleIndexColumnOffset),
      phase: this.at(this.samples.columns.phaseColumnOffset) };
  }
  private bodyStart(bodyIndex: number): number {
    return this.samples.columns.bodyStatesColumnOffset + bodyIndex * this.samples.columns.valuesPerBody;
  }
  bodyPose(bodyIndex: number): BodyPose {
    const b = this.bodyStart(bodyIndex);
    return { position: [this.at(b), this.at(b + 1), this.at(b + 2)],
      orientationWxyz: [this.at(b + 3), this.at(b + 4), this.at(b + 5), this.at(b + 6)] };
  }
  bodyLinearVelocity(bodyIndex: number): [number, number, number] {
    const b = this.bodyStart(bodyIndex) + 7;
    return [this.at(b), this.at(b + 1), this.at(b + 2)];
  }
  bodyAngularVelocity(bodyIndex: number): [number, number, number] {
    const b = this.bodyStart(bodyIndex) + 10;
    return [this.at(b), this.at(b + 1), this.at(b + 2)];
  }
  wheelSpinAngle(index: number): number | null {
    if (!Number.isInteger(index) || index < 0) return null;
    if (index < this.samples.columns.wheelSpinAngleCount) {
      const angle = this.at(this.samples.columns.wheelSpinAnglesColumnOffset + index);
      return Number.isFinite(angle) ? angle : null;
    }
    const binding = this.samples.wheelSpinScalarBindings[index];
    if (binding === undefined || binding === null) return null;
    const sample = this.scalar(binding.scalarIndex);
    return sample.status === ScalarStatus.valid && Number.isFinite(sample.value) ? binding.sign * sample.value : null;
  }
  scalar(index: number): ScalarSample {
    return { value: this.at(this.samples.columns.scalarValuesColumnOffset + index),
      status: this.samples.statuses[this.statusRow + index] as number };
  }
}
