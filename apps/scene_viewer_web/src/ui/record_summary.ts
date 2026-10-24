import { selectWheelSpin, type WheelSpinBinding } from '../scene/pose_interpolation.ts';
import type { RecordedSamples } from '../record/recorded_samples.ts';
import type { VehicleReadoutModel } from './vehicle_readout_model.ts';
import { niceCeiling } from './format.ts';

export interface ContactSpan {
  startFrameIndex: number;
  endFrameIndex: number;
  patches: number;
  code: string;
}

export interface ReadoutScales { forceKn: number; bodyLateralMm: number; bodyYawMrad: number }

/** Offline-only statistics. Current readings never depend on this history. */
export interface RecordSummary {
  scales: ReadoutScales;
  contactSpans: ContactSpan[];
  carbodyStation: Float64Array;
  sampleIntervalSeconds: number;
  maximumWheelRotationRadians: number;
  hasMissingWheelSpinAngles: boolean;
}

export function summarizeRecord(samples: RecordedSamples, model: VehicleReadoutModel): RecordSummary {
  const { wheels, carbody, bogies } = model;
  // Record-wide ceilings for bars; only valid samples count.
  let hasMissingWheelSpinAngles = false;
  const spinCandidatesByBody = new Map<number, WheelSpinBinding[]>();
  for (const wheel of wheels) {
    let candidates = spinCandidatesByBody.get(wheel.wheelBodyIndex);
    if (candidates === undefined) {
      candidates = [];
      spinCandidatesByBody.set(wheel.wheelBodyIndex, candidates);
    }
    candidates.push({ wheelPlacementIndex: wheel.placementIndex, spinAxisInBody: wheel.spinAxisInWheelBodyFrame });
  }
  let forceMax = 0;
  let lateralMax = 0;
  let yawMax = 0;
  for (let frameIndex = 0; frameIndex < samples.frameCount; ++frameIndex) {
    const sample = samples.viewAt(frameIndex);
    for (const wheel of wheels) {
      const force = model.read(sample, wheel.verticalSupportForceScalar);
      if (force?.valid === true) {
        forceMax = Math.max(forceMax, Math.abs(force.value));
      }
    }
    for (const body of [carbody, ...bogies]) {
      if (body === null) {
        continue;
      }
      const lateral = model.read(sample, body.lateralScalar);
      const yaw = model.read(sample, body.yawScalar);
      if (lateral?.valid === true) {
        lateralMax = Math.max(lateralMax, Math.abs(lateral.value));
      }
      if (yaw?.valid === true) {
        yawMax = Math.max(yawMax, Math.abs(yaw.value));
      }
    }
  }

  const contactSpans: ContactSpan[] = [];
  for (const wheel of wheels) {
    let open: ContactSpan | null = null;
    for (let frameIndex = 0; frameIndex < samples.frameCount; ++frameIndex) {
      const patches = model.read(samples.viewAt(frameIndex), wheel.contactPatchCountScalar);
      // A sample that is not ready is no event and ends any open span.
      if (patches?.valid !== true) {
        open = null;
        continue;
      }
      const count = patches.value;
      if (count !== 1) {
        if (open !== null && open.patches === count && open.endFrameIndex === frameIndex - 1) {
          open.endFrameIndex = frameIndex;
        } else {
          open = { startFrameIndex: frameIndex, endFrameIndex: frameIndex, patches: count, code: wheel.code };
          contactSpans.push(open);
        }
      }
    }
  }
  contactSpans.sort((a, b) => a.startFrameIndex - b.startFrameIndex);

  const intervals: number[] = [];
  for (let frameIndex = 1; frameIndex < Math.min(samples.frameCount, 200); ++frameIndex) {
    intervals.push(samples.timeAt(frameIndex) - samples.timeAt(frameIndex - 1));
  }
  intervals.sort((a, b) => a - b);
  const carbodyStation = Float64Array.from({ length: samples.frameCount }, (_, index) =>
    model.stationAt(samples.viewAt(index)) ?? Number.NaN);
  let maximumWheelRotationRadians = 0;
  for (let index = 1; index < samples.frameCount; ++index) {
    const sample = samples.viewAt(index);
    const previous = samples.viewAt(index - 1);
    const dt = sample.timeSeconds - previous.timeSeconds;
    for (const [bodyIndex, candidates] of spinCandidatesByBody) {
      if (selectWheelSpin(previous, sample, candidates) === null) hasMissingWheelSpinAngles = true;
      maximumWheelRotationRadians = Math.max(maximumWheelRotationRadians,
        Math.hypot(...sample.bodyAngularVelocity(bodyIndex)) * dt);
    }
  }
  return {
    // Display floors keep numerical noise on a quiet record from filling a bar.
    scales: {
      forceKn: niceCeiling(Math.max(10, forceMax / 1000)),
      bodyLateralMm: niceCeiling(Math.max(1, lateralMax * 1000)),
      bodyYawMrad: niceCeiling(Math.max(1, yawMax * 1000)),
    },
    contactSpans, carbodyStation, maximumWheelRotationRadians, hasMissingWheelSpinAngles,
    sampleIntervalSeconds: intervals[intervals.length >> 1] ?? 0,
  };
}
