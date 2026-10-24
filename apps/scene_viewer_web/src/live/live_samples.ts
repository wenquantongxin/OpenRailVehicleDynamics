import type { SceneDescription } from '../record/scene_definition.ts';
import { ScalarStatus, type BodyPose, type SceneSampleView } from '../record/scene_sample.ts';
import type { StateMessage } from '../stream/protocol.ts';

/** Resolved once per description; stream wheel angles come only from named scalars. */
export function resolveLiveSpinBindings(description: SceneDescription) {
  const indices = new Map(description.scalars.map((scalar, index) => [scalar.name, index]));
  return description.wheelPlacements.map((wheel) => wheel.spinAngleScalar === null ? null : {
    index: indices.get(wheel.spinAngleScalar.name)!, sign: wheel.spinAngleScalar.sign,
  });
}

/** Owns its values. Evicting or replacing a buffer slot never changes a held sample. */
export class LiveSample implements SceneSampleView {
  readonly timeSeconds: number;
  readonly identity: { readonly kind: 'stream'; readonly runId: bigint; readonly sequence: bigint };
  readonly byteLength: number;
  private readonly bodies: Float64Array;
  private readonly values: Float64Array;
  private readonly statuses: Uint8Array;
  private readonly spins: ReturnType<typeof resolveLiveSpinBindings>;

  constructor(message: StateMessage, spins: ReturnType<typeof resolveLiveSpinBindings>) {
    this.timeSeconds = message.header.simulationTimeSeconds;
    this.identity = Object.freeze({ kind: 'stream', runId: message.header.runId, sequence: message.header.sequence });
    this.bodies = message.state.bodyValues.slice();
    this.values = message.state.scalarValues.slice();
    this.statuses = message.state.scalarStatuses.slice();
    this.byteLength = 8 + this.bodies.byteLength + this.values.byteLength + this.statuses.byteLength;
    this.spins = spins;
  }
  private triple(offset: number): [number, number, number] {
    return [this.bodies[offset]!, this.bodies[offset + 1]!, this.bodies[offset + 2]!];
  }
  bodyPose(index: number): BodyPose {
    const offset = 13 * index;
    return { position: this.triple(offset), orientationWxyz: [this.bodies[offset + 3]!, this.bodies[offset + 4]!, this.bodies[offset + 5]!, this.bodies[offset + 6]!] };
  }
  bodyLinearVelocity(index: number) { return this.triple(13 * index + 7); }
  bodyAngularVelocity(index: number) { return this.triple(13 * index + 10); }
  scalar(index: number) { return { value: this.values[index]!, status: this.statuses[index]! }; }
  wheelSpinAngle(index: number): number | null {
    const binding = this.spins[index];
    if (binding === null || binding === undefined) return null;
    return this.statuses[binding.index] === ScalarStatus.valid ? binding.sign * this.values[binding.index]! : null;
  }
}
