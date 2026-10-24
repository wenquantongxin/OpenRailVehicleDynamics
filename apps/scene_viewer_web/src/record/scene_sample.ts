export const ScalarStatus = { notReady: 0, valid: 1, placeholder: 2 } as const;

export interface BodyPose {
  position: [number, number, number];
  /** w, x, y, z; body to world. */
  orientationWxyz: [number, number, number, number];
}

export interface ScalarSample {
  value: number;
  status: number;
}

export type SceneSampleIdentity =
  | { readonly kind: 'record'; readonly timeNanoseconds: number; readonly sampleIndex: number; readonly phase: number }
  | { readonly kind: 'stream'; readonly runId: bigint; readonly sequence: bigint };

/** A borrowed sample; its binding remains stable while other samples are read. */
export interface SceneSampleView {
  readonly timeSeconds: number;
  readonly identity: SceneSampleIdentity;
  bodyPose(bodyIndex: number): BodyPose;
  bodyLinearVelocity(bodyIndex: number): [number, number, number];
  bodyAngularVelocity(bodyIndex: number): [number, number, number];
  wheelSpinAngle(wheelPlacementIndex: number): number | null;
  scalar(scalarIndex: number): ScalarSample;
}
