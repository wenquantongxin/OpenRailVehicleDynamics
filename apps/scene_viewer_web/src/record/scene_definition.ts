export type WheelSide = 'left' | 'right';

export interface SceneBody {
  name: string;
  movesFreelyInWorld: boolean;
}

export interface WheelPlacement {
  interfaceName: string;
  wheelBodyName: string;
  /**
   * The body carrying this wheel's non-spinning profile frame: the axle bridge
   * of an independently rotating wheel, or the wheelset body itself for a
   * rigid wheelset, in which case it equals `wheelBodyName`. The left and
   * right wheels of one carrier are paired through it.
   */
  carrierBodyName: string;
  side: WheelSide;
  datumInWheelBodyFrameMeters: [number, number, number];
  spinAxisInWheelBodyFrame: [number, number, number];
  nominalRollingRadiusMeters: number;
}

export interface ScalarDefinition {
  name: string;
  unit: string;
  referenceFrame: string;
  quantity: string;
  method: string;
  sampleSemantics: string;
}

export interface TrackTable {
  stationsMeters: number[];
  centerlineInInertialMeters: number[][];
  rotationInertialFromTrackWxyz: number[][];
  curvatureRadiansPerMeter: number[];
  superelevationMeters: number[];
  leftRailDatumInInertialMeters: number[][];
  rightRailDatumInInertialMeters: number[][];
}

/** Static scene data shared by rendering and sample sources. */
export interface SceneDefinition {
  worldFrame: string;
  bodies: SceneBody[];
  wheelPlacements: WheelPlacement[];
  scalars: ScalarDefinition[];
  visualDefinitionText: string | null;
  track: TrackTable | null;
}
