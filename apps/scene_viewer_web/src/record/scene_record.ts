// The scene record as written by ORVD::scene_record. The layout is read from
// scene.json, never assumed: column offsets, body count and scalar count all
// come from the file, and the frame table is refused when it disagrees.

export type WheelSide = 'left' | 'right';

export interface SceneBody {
  name: string;
  movesFreelyInWorld: boolean;
}

export interface WheelPlacement {
  interfaceName: string;
  wheelBodyName: string;
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

export interface FrameColumns {
  timeSeconds: number;
  timeNanoseconds: number;
  sampleIndex: number;
  phase: number;
  bodyOffset: number;
  valuesPerBody: number;
  /** Unwrapped spin angle per wheel placement; count 0 when the record has none. */
  wheelSpinOffset: number;
  wheelSpinCount: number;
  scalarOffset: number;
  scalarCount: number;
  rowValueCount: number;
}

export const ScalarStatus = { notReady: 0, valid: 1, placeholder: 2 } as const;
export const SamplePhaseNames: Record<number, string> = {
  0: 'initial accepted state',
  1: 'dense intermediate sample',
  2: 'accepted endpoint',
};

export interface BodyPose {
  position: [number, number, number];
  /** w, x, y, z; body to world. */
  orientationWxyz: [number, number, number, number];
}

export interface SceneRecord {
  worldFrame: string;
  bodies: SceneBody[];
  wheelPlacements: WheelPlacement[];
  scalars: ScalarDefinition[];
  visualDefinitionText: string | null;
  track: TrackTable | null;
  frameCount: number;
  columns: FrameColumns;
  /** The whole frame table, row-major, in host byte order. */
  values: Float64Array;
  statuses: Uint8Array;
  /** time_seconds of every frame, extracted once for seeking. */
  timesSeconds: Float64Array;
}

function requireNumber(value: unknown, what: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new Error(`scene.json: ${what} is not a finite number`);
  }
  return value;
}

function requireString(value: unknown, what: string): string {
  if (typeof value !== 'string') {
    throw new Error(`scene.json: ${what} is not a string`);
  }
  return value;
}

function requireArray(value: unknown, what: string): unknown[] {
  if (!Array.isArray(value)) {
    throw new Error(`scene.json: ${what} is not an array`);
  }
  return value;
}

function requireTriple(value: unknown, what: string): [number, number, number] {
  const array = requireArray(value, what);
  if (array.length !== 3) {
    throw new Error(`scene.json: ${what} does not have three entries`);
  }
  return [requireNumber(array[0], what), requireNumber(array[1], what), requireNumber(array[2], what)];
}

function numberRows(value: unknown, what: string): number[][] {
  return requireArray(value, what).map((row, index) =>
    requireArray(row, `${what}[${index}]`).map((entry) => requireNumber(entry, `${what}[${index}]`)),
  );
}

function hostIsLittleEndian(): boolean {
  return new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;
}

function decodeLittleEndianDoubles(buffer: ArrayBuffer): Float64Array {
  if (buffer.byteLength % 8 !== 0) {
    throw new Error('frames file length is not a multiple of eight bytes');
  }
  if (hostIsLittleEndian()) {
    return new Float64Array(buffer);
  }
  const view = new DataView(buffer);
  const out = new Float64Array(buffer.byteLength / 8);
  for (let index = 0; index < out.length; ++index) {
    out[index] = view.getFloat64(index * 8, true);
  }
  return out;
}

export function parseSceneRecord(
  sceneJsonText: string,
  framesBuffer: ArrayBuffer,
  statusesBuffer: ArrayBuffer,
  visualDefinitionText: string | null,
): SceneRecord {
  const scene = JSON.parse(sceneJsonText) as Record<string, unknown>;
  if (scene['record'] !== 'orvd.scene_record') {
    throw new Error('scene.json is not an ORVD scene record');
  }
  const bodies: SceneBody[] = requireArray(scene['bodies'], 'bodies').map((entry, index) => {
    const body = entry as Record<string, unknown>;
    return {
      name: requireString(body['name'], `bodies[${index}].name`),
      movesFreelyInWorld: body['moves_freely_in_world'] === true,
    };
  });
  if (bodies.length === 0) {
    throw new Error('scene.json names no rigid body');
  }
  const wheelPlacements: WheelPlacement[] = requireArray(scene['wheel_placements'], 'wheel_placements').map(
    (entry, index) => {
      const wheel = entry as Record<string, unknown>;
      const side = requireString(wheel['side'], `wheel_placements[${index}].side`);
      if (side !== 'left' && side !== 'right') {
        throw new Error(`scene.json: unknown wheel side '${side}'`);
      }
      return {
        interfaceName: requireString(wheel['interface_name'], 'interface_name'),
        wheelBodyName: requireString(wheel['wheel_body_name'], 'wheel_body_name'),
        side,
        datumInWheelBodyFrameMeters: requireTriple(wheel['datum_in_wheel_body_frame_meters'], 'datum'),
        spinAxisInWheelBodyFrame: requireTriple(wheel['spin_axis_in_wheel_body_frame'], 'spin axis'),
        nominalRollingRadiusMeters: requireNumber(wheel['nominal_rolling_radius_meters'], 'rolling radius'),
      };
    },
  );
  const scalars: ScalarDefinition[] = requireArray(scene['scalars'], 'scalars').map((entry) => {
    const definition = entry as Record<string, unknown>;
    return {
      name: requireString(definition['name'], 'scalar name'),
      unit: requireString(definition['unit'], 'scalar unit'),
      referenceFrame: requireString(definition['reference_frame'], 'scalar reference_frame'),
      quantity: requireString(definition['quantity'], 'scalar quantity'),
      method: requireString(definition['method'], 'scalar method'),
      sampleSemantics: requireString(definition['sample_semantics'], 'scalar sample_semantics'),
    };
  });

  const frameTable = scene['frame_table'] as Record<string, unknown>;
  const columnsJson = frameTable['columns'] as Record<string, unknown>;
  const bodyColumns = columnsJson['body_states'] as Record<string, unknown>;
  const spinColumns = columnsJson['wheel_spin_angles'] as Record<string, unknown>;
  const scalarColumns = columnsJson['scalar_values'] as Record<string, unknown>;
  const columns: FrameColumns = {
    timeSeconds: requireNumber(columnsJson['time_seconds'], 'columns.time_seconds'),
    timeNanoseconds: requireNumber(columnsJson['time_nanoseconds'], 'columns.time_nanoseconds'),
    sampleIndex: requireNumber(columnsJson['sample_index'], 'columns.sample_index'),
    phase: requireNumber(columnsJson['phase'], 'columns.phase'),
    bodyOffset: requireNumber(bodyColumns['offset'], 'body_states.offset'),
    valuesPerBody: requireNumber(bodyColumns['values_per_body'], 'body_states.values_per_body'),
    wheelSpinOffset: requireNumber(spinColumns['offset'], 'wheel_spin_angles.offset'),
    wheelSpinCount: requireNumber(spinColumns['count'], 'wheel_spin_angles.count'),
    scalarOffset: requireNumber(scalarColumns['offset'], 'scalar_values.offset'),
    scalarCount: requireNumber(scalarColumns['count'], 'scalar_values.count'),
    rowValueCount: requireNumber(frameTable['row_value_count'], 'row_value_count'),
  };
  const frameCount = requireNumber(frameTable['frame_count'], 'frame_count');
  const bodyCount = requireNumber(bodyColumns['body_count'], 'body_states.body_count');
  if (
    bodyCount !== bodies.length ||
    columns.valuesPerBody !== 13 ||
    columns.wheelSpinOffset !== columns.bodyOffset + columns.valuesPerBody * bodyCount ||
    (columns.wheelSpinCount !== 0 && columns.wheelSpinCount !== wheelPlacements.length) ||
    columns.scalarCount !== scalars.length ||
    columns.scalarOffset !== columns.wheelSpinOffset + columns.wheelSpinCount ||
    columns.rowValueCount !== columns.scalarOffset + columns.scalarCount
  ) {
    throw new Error('scene.json: the frame table layout disagrees with the bodies, wheels or scalars');
  }
  const values = decodeLittleEndianDoubles(framesBuffer);
  if (values.length !== frameCount * columns.rowValueCount) {
    throw new Error(
      `frames file holds ${values.length} values, expected ${frameCount * columns.rowValueCount}`,
    );
  }
  const statuses = new Uint8Array(statusesBuffer);
  if (statuses.length !== frameCount * columns.scalarCount) {
    throw new Error('scalar status file length disagrees with the frame and scalar counts');
  }
  const timesSeconds = new Float64Array(frameCount);
  for (let frame = 0; frame < frameCount; ++frame) {
    const time = values[frame * columns.rowValueCount + columns.timeSeconds];
    if (time === undefined || !Number.isFinite(time)) {
      throw new Error(`frame ${frame} has no finite time`);
    }
    if (frame > 0 && time <= (timesSeconds[frame - 1] as number)) {
      throw new Error(`frame ${frame} time does not increase`);
    }
    timesSeconds[frame] = time;
  }

  let track: TrackTable | null = null;
  const trackJson = scene['track'];
  if (trackJson !== null && trackJson !== undefined) {
    const table = trackJson as Record<string, unknown>;
    track = {
      stationsMeters: requireArray(table['stations_meters'], 'track.stations_meters').map((entry) =>
        requireNumber(entry, 'station'),
      ),
      centerlineInInertialMeters: numberRows(table['centerline_in_inertial_meters'], 'track.centerline'),
      rotationInertialFromTrackWxyz: numberRows(table['rotation_inertial_from_track_wxyz'], 'track.rotation'),
      curvatureRadiansPerMeter: requireArray(table['curvature_radians_per_meter'], 'track.curvature').map(
        (entry) => requireNumber(entry, 'curvature'),
      ),
      superelevationMeters: requireArray(table['superelevation_meters'], 'track.superelevation').map(
        (entry) => requireNumber(entry, 'superelevation'),
      ),
      leftRailDatumInInertialMeters: numberRows(table['left_rail_datum_in_inertial_meters'], 'track.left rail'),
      rightRailDatumInInertialMeters: numberRows(table['right_rail_datum_in_inertial_meters'], 'track.right rail'),
    };
    const count = track.stationsMeters.length;
    if (
      count === 0 ||
      track.centerlineInInertialMeters.length !== count ||
      track.leftRailDatumInInertialMeters.length !== count ||
      track.rightRailDatumInInertialMeters.length !== count
    ) {
      throw new Error('scene.json: the track table columns disagree in length');
    }
  }

  return {
    worldFrame: requireString(scene['world_frame'], 'world_frame'),
    bodies,
    wheelPlacements,
    scalars,
    visualDefinitionText,
    track,
    frameCount,
    columns,
    values,
    statuses,
    timesSeconds,
  };
}

export function frameTimeSeconds(record: SceneRecord, frame: number): number {
  return record.timesSeconds[frame] as number;
}

export function frameSampleIndex(record: SceneRecord, frame: number): number {
  return record.values[frame * record.columns.rowValueCount + record.columns.sampleIndex] as number;
}

export function framePhase(record: SceneRecord, frame: number): number {
  return record.values[frame * record.columns.rowValueCount + record.columns.phase] as number;
}

export function bodyPose(record: SceneRecord, frame: number, body: number): BodyPose {
  const base = frame * record.columns.rowValueCount + record.columns.bodyOffset + body * record.columns.valuesPerBody;
  const v = record.values;
  return {
    position: [v[base] as number, v[base + 1] as number, v[base + 2] as number],
    orientationWxyz: [v[base + 3] as number, v[base + 4] as number, v[base + 5] as number, v[base + 6] as number],
  };
}

/** The unwrapped spin angle of wheel placement `wheel`, or null when the record has none. */
export function wheelSpinAngle(record: SceneRecord, frame: number, wheel: number): number | null {
  if (record.columns.wheelSpinCount === 0) {
    return null;
  }
  return record.values[frame * record.columns.rowValueCount + record.columns.wheelSpinOffset + wheel] as number;
}

export function bodyAngularVelocity(record: SceneRecord, frame: number, body: number): [number, number, number] {
  const base = frame * record.columns.rowValueCount + record.columns.bodyOffset + body * record.columns.valuesPerBody;
  const v = record.values;
  return [v[base + 10] as number, v[base + 11] as number, v[base + 12] as number];
}

export interface ScalarSample {
  value: number;
  status: number;
}

export function scalarSample(record: SceneRecord, frame: number, scalar: number): ScalarSample {
  return {
    value: record.values[frame * record.columns.rowValueCount + record.columns.scalarOffset + scalar] as number,
    status: record.statuses[frame * record.columns.scalarCount + scalar] as number,
  };
}

/** The last frame whose time is not after `timeSeconds`; -1 before the first frame. */
export function frameAtOrBefore(record: SceneRecord, timeSeconds: number): number {
  const times = record.timesSeconds;
  let low = 0;
  let high = times.length - 1;
  if (timeSeconds < (times[0] as number)) {
    return -1;
  }
  while (low < high) {
    const middle = (low + high + 1) >> 1;
    if ((times[middle] as number) <= timeSeconds) {
      low = middle;
    } else {
      high = middle - 1;
    }
  }
  return low;
}
