import { parseSceneDefinition, type SceneDefinition } from './scene_definition.ts';
import { RecordedSamples, type FrameColumns } from './recorded_samples.ts';
import { composeSceneDefinition, parseSceneAssets, type SceneAssets } from './scene_assets.ts';
import * as field from './json_fields.ts';

export interface SceneRecord { definition: SceneDefinition; samples: RecordedSamples; }

export function parseRecordEnvelope(value: unknown): Record<string, unknown> {
  const scene = field.object(value, 'scene.json');
  field.exact(scene['schema'], 'orvd.scene_record', 'scene.json.schema');
  if ('record' in scene || 'bodies' in scene || 'track' in scene || 'world_frame' in scene || 'visual_definition_file' in scene) {
    throw new Error('scene.json: legacy record fields are not supported');
  }
  return scene;
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
  sceneDocument: unknown,
  framesBuffer: ArrayBuffer,
  statusesBuffer: ArrayBuffer,
  visualDefinitionText: string | null,
  bundleAssets: SceneAssets | null = null,
): SceneRecord {
  const scene = parseRecordEnvelope(sceneDocument);
  const description = parseSceneDefinition(scene['description']);
  const { bodies, wheelPlacements, scalars } = description;
  const definition = composeSceneDefinition(description, parseSceneAssets(scene['assets'], visualDefinitionText), bundleAssets);
  const frameTable = field.object(scene['frame_table'], 'frame_table');
  const columnsJson = field.object(frameTable['columns'], 'columns');
  const bodyColumns = field.object(columnsJson['body_states'], 'body_states');
  const spinColumns = field.object(columnsJson['wheel_spin_angles'], 'wheel_spin_angles');
  const scalarColumns = field.object(columnsJson['scalar_values'], 'scalar_values');
  const columns: FrameColumns = {
    timeSecondsColumnOffset: field.integer(columnsJson['time_seconds'], 'columns.time_seconds'),
    timeNanosecondsColumnOffset: field.integer(columnsJson['time_nanoseconds'], 'columns.time_nanoseconds'),
    sampleIndexColumnOffset: field.integer(columnsJson['sample_index'], 'columns.sample_index'),
    phaseColumnOffset: field.integer(columnsJson['phase'], 'columns.phase'),
    bodyStatesColumnOffset: field.integer(bodyColumns['offset'], 'body_states.offset'),
    valuesPerBody: field.integer(bodyColumns['values_per_body'], 'body_states.values_per_body'),
    wheelSpinAnglesColumnOffset: field.integer(spinColumns['offset'], 'wheel_spin_angles.offset'),
    wheelSpinAngleCount: field.integer(spinColumns['count'], 'wheel_spin_angles.count'),
    scalarValuesColumnOffset: field.integer(scalarColumns['offset'], 'scalar_values.offset'),
    scalarCount: field.integer(scalarColumns['count'], 'scalar_values.count'),
    rowValueCount: field.integer(frameTable['row_value_count'], 'row_value_count'),
  };
  const frameCount = field.integer(frameTable['frame_count'], 'frame_count');
  const bodyCount = field.integer(bodyColumns['body_count'], 'body_states.body_count');
  if (
    frameCount === 0 || columns.timeSecondsColumnOffset !== 0 || columns.timeNanosecondsColumnOffset !== 1 ||
    columns.sampleIndexColumnOffset !== 2 || columns.phaseColumnOffset !== 3 || columns.bodyStatesColumnOffset !== 4 ||
    bodyCount !== bodies.length ||
    columns.valuesPerBody !== 13 ||
    columns.wheelSpinAnglesColumnOffset !== columns.bodyStatesColumnOffset + columns.valuesPerBody * bodyCount ||
    (columns.wheelSpinAngleCount !== 0 && columns.wheelSpinAngleCount !== wheelPlacements.length) ||
    columns.scalarCount !== scalars.length ||
    columns.scalarValuesColumnOffset !== columns.wheelSpinAnglesColumnOffset + columns.wheelSpinAngleCount ||
    columns.rowValueCount !== columns.scalarValuesColumnOffset + columns.scalarCount
  ) {
    throw new Error('scene.json: the frame table layout disagrees with the bodies, wheels or scalars');
  }
  const values = decodeLittleEndianDoubles(framesBuffer);
  if (values.length !== frameCount * columns.rowValueCount) {
    throw new Error(`frames file holds ${values.length} values, expected ${frameCount * columns.rowValueCount}`);
  }
  const statuses = new Uint8Array(statusesBuffer);
  if (statuses.length !== frameCount * columns.scalarCount) {
    throw new Error('scalar status file length disagrees with the frame and scalar counts');
  }
  const statusTable = field.object(scene['scalar_status_table'], 'scalar_status_table');
  if (field.integer(statusTable['frame_count'], 'status frame_count') !== frameCount || field.integer(statusTable['scalar_count'], 'status scalar_count') !== scalars.length) {
    throw new Error('scalar status table layout disagrees with the frame table');
  }
  if (statuses.some((status) => status > 2)) throw new Error('scalar status table contains an unknown status');
  const samples = new RecordedSamples(columns, values, statuses, description);
  for (let index = 0; index < frameCount; ++index) {
    const time = samples.timeAt(index);
    if (!Number.isFinite(time) || (index > 0 && time <= samples.timeAt(index - 1))) {
      throw new Error(`frame ${index} time must be finite and strictly increasing`);
    }
  }

  return { definition, samples };
}
