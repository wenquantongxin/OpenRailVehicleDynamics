// Reads a scene record directory with the viewer's own parser and prints what
// it contains. Exits non-zero when the record is incomplete or inconsistent.
// Usage: node --experimental-strip-types scripts/check_record.ts <record-directory>

import { readFile } from 'node:fs/promises';
import { join } from 'node:path';

import { bodyPose, frameTimeSeconds, parseSceneRecord, scalarSample } from '../src/record/scene_record.ts';

async function main(): Promise<void> {
  const directory = process.argv[2];
  if (directory === undefined) {
    throw new Error('usage: check_record.ts <record-directory>');
  }
  const sceneText = await readFile(join(directory, 'scene.json'), 'utf8');
  const scene = JSON.parse(sceneText) as Record<string, unknown>;
  const frameTable = scene['frame_table'] as Record<string, unknown>;
  const statusTable = scene['scalar_status_table'] as Record<string, unknown>;
  const visualFile = scene['visual_definition_file'];
  const toArrayBuffer = (buffer: Buffer): ArrayBuffer =>
    buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength) as ArrayBuffer;
  const frames = toArrayBuffer(await readFile(join(directory, String(frameTable['file']))));
  const statuses = toArrayBuffer(await readFile(join(directory, String(statusTable['file']))));
  const visualText = typeof visualFile === 'string' ? await readFile(join(directory, visualFile), 'utf8') : null;
  const record = parseSceneRecord(sceneText, frames, statuses, visualText);
  const last = record.frameCount - 1;
  const carbody = record.bodies.findIndex((body) => body.name === 'carbody');
  const bodyIndex = carbody >= 0 ? carbody : 0;
  const first = bodyPose(record, 0, bodyIndex);
  const final = bodyPose(record, last, bodyIndex);
  console.log(
    JSON.stringify(
      {
        frames: record.frameCount,
        bodies: record.bodies.length,
        wheels: record.wheelPlacements.length,
        scalars: record.scalars.length,
        track_stations: record.track?.stationsMeters.length ?? 0,
        visual_definition: visualText !== null,
        time_span_seconds: [frameTimeSeconds(record, 0), frameTimeSeconds(record, last)],
        reference_body: record.bodies[bodyIndex]?.name,
        first_position: first.position,
        last_position: final.position,
        last_orientation_wxyz: final.orientationWxyz,
        first_scalar: record.scalars[0] ? { name: record.scalars[0].name, ...scalarSample(record, last, 0) } : null,
      },
      null,
      2,
    ),
  );
}

main().catch((error: unknown) => {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(1);
});
