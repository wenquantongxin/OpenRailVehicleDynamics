// Reads a scene record directory with the viewer's own parser, resolves its
// display bindings the way the viewer does, and prints what it contains.
// Exits non-zero when the record is incomplete or inconsistent.
// Usage: node --experimental-strip-types scripts/check_record.ts <record-directory>

import { readFile } from 'node:fs/promises';
import { join } from 'node:path';

import { parseSceneRecord } from '../src/record/scene_record.ts';
import { parseVisualDefinition } from '../src/record/visual_definition.ts';
import { resolveVehicleDisplayBindings } from '../src/scene/vehicle_display_bindings.ts';

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
  const visualDefinition = visualText === null ? null : parseVisualDefinition(visualText, new Set(record.definition.bodies.map((body) => body.name)));
  const bindings = resolveVehicleDisplayBindings(record.definition, visualDefinition);
  const lastFrameIndex = record.samples.frameCount - 1;
  // The reference body is the bound carbody; without one, the first body, which is a slot, not a role.
  const referenceBodyIndex = bindings.carbody?.bodyIndex ?? 0;
  const first = record.samples.viewAt(0).bodyPose(referenceBodyIndex);
  const final = record.samples.viewAt(lastFrameIndex).bodyPose(referenceBodyIndex);
  console.log(
    JSON.stringify(
      {
        frames: record.samples.frameCount,
        bodies: record.definition.bodies.length,
        wheels: record.definition.wheelPlacements.length,
        scalars: record.definition.scalars.length,
        track_stations: record.definition.track?.stationsMeters.length ?? 0,
        visual_definition: visualText !== null,
        carbody: bindings.carbody?.bodyName ?? null,
        bogies: bindings.bogies.map((bogie) => ({ body: bogie.bodyName, members: bogie.memberBodyIndices.length })),
        carriers: bindings.carriers.map((carrier) => ({
          body: carrier.bodyName,
          left: carrier.leftWheelPlacementIndex === null ? null : record.definition.wheelPlacements[carrier.leftWheelPlacementIndex]?.interfaceName,
          right: carrier.rightWheelPlacementIndex === null ? null : record.definition.wheelPlacements[carrier.rightWheelPlacementIndex]?.interfaceName,
          bogie: carrier.bogieIndex,
        })),
        time_span_seconds: [record.samples.timeAt(0), record.samples.timeAt(lastFrameIndex)],
        reference_body: record.definition.bodies[referenceBodyIndex]?.name,
        first_position: first.position,
        last_position: final.position,
        last_orientation_wxyz: final.orientationWxyz,
        first_scalar: record.definition.scalars[0] ? { name: record.definition.scalars[0].name, ...record.samples.viewAt(lastFrameIndex).scalar(0) } : null,
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
