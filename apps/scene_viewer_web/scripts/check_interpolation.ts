// Display interpolation checked against a real record: frames ten samples
// apart are interpolated to the time of the sample halfway between them and
// compared with that recorded sample. Wheels must land close to the recorded
// orientation when the record carries unwrapped spin angles; the same check
// without spin angles shows how far the shortest path goes wrong.
// Usage: node --experimental-strip-types scripts/check_interpolation.ts <record-directory>

import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import * as THREE from 'three';

import { bodyPose, frameTimeSeconds, parseSceneRecord } from '../src/record/scene_record.ts';
import { wheelSpinBindings } from '../src/scene/apply_frame.ts';
import { interpolateOrientation } from '../src/scene/pose_interpolation.ts';

function geodesicAngle(a: THREE.Quaternion, b: THREE.Quaternion): number {
  const dot = Math.min(1, Math.abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w));
  return 2 * Math.acos(dot);
}

async function main(): Promise<void> {
  const directory = process.argv[2];
  if (directory === undefined) {
    throw new Error('usage: check_interpolation.ts <record-directory>');
  }
  const sceneText = await readFile(join(directory, 'scene.json'), 'utf8');
  const scene = JSON.parse(sceneText) as Record<string, unknown>;
  const frameTable = scene['frame_table'] as Record<string, unknown>;
  const statusTable = scene['scalar_status_table'] as Record<string, unknown>;
  const toArrayBuffer = (buffer: Buffer): ArrayBuffer =>
    buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength) as ArrayBuffer;
  const record = parseSceneRecord(
    sceneText,
    toArrayBuffer(await readFile(join(directory, String(frameTable['file'])))),
    toArrayBuffer(await readFile(join(directory, String(statusTable['file'])))),
    null,
  );
  if (record.frameCount < 11) {
    throw new Error('the record needs at least eleven frames');
  }
  const frameA = 0;
  const frameB = 10;
  const frameMid = 5;
  const alpha =
    (frameTimeSeconds(record, frameMid) - frameTimeSeconds(record, frameA)) /
    (frameTimeSeconds(record, frameB) - frameTimeSeconds(record, frameA));
  const bindings = wheelSpinBindings(record);
  const interpolated = new THREE.Quaternion();
  const recorded = new THREE.Quaternion();
  let worstBody = 0;
  let worstWheelWithSpin = 0;
  let worstWheelShortestPath = 0;
  record.bodies.forEach((_body, index) => {
    const [w, x, y, z] = bodyPose(record, frameMid, index).orientationWxyz;
    recorded.set(x, y, z, w);
    const binding = bindings.get(index);
    interpolateOrientation(record, frameA, frameB, alpha, index, binding, interpolated);
    const error = geodesicAngle(interpolated, recorded);
    if (binding === undefined) {
      worstBody = Math.max(worstBody, error);
    } else {
      worstWheelWithSpin = Math.max(worstWheelWithSpin, error);
      interpolateOrientation(record, frameA, frameB, alpha, index, undefined, interpolated);
      worstWheelShortestPath = Math.max(worstWheelShortestPath, geodesicAngle(interpolated, recorded));
    }
  });
  const report = {
    frames: [frameA, frameMid, frameB],
    interval_seconds: frameTimeSeconds(record, frameB) - frameTimeSeconds(record, frameA),
    wheels_with_spin_angles: bindings.size,
    worst_non_wheel_error_radians: worstBody,
    worst_wheel_error_with_spin_angles_radians: worstWheelWithSpin,
    worst_wheel_error_shortest_path_radians: worstWheelShortestPath,
  };
  console.log(JSON.stringify(report, null, 2));
  if (bindings.size === 0) {
    throw new Error('the record carries no wheel spin angles; nothing to verify');
  }
  if (worstWheelWithSpin > 0.05 || worstBody > 0.05) {
    throw new Error('interpolated orientations differ from the recorded sample by more than 0.05 rad');
  }
}

main().catch((error: unknown) => {
  console.error(error instanceof Error ? error.message : String(error));
  process.exit(1);
});
