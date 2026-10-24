import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';
import { interpolateOrientation } from '../src/scene/pose_interpolation.ts';
import { LiveController } from '../src/live/live_controller.ts';
import { LiveSample, resolveLiveSpinBindings } from '../src/live/live_samples.ts';
import { LiveReadoutScales } from '../src/live/live_readout_scales.ts';
import { buildVehicleReadoutModel } from '../src/ui/vehicle_readout_model.ts';
import { resolveVehicleDisplayBindings } from '../src/scene/vehicle_display_bindings.ts';
import type { StateMessage } from '../src/stream/protocol.ts';
import { contractFixture } from './scene_contract_fixtures.ts';
import { makeRecord, placement } from './record_fixtures.ts';

const definition = contractFixture().record.definition;
const spins = resolveLiveSpinBindings(definition);
function message(time: number, sequence: bigint, bodies = 1): StateMessage {
  const bodyValues = new Float64Array(bodies * 13);
  bodyValues[0] = time;
  bodyValues[3] = 1;
  return { kind: 'state', header: { kind: 2, runId: 2n ** 63n + 9n, sequence, simulationTimeSeconds: time,
    fragmentIndex: 0, fragmentCount: 1, payloadBytes: 121 }, bytes: new Uint8Array(),
    state: { bodyCount: bodies, scalarCount: 1, bodyValues, scalarValues: new Float64Array([4 * Math.PI]), scalarStatuses: new Uint8Array([1]) } };
}
function sample(time: number, sequence = BigInt(Math.round(time * 1000)), bodies = 1) { return new LiveSample(message(time, sequence, bodies), spins); }

test('live samples preserve uint64 identities, own values and bind signed scalar angles', () => {
  const input = message(3.5, 2n ** 53n + 7n);
  const current = new LiveSample(input, spins);
  input.state.bodyValues[0] = 999;
  input.state.scalarStatuses[0] = 0;
  assert.equal(current.bodyPose(0).position[0], 3.5);
  assert.deepEqual(current.identity, { kind: 'stream', runId: 2n ** 63n + 9n, sequence: 2n ** 53n + 7n });
  assert.equal('timeNanoseconds' in current.identity, false);
  assert.equal(current.wheelSpinAngle(0), -4 * Math.PI);
  input.state.scalarValues[0] = 0;
  assert.equal(new LiveSample(input, spins).wheelSpinAngle(0), null);
  const positive = new LiveSample(message(0, 0n), [{ index: 0, sign: 1 }]);
  assert.equal(positive.wheelSpinAngle(0), 4 * Math.PI);
});

test('live delay brackets received times, never extrapolates and marks stale without changing values', () => {
  let wall = 0;
  const controller = new LiveController(sample(0), () => wall);
  wall = 200;
  controller.append(sample(0.2));
  controller.advance(0.2);
  assert.equal(controller.interval().displayTimeSeconds, 0.1);
  assert.equal(controller.interval().alpha, 0.5);
  wall = 5000;
  controller.advance(4.8);
  assert.equal(controller.interval().displayTimeSeconds, 0.2);
  assert.equal(controller.interval().first.scalar(0).status, 1);
  assert.equal(controller.isStale, true);
});

test('pause retains one interval while receiving and eviction stay bounded; resume returns live', () => {
  let wall = 0;
  const controller = new LiveController(sample(0), () => wall);
  wall = 200;
  controller.append(sample(0.2));
  controller.advance(0.2);
  controller.pause();
  const frozen = controller.interval();
  for (let index = 21; index <= 1200; ++index) {
    wall = index * 10;
    controller.append(sample(index / 100, BigInt(index + 1000)));
    controller.advance(0.01);
  }
  assert.equal(controller.interval(), frozen);
  assert.equal(frozen.first.timeSeconds, 0);
  assert.equal(controller.sampleCount, 512);
  assert.ok(controller.bufferedBytes <= 8 * 1024 * 1024);
  controller.resume();
  assert.equal(controller.interval().displayTimeSeconds, 11.9);
  controller.disconnect();
  assert.equal(controller.sampleCount, 0);
  const disconnected = controller.interval();
  controller.advance(5);
  assert.equal(controller.interval(), disconnected);
});

test('reconnect seeds a fresh interval and arrival clock without interpolating across the outage', () => {
  let wall = 0;
  const controller = new LiveController(sample(0), () => wall);
  wall = 200;
  controller.append(sample(0.2));
  controller.advance(0.2);
  const before = controller.interval();
  controller.disconnect();
  wall = 5000;
  controller.advance(4.8);
  assert.equal(controller.interval(), before);
  assert.equal(controller.playing, false);
  assert.equal(controller.sampleCount, 0);
  assert.equal(controller.bufferedBytes, 0);
  assert.equal(controller.isStale, true);

  const first = sample(10);
  assert.equal(controller.reconnect(first), true);
  assert.equal(controller.playing, true);
  assert.equal(controller.isStale, false);
  assert.equal(controller.sampleCount, 1);
  assert.equal(controller.bufferedBytes, first.byteLength);
  assert.deepEqual(controller.interval(), { first, second: first, alpha: 0, displayTimeSeconds: 10 });
  wall = 5100;
  controller.append(sample(10.2));
  controller.advance(0.1);
  assert.equal(controller.interval().first, first);
  assert.equal(controller.interval().displayTimeSeconds, 10.1);
  wall = 10000;
  controller.advance(4.9);
  assert.equal(controller.interval().displayTimeSeconds, 10.2);
});

test('paused reconnect keeps the same displayed interval while the fresh buffer remains bounded', () => {
  let wall = 0;
  const controller = new LiveController(sample(0), () => wall);
  wall = 200;
  controller.append(sample(0.2));
  controller.advance(0.2);
  controller.pause();
  const frozen = controller.interval();
  controller.disconnect();
  wall = 5000;
  assert.equal(controller.reconnect(sample(10)), true);
  assert.equal(controller.interval(), frozen);
  assert.equal(controller.isPaused, true);
  assert.equal(controller.playing, false);
  assert.equal(controller.isStale, false);
  for (let index = 1; index <= 600; ++index) {
    wall += 10;
    controller.append(sample(10 + index / 100));
    controller.advance(0.01);
  }
  assert.equal(controller.interval(), frozen);
  assert.equal(controller.sampleCount, 512);
  assert.ok(controller.bufferedBytes <= 8 * 1024 * 1024);
  assert.equal(frozen.first.timeSeconds, 0);
  controller.resume();
  assert.equal(controller.playing, true);
  assert.equal(controller.interval().displayTimeSeconds, 15.9);
  assert.ok(controller.interval().first.timeSeconds >= 10);
});

test('reconnect accepts the connection-validated cached state and keeps the buffer in its original run', () => {
  const first = sample(5, 50n);
  const controller = new LiveController(first);
  controller.disconnect();
  const otherRun = message(5, 50n);
  otherRun.header.runId += 1n;
  assert.throws(() => controller.reconnect(new LiveSample(otherRun, spins)), /one run/);
  assert.equal(controller.sampleCount, 0);
  assert.equal(controller.playing, false);
  assert.equal(controller.reconnect(first), true);
  assert.equal(controller.interval().first, first);
  assert.equal(controller.append(sample(5, 50n)), false);
  assert.equal(controller.append(sample(5, 51n)), true);
  controller.advance(0);
  assert.deepEqual(controller.interval().first.identity, { kind: 'stream', runId: first.identity.runId, sequence: 51n });
  controller.disconnect();
  assert.throws(() => controller.append(new LiveSample(otherRun, spins)), /one run/);
});

test('byte bound evicts large bodies and held samples stay intact', () => {
  const first = sample(0, 0n, 1000);
  const controller = new LiveController(first);
  for (let index = 1; index < 100; ++index) controller.append(sample(index, BigInt(index), 1000));
  assert.ok(controller.sampleCount < 100);
  assert.ok(controller.bufferedBytes <= 8 * 1024 * 1024);
  assert.equal(first.bodyPose(0).position[0], 0);
});

test('pause and resume never rewind an already displayed time when no newer state arrives', () => {
  let wall = 0;
  const controller = new LiveController(sample(9), () => wall);
  wall = 1000;
  controller.append(sample(10));
  wall = 1100;
  controller.advance(0.1);
  assert.equal(controller.interval().displayTimeSeconds, 10);
  controller.pause();
  wall = 1200;
  controller.resume();
  assert.equal(controller.interval().displayTimeSeconds, 10);
  controller.pause();
  controller.append(sample(10.04));
  controller.resume();
  assert.equal(controller.interval().displayTimeSeconds, 10);
});

test('same-time higher sequence replaces without zero intervals; backwards time freezes until a new controller', () => {
  const controller = new LiveController(sample(1, 10n));
  const old = controller.interval().first;
  assert.equal(controller.append(sample(1, 9n)), false);
  assert.equal(controller.append(sample(1, 11n)), true);
  controller.advance(1);
  assert.equal(controller.sampleCount, 1);
  assert.equal(controller.interval().first.identity.kind, 'stream');
  assert.equal(old.identity.kind === 'stream' && old.identity.sequence, 10n);
  const before = controller.interval();
  assert.throws(() => controller.append(sample(0.9, 12n)), /backwards/);
  controller.advance(1);
  assert.equal(controller.interval(), before);
  assert.equal(controller.hasError, true);
  assert.equal(controller.append(sample(2, 13n)), false);
  controller.disconnect();
  assert.equal(controller.reconnect(sample(2, 13n)), false);
  assert.equal(controller.interval(), before);
  assert.equal(controller.sampleCount, 0);
  assert.equal(controller.playing, false);
  assert.equal(controller.hasError, true);
});

test('live scales keep only valid per-run extrema and retain existing display floors', () => {
  const record = makeRecord({ bodies: ['wheel'], placements: [placement('contact', 'wheel', 'wheel', 'left')],
    scalars: ['contact.vertical_support_force_on_wheel_newtons'],
    frames: [{ timeSeconds: 0, scalarValues: [120000] }, { timeSeconds: 1, scalarValues: [0] },
      { timeSeconds: 2, scalarValues: [999999], scalarStatuses: [0] }] });
  const model = buildVehicleReadoutModel(record.definition, null, resolveVehicleDisplayBindings(record.definition, null), record.samples.viewAt(0));
  const scales = new LiveReadoutScales(model);
  assert.deepEqual(scales.scales, { forceKn: 10, bodyLateralMm: 1, bodyYawMrad: 1 });
  for (let index = 0; index < 3; ++index) scales.observe(record.samples.viewAt(index));
  assert.equal(scales.scales.forceKn, 120);
});


test('live scalar wheel turns select the multi-turn pose branch without adding endpoint spin', () => {
  const a = message(0, 0n);
  a.state.scalarValues[0] = 0;
  const b = message(1, 1n);
  const first = new LiveSample(a, spins);
  const second = new LiveSample(b, spins);
  const out = new THREE.Quaternion();
  const bindings = [{ wheelPlacementIndex: 0, spinAxisInBody: definition.wheelPlacements[0]!.spinAxisInWheelBodyFrame }];
  interpolateOrientation(first, second, 0.125, 0, bindings, out);
  const expected = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, -1, 0), -Math.PI / 2);
  assert.ok(out.angleTo(expected) < 1e-7);
  interpolateOrientation(first, second, 1, 0, bindings, out);
  assert.deepEqual(out.toArray(), [0, 0, 0, 1]);
});
