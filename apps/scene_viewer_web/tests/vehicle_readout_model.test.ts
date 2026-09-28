import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { ScalarStatus, frameIndexAtOrBefore, frameSampleIndex } from '../src/record/scene_record.ts';
import { parseVisualDefinition } from '../src/record/visual_definition.ts';
import { TrackModel } from '../src/scene/track_model.ts';
import { resolveVehicleDisplayBindings } from '../src/scene/vehicle_display_bindings.ts';
import { buildVehicleReadoutModel } from '../src/ui/vehicle_readout_model.ts';
import type { WheelPlacement } from '../src/record/scene_record.ts';
import { makeRecord, placement, visualDefinitionWithBindings, type FixtureFrame } from './record_fixtures.ts';
import { range, straightTable, turnedRotation, turnedTable } from './track_fixtures.ts';

// Neutral body names: the readout model must get every role from the bindings.
const bodies = ['body_a', 'body_b', 'body_c', 'body_d', 'body_e'];
const placements = [placement('if_left', 'body_d', 'body_c', 'left'), placement('if_right', 'body_e', 'body_c', 'right')];
const scalars = ['body_c.track_station_meters', 'body_c.lateral_meters', 'body_c.yaw_radians', 'if_left.contact_patch_count', 'if_right.contact_patch_count'];
const positions: Record<string, [number, number, number]> = {
  body_a: [0, 0, -1],
  body_b: [8.75, 0, -0.43],
  body_c: [10, 0, -0.43],
  body_d: [10, 0, -0.43],
  body_e: [10, 0, -0.43],
};
const bindingsJson = {
  carbody: { body_name: 'body_a', display_name: { en: 'Carbody', zh: '车体' } },
  bogies: [{ body_name: 'body_b', display_name: { en: 'Bogie frame', zh: '构架' }, member_body_names: ['body_c'] }],
  carriers: [{ carrier_body_name: 'body_c', display_name: { en: 'Axle bridge', zh: '轴桥' } }],
};

function frame(timeSeconds: number, spec: { carrierQuaternion?: THREE.Quaternion; patches: [number, number]; statuses: [number, number]; sampleIndex?: number }): FixtureFrame {
  const q = spec.carrierQuaternion ?? new THREE.Quaternion();
  return {
    timeSeconds,
    sampleIndex: spec.sampleIndex,
    poses: Object.fromEntries(
      bodies.map((name) => [name, { position: positions[name], orientationWxyz: name === 'body_c' ? [q.w, q.x, q.y, q.z] : [1, 0, 0, 0] }]),
    ),
    scalarValues: [10, 0, 0, spec.patches[0], spec.patches[1]],
    scalarStatuses: [ScalarStatus.valid, ScalarStatus.valid, ScalarStatus.valid, spec.statuses[0], spec.statuses[1]],
  };
}

function build(frames: FixtureFrame[], bindingsSpec: unknown = bindingsJson, track: TrackModel | null = null) {
  const text = visualDefinitionWithBindings(bindingsSpec);
  const record = makeRecord({ bodies, placements, scalars, frames, visualDefinitionText: text });
  const bindings = resolveVehicleDisplayBindings(record, parseVisualDefinition(text, new Set(bodies)));
  return { record, bindings, model: buildVehicleReadoutModel(record, track, bindings) };
}

test('a wheel whose contact is not ready stays unknown', () => {
  const { model } = build([
    frame(0, { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.notReady] }),
    frame(0.01, { patches: [1, 1], statuses: [ScalarStatus.notReady, ScalarStatus.notReady] }),
  ]);
  assert.deepEqual(model.contactAt(0), { state: 'incomplete', codes: [], known: 1, total: 2 });
  assert.deepEqual(model.contactAt(1), { state: 'unknown', codes: [], known: 0, total: 2 });
  const patches = new Float64Array(2);
  model.contactPatchesAt(0, patches);
  assert.equal(patches[0], 1);
  assert.ok(Number.isNaN(patches[1]), 'a not-ready count is NaN, not 1');
  assert.equal(model.contactSpans.length, 0, 'not ready is no contact event');
});

test('recorded loss and two-point contact are reported with the coverage, by frame row', () => {
  const { model } = build([
    frame(0, { patches: [0, 1], statuses: [ScalarStatus.valid, ScalarStatus.notReady] }),
    frame(0.01, { patches: [2, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] }),
  ]);
  assert.deepEqual(model.contactAt(0), { state: 'loss', codes: ['1L'], known: 1, total: 2 });
  assert.deepEqual(model.contactAt(1), { state: 'two', codes: ['1L'], known: 2, total: 2 });
  assert.deepEqual(
    model.contactSpans.map((span) => [span.startFrameIndex, span.patches, span.code]),
    [
      [0, 0, '1L'],
      [1, 2, '1L'],
    ],
  );
});

test('body plan heading comes from the +x axis of a body that does not spin, whatever its basis', () => {
  const track = new TrackModel(straightTable(range(-20, 40, 0.5), () => 0));
  const theta = 0.004;
  // Nose to the right: a turn about the inertial +z axis, which points down.
  const turn = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), theta);
  const halfTurn = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
  for (const [basis, orientation] of [
    ['identity basis (y right, z down)', turn.clone()],
    ['half-turn basis (y left, z up)', turn.clone().multiply(halfTurn)],
  ] as const) {
    const { model } = build([frame(0, { carrierQuaternion: orientation, patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] })], bindingsJson, track);
    const carrier = model.carriers[0];
    assert.ok(carrier !== undefined);
    const heading = model.headingAt(0, carrier);
    assert.ok(heading !== null && Math.abs(heading - theta) < 1e-12, `${basis}: heading ${heading}`);
  }
});

test('frame rows are looked up by row number while the sample identity is only shown', () => {
  const { record, model } = build([
    frame(0, { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid], sampleIndex: 42 }),
    frame(0.01, { patches: [0, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid], sampleIndex: 105 }),
    frame(0.02, { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid], sampleIndex: 901 }),
  ]);
  assert.deepEqual([0, 1, 2].map((frameIndex) => frameSampleIndex(record, frameIndex)), [42, 105, 901]);
  assert.equal(frameIndexAtOrBefore(record, 0.015), 1, 'seeking resolves to the row, not the identity');
  assert.deepEqual(model.contactSpans.map((span) => [span.startFrameIndex, span.endFrameIndex]), [[1, 1]]);
  assert.deepEqual(model.contactAt(1).codes, ['1L']);
  const absent = makeRecord({ bodies, frames: [{ timeSeconds: 0 }] });
  assert.equal(frameSampleIndex(absent, 0), -1, 'an absent identity keeps its stated meaning');
});

test('the carrier order and names come from the bindings; offsets follow the carbody axis', () => {
  const { model } = build([frame(0, { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] })]);
  const carrier = model.carriers[0];
  assert.ok(carrier !== undefined);
  assert.equal(carrier.number, 1);
  assert.deepEqual(carrier.displayName, { en: 'Axle bridge', zh: '轴桥' });
  assert.equal(carrier.initialLongitudinalOffsetInCarbodyFrameMeters, 10);
  assert.equal(carrier.leftWheel?.code, '1L');
  assert.equal(carrier.rightWheel?.code, '1R');
  assert.equal(carrier.bogieIndex, 0);
  assert.equal(model.bogies[0]?.initialLongitudinalOffsetInCarbodyFrameMeters, 8.75);
});

test('without a carbody the carbody-relative readouts are unavailable but the carriers remain', () => {
  const { model } = build([frame(0, { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] })], { ...bindingsJson, carbody: null });
  assert.equal(model.carbody, null);
  assert.equal(model.speedKmhAt(0), null);
  assert.ok(Number.isNaN(model.carbodyStation[0] ?? 0));
  assert.equal(model.carriers[0]?.initialLongitudinalOffsetInCarbodyFrameMeters, null, 'no world-axis stand-in');
  assert.equal(model.carriers.length, 1);
  assert.deepEqual(model.contactAt(0), { state: 'single', codes: [], known: 2, total: 2 });
});

test('a scalar the record never exported reads as null while a wrong body reference is refused earlier', () => {
  const { model } = build([frame(0, { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] })]);
  const bogie = model.bogies[0];
  assert.ok(bogie !== undefined);
  assert.equal(bogie.lateralScalar, null, 'the fixture exports no track coordinates for the bogie');
  assert.equal(model.read(0, bogie.lateralScalar), null);
  assert.throws(
    () => build([frame(0, { patches: [1, 1], statuses: [ScalarStatus.valid, ScalarStatus.valid] })], { ...bindingsJson, bogies: [{ ...bindingsJson.bogies[0], body_name: 'body_x' }] }),
    /does not list/,
  );
});

// --- carrier heading from the wheel axle ---
//
// Two vehicles with neutral names. The IRW-like one has a separate carrier and
// two wheel bodies in the half-turn basis (y left, z up) with the spin axis
// (0, -1, 0); only the wheels spin. The rigid-wheelset-like one has one body
// that is both carrier and wheel, in the identity basis with the spin axis
// (0, 1, 0); the whole body spins. Every pose is built from the same yaw and
// roll, so what is proved is that the spin does not enter the heading.

interface AxleVehicle {
  bodies: string[];
  placements: WheelPlacement[];
  basis: THREE.Quaternion;
  spinAxis: [number, number, number];
  carrierBody: string;
  wheelBodies: string[];
}

const halfTurnBasis = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
const irwLike: AxleVehicle = {
  bodies: ['body_c', 'body_d', 'body_e'],
  placements: [placement('if_left', 'body_d', 'body_c', 'left', -1), placement('if_right', 'body_e', 'body_c', 'right', -1)],
  basis: halfTurnBasis,
  spinAxis: [0, -1, 0],
  carrierBody: 'body_c',
  wheelBodies: ['body_d', 'body_e'],
};
const wheelsetLike: AxleVehicle = {
  bodies: ['body_f'],
  placements: [placement('if_left_f', 'body_f', 'body_f', 'left', 1), placement('if_right_f', 'body_f', 'body_f', 'right', 1)],
  basis: new THREE.Quaternion(),
  spinAxis: [0, 1, 0],
  carrierBody: 'body_f',
  wheelBodies: ['body_f'],
};

const yaw = (psi: number): THREE.Quaternion => new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), psi);
const roll = (phi: number): THREE.Quaternion => new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), phi);

function axleModel(vehicle: AxleVehicle, track: TrackModel | null, trackTransform: THREE.Quaternion, psi: number, phi: number, theta: number) {
  const carrierPose = trackTransform.clone().multiply(yaw(psi)).multiply(roll(phi)).multiply(vehicle.basis);
  const spin = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...vehicle.spinAxis), theta);
  const position = new THREE.Vector3(10, 0, -0.43).applyQuaternion(trackTransform).toArray() as [number, number, number];
  const poses = Object.fromEntries(
    vehicle.bodies.map((body) => {
      const q = vehicle.wheelBodies.includes(body) ? carrierPose.clone().multiply(spin) : carrierPose;
      return [body, { position, orientationWxyz: [q.w, q.x, q.y, q.z] as [number, number, number, number] }];
    }),
  );
  const text = visualDefinitionWithBindings({
    carbody: null,
    bogies: [],
    carriers: [{ carrier_body_name: vehicle.carrierBody, display_name: { en: 'Carrier', zh: '载体' } }],
  });
  const record = makeRecord({ bodies: vehicle.bodies, placements: vehicle.placements, frames: [{ timeSeconds: 0, poses }], visualDefinitionText: text });
  const bindings = resolveVehicleDisplayBindings(record, parseVisualDefinition(text, new Set(vehicle.bodies)));
  return buildVehicleReadoutModel(record, track, bindings);
}

test('carrier heading follows the wheel axle, not the turn of the wheel about it', () => {
  const track = new TrackModel(straightTable(range(-20, 40, 0.5), () => 0));
  for (const [label, vehicle] of [['independent wheels', irwLike], ['rigid wheelset', wheelsetLike]] as const) {
    for (const psi of [0.004, -0.02]) {
      for (const phi of [0, 0.05]) {
        for (const theta of [0, Math.PI / 2, Math.PI, 13]) {
          const model = axleModel(vehicle, track, new THREE.Quaternion(), psi, phi, theta);
          const carrier = model.carriers[0];
          assert.ok(carrier !== undefined);
          const heading = model.carrierHeadingAt(0, carrier);
          assert.ok(heading !== null && Math.abs(heading - psi) < 1e-9, `${label}: psi=${psi} phi=${phi} theta=${theta} gave ${heading}`);
        }
      }
    }
  }
});

test('carrier heading is relative to the local track frame, not to the world axes', () => {
  const alpha = 0.35;
  const cant = 0.08;
  const track = new TrackModel(turnedTable(range(-20, 40, 0.5), alpha, cant));
  const transform = turnedRotation(alpha, cant);
  for (const vehicle of [irwLike, wheelsetLike]) {
    for (const theta of [0, Math.PI / 2]) {
      const model = axleModel(vehicle, track, transform, 0.004, 0, theta);
      const carrier = model.carriers[0];
      assert.ok(carrier !== undefined);
      const heading = model.carrierHeadingAt(0, carrier);
      assert.ok(heading !== null && Math.abs(heading - 0.004) < 1e-9, `turned track, theta=${theta} gave ${heading}`);
    }
  }
});

test('a carrier without a horizontal axle, or without a track, reads unavailable', () => {
  const track = new TrackModel(straightTable(range(-20, 40, 0.5), () => 0));
  const vertical = axleModel(irwLike, track, new THREE.Quaternion(), 0.004, Math.PI / 2, 0.3);
  const carrier = vertical.carriers[0];
  assert.ok(carrier !== undefined);
  assert.equal(vertical.carrierHeadingAt(0, carrier), null, 'a vertical axle has no plan heading');
  const noTrack = axleModel(irwLike, null, new THREE.Quaternion(), 0.004, 0, 0.3);
  const untracked = noTrack.carriers[0];
  assert.ok(untracked !== undefined);
  assert.equal(noTrack.carrierHeadingAt(0, untracked), null, 'no track, no heading');
});
