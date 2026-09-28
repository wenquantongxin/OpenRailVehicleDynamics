import assert from 'node:assert/strict';
import { test } from 'node:test';

import { TrackModel } from '../src/scene/track_model.ts';
import { range, straightTable } from './track_fixtures.ts';

test('curves of different radius stay separate across a short transition', () => {
  // R300 to 150 m, a 3 m linear transition, R500 to 250 m, then back to tangent.
  const k = (s: number): number => {
    if (s < 50) return 0;
    if (s < 100) return (s - 50) / 50 / 300;
    if (s < 150) return 1 / 300;
    if (s < 153) return 1 / 300 + ((s - 150) / 3) * (1 / 500 - 1 / 300);
    if (s < 250) return 1 / 500;
    if (s < 300) return (1 - (s - 250) / 50) / 500;
    return 0;
  };
  const model = new TrackModel(straightTable(range(0, 400, 0.5), k));
  const circular = model.sections.filter((section) => section.kind === 'circular');
  assert.equal(circular.length, 2);
  assert.ok(Math.abs((circular[0]?.radiusMeters ?? 0) - 300) < 1e-6, `first radius ${circular[0]?.radiusMeters}`);
  assert.ok(Math.abs((circular[1]?.radiusMeters ?? 0) - 500) < 1e-6, `second radius ${circular[1]?.radiusMeters}`);
  for (const section of model.sections) {
    assert.ok(section.radiusMeters === null || Number.isFinite(section.radiusMeters), 'no infinite radius');
  }
  assert.deepEqual(
    model.elementPoints.map((point) => point.code),
    ['TS', 'SC', 'CS', 'SC', 'CS', 'ST'],
  );
});

test('two circular curves meeting directly give a compound-curve point', () => {
  const k = (s: number): number => (s < 50 ? 1 / 300 : 1 / 500);
  const model = new TrackModel(straightTable(range(0, 100, 0.5), k));
  assert.deepEqual(
    model.sections.map((section) => [section.kind, Math.round(section.radiusMeters ?? 0)]),
    [
      ['circular', 300],
      ['circular', 500],
    ],
  );
  assert.deepEqual(model.elementPoints.map((point) => point.code), ['CC']);
});

test('rail offsets, datum spacing and lowest point come from the recorded table', () => {
  const table = straightTable(range(0, 20, 0.5), () => 0, 0.8);
  // Raise the line by 0.3 m towards its far end (inertial z points down).
  table.centerlineInInertialMeters = table.stationsMeters.map((s) => [s, 0, -0.015 * s]);
  table.leftRailDatumInInertialMeters = table.stationsMeters.map((s) => [s, -0.8, -0.015 * s - 0.00018]);
  table.rightRailDatumInInertialMeters = table.stationsMeters.map((s) => [s, 0.8, -0.015 * s - 0.00018]);
  const model = new TrackModel(table);
  assert.ok(Math.abs(model.medianRailDatumSpacingMeters - 1.6) < 1e-12);
  assert.ok(Math.abs(model.lowestCentrelineInertialZMeters - 0) < 1e-12, 'the start is the lowest point');
  const [vl, wl] = model.railDatumOffsetAtStation(7.25, 'left');
  const [vr, wr] = model.railDatumOffsetAtStation(7.25, 'right');
  assert.ok(Math.abs(vl + 0.8) < 1e-12 && Math.abs(vr - 0.8) < 1e-12);
  assert.ok(Math.abs(wl + 0.00018) < 1e-12 && Math.abs(wr + 0.00018) < 1e-12);
});

test('canted track: rail offsets are read in the rolled Track-T frame', () => {
  const stations = range(0, 10, 0.5);
  const roll = 0.08;
  const table = straightTable(stations, () => 0);
  const w = Math.cos(roll / 2);
  const x = Math.sin(roll / 2);
  table.rotationInertialFromTrackWxyz = stations.map(() => [w, x, 0, 0]);
  // Rails at ±0.75315 along the rolled lateral axis (0, cos, sin).
  table.leftRailDatumInInertialMeters = stations.map((s) => [s, -0.75315 * Math.cos(roll), -0.75315 * Math.sin(roll)]);
  table.rightRailDatumInInertialMeters = stations.map((s) => [s, 0.75315 * Math.cos(roll), 0.75315 * Math.sin(roll)]);
  const model = new TrackModel(table);
  const [vl, wl] = model.railDatumOffsetAtStation(5, 'left');
  const [vr, wr] = model.railDatumOffsetAtStation(5, 'right');
  assert.ok(Math.abs(vl + 0.75315) < 1e-12 && Math.abs(vr - 0.75315) < 1e-12);
  assert.ok(Math.abs(wl) < 1e-12 && Math.abs(wr) < 1e-12);
});
