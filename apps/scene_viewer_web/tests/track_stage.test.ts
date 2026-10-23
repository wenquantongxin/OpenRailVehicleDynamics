import assert from 'node:assert/strict';
import { after, before, test } from 'node:test';
import * as THREE from 'three';

import { createPalette, type ScenePalette } from '../src/scene/materials.ts';
import { buildGround, buildTrackSegment, disposeTrackSegment, groundDepth, moveGround } from '../src/scene/track_builder.ts';
import { TrackModel } from '../src/scene/track_model.ts';
import { TrackStage } from '../src/scene/track_stage.ts';
import { ownedGridStations, ownsStation, segmentSurfaceStations, trackSegment, trackSegmentAt, trackSegmentBounds } from '../src/scene/track_segments.ts';
import { range, straightTable } from './track_fixtures.ts';

// Geometry/lifecycle tests need canvas-backed textures but no rasterization or WebGL.
const previousDocument = globalThis.document;
let palette: ScenePalette;
before(() => {
  const context = new Proxy({}, { get: () => () => undefined, set: () => true });
  globalThis.document = { createElement: () => ({ width: 0, height: 0, getContext: () => context }) } as unknown as Document;
  palette = createPalette();
});
after(() => { palette.dispose(); globalThis.document = previousDocument; });

test('absolute segment identity and half-open ownership survive clipping and negative stations', () => {
  const model = new TrackModel(straightTable([-61.2, -60, 0, 60, 60.0000001], () => 0));
  assert.deepEqual(trackSegmentBounds(model), [-2, 1]);
  assert.deepEqual(trackSegment(model, -2), { id: -2, start: -61.2, end: -60, includesEnd: false });
  assert.deepEqual(trackSegment(model, 1), { id: 1, start: 60, end: 60.0000001, includesEnd: true });
  assert.deepEqual(segmentSurfaceStations(model, trackSegment(model, 1), 'simplified'), [60, 60.0000001]);
  const whole = new TrackModel(straightTable([-120, 0, 120], () => 0));
  assert.equal(trackSegmentAt(model, 10), trackSegmentAt(whole, 10));
  assert.equal(trackSegmentAt(whole, 120), 1, 'final endpoint belongs to the last nonempty segment');
  const left = trackSegment(whole, -1);
  const right = trackSegment(whole, 0);
  assert.equal(ownsStation(left, 0), false);
  assert.equal(ownsStation(right, 0), true);
  for (const spacing of [0.6, 50]) {
    const stations = [ownedGridStations(left, spacing), ownedGridStations(right, spacing)].flat();
    assert.equal(stations.filter((s) => s === 0).length, 1);
    assert.equal(new Set(stations).size, stations.length);
    stations.forEach((s) => assert.equal(s, Math.round(s / spacing) * spacing));
  }
  assert.deepEqual(segmentSurfaceStations(whole, { id: 0, start: 1.1, end: 8.1, includesEnd: true }, 'simplified'), [1.1, 3, 6, 8.1]);
});

function longTrack(): TrackModel {
  const table = straightTable(range(-30.25, 22030.25, 0.5), () => 0);
  table.stationsMeters.forEach((s, i) => {
    const y = 80 * Math.sin(s / 900);
    const z = 8 * Math.sin(s / 1300);
    const heading = Math.atan((80 / 900) * Math.cos(s / 900));
    const pitch = -Math.atan((8 / 1300) * Math.cos(s / 1300));
    const cant = 0.06 * Math.sin(s / 1000);
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(cant, pitch, heading, 'ZYX'));
    const p = new THREE.Vector3(18700.123456 + s, y, z);
    table.centerlineInInertialMeters[i] = p.toArray();
    table.rotationInertialFromTrackWxyz[i] = [q.w, q.x, q.y, q.z];
    table.curvatureRadiansPerMeter[i] = -80 / (900 * 900) * Math.sin(s / 900);
    table.superelevationMeters[i] = 1.5 * Math.sin(cant);
    table.leftRailDatumInInertialMeters[i] = new THREE.Vector3(0, -0.75315, -0.00018).applyQuaternion(q).add(p).toArray();
    table.rightRailDatumInInertialMeters[i] = new THREE.Vector3(0, 0.75315, -0.00018).applyQuaternion(q).add(p).toArray();
  });
  return new TrackModel(table);
}

function mesh(group: THREE.Group, name: string): THREE.Mesh {
  const object = group.getObjectByName(name);
  assert.ok(object instanceof THREE.Mesh, name);
  return object;
}

test('neighbouring detailed and simplified rails and ballast meet in world coordinates', () => {
  const model = longTrack();
  for (const id of [0, 155, 310]) {
    const a = buildTrackSegment(model, palette, trackSegment(model, id), 'detailed');
    const b = buildTrackSegment(model, palette, trackSegment(model, id + 1), 'simplified');
    for (const [name, rowSize] of [['left rail', 58], ['right rail', 58], ['ballast', 6]] as const) {
      const av = mesh(a, name).geometry.getAttribute('position');
      const bv = mesh(b, name).geometry.getAttribute('position');
      for (let j = 0; j < rowSize; ++j) {
        const pa = new THREE.Vector3().fromBufferAttribute(av, av.count - rowSize + j).add(a.position);
        const pb = new THREE.Vector3().fromBufferAttribute(bv, j).add(b.position);
        assert.ok(pa.distanceTo(pb) <= 1e-5, `${name} seam ${pa.distanceTo(pb)}`);
      }
    }
    disposeTrackSegment(a); disposeTrackSegment(b);
  }
});

test('local vertices and instance matrices retain a 0.1 mm displacement at distant coordinates', () => {
  const table = straightTable([0, 0.6, 1.2, 60], () => 0);
  for (const rows of [table.centerlineInInertialMeters, table.leftRailDatumInInertialMeters, table.rightRailDatumInInertialMeters]) {
    rows.forEach((p) => { p[0] = (p[0] ?? 0) + 18700.123456; });
  }
  const make = () => {
    const model = new TrackModel(table);
    return buildTrackSegment(model, palette, trackSegment(model, 0), 'detailed');
  };
  const a = make();
  for (const rows of [table.centerlineInInertialMeters, table.leftRailDatumInInertialMeters, table.rightRailDatumInInertialMeters]) rows[1]![0]! += 0.0001;
  const b = make();
  const x = (g: THREE.Group) => mesh(g, 'left rail').geometry.getAttribute('position').getX(58) + g.position.x;
  assert.ok(Math.abs((x(b) - x(a)) - 0.0001) < 1e-7);
  const ia = new THREE.Matrix4(); const ib = new THREE.Matrix4();
  (mesh(a, 'sleepers') as THREE.InstancedMesh).getMatrixAt(1, ia);
  (mesh(b, 'sleepers') as THREE.InstancedMesh).getMatrixAt(1, ib);
  assert.ok(Math.abs((ib.elements[12]! - ia.elements[12]!) - 0.0001) < 1e-7);
  assert.ok(Math.abs((Math.fround(x(b)) - Math.fround(x(a))) - 0.0001) > 0.00005, 'absolute float storage loses the displacement');
  disposeTrackSegment(a); disposeTrackSegment(b);
});

test('forward, backward and distant seeks retain at most 17 segments and release replaced resources', () => {
  const model = longTrack();
  let sharedMaterialDisposals = 0;
  palette.railHead.addEventListener('dispose', () => { ++sharedMaterialDisposals; });
  for (let load = 0; load < 2; ++load) {
    const stage = new TrackStage(model, palette);
    const tracked = new Map<object, { count: number }>();
    const observe = () => stage.group.traverse((object) => {
      const resources: { addEventListener: (type: 'dispose', listener: () => void) => void }[] = [];
      if (object instanceof THREE.Mesh) resources.push(object.geometry);
      if (object instanceof THREE.InstancedMesh) resources.push(object);
      if (object instanceof THREE.Sprite) resources.push(object.material, object.material.map!);
      for (const resource of resources) {
        if (tracked.has(resource)) continue;
        const state = { count: 0 };
        tracked.set(resource, state);
        resource.addEventListener('dispose', () => { ++state.count; });
      }
    });
    for (const station of [-30, 1, 59.99, 60, 120, 600, 18700, 22030, 120, 0, -30]) {
      stage.update(station);
      assert.ok(stage.group.children.length <= 17);
      assert.ok(stage.group.children.filter((g) => g.name.endsWith(' detailed')).length <= 5);
      observe();
      const before = [...stage.group.children];
      stage.update(station + 0.00001);
      assert.deepEqual(stage.group.children, before, 'unchanged segment and LOD reuse geometry');
    }
    stage.dispose();
    assert.equal(stage.group.children.length, 0);
    for (const { count } of tracked.values()) assert.equal(count, 1, 'owned resource released exactly once');
  }
  assert.equal(sharedMaterialDisposals, 0, 'segment release must not dispose the shared palette');
  const points = model.overviewPoints();
  assert.ok(points.length <= 1024 + model.elementPoints.length);
  assert.equal(points[0]?.stationMeters, model.firstStationMeters);
  assert.equal(points.at(-1)?.stationMeters, model.lastStationMeters);
  for (const point of model.elementPoints) assert.ok(points.some((p) => p.stationMeters === point.stationMeters));
});

test('the finite ground moves without changing height or world texture phase', () => {
  const model = longTrack();
  const ground = buildGround(model, palette);
  const geometry = ground.geometry;
  const position = geometry.getAttribute('position');
  const uv = geometry.getAttribute('uv');
  for (const [x, y] of [[0, 0], [18700.125, 52.625], [-120.25, -18.5]] as const) {
    moveGround(ground, x, y);
    assert.equal(ground.geometry, geometry, 'movement must reuse the plane');
    assert.equal(ground.position.z, model.lowestCentrelineInertialZMeters + groundDepth);
    for (let index = 0; index < position.count; ++index) {
      const worldX = ground.position.x + position.getX(index);
      const worldY = ground.position.y + position.getY(index);
      assert.ok(Math.abs(uv.getX(index) - (worldX / 10 + 200)) < 0.0002);
      assert.ok(Math.abs(uv.getY(index) - (-worldY / 10 + 200)) < 0.0002);
    }
  }
  geometry.dispose();
});
