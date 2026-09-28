import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import { CameraRig, presetAvailableFor, type CameraAnchors, type FramedExtents } from '../src/scene/camera_rig.ts';
import type { ViewportArea } from '../src/scene/camera_framing.ts';

// The rig with a stand-in for the orbit controls: refitting follows the
// picture until the user takes over, an unframeable preset is refused without
// moving the camera, and following carries the camera with its anchor.

function makeRig() {
  const root = new THREE.Group();
  root.rotation.x = Math.PI / 2;
  const carbody = new THREE.Object3D();
  carbody.position.set(5, 0, 0);
  root.add(carbody);
  root.updateMatrixWorld(true);
  const anchors: CameraAnchors = { root, carbody, bogie: null, bodies: [carbody] };
  const extents: FramedExtents = { vehicle: { minForward: -11, maxForward: 11, minLeft: -1.3, maxLeft: 1.3, minUp: 0, maxUp: 3.8 }, bogie: null };
  const listeners: Record<string, () => void> = {};
  const controls = {
    target: new THREE.Vector3(),
    update: () => undefined,
    addEventListener: (type: string, listener: () => void) => {
      listeners[type] = listener;
    },
  } as unknown as OrbitControls;
  const camera = new THREE.PerspectiveCamera(30, 1400 / 900, 0.1, 2500);
  const rig = new CameraRig(camera, controls, anchors, null, extents);
  return { rig, camera, controls, listeners, carbody, root, extents };
}

const wide: ViewportArea = { width: 1400, height: 900, free: { x0: 294, y0: 86, x1: 1106, y1: 724 } };
const narrow: ViewportArea = { width: 1000, height: 700, free: { x0: 276, y0: 82, x1: 724, y1: 540 } };

test('a preset that cannot be framed is refused without moving the camera', () => {
  const { rig, camera, extents } = makeRig();
  assert.ok(rig.applyPreset('overview', wide, true));
  const before = camera.position.clone();
  assert.equal(rig.presetAvailable('bogie'), false);
  assert.equal(rig.applyPreset('bogie', wide, true), false);
  assert.ok(camera.position.equals(before));
  assert.equal(presetAvailableFor('headon', extents, false), true, 'head-on falls back to the vehicle without a bogie');
  assert.equal(presetAvailableFor('headon', { vehicle: null, bogie: null }, false), false);
});

test('a changed picture refits an untouched preset and leaves a user-adjusted view alone', () => {
  const { rig, camera, listeners } = makeRig();
  rig.applyPreset('overview', wide, true);
  const fittedWide = camera.position.clone();
  rig.refit(narrow);
  const refitted = camera.position.clone();
  assert.ok(!refitted.equals(fittedWide), 'the narrower picture moves the camera');
  const fresh = makeRig();
  fresh.rig.applyPreset('overview', narrow, true);
  assert.ok(refitted.distanceTo(fresh.camera.position) < 1e-9, 'a refit equals a fresh fit to that picture');
  rig.refit(narrow);
  assert.ok(camera.position.equals(refitted), 'an unchanged picture does nothing');
  const start = listeners['start'];
  assert.ok(start !== undefined);
  start();
  camera.position.x += 3;
  const adjusted = camera.position.clone();
  rig.refit(wide);
  assert.ok(camera.position.equals(adjusted), 'the user view is kept through a picture change');
  rig.applyPreset('overview', wide, true);
  assert.ok(camera.position.distanceTo(fittedWide) < 1e-9, 'choosing a preset again takes over from the user');
});

test('following carries the camera with the anchor', () => {
  const { rig, camera, controls, carbody, root } = makeRig();
  rig.applyPreset('side', wide, true);
  const previousCamera = camera.position.clone();
  const previousTarget = controls.target.clone();
  const offset = camera.position.clone().sub(controls.target);
  carbody.position.x += 7;
  root.updateMatrixWorld(true);
  rig.update(0.016, wide);
  const translation = new THREE.Vector3(7, 0, 0);
  assert.ok(camera.position.distanceTo(previousCamera.add(translation)) < 1e-9, 'the camera follows the anchor translation');
  assert.ok(controls.target.distanceTo(previousTarget.add(translation)) < 1e-9, 'the target follows the anchor translation');
  const moved = camera.position.clone().sub(controls.target);
  assert.ok(moved.distanceTo(offset) < 1e-9, 'the camera keeps its offset from the target');
  assert.ok(Math.abs(camera.position.x - offset.x - controls.target.x) < 1e-9);
});
