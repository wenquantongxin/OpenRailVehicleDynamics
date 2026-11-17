import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { forwardArrowPlacement } from '../src/scene/track_plane_grid_with_forward_arrow.ts';

// The forward arrow on the grid is drawn at a vehicle's size beside a vehicle and in proportion beside a smaller
// drawing, so that beside a single cube it does not outgrow what the camera frames.

test('beside a vehicle the arrow is 2.4 m long, its tip half a metre behind the front end and a metre out to the side', () => {
  const vehicle = new THREE.Box3(new THREE.Vector3(-11, 0, -1.6), new THREE.Vector3(11, 3.9, 1.6));
  assert.deepEqual(forwardArrowPlacement(vehicle), { tip: 10.5, side: -2.6, scale: 1, lengthMeters: 2.4 });
});

test('beside a drawing shorter than a vehicle the arrow shrinks with the length framed, never longer than a third of it', () => {
  const cube = new THREE.Box3(new THREE.Vector3(-0.5, 0, -0.5), new THREE.Vector3(0.5, 1, 0.5));
  const besideCube = forwardArrowPlacement(cube);
  assert.ok(Math.abs(besideCube.scale - 0.4) < 1e-12, 'a cube of 1 m is framed as 3 m long');
  assert.ok(besideCube.lengthMeters <= 3 / 3, `the arrow is ${besideCube.lengthMeters} m long beside 3 m framed`);
  assert.ok(Math.abs(besideCube.tip - 0.3) < 1e-12 && Math.abs(besideCube.side + 0.9) < 1e-12, 'its offsets shrink with it');
  for (const length of [0.1, 1, 3, 5, 7.5, 10, 30]) {
    const drawn = new THREE.Box3(new THREE.Vector3(0, 0, -0.5), new THREE.Vector3(length, 1, 0.5));
    const framedLength = Math.max(length, 3);
    assert.ok(forwardArrowPlacement(drawn).lengthMeters <= framedLength / 3, `beside a drawing ${length} m long`);
  }
});
