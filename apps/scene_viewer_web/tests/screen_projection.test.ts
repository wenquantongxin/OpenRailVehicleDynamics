import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { projectClippedBoxToScreen } from '../src/scene/label_layer.ts';

// A camera at the origin looking down -z with a 90 degree vertical field of
// view and square aspect: a point at camera-space (x, y, -d) projects to NDC
// (x / d, y / d). The near plane is 1 m away.
const camera = new THREE.PerspectiveCamera(90, 1, 1, 100);
camera.updateProjectionMatrix();
const width = 200;
const height = 100;

function corners(x: [number, number], y: [number, number], z: [number, number]): THREE.Vector3[] {
  return Array.from({ length: 8 }, (_, k) => new THREE.Vector3(k & 1 ? x[1] : x[0], k & 2 ? y[1] : y[0], k & 4 ? z[1] : z[0]));
}

test('a box in front of the camera projects to the rectangle of its corners', () => {
  const rect = projectClippedBoxToScreen(corners([-1, 1], [-0.5, 0.5], [-3, -2]), camera.projectionMatrix, camera.near, width, height);
  assert.ok(rect !== null);
  // The near face at z = -2 sets the extent: x = ±0.5, y = ±0.25 in NDC.
  assert.ok(Math.abs(rect.x0 - 0.25 * width) < 1e-9 && Math.abs(rect.x1 - 0.75 * width) < 1e-9);
  assert.ok(Math.abs(rect.y0 - 0.375 * height) < 1e-9 && Math.abs(rect.y1 - 0.625 * height) < 1e-9);
});

test('a box straddling the near plane is bounded by its near-plane intersections', () => {
  // x in [1, 2], y in [-0.5, 0.5], z from 0.5 m to 3 m in front of the camera;
  // the nearer face is behind the near plane, not behind the camera.
  const rect = projectClippedBoxToScreen(corners([1, 2], [-0.5, 0.5], [-3, -0.5]), camera.projectionMatrix, camera.near, width, height);
  assert.ok(rect !== null);
  // The edge from (2, y, -0.5) to (2, y, -3) meets the near plane at (2, y, -1),
  // which projects to NDC x = 2, i.e. 1.5 widths. Dropping the corners behind the
  // near plane would stop at the retained corner (2, y, -3), NDC x = 2/3.
  assert.ok(Math.abs(rect.x1 - 1.5 * width) < 1e-9, `right edge ${rect.x1}`);
  // The in-front corners at x = 1, z = -3 give the left edge: NDC x = 1/3.
  assert.ok(Math.abs(rect.x0 - (1 + 1 / 3) / 2 * width) < 1e-9, `left edge ${rect.x0}`);
  // At the near plane y = ±0.5 projects to NDC ±0.5.
  assert.ok(Math.abs(rect.y0 - 0.25 * height) < 1e-9 && Math.abs(rect.y1 - 0.75 * height) < 1e-9);
  // Nothing mirrored across the picture: the whole rectangle lies to the right of the centre.
  assert.ok(rect.x0 > width / 2);
});

test('a box entirely behind the camera gives no rectangle', () => {
  assert.equal(projectClippedBoxToScreen(corners([-1, 1], [-1, 1], [0.5, 3]), camera.projectionMatrix, camera.near, width, height), null);
});
