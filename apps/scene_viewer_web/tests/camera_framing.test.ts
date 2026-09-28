import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { fitDistanceMeters, viewOffsetPixels, type ViewportArea } from '../src/scene/camera_framing.ts';
import { presetDestination, presetTaste, viewPresets } from '../src/scene/camera_rig.ts';
import { extentCorners, type DisplayExtent } from '../src/scene/display_extents.ts';

// The fit is checked the way the viewer uses it: a real perspective camera
// placed at the destination, looking at the target with the vertical up, its
// view offset aiming the axis at the free rectangle's centre. Every corner of
// the extent must project inside the free rectangle shrunk by the margin, and
// one corner must touch that shrunk boundary, unless the one-metre clearance
// in front of the nearest corner governed the distance.

const extents: Record<string, DisplayExtent> = {
  irw: { minForward: -11, maxForward: 11, minLeft: -1.332, maxLeft: 1.332, minUp: -0.028, maxUp: 3.8 },
  tall: { minForward: -1, maxForward: 1, minLeft: -1, maxLeft: 1, minUp: 0, maxUp: 8 },
  longFlat: { minForward: -20, maxForward: 20, minLeft: -1, maxLeft: 1, minUp: 0, maxUp: 1 },
  offset: { minForward: 2, maxForward: 8, minLeft: 0, maxLeft: 2, minUp: 0.5, maxUp: 3.5 },
};

const viewports: Record<string, ViewportArea> = {
  wide: { width: 1400, height: 900, free: { x0: 294, y0: 86, x1: 1106, y1: 724 } },
  narrow: { width: 1200, height: 800, free: { x0: 276, y0: 82, x1: 924, y1: 640 } },
  portrait: { width: 600, height: 900, free: { x0: 20, y0: 50, x1: 580, y1: 850 } },
};

function projectedPixels(camera: THREE.PerspectiveCamera, viewport: ViewportArea, point: [number, number, number]): [number, number] {
  const ndc = new THREE.Vector3(...point).project(camera);
  return [((ndc.x + 1) / 2) * viewport.width, ((1 - ndc.y) / 2) * viewport.height];
}

test('every preset fits every extent inside the free rectangle, tightly, in every viewport', () => {
  for (const preset of viewPresets) {
    const { direction, fov, margin } = presetTaste(preset);
    for (const [extentName, extent] of Object.entries(extents)) {
      for (const [viewportName, viewport] of Object.entries(viewports)) {
        const destination = presetDestination(preset, extent, viewport);
        const camera = new THREE.PerspectiveCamera(fov, viewport.width / viewport.height, 0.1, 5000);
        camera.up.set(0, 0, 1);
        camera.position.copy(destination.camera);
        camera.lookAt(destination.target);
        camera.updateMatrixWorld(true);
        const { offsetX, offsetY } = viewOffsetPixels(viewport);
        camera.setViewOffset(viewport.width, viewport.height, offsetX, offsetY, viewport.width, viewport.height);
        const label = `${preset} / ${extentName} / ${viewportName}`;
        const centreX = 0.5 * (viewport.free.x0 + viewport.free.x1);
        const centreY = 0.5 * (viewport.free.y0 + viewport.free.y1);
        const [targetX, targetY] = projectedPixels(camera, viewport, destination.target.toArray() as [number, number, number]);
        assert.ok(Math.abs(targetX - centreX) < 1e-6 && Math.abs(targetY - centreY) < 1e-6, `${label}: the target sits at the free centre`);
        const bound = 1 / (1 + margin);
        let largest = 0;
        for (const corner of extentCorners(extent)) {
          const [x, y] = projectedPixels(camera, viewport, corner);
          const ratio = Math.max(Math.abs(x - centreX) / (0.5 * (viewport.free.x1 - viewport.free.x0)), Math.abs(y - centreY) / (0.5 * (viewport.free.y1 - viewport.free.y0)));
          assert.ok(ratio <= bound + 1e-6, `${label}: corner ${corner.join(',')} projects to ratio ${ratio}, beyond the margin`);
          largest = Math.max(largest, ratio);
        }
        // Tight unless the one-metre clearance in front of the nearest corner governed.
        const toCamera = new THREE.Vector3(...direction).normalize();
        const nearestDepth = Math.min(...extentCorners(extent).map((corner) => -new THREE.Vector3(...corner).sub(destination.target).dot(toCamera)));
        const distance = destination.camera.distanceTo(destination.target);
        const clearanceGoverned = Math.abs(distance - (1 - nearestDepth)) < 1e-9;
        assert.ok(clearanceGoverned || largest >= bound - 1e-6, `${label}: largest ratio ${largest} does not reach the margin boundary ${bound}`);
      }
    }
  }
});

test('the distance grows with the extent and shrinks with the free picture', () => {
  const viewport = viewports['wide'] as ViewportArea;
  const { direction, fov, margin } = presetTaste('overview');
  const small = fitDistanceMeters(extentCorners(extents['tall'] as DisplayExtent), [0, 0, 4], direction, fov, viewport, margin);
  const doubled: DisplayExtent = { minForward: -2, maxForward: 2, minLeft: -2, maxLeft: 2, minUp: 0, maxUp: 16 };
  const large = fitDistanceMeters(extentCorners(doubled), [0, 0, 8], direction, fov, viewport, margin);
  assert.ok(large > 1.9 * small && large < 2.1 * small, `${small} vs ${large}`);
  const cramped: ViewportArea = { ...viewport, free: { x0: 500, y0: 86, x1: 900, y1: 724 } };
  assert.ok(fitDistanceMeters(extentCorners(extents['irw'] as DisplayExtent), [0, 0, 1], direction, fov, cramped, margin) > fitDistanceMeters(extentCorners(extents['irw'] as DisplayExtent), [0, 0, 1], direction, fov, viewport, margin));
});
