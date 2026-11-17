import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { framedBox, framesCamera, presetPlacement, viewPresets } from '../src/scene/vehicle_model_view_camera_presets.ts';
import { createModelPalette } from '../src/scene/vehicle_model_view_colors_and_materials.ts';
import { VehiclePreviewScene } from '../src/scene/vehicle_preview_scene.ts';
import { readPreview } from '../src/service/vehicle_modeling_service_protocol.ts';

// Each preset is checked the way the view uses it: a real perspective camera
// placed where the preset says, looking at its target with the vertical up.
// Every corner of the framed box must project inside the picture, and the
// box must fill it to the margin, unless the one-metre clearance governed. A
// drawing narrower than three metres along or across is framed as that wide.

const boxes: Record<string, THREE.Box3> = {
  vehicle: new THREE.Box3(new THREE.Vector3(-11, 0, -1.6), new THREE.Vector3(11, 3.9, 1.6)),
  offset: new THREE.Box3(new THREE.Vector3(2, 0.2, -3), new THREE.Vector3(9, 1.4, -0.5)),
  tiny: new THREE.Box3(new THREE.Vector3(-0.05, 0, -0.05), new THREE.Vector3(0.05, 0.1, 0.05)),
};

test('every preset frames every box inside the picture, at every aspect', () => {
  for (const preset of viewPresets) {
    for (const [name, drawn] of Object.entries(boxes)) {
      const box = framedBox(drawn);
      for (const aspect of [16 / 10, 4 / 3, 0.8]) {
        const placement = presetPlacement(preset, drawn, aspect);
        const camera = new THREE.PerspectiveCamera(placement.fov, aspect, 0.01, 5000);
        camera.position.copy(placement.position);
        camera.lookAt(placement.target);
        camera.updateMatrixWorld(true);
        const toCamera = placement.position.clone().sub(placement.target).normalize();
        let largest = 0;
        let nearestDepth = Infinity;
        for (let corner = 0; corner < 8; ++corner) {
          const point = new THREE.Vector3(corner & 1 ? box.max.x : box.min.x, corner & 2 ? box.max.y : box.min.y, corner & 4 ? box.max.z : box.min.z);
          nearestDepth = Math.min(nearestDepth, -point.clone().sub(placement.target).dot(toCamera));
          const projected = point.project(camera);
          largest = Math.max(largest, Math.abs(projected.x), Math.abs(projected.y));
        }
        const label = `${preset} / ${name} / ${aspect}`;
        assert.ok(largest <= 1, `${label}: a corner leaves the picture (${largest})`);
        const clearanceGoverned = Math.abs(placement.position.distanceTo(placement.target) - (1 - nearestDepth)) < 1e-9;
        assert.ok(largest > 0.9 || clearanceGoverned, `${label}: the box fills only ${largest} of the picture`);
      }
    }
  }
});

test('a drawing at least three metres along and across is framed as it is; a smaller one as three metres wide about its centre, its height as it is', () => {
  assert.deepEqual(framedBox(boxes['vehicle']!), boxes['vehicle'], 'a vehicle 22 m long and 3.2 m wide');
  const cube = new THREE.Box3(new THREE.Vector3(-0.5, 0, -0.5), new THREE.Vector3(0.5, 1, 0.5));
  assert.deepEqual(framedBox(cube), new THREE.Box3(new THREE.Vector3(-1.5, 0, -1.5), new THREE.Vector3(1.5, 1, 1.5)));
  const long = new THREE.Box3(new THREE.Vector3(0, 0, -0.5), new THREE.Vector3(10, 1, 0.5));
  assert.deepEqual(framedBox(long), new THREE.Box3(new THREE.Vector3(0, 0, -1.5), new THREE.Vector3(10, 1, 1.5)), 'only what is narrow is widened');
  assert.deepEqual(framedBox(boxes['offset']!), new THREE.Box3(new THREE.Vector3(2, 0.2, -3.25), new THREE.Vector3(9, 1.4, -0.25)), 'about the centre, wherever it is');
});

test('the plan looks down with forward to the right of the picture', () => {
  const placement = presetPlacement('plan', boxes['vehicle']!, 1.6);
  const camera = new THREE.PerspectiveCamera(placement.fov, 1.6, 0.01, 5000);
  camera.position.copy(placement.position);
  camera.lookAt(placement.target);
  camera.updateMatrixWorld(true);
  const ahead = placement.target.clone().add(new THREE.Vector3(5, 0, 0)).project(camera);
  const centre = placement.target.clone().project(camera);
  assert.ok(placement.position.y > placement.target.y + 10);
  assert.ok(ahead.x > centre.x + 0.1 && Math.abs(ahead.y - centre.y) < 0.05);
});

test('only a preview that draws something frames the camera, and only once per opened vehicle', () => {
  const lists = { rigid_bodies: [], fixed_frames: [], revolute_joints: [], ball_rpy_joints: [], weld_joints: [], force_elements: [], wheels: [] };
  const drawsNothing = readPreview({ preview: { project_revision: 0, ...lists, not_drawable_elements: [{ kind: 'kind_body', name: 'unplaced' }] } })!;
  const identity = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  const drawsABody = readPreview({
    preview: {
      project_revision: 1,
      ...lists,
      rigid_bodies: [
        { element: { kind: 'kind_body', name: 'placed' }, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [0, 0, -1], visual_definition_body_name: null },
      ],
      not_drawable_elements: [],
    },
  })!;
  // Nothing drawn has no extent: framing it would put the camera at non-finite numbers.
  assert.ok(new VehiclePreviewScene(drawsNothing, null, createModelPalette()).bounds().isEmpty());
  assert.equal(framesCamera(null, false), false);
  assert.equal(framesCamera(drawsNothing, false), false);
  // The first later preview that draws something is framed as a newly opened vehicle is.
  assert.equal(framesCamera(drawsABody, false), true);
  assert.equal(framesCamera(drawsABody, true), false);
});
