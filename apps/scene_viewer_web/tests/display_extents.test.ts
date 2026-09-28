import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { anchorLocalFrame, measureDisplayExtent } from '../src/scene/display_extents.ts';
import type { ScenePart } from '../src/scene/scene_builder.ts';

// An IRW-shaped tree built by hand: the root turns the inertial frame (x
// forward, y right, z down) into Three.js Y-up, the bodies keep the half-turn
// basis (y left, z up) and every part is a plain geometry under its body.

const halfTurn = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);

interface Tree {
  root: THREE.Group;
  bodies: Map<string, THREE.Object3D>;
  parts: ScenePart[];
}

function buildTree(yawRadians: number): Tree {
  const root = new THREE.Group();
  root.rotation.x = Math.PI / 2;
  const yaw = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), yawRadians);
  const bodies = new Map<string, THREE.Object3D>();
  const parts: ScenePart[] = [];
  const addBody = (name: string, position: [number, number, number]): THREE.Object3D => {
    const body = new THREE.Object3D();
    body.position.set(...position).applyQuaternion(yaw);
    body.quaternion.copy(yaw).multiply(halfTurn);
    root.add(body);
    bodies.set(name, body);
    return body;
  };
  const addPart = (bodyName: string, group: ScenePart['group'], geometry: THREE.BufferGeometry, at: [number, number, number]): void => {
    const mesh = new THREE.Mesh(geometry);
    mesh.position.set(...at);
    const body = bodies.get(bodyName);
    if (body === undefined) {
      throw new Error(bodyName);
    }
    body.add(mesh);
    parts.push({ name: `${bodyName}:${group}`, bodyName, group, object: mesh });
  };
  addBody('carbody', [0, 0, 0]);
  addBody('frame_front', [8.75, 0, -0.43]);
  addBody('wheel_ff', [10, 0, -0.43]);
  addBody('wheel_fr', [7.5, 0, -0.43]);
  addPart('carbody', 'carbody', new THREE.BoxGeometry(22, 2.65, 2.8), [0, 0, 2.4]);
  addPart('frame_front', 'running_gear', new THREE.BoxGeometry(0.4, 2.32, 0.2), [0, 0, 0.26]);
  for (const wheelBody of ['wheel_ff', 'wheel_fr']) {
    for (const lateral of [0.7465, -0.7465]) {
      // A cylinder's axis is its y axis, the wheel body's lateral axis.
      addPart(wheelBody, 'wheels', new THREE.CylinderGeometry(0.458, 0.458, 0.135), [0, lateral, 0]);
    }
  }
  // Not framed: the line and a coordinate-axes part.
  const track = new THREE.Mesh(new THREE.BoxGeometry(400, 3, 0.5));
  root.add(track);
  parts.push({ name: 'track', bodyName: '', group: 'track', object: track });
  addPart('carbody', 'axes', new THREE.BoxGeometry(50, 50, 50), [0, 0, 0]);
  root.updateMatrixWorld(true);
  return { root, bodies, parts };
}

function vehicleFrame(tree: Tree) {
  const carbody = tree.bodies.get('carbody') as THREE.Object3D;
  const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(carbody.getWorldQuaternion(new THREE.Quaternion()));
  return anchorLocalFrame(carbody.getWorldPosition(new THREE.Vector3()), null, tree.root, forward);
}

/** Geometry vertices are single precision, so extents agree to about 1e-7 m. */
const near = (a: number, b: number): boolean => Math.abs(a - b) < 1e-6;

test('the vehicle extent is measured in the anchor frame from the parts that belong to it', () => {
  const tree = buildTree(0);
  const extent = measureDisplayExtent(tree.parts, vehicleFrame(tree), null);
  assert.ok(extent !== null);
  assert.ok(near(extent.minForward, -11) && near(extent.maxForward, 11), 'length from the carbody box');
  assert.ok(near(extent.minLeft, -1.325) && near(extent.maxLeft, 1.325), 'width from the carbody box, wider than the wheels');
  assert.ok(near(extent.minUp, -0.028) && near(extent.maxUp, 3.8), 'from the wheel flange bottom to the carbody roof');
});

test('a vehicle turned along the line measures the same as a straight one', () => {
  const straight = measureDisplayExtent(buildTree(0).parts, vehicleFrame(buildTree(0)), null);
  const turned = buildTree(0.33);
  const extent = measureDisplayExtent(turned.parts, vehicleFrame(turned), null);
  assert.ok(straight !== null && extent !== null);
  for (const key of Object.keys(straight) as (keyof typeof straight)[]) {
    assert.ok(near(straight[key], extent[key]), `${key}: ${straight[key]} vs ${extent[key]}`);
  }
});

test('a bogie group is measured in its own frame from its member bodies only', () => {
  const tree = buildTree(0.1);
  const frame = tree.bodies.get('frame_front') as THREE.Object3D;
  const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(frame.getWorldQuaternion(new THREE.Quaternion()));
  const localFrame = anchorLocalFrame(frame.getWorldPosition(new THREE.Vector3()), null, tree.root, forward);
  const extent = measureDisplayExtent(tree.parts, localFrame, new Set(['frame_front', 'wheel_ff', 'wheel_fr']));
  assert.ok(extent !== null);
  assert.ok(near(extent.minForward, -1.708) && near(extent.maxForward, 1.708), 'axles at ±1.25 m plus the wheel radius');
  assert.ok(near(extent.minLeft, -1.16) && near(extent.maxLeft, 1.16), 'the transom is the widest member');
  assert.ok(near(extent.minUp, -0.458) && near(extent.maxUp, 0.458), 'wheels above and below the frame level');
});

test('nothing framed gives null', () => {
  const tree = buildTree(0);
  assert.equal(measureDisplayExtent(tree.parts, vehicleFrame(tree), new Set(['nobody'])), null);
  assert.equal(measureDisplayExtent([], vehicleFrame(tree), null), null);
});
