import assert from 'node:assert/strict';
import { test } from 'node:test';
import * as THREE from 'three';

import { createModelPalette } from '../src/scene/materials.ts';
import { assignPartsToDrawnBodies, PreviewScene } from '../src/scene/preview_scene.ts';
import { readPreview, sameElement, type ElementReference } from '../src/service/protocol.ts';
import { parseVisualDefinition } from '../src/visual/visual_definition.ts';

// The drawing is built from the preview alone: poses, parts in their body's
// frame, marks at the drawn places, each drawing showing the element the
// preview names. Checked through world positions, which is what the camera
// sees. The frame shares its name with a body on purpose: names are shared
// across kinds, elements are not.

// A quarter turn about the drawing frame's z axis: body x points along drawing y.
const quarterTurn = [
  [0, -1, 0],
  [1, 0, 0],
  [0, 0, 1],
];
const identity = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
];

const turnedBody = { kind: 'kind_body', name: 'turned' };
const frameNamedLikeBody = { kind: 'kind_frame', name: 'turned' };
// A rigid wheelset turning its two wheels, and a wheel turning on its own body.
const wheelset = { kind: 'kind_body', name: 'wheelset' };
const ownWheelBody = { kind: 'kind_body', name: 'own_wheel' };
const leftWheel = { kind: 'kind_interface', name: 'wheelset.left' };
const rightWheel = { kind: 'kind_interface', name: 'wheelset.right' };
const ownWheel = { kind: 'kind_interface', name: 'own_wheel.right' };
// The joint the independently rotating wheel turns in, at the wheel's datum: inside the wheel, as seen from above.
const bearing = { kind: 'kind_hinge', name: 'own_wheel_bearing' };

const previewJson = {
  project_revision: 2,
  rigid_bodies: [
    { element: turnedBody, rotation_drawing_from_body: quarterTurn, origin_in_drawing_frame_meters: [4, 1, -2] },
    { element: { kind: 'kind_body', name: 'bare' }, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [-3, 0, -0.5] },
    { element: wheelset, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [8, 0, -0.43] },
    { element: ownWheelBody, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [12, 0.75, -0.43] },
  ],
  fixed_frames: [{ element: frameNamedLikeBody, rotation_drawing_from_frame: identity, origin_in_drawing_frame_meters: [4, 1, -1] }],
  revolute_joints: [
    { element: { kind: 'kind_hinge', name: 'hinge' }, location_in_drawing_frame_meters: [1, 0, -0.4], axis_in_drawing_frame: [0, 1, 0] },
    { element: bearing, location_in_drawing_frame_meters: [12, 0.75, -0.43], axis_in_drawing_frame: [0, 1, 0] },
  ],
  ball_rpy_joints: [{ element: { kind: 'kind_ball', name: 'ball' }, location_in_drawing_frame_meters: [2, 0, -0.4] }],
  weld_joints: [{ element: { kind: 'kind_weld', name: 'weld' }, location_in_drawing_frame_meters: [3, 0, -0.4] }],
  force_elements: [
    { element: { kind: 'kind_link', name: 'coincident' }, reference_end_in_drawing_frame_meters: [0, 1, -1], opposite_end_in_drawing_frame_meters: [0, 1, -1] },
    { element: { kind: 'kind_link', name: 'apart' }, reference_end_in_drawing_frame_meters: [0, -1, -1], opposite_end_in_drawing_frame_meters: [3, -1, -1] },
  ],
  wheels: [
    { element: leftWheel, wheel_body: wheelset, datum_in_drawing_frame_meters: [8, -0.75, -0.43], outward_spin_axis_in_drawing_frame: [0, -1, 0], nominal_rolling_radius_meters: 0.43 },
    { element: rightWheel, wheel_body: wheelset, datum_in_drawing_frame_meters: [8, 0.75, -0.43], outward_spin_axis_in_drawing_frame: [0, 1, 0], nominal_rolling_radius_meters: 0.43 },
    { element: ownWheel, wheel_body: ownWheelBody, datum_in_drawing_frame_meters: [12, 0.75, -0.43], outward_spin_axis_in_drawing_frame: [0, 1, 0], nominal_rolling_radius_meters: 0.43 },
  ],
  not_drawable_elements: [{ kind: 'kind_body', name: 'unplaced' }],
};

const visualJson = {
  length_unit: 'meter',
  parts: [
    { name: 'nose', body_name: 'turned', shape: 'sphere', appearance: 'structure', center_in_body_frame_meters: [1, 0, 0], radius_meters: 0.2 },
    { name: 'skin', body_name: 'turned', shape: 'box', appearance: 'shell', center_in_body_frame_meters: [0, 0, -1], size_meters: [2, 1, 1] },
    { name: 'away', body_name: 'unplaced', shape: 'sphere', appearance: 'dark', center_in_body_frame_meters: [0, 0, 0], radius_meters: 0.1 },
    { name: 'gone', body_name: 'removed', shape: 'sphere', appearance: 'dark', center_in_body_frame_meters: [0, 0, 0], radius_meters: 0.1 },
    { name: 'axle', body_name: 'wheelset', shape: 'cylinder', appearance: 'dark', start_in_body_frame_meters: [0, -0.8, 0], end_in_body_frame_meters: [0, 0.8, 0], radius_meters: 0.08 },
  ],
  wheel_visual: {
    width_meters: 0.135,
    back_face_offset_meters: 0.07,
    flange_height_meters: 0.028,
    flange_thickness_meters: 0.032,
    rim_depth_meters: 0.045,
    web_thickness_meters: 0.03,
    hub_radius_meters: 0.11,
    hub_length_meters: 0.13,
  },
};

/** Three's world position of a drawing-frame point: (x, y, z) goes to (x, -z, y). */
function world(x: number, y: number, z: number): THREE.Vector3 {
  return new THREE.Vector3(x, -z, y);
}

const preview = readPreview({ preview: previewJson })!;
const visualDefinition = parseVisualDefinition(JSON.stringify(visualJson));

function build(palette = createModelPalette()) {
  return new PreviewScene(preview, visualDefinition, palette);
}

function drawingOf(scene: PreviewScene, element: ElementReference) {
  const drawing = scene.drawings.find((candidate) => sameElement(candidate.element, element));
  assert.ok(drawing, element.name);
  return drawing;
}

const link = (name: string): ElementReference => ({ kind: 'kind_link', name });

function near(actual: THREE.Vector3, expected: THREE.Vector3, what: string): void {
  assert.ok(actual.distanceTo(expected) < 1e-9, `${what}: ${actual.toArray()} vs ${expected.toArray()}`);
}

test('parts hang under their body at its pose, in the body’s own frame', () => {
  const turned = drawingOf(build(), turnedBody);
  assert.equal(turned.parts.length, 2);
  // The nose is one metre along body x, which the quarter turn points along drawing +y.
  const nose = turned.parts.find((part) => part.mesh.geometry instanceof THREE.SphereGeometry)!;
  near(nose.mesh.getWorldPosition(new THREE.Vector3()), world(4, 2, -2), 'nose');
});

test('a body without parts is drawn as one marker at its origin', () => {
  const bare = drawingOf(build(), { kind: 'kind_body', name: 'bare' });
  assert.equal(bare.parts.length, 0);
  assert.equal(bare.marks.length, 1);
  near(bare.marks[0]!.mesh.getWorldPosition(new THREE.Vector3()), world(-3, 0, -0.5), 'marker');
});

test('parts whose body is not drawn are left out of the drawing and listed', () => {
  const { withoutBody } = assignPartsToDrawnBodies(visualDefinition, preview);
  assert.deepEqual(
    withoutBody.map((part) => part.name),
    ['away', 'gone'],
  );
  const scene = build();
  assert.equal(drawingOf(scene, turnedBody).parts.length, 2);
  assert.equal(
    scene.drawings.some((drawing) => drawing.element.name === 'unplaced' || drawing.element.name === 'removed'),
    false,
  );
});

test('force elements are a point where their ends meet and a bar between distinct ends', () => {
  const scene = build();
  const point = drawingOf(scene, link('coincident')).marks[0]!;
  assert.equal(point.shape, 'point');
  near(point.mesh.getWorldPosition(new THREE.Vector3()), world(0, 1, -1), 'point');
  const bar = drawingOf(scene, link('apart')).marks[0]!;
  assert.equal(bar.shape, 'bar');
  const box = new THREE.Box3().setFromObject(bar.mesh);
  assert.ok(Math.abs(box.max.x - box.min.x - 3) < 1e-6, 'the bar spans the three metres between its ends');
  near(box.getCenter(new THREE.Vector3()), world(1.5, -1, -1), 'bar centre');
});

test('joints are marks at their locations, a revolute joint along its axis', () => {
  const scene = build();
  const hinge = drawingOf(scene, { kind: 'kind_hinge', name: 'hinge' }).marks[0]!;
  const box = new THREE.Box3().setFromObject(hinge.mesh);
  // The axis is drawing +y, which is three's +z.
  assert.ok(box.max.z - box.min.z > 5 * (box.max.x - box.min.x));
  near(drawingOf(scene, { kind: 'kind_ball', name: 'ball' }).marks[0]!.mesh.getWorldPosition(new THREE.Vector3()), world(2, 0, -0.4), 'ball');
  near(drawingOf(scene, { kind: 'kind_weld', name: 'weld' }).marks[0]!.mesh.getWorldPosition(new THREE.Vector3()), world(3, 0, -0.4), 'weld');
});

test('frames are hidden until asked for, and a selected frame shows anyway', () => {
  const scene = build();
  const frame = drawingOf(scene, frameNamedLikeBody);
  assert.equal(scene.shown(frame), false);
  scene.setDisplay({ parts: 'solid', frames: true });
  assert.equal(scene.shown(frame), true);
  scene.setDisplay({ parts: 'solid', frames: false });
  scene.setHighlight(frameNamedLikeBody);
  assert.equal(scene.shown(frame), true);
});

test('selecting a body recolours all its parts and nothing else, not even a frame of the same name', () => {
  const palette = createModelPalette();
  const scene = build(palette);
  scene.setHighlight(turnedBody);
  const selectedMaterials = new Set([palette.selectedPart, palette.selectedTranslucentPart, palette.selectedMark]);
  for (const drawing of scene.drawings) {
    const meshes = [...drawing.parts.map((part) => part.mesh), ...drawing.marks.map((each) => each.mesh)];
    const highlighted = meshes.filter((mesh) => selectedMaterials.has(mesh.material as THREE.Material)).length;
    assert.equal(highlighted, sameElement(drawing.element, turnedBody) ? meshes.length : 0, `${drawing.element.kind} ${drawing.element.name}`);
  }
});

test('a click picks what it hits; through x-ray parts, a mark comes before the body around it', () => {
  const scene = build();
  const raycaster = new THREE.Raycaster();
  // Straight down onto the marker of 'bare'.
  raycaster.set(world(-3, 0, -5), new THREE.Vector3(0, -1, 0));
  assert.deepEqual(scene.pick(raycaster)?.element, { kind: 'kind_body', name: 'bare' });
  // Through the shell of the turned body (drawing x 3.5..4.5, y 0.5..1.5, z -3.5..-2.5) onto the frame inside it.
  scene.setDisplay({ parts: 'solid', frames: true });
  raycaster.set(world(4, 1, -10), new THREE.Vector3(0, -1, 0));
  assert.deepEqual(scene.pick(raycaster)?.element, turnedBody);
  scene.setDisplay({ parts: 'xray', frames: true });
  assert.deepEqual(scene.pick(raycaster)?.element, frameNamedLikeBody);
});

test('marks are never drawn smaller than their least size on screen, and never smaller than they are', () => {
  const scene = build();
  const camera = new THREE.PerspectiveCamera(30, 1, 0.1, 1000);
  const point = drawingOf(scene, link('coincident')).marks[0]!;
  camera.position.copy(world(0, 1, -1)).add(new THREE.Vector3(0, 0, 200));
  scene.fitMarksToScreen(camera, 800);
  assert.ok(point.mesh.scale.x > 1);
  camera.position.copy(world(0, 1, -1)).add(new THREE.Vector3(0, 0, 0.5));
  scene.fitMarksToScreen(camera, 800);
  assert.equal(point.mesh.scale.x, 1);
  const bar = drawingOf(scene, link('apart')).marks[0]!;
  camera.position.set(0, 0, 300);
  scene.fitMarksToScreen(camera, 800);
  assert.equal(bar.mesh.scale.y, 1, 'a bar keeps its length');
  assert.ok(bar.mesh.scale.x > 1);
});

/** The meshes of one wheel: those standing for its contact interface. */
function wheelMeshes(scene: PreviewScene, wheel: ElementReference): THREE.Mesh[] {
  return scene.drawings.flatMap((drawing) => drawing.parts.filter((part) => sameElement(part.ownElement, wheel)).map((part) => part.mesh));
}

test('a wheel is a part of the body it turns with: a rigid wheelset and its two wheels are one drawing', () => {
  const scene = build();
  const drawing = drawingOf(scene, wheelset);
  const left = wheelMeshes(scene, leftWheel);
  const right = wheelMeshes(scene, rightWheel);
  assert.ok(left.length > 0 && right.length > 0);
  assert.deepEqual(
    new Set(drawing.parts.map((part) => part.mesh)),
    new Set([...drawing.parts.filter((part) => part.ownElement === null).map((part) => part.mesh), ...left, ...right]),
  );
  assert.equal(drawing.parts.filter((part) => part.ownElement === null).length, 1, 'the axle');
});

test('a wheel stands on its datum, turned about its outward spin axis, and a body of wheels alone has no marker', () => {
  const scene = build();
  const own = drawingOf(scene, ownWheelBody);
  assert.equal(own.marks.length, 0);
  assert.deepEqual(new Set(own.parts.map((part) => part.mesh)), new Set(wheelMeshes(scene, ownWheel)));
  for (const [wheel, datum, outward] of [
    [leftWheel, world(8, -0.75, -0.43), world(0, -1, 0)],
    [rightWheel, world(8, 0.75, -0.43), world(0, 1, 0)],
  ] as const) {
    const turning = wheelMeshes(scene, wheel)[0]!.parent!;
    near(turning.getWorldPosition(new THREE.Vector3()), datum, `${wheel.name} datum`);
    near(new THREE.Vector3(0, 1, 0).applyQuaternion(turning.getWorldQuaternion(new THREE.Quaternion())), outward, `${wheel.name} axis`);
  }
});

test('a click on a wheel picks the body it turns with', () => {
  const scene = build();
  const raycaster = new THREE.Raycaster();
  // Straight down onto the top of the independently rotating wheel's tread.
  raycaster.set(world(12, 0.75, -2), new THREE.Vector3(0, -1, 0));
  assert.deepEqual(scene.pick(raycaster)?.element, ownWheelBody);
  raycaster.set(world(8, -0.75, -2), new THREE.Vector3(0, -1, 0));
  assert.deepEqual(scene.pick(raycaster)?.element, wheelset);
});

test('what is drawn is what a ray can pick: a wheel highlighted while parts are hidden hides the joint behind it and picks its body', () => {
  const scene = build();
  const raycaster = new THREE.Raycaster();
  // Straight down onto the top of the independently rotating wheel, with its bearing behind it.
  raycaster.set(world(12, 0.75, -2), new THREE.Vector3(0, -1, 0));
  scene.setDisplay({ parts: 'hidden', frames: false });
  assert.deepEqual(scene.pick(raycaster)?.element, bearing, 'the hidden wheel is passed through');
  scene.setHighlight(ownWheel);
  assert.deepEqual(scene.pick(raycaster)?.element, ownWheelBody, 'the highlighted wheel is drawn, solid, in front of the bearing');
  // A wheel that is not drawn cannot be picked.
  raycaster.set(world(8, -0.75, -2), new THREE.Vector3(0, -1, 0));
  assert.equal(scene.pick(raycaster), null);
  // A selected mark is drawn over everything, so it is picked even through the solid wheel in front of it.
  scene.setDisplay({ parts: 'solid', frames: false });
  scene.setHighlight(bearing);
  raycaster.set(world(12, 0.75, -2), new THREE.Vector3(0, -1, 0));
  assert.deepEqual(scene.pick(raycaster)?.element, bearing);
});

test('selecting a contact interface highlights its wheel alone, also while parts are hidden; selecting the body highlights all its wheels', () => {
  const palette = createModelPalette();
  const scene = build(palette);
  const highlightedMeshes = (): THREE.Mesh[] =>
    scene.drawings.flatMap((drawing) => drawing.parts.map((part) => part.mesh)).filter((mesh) => mesh.material === palette.selectedPart);
  scene.setDisplay({ parts: 'hidden', frames: false });
  scene.setHighlight(rightWheel);
  assert.deepEqual(new Set(highlightedMeshes()), new Set(wheelMeshes(scene, rightWheel)));
  assert.ok(wheelMeshes(scene, rightWheel).every((mesh) => mesh.visible));
  assert.ok(wheelMeshes(scene, leftWheel).every((mesh) => !mesh.visible));
  scene.setDisplay({ parts: 'solid', frames: false });
  scene.setHighlight(wheelset);
  assert.deepEqual(new Set(highlightedMeshes()), new Set(drawingOf(scene, wheelset).parts.map((part) => part.mesh)));
});
