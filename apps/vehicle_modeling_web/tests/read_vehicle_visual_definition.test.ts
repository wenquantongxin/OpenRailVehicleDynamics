import assert from 'node:assert/strict';
import { test } from 'node:test';

import { readVisualDefinition } from '../src/visual/read_vehicle_visual_definition.ts';

const wheelVisual = {
  width_meters: 0.135,
  back_face_offset_meters: 0.07,
  flange_height_meters: 0.028,
  flange_thickness_meters: 0.032,
  rim_depth_meters: 0.045,
  web_thickness_meters: 0.03,
  hub_radius_meters: 0.11,
  hub_length_meters: 0.13,
};

/** A document as the service gives it: a JSON object, here one with the given parts. */
function definition(parts: unknown[], extra: Record<string, unknown> = {}): unknown {
  return JSON.parse(JSON.stringify({ length_unit: 'meter', parts, wheel_visual: wheelVisual, ...extra }));
}

const box = { name: 'shell', body_name: 'body_a', shape: 'box', appearance: 'shell', center_in_body_frame_meters: [0, 0, -2], size_meters: [20, 3, 2.7], corner_radius_meters: 0.3 };

test('parts of every shape are read in their body frame; bindings and labels are not read', () => {
  const read = readVisualDefinition(
    definition(
      [
        { ...box, label: { en: 'Carbody', zh: '车体' } },
        { name: 'axle', body_name: 'body_b', shape: 'cylinder', appearance: 'dark', start_in_body_frame_meters: [0, -1, 0], end_in_body_frame_meters: [0, 1, 0], radius_meters: 0.08 },
        { name: 'joint', body_name: 'body_b', shape: 'sphere', appearance: 'accent', center_in_body_frame_meters: [1, 0, 0], radius_meters: 0.1 },
        { name: 'origin', body_name: 'body_a', shape: 'axes', origin_in_body_frame_meters: [0, 0, 0], length_meters: 2 },
      ],
      { display_bindings: null },
    ),
  );
  assert.deepEqual(
    read.parts.map((part) => [part.kind, part.bodyName]),
    [
      ['box', 'body_a'],
      ['cylinder', 'body_b'],
      ['sphere', 'body_b'],
      ['axes', 'body_a'],
    ],
  );
  assert.deepEqual(read.parts[0], { name: 'shell', bodyName: 'body_a', appearance: 'shell', kind: 'box', centerInBodyFrameMeters: [0, 0, -2], sizeMeters: [20, 3, 2.7], cornerRadiusMeters: 0.3 });
});

test('a part that cannot be drawn as stated is refused', () => {
  assert.throws(() => readVisualDefinition({ length_unit: 'millimeter', parts: [] }), /length_unit/);
  assert.throws(() => readVisualDefinition({ unrelated: 'document' }), /length_unit/);
  const axle = { name: 'axle', body_name: 'body_b', shape: 'cylinder', appearance: 'dark', start_in_body_frame_meters: [0, 1, 0], end_in_body_frame_meters: [0, 1, 0], radius_meters: 0.08 };
  assert.throws(() => readVisualDefinition(definition([axle])), /axle' starts and ends at the same point/);
  assert.throws(() => readVisualDefinition(definition([{ ...box, shape: 'cone' }])), /unknown shape/);
  assert.throws(() => readVisualDefinition(definition([{ ...box, appearance: 'chrome' }])), /appearance/);
  assert.throws(() => readVisualDefinition(definition([{ ...box, size_meters: [1, 2] }])), /three-vector/);
  assert.throws(() => readVisualDefinition(definition([{ ...box, body_name: undefined }])), /names no body/);
});

test('of wheel_visual, the eight lengths of the section are read and checked; nothing else', () => {
  const read = readVisualDefinition(definition([], { wheel_visual: { ...wheelVisual, rim_marker: true, labels: 'not read' } }));
  assert.deepEqual(read.wheelSection, {
    widthMeters: 0.135,
    backFaceOffsetMeters: 0.07,
    flangeHeightMeters: 0.028,
    flangeThicknessMeters: 0.032,
    rimDepthMeters: 0.045,
    webThicknessMeters: 0.03,
    hubRadiusMeters: 0.11,
    hubLengthMeters: 0.13,
  });
  assert.throws(() => readVisualDefinition(definition([], { wheel_visual: undefined })), /wheel_visual/);
  assert.throws(() => readVisualDefinition(definition([], { wheel_visual: { ...wheelVisual, hub_radius_meters: 0 } })), /hub_radius_meters/);
  assert.throws(() => readVisualDefinition(definition([], { wheel_visual: { ...wheelVisual, back_face_offset_meters: 0.2 } })), /back face/);
});
