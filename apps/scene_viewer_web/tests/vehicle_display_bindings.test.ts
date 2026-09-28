import assert from 'node:assert/strict';
import { test } from 'node:test';

import { parseVisualDefinition } from '../src/record/visual_definition.ts';
import { resolveVehicleDisplayBindings } from '../src/scene/vehicle_display_bindings.ts';
import { makeRecord, placement, visualDefinitionWithBindings } from './record_fixtures.ts';

// Bodies carry no conventional prefix on purpose: every role must come from
// the bindings, every wheel pairing from the record's carrier body names.
// body_c is an axle-bridge-like carrier with two separate wheel bodies;
// body_f is a rigid-wheelset-like carrier whose two wheels are itself.
const bodies = ['body_a', 'body_b', 'body_c', 'body_d', 'body_e', 'body_f', 'body_g'];
const placements = [
  placement('if_right_c', 'body_e', 'body_c', 'right'),
  placement('if_left_f', 'body_f', 'body_f', 'left', 1),
  placement('if_left_c', 'body_d', 'body_c', 'left'),
  placement('if_right_f', 'body_f', 'body_f', 'right', 1),
];
const positions = {
  // Two carriers 0.12 m apart in x: closer than any grouping distance the old code used.
  body_a: [0, 0, -1] as [number, number, number],
  body_b: [8.75, 0, -0.4] as [number, number, number],
  body_c: [10.0, 0, -0.43] as [number, number, number],
  body_d: [10.0, 0, -0.43] as [number, number, number],
  body_e: [10.0, 0, -0.43] as [number, number, number],
  body_f: [9.88, 0, -0.43] as [number, number, number],
  body_g: [8.75, 0, -1.0] as [number, number, number],
};

function record(visualDefinitionText: string | null) {
  return makeRecord({
    bodies,
    placements,
    frames: [{ timeSeconds: 0, poses: Object.fromEntries(Object.entries(positions).map(([name, position]) => [name, { position }])) }],
    visualDefinitionText,
  });
}

const explicitBindings = {
  carbody: { body_name: 'body_a', display_name: { en: 'Carbody', zh: '车体' } },
  bogies: [{ body_name: 'body_b', display_name: { en: 'Bogie frame · end 1', zh: '一位端构架' }, member_body_names: ['body_c', 'body_f', 'body_g'] }],
  // Reverse of the geometric order: body_f sits behind body_c but is listed first.
  carriers: [
    { carrier_body_name: 'body_f', display_name: { en: 'Wheelset 1', zh: '1 位轮对' } },
    { carrier_body_name: 'body_c', display_name: { en: 'Axle bridge 2', zh: '2 位轴桥' } },
  ],
};

function resolve(bindings: unknown) {
  const text = visualDefinitionWithBindings(bindings);
  const rec = record(text);
  return resolveVehicleDisplayBindings(rec, parseVisualDefinition(text, new Set(rec.bodies.map((body) => body.name))));
}

test('explicit bindings pair wheels through carriers and keep the stated order', () => {
  const bindings = resolve(explicitBindings);
  assert.equal(bindings.hasVisualDefinition, true);
  assert.deepEqual(bindings.carbody?.bodyName, 'body_a');
  assert.deepEqual(
    bindings.carriers.map((carrier) => carrier.bodyName),
    ['body_f', 'body_c'],
    'the display order is the binding order, not the geometric order',
  );
  const [wheelset, bridge] = bindings.carriers;
  assert.ok(wheelset !== undefined && bridge !== undefined);
  assert.equal(wheelset.leftWheelPlacementIndex, 1);
  assert.equal(wheelset.rightWheelPlacementIndex, 3);
  assert.equal(bridge.leftWheelPlacementIndex, 2, 'the left wheel is found by side, not by array position');
  assert.equal(bridge.rightWheelPlacementIndex, 0);
  assert.equal(wheelset.bogieIndex, 0);
  assert.equal(bridge.bogieIndex, 0);
  // Separate wheel bodies join their carrier's group without being listed.
  const group = bindings.bogies[0];
  assert.ok(group !== undefined);
  assert.deepEqual(
    group.memberBodyIndices.map((index) => bodies[index]),
    ['body_b', 'body_c', 'body_d', 'body_e', 'body_f', 'body_g'],
  );
  assert.equal(bindings.bogieIndexOfBody[0], -1, 'the carbody belongs to no bogie group');
});

test('reversing the placement array changes nothing about the pairing', () => {
  const text = visualDefinitionWithBindings(explicitBindings);
  const base = record(text);
  const reversed = { ...base, wheelPlacements: [...base.wheelPlacements].reverse() };
  const bindings = resolveVehicleDisplayBindings(reversed, parseVisualDefinition(text, new Set(bodies)));
  const bridge = bindings.carriers[1];
  assert.ok(bridge !== undefined);
  assert.equal(reversed.wheelPlacements[bridge.leftWheelPlacementIndex ?? -1]?.wheelBodyName, 'body_d');
  assert.equal(reversed.wheelPlacements[bridge.rightWheelPlacementIndex ?? -1]?.wheelBodyName, 'body_e');
});

test('without a visual definition carriers keep their raw names and there are no roles', () => {
  const rec = record(null);
  const bindings = resolveVehicleDisplayBindings(rec, null);
  assert.equal(bindings.hasVisualDefinition, false);
  assert.equal(bindings.carbody, null);
  assert.deepEqual(bindings.bogies, []);
  assert.deepEqual(
    bindings.carriers.map((carrier) => [carrier.bodyName, carrier.displayName.en, carrier.bogieIndex]),
    [
      ['body_c', 'body_c', null],
      ['body_f', 'body_f', null],
    ],
  );
});

test('a wrong reference is refused; an omitted role is an absence', () => {
  assert.throws(() => resolve({ ...explicitBindings, carbody: { body_name: 'body_x', display_name: { en: 'x', zh: 'x' } } }), /does not list/);
  assert.throws(() => resolve({ ...explicitBindings, carriers: explicitBindings.carriers.slice(0, 1) }), /has no display binding/);
  assert.throws(() => resolve({ ...explicitBindings, carriers: [...explicitBindings.carriers, explicitBindings.carriers[0]] }), /twice/);
  assert.throws(
    () =>
      resolve({
        ...explicitBindings,
        bogies: [
          ...explicitBindings.bogies,
          { body_name: 'body_g', display_name: { en: 'other', zh: '其他' }, member_body_names: ['body_c'] },
        ],
      }),
    /two bogies/,
  );
  assert.throws(() => resolve({ ...explicitBindings, bogies: [{ ...explicitBindings.bogies[0], member_body_names: ['body_a'] }] }), /bogie member/);
  const withoutCarbody = resolve({ ...explicitBindings, carbody: null });
  assert.equal(withoutCarbody.carbody, null);
  assert.equal(withoutCarbody.carriers.length, 2, 'carriers do not depend on the carbody');
});

test('a record whose placement names an unlisted carrier body is refused', () => {
  const rec = record(null);
  const broken = { ...rec, wheelPlacements: [placement('if', 'body_d', 'body_missing', 'left')] };
  assert.throws(() => resolveVehicleDisplayBindings(broken, null), /does not list/);
  const doubled = { ...rec, wheelPlacements: [placement('if1', 'body_d', 'body_c', 'left'), placement('if2', 'body_e', 'body_c', 'left')] };
  assert.throws(() => resolveVehicleDisplayBindings(doubled, null), /two left wheels/);
});
