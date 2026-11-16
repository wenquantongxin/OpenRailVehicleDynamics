// Drives the real modelling service through the modelling server the way the
// interface does, with the interface's own session, readers and drawing, and
// asserts the structure of what comes back, never its numbers. For each
// reference vehicle: open, outline, the properties of one element of every
// kind, the drawing with one wheel per contact interface, an edit the check
// passes, an edit the check refuses, undo and redo. Then a vehicle whose
// contact record does not fit it: it opens, its check refuses it, there is no
// drawing, and its outline and properties read.
// The reference vehicles are this app's catalogue; the vehicle that does not
// fit is made of the repository's IRW vehicle and GZ18 contact record.
// Exits non-zero at the first broken expectation.
//
// Usage: npm run check-service -- <service binary> <repository root>

import assert from 'node:assert/strict';
import { join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { readReferenceVehicleCatalogue, type ReferenceVehicle as CatalogueVehicle } from '../server/reference_vehicle_catalogue.ts';
import { startServer } from '../server/server.ts';
import { createModelPalette } from '../src/scene/materials.ts';
import { assignPartsToDrawnBodies, PreviewScene } from '../src/scene/preview_scene.ts';
import { commandChannel } from '../src/service/command_channel.ts';
import { elementKey, type ElementReference, type PropertyReading } from '../src/service/protocol.ts';
import { fetchReferenceVehicles, fetchVisualDefinitionText, type ReferenceVehicle } from '../src/service/reference_vehicles.ts';
import { checkStatus, EditingSession, previewIsStale, type SessionState } from '../src/session/editing_session.ts';
import { parseVisualDefinition } from '../src/visual/visual_definition.ts';

const [servicePath, repositoryRoot] = process.argv.slice(2);
if (servicePath === undefined || repositoryRoot === undefined) {
  throw new Error('usage: npm run check-service -- <service binary> <repository root>');
}

const library = (folder: string, ...path: string[]): string => resolve(join(repositoryRoot, 'vehicle_library', folder, ...path));

/** The IRW vehicle and start-up state with the GZ18 contact record, which does not fit them. */
const nonFittingVehicle: CatalogueVehicle = {
  name: 'IRW_WITH_GZ18_CONTACT_RECORD',
  title: { en: 'IRW vehicle with the GZ18 contact record', zh: 'IRW 车辆配 GZ18 接触记录' },
  openArguments: {
    vehicle_definition_path: library('irw', 'vehicle_definition.json'),
    startup_state_path: library('irw', 'startup_states', 'moving_startup_60kmh.json'),
    contact_record_path: library('gz18', 'contact_record.json'),
  },
  visualDefinitionPath: null,
};

function fail(message: string): never {
  throw new Error(message);
}

/** The session's state once the action is done, with nothing refused and nothing failed. */
async function settled(session: EditingSession, action: Promise<unknown>): Promise<SessionState> {
  await action;
  const state = session.state;
  if (state.failure !== null) fail(`the session failed: ${state.failure}`);
  if (state.commandRefusal !== null) fail(`the service refused: ${state.commandRefusal}`);
  return state;
}

/** The editable number properties of every element, in outline order, with their values. */
async function editableNumbers(session: EditingSession, elements: ElementReference[]): Promise<{ element: ElementReference; property: PropertyReading & { valueKind: 'number' } }[]> {
  const found = [];
  for (const element of elements) {
    const state = await settled(session, session.select(element));
    for (const property of state.properties) {
      if (property.editable && property.valueKind === 'number') found.push({ element, property });
    }
  }
  return found;
}

function sessionAt(origin: string): EditingSession {
  return new EditingSession(commandChannel(origin), async (reference) =>
    reference.visualDefinitionAvailable ? parseVisualDefinition(await fetchVisualDefinitionText(origin, reference)) : null,
  );
}

/**
 * Reads the properties of every element: each has some, all in value kinds the
 * interface shows, and a value the element does not hold is never editable.
 */
async function readEveryElement(session: EditingSession, elements: ElementReference[]): Promise<{ properties: number; absent: number }> {
  const counts = { properties: 0, absent: 0 };
  for (const element of elements) {
    const state = await settled(session, session.select(element));
    assert.ok(state.properties.length > 0, `${element.kind} ${element.name} has no properties`);
    for (const property of state.properties) {
      if (property.value === null) {
        assert.equal(property.editable, false, `${element.name}.${property.name} holds no value but is editable`);
        counts.absent += 1;
      }
    }
    counts.properties += state.properties.length;
  }
  return counts;
}

async function checkVehicle(origin: string, vehicle: ReferenceVehicle): Promise<string> {
  const session = sessionAt(origin);
  let state = await settled(session, session.open(vehicle));
  const outline = state.outline ?? fail('no outline after opening');
  assert.ok(outline.vehicleName.length > 0);
  assert.equal(state.revision, 0);
  assert.equal(checkStatus(state), 'passed', `${vehicle.name} does not pass its check when opened: ${state.check?.report.refusals.join('; ')}`);
  const kinds = [...new Set(outline.elements.map((element) => element.kind))];
  // The outline lists the elements kind by kind: each kind is one run.
  assert.equal(kinds.length, outline.elements.filter((element, index) => outline.elements[index - 1]?.kind !== element.kind).length);
  // The interface does not read contact_record_present; it is reported to tell which service was checked.
  const rawOutline = await (await fetch(`${origin}/api/command`, { method: 'POST', body: JSON.stringify({ id: 0, command: 'get_outline' }) })).json();

  const read = await readEveryElement(session, outline.elements);
  state = session.state;

  // The drawing: every drawing shows one outline element and no element is drawn twice; the visual parts all find their body.
  const preview = state.preview ?? fail('no preview after a passed check');
  assert.equal(preview.projectRevision, state.revision);
  const drawn = new PreviewScene(preview, state.vehicle?.visualDefinition ?? null, createModelPalette());
  const outlineKeys = new Set(outline.elements.map(elementKey));
  const drawnKeys = drawn.drawings.map((drawing) => elementKey(drawing.element));
  assert.ok(drawnKeys.length > 0 && drawnKeys.every((key) => outlineKeys.has(key)));
  assert.equal(new Set(drawnKeys).size, drawnKeys.length);
  assert.deepEqual(assignPartsToDrawnBodies(state.vehicle?.visualDefinition ?? null, preview).withoutBody, []);
  const sharedNames = drawn.drawings.filter((drawing) =>
    outline.elements.some((element) => element.name === drawing.element.name && element.kind !== drawing.element.kind),
  ).length;

  // The wheels: a wheel names its contact interface, so the contact interfaces are the outline's elements of the
  // wheels' kind. There is one wheel per contact interface, and every wheel turns with a drawn rigid body.
  const wheelElements = preview.wheels.map((wheel) => elementKey(wheel.element));
  assert.ok(wheelElements.length > 0, `${vehicle.name} is drawn without wheels`);
  const contactInterfaceKind = preview.wheels[0]!.element.kind;
  const contactInterfaces = outline.elements.filter((element) => element.kind === contactInterfaceKind).map(elementKey);
  assert.deepEqual([...wheelElements].sort(), [...contactInterfaces].sort(), 'the wheels are not one per contact interface of the outline');
  const drawnBodies = new Set(preview.rigidBodies.map((body) => elementKey(body.element)));
  const wheelBodies = new Set(preview.wheels.map((wheel) => elementKey(wheel.wheelBody)));
  for (const body of wheelBodies) assert.ok(drawnBodies.has(body), `the wheel body ${body} is not a drawn rigid body`);

  // An edit: twice an editable positive number, read back as sent and checked; the check passes it and draws it.
  const numbers = await editableNumbers(session, outline.elements.filter((element, index) => outline.elements.findIndex((other) => other.kind === element.kind) === index));
  const edited = numbers.find((entry) => entry.property.value! > 0) ?? fail('no editable positive number');
  const doubled = 2 * edited.property.value!;
  await settled(session, session.select(edited.element));
  assert.equal(await session.setProperty(edited.element, edited.property.name, doubled), true);
  state = session.state;
  assert.equal(state.revision, 1);
  assert.equal(state.canUndo, true);
  assert.equal(state.properties.find((property) => property.name === edited.property.name)?.value, doubled);
  assert.equal(checkStatus(state), 'passed', `doubling ${edited.property.name} was refused: ${state.check?.report.refusals.join('; ')}`);
  assert.equal(state.preview?.projectRevision, state.revision);

  // An edit the check refuses: the negative of an editable positive number, until one is refused.
  let refused: SessionState | null = null;
  for (const entry of numbers.filter((candidate) => candidate.property.value! > 0)) {
    await settled(session, session.select(entry.element));
    assert.equal(await session.setProperty(entry.element, entry.property.name, -entry.property.value!), true);
    if (checkStatus(session.state) === 'failed') {
      refused = session.state;
      break;
    }
    await settled(session, session.undo());
  }
  refused ??= fail('no negated number made the check refuse');
  assert.ok(refused.check!.report.refusals.length > 0);
  assert.ok(previewIsStale(refused), 'a refused check keeps the older drawing');

  // Undo brings back a project the check passes, and a drawing of it; redo steps forward again.
  state = await settled(session, session.undo());
  assert.equal(state.canRedo, true);
  assert.equal(checkStatus(state), 'passed');
  assert.equal(state.preview?.projectRevision, state.revision);
  const beforeRedo = state.revision;
  state = await settled(session, session.redo());
  assert.equal(state.revision, beforeRedo + 1);
  assert.equal(checkStatus(state), 'failed');

  return [
    `${vehicle.name}: ${outline.elements.length} elements in ${kinds.length} kinds, ${read.properties} properties read, ${read.absent} without a value and none of those editable,`,
    `  contact_record_present ${String(rawOutline.result.contact_record_present)},`,
    `  ${drawn.drawings.length} drawings, each of its own element; ${sharedNames} of them share their name with an element of another kind,`,
    `  ${wheelElements.length} wheels, one per contact interface, turning with ${wheelBodies.size} drawn rigid bodies,`,
    `  check passed on open, refused '${refused.check!.report.refusals[0]}', revision ${state.revision} after undo and redo`,
  ].join('\n');
}

async function checkNonFittingVehicle(origin: string, vehicle: ReferenceVehicle): Promise<string> {
  const session = sessionAt(origin);
  const state = await settled(session, session.open(vehicle));
  assert.equal(checkStatus(state), 'failed', `${vehicle.name} passes its check`);
  const refusals = state.check!.report.refusals;
  assert.ok(refusals.length > 0);
  assert.equal(state.preview, null);
  const outline = state.outline ?? fail('no outline after opening');
  assert.ok(outline.elements.length > 0);
  const read = await readEveryElement(session, outline.elements);
  assert.equal(session.state.preview, null);
  return [
    `${vehicle.name}: opened, check refused '${refusals[0]}' (${refusals.length} refusal${refusals.length === 1 ? '' : 's'}), no preview,`,
    `  ${outline.elements.length} elements outlined, ${read.properties} properties read`,
  ].join('\n');
}

// The reference vehicles exactly as serve:reference offers them, and the one that does not fit; any free port.
const referenceVehicles = await readReferenceVehicleCatalogue(fileURLToPath(new URL('../reference_vehicles.json', import.meta.url)));
const options = {
  servicePath: resolve(servicePath),
  host: '127.0.0.1',
  port: 0,
  staticDirectory: fileURLToPath(new URL('../dist', import.meta.url)),
  referenceVehicles: [...referenceVehicles, nonFittingVehicle],
};
const server = await startServer(options, (ending) => {
  console.error(`check-service: the service ${ending}`);
  process.exit(1);
});
try {
  const origin = `http://127.0.0.1:${server.port}`;
  for (const vehicle of await fetchReferenceVehicles(origin)) {
    console.log(vehicle.name === nonFittingVehicle.name ? await checkNonFittingVehicle(origin, vehicle) : await checkVehicle(origin, vehicle));
  }
  console.log('check-service: passed');
} catch (error) {
  console.error(`check-service: ${error instanceof Error ? error.message : String(error)}`);
  process.exitCode = 1;
} finally {
  await server.close();
}
