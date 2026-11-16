// Drives the real modelling service through the modelling server the way the
// interface does, with the interface's own session, readers and drawing, and
// asserts the structure of what comes back, never its numbers. For each
// reference vehicle: open, outline, the properties of every element, the
// drawing with one wheel per contact interface, an edit the check passes, an
// edit the check refuses, undo and redo. On GZ18, structural editing: a body,
// a frame on it and a force element from that frame to one the vehicle had
// are created; the body is renamed and the frame follows; removing the body
// takes the frame and the force element with it; undoing everything gives
// back the outline the vehicle opened with. Then a vehicle whose contact
// record does not fit it: it opens, its check refuses it, there is no drawing,
// and its outline and properties read.
// The reference vehicles are this app's catalogue; the vehicle that does not
// fit is made of the repository's IRW vehicle and GZ18 contact record. Kinds
// are told apart by the preview's drawings and properties by their
// descriptions: no kind and no property is named here.
// Exits non-zero at the first broken expectation.
//
// Usage: npm run check-service -- <service binary> <repository root>

import assert from 'node:assert/strict';
import { join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { readReferenceVehicleCatalogue, type ReferenceVehicleCatalogueEntry } from '../server/read_reference_vehicle_catalogue.ts';
import { startVehicleModelingHttpServer } from '../server/vehicle_modeling_http_server.ts';
import { createModelPalette } from '../src/scene/vehicle_model_view_colors_and_materials.ts';
import { assignPartsToDrawnBodies, VehiclePreviewScene } from '../src/scene/vehicle_preview_scene.ts';
import { fetchReferenceVehicles, fetchVisualDefinitionText, type ReferenceVehicle } from '../src/service/fetch_reference_vehicles.ts';
import { modelingServerCommandChannel } from '../src/service/modeling_server_command_channel.ts';
import { elementKey, sameElement, type ElementReference, type PropertyDescription, type PropertyReading } from '../src/service/vehicle_modeling_service_protocol.ts';
import { checkStatus, VehicleEditingSession, previewIsStale, type SessionState } from '../src/session/vehicle_editing_session.ts';
import type { ReferenceDescription } from '../src/ui/element_reference_picker_rules.ts';
import { parseVisualDefinition } from '../src/visual/read_vehicle_visual_definition.ts';

const [servicePath, repositoryRoot] = process.argv.slice(2);
if (servicePath === undefined || repositoryRoot === undefined) {
  throw new Error('usage: npm run check-service -- <service binary> <repository root>');
}

const library = (folder: string, ...path: string[]): string => resolve(join(repositoryRoot, 'vehicle_library', folder, ...path));

/** The IRW vehicle and start-up state with the GZ18 contact record, which does not fit them. */
const nonFittingVehicle: ReferenceVehicleCatalogueEntry = {
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
async function settled(session: VehicleEditingSession, action: Promise<unknown>): Promise<SessionState> {
  await action;
  const state = session.state;
  if (state.failure !== null) fail(`the session failed: ${state.failure}`);
  if (state.commandRefusal !== null) fail(`the service refused: ${state.commandRefusal.message}`);
  return state;
}

/** The editable number properties of every element, in outline order, with their values. */
async function editableNumbers(session: VehicleEditingSession, elements: ElementReference[]): Promise<{ element: ElementReference; property: PropertyReading & { valueKind: 'number' } }[]> {
  const found = [];
  for (const element of elements) {
    const state = await settled(session, session.select(element));
    for (const property of state.properties) {
      if (property.editable && property.valueKind === 'number') found.push({ element, property });
    }
  }
  return found;
}

function sessionAt(origin: string): VehicleEditingSession {
  return new VehicleEditingSession(modelingServerCommandChannel(origin), async (reference) =>
    reference.visualDefinitionAvailable ? parseVisualDefinition(await fetchVisualDefinitionText(origin, reference)) : null,
  );
}

/**
 * Reads the properties of every element: each has some, all in value kinds the
 * interface shows. A value the element does not hold is never editable, apart
 * from an optional reference that names none, which is.
 */
async function readEveryElement(session: VehicleEditingSession, elements: ElementReference[]): Promise<{ properties: number; absent: number; namingNone: number }> {
  const counts = { properties: 0, absent: 0, namingNone: 0 };
  for (const element of elements) {
    const state = await settled(session, session.select(element));
    assert.ok(state.properties.length > 0, `${element.kind} ${element.name} has no properties`);
    for (const property of state.properties) {
      if (property.value !== null) continue;
      if (property.valueKind === 'element_reference' && property.optional) {
        assert.equal(property.editable, true, `${element.name}.${property.name} names none and cannot be set`);
        counts.namingNone += 1;
      } else {
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
  const drawn = new VehiclePreviewScene(preview, state.vehicle?.visualDefinition ?? null, createModelPalette());
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
    `  ${read.namingNone} optional references naming none, each of them editable,`,
    `  contact_record_present ${String(rawOutline.result.contact_record_present)},`,
    `  ${drawn.drawings.length} drawings, each of its own element; ${sharedNames} of them share their name with an element of another kind,`,
    `  ${wheelElements.length} wheels, one per contact interface, turning with ${wheelBodies.size} drawn rigid bodies,`,
    `  check passed on open, refused '${refused.check!.report.refusals[0]}', revision ${state.revision} after undo and redo`,
  ].join('\n');
}

/** A name no element of the outline has. */
function unusedName(elements: readonly ElementReference[], stem: string): string {
  let name = stem;
  for (let count = 2; elements.some((element) => element.name === name); count += 1) name = `${stem}_${count}`;
  return name;
}

/**
 * Structural editing through the interface's session. The kinds come from
 * the preview's lists, a body's, a frame's and a force element's, and every
 * property sent is found by its description: the name by its value kind, the
 * references a new element must give by being required.
 */
async function checkStructuralEditing(origin: string, vehicle: ReferenceVehicle): Promise<string> {
  const session = sessionAt(origin);
  let state = await settled(session, session.open(vehicle));
  const opened = state.outline ?? fail('no outline after opening');
  const preview = state.preview ?? fail('no preview after opening');
  const bodyKind = (preview.rigidBodies[0] ?? fail('nothing drawn as a rigid body')).element.kind;
  const frameKind = (preview.fixedFrames[0] ?? fail('nothing drawn as a fixed frame')).element.kind;
  const forceKind = (preview.forceElements[0] ?? fail('nothing drawn as a force element')).element.kind;
  const properties = (kind: string): PropertyDescription[] => (state.elementKinds.find((each) => each.kind === kind) ?? fail(`'${kind}' is not described`)).properties;
  const requiredReferences = (kind: string): ReferenceDescription[] =>
    properties(kind).filter((property): property is ReferenceDescription => property.valueKind === 'element_reference' && !property.optional);
  const created = async (kind: string, stem: string, given: Record<string, string>): Promise<ElementReference> => {
    const name = unusedName(opened.elements, stem);
    const refusal = await session.createElement(kind, name, given);
    assert.equal(refusal, null, `creating a ${kind} was refused: ${refusal?.message}`);
    const element = { kind, name };
    assert.deepEqual(session.state.selection, element, 'a created element is selected');
    return element;
  };

  // A body, given nothing but its name: the service's defaults stand for the rest.
  const body = await created(bodyKind, 'structural_check_body', {});
  // A frame on it: every reference a frame must give names the body.
  const frameReferences = requiredReferences(frameKind);
  assert.ok(frameReferences.length > 0 && frameReferences.every((reference) => reference.referencedKinds.includes(bodyKind)));
  const frame = await created(frameKind, 'structural_check_frame', Object.fromEntries(frameReferences.map((reference) => [reference.name, body.name])));
  // A force element from that frame to a frame the vehicle had.
  const ends = requiredReferences(forceKind);
  assert.ok(ends.length >= 2 && ends.every((end) => end.referencedKinds.includes(frameKind)), 'a force element names the frames at its ends');
  const vehicleFrame = opened.elements.find((element) => element.kind === frameKind)!;
  const force = await created(forceKind, 'structural_check_force', Object.fromEntries(ends.map((end, index) => [end.name, index === 0 ? frame.name : vehicleFrame.name])));

  // Renaming the selected body: the selection and the frame on it follow.
  const nameProperty = properties(bodyKind).find((property) => property.valueKind === 'element_name') ?? fail('a body has no name to set');
  const renamed = { kind: bodyKind, name: unusedName(opened.elements, 'structural_check_body_renamed') };
  await settled(session, session.select(body));
  assert.equal(await session.setProperty(body, nameProperty.name, renamed.name), true, `the rename was refused: ${session.state.editRefusal?.message}`);
  assert.deepEqual(session.state.selection, renamed);
  state = await settled(session, session.select(frame));
  for (const reference of frameReferences) assert.equal(state.properties.find((property) => property.name === reference.name)?.value, renamed.name);

  // Removing the body takes the frame and the force element with it; it waits to be confirmed.
  await settled(session, session.select(renamed));
  state = await settled(session, session.requestRemoval(renamed));
  const pending = state.pendingRemoval ?? fail('removing the body would take nothing but the body');
  for (const goes of [renamed, frame, force]) assert.ok(pending.removedElements.some((each) => sameElement(each, goes)), `${goes.name} would not be removed with the body`);
  state = await settled(session, session.confirmRemoval());
  assert.deepEqual(state.removedElements, pending.removedElements);
  assert.ok(pending.removedElements.every((gone) => !state.outline!.elements.some((each) => sameElement(each, gone))));
  assert.equal(state.selection, null);

  // Five edits undone give back the vehicle as it was opened.
  for (let step = 0; step < 5; step += 1) state = await settled(session, session.undo());
  assert.equal(state.canUndo, false);
  assert.deepEqual(state.outline!.elements, opened.elements);

  return [
    `${vehicle.name} structural editing: created ${bodyKind} '${body.name}', ${frameKind} '${frame.name}' on it and ${forceKind} '${force.name}' from it to '${vehicleFrame.name}',`,
    `  renamed the body to '${renamed.name}' and the frame followed, removed it with ${pending.removedElements.length - 1} elements that referred to it,`,
    `  five undos gave back the outline of ${opened.elements.length} elements the vehicle opened with; check ${checkStatus(state)} at revision ${state.revision}`,
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
const referenceVehicles = await readReferenceVehicleCatalogue(fileURLToPath(new URL('../reference_vehicle_catalogue.json', import.meta.url)));
const options = {
  servicePath: resolve(servicePath),
  host: '127.0.0.1',
  port: 0,
  staticDirectory: fileURLToPath(new URL('../dist', import.meta.url)),
  referenceVehicles: [...referenceVehicles, nonFittingVehicle],
};
const server = await startVehicleModelingHttpServer(options, (ending) => {
  console.error(`check-service: the service ${ending}`);
  process.exit(1);
});
try {
  const origin = `http://127.0.0.1:${server.port}`;
  const vehicles = await fetchReferenceVehicles(origin);
  for (const vehicle of vehicles) {
    console.log(vehicle.name === nonFittingVehicle.name ? await checkNonFittingVehicle(origin, vehicle) : await checkVehicle(origin, vehicle));
  }
  console.log(await checkStructuralEditing(origin, vehicles.find((vehicle) => vehicle.name === 'GZ18') ?? fail('the catalogue has no GZ18')));
  console.log('check-service: passed');
} catch (error) {
  console.error(`check-service: ${error instanceof Error ? error.message : String(error)}`);
  process.exitCode = 1;
} finally {
  await server.close();
}
