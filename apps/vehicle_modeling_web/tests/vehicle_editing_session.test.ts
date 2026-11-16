import assert from 'node:assert/strict';
import { test } from 'node:test';

import type { ReferenceVehicle } from '../src/service/fetch_reference_vehicles.ts';
import type { ElementReference } from '../src/service/vehicle_modeling_service_protocol.ts';
import { checkStatus, VehicleEditingSession, previewIsStale, type SessionState } from '../src/session/vehicle_editing_session.ts';
import { FakeVehicleModelingService, fakeVehicle } from './fake_vehicle_modeling_service.ts';

// The session's sequencing against a service that follows the protocol: what
// is sent after each action, and that what the session shows is what the
// service answered last.

const vehicle: ReferenceVehicle = {
  name: 'FAKE',
  title: { en: 'Fake vehicle', zh: '假车辆' },
  openArguments: { vehicle_definition_path: '/fake/vehicle.json', startup_state_path: '/fake/startup.json', contact_record_path: '/fake/contact_record.json' },
  visualDefinitionAvailable: false,
};

const body = { kind: 'kind_body', name: 'body_a' };
const otherBody = { kind: 'kind_body', name: 'body_b' };
const frame = { kind: 'kind_frame', name: 'frame_a' };
const spring = { kind: 'kind_spring', name: 'spring_a' };

async function openedSession() {
  const service = new FakeVehicleModelingService({ '/fake/vehicle.json': fakeVehicle() });
  const session = new VehicleEditingSession(service.send, async () => null);
  await session.open(vehicle);
  return { service, session };
}

function commands(service: FakeVehicleModelingService): string[] {
  return service.log.map((entry) => entry.command);
}

function valueOf(state: SessionState, property: string): unknown {
  return state.properties.find((each) => each.name === property)?.value;
}

function outlineHas(state: SessionState, element: ElementReference): boolean {
  return state.outline!.elements.some((each) => each.kind === element.kind && each.name === element.name);
}

/** Every element the service was asked about, by get_properties, set_property or a removal. */
function elementsAskedAbout(service: FakeVehicleModelingService, since: number): unknown[] {
  return service.log.slice(since).flatMap((entry) => (entry.commandArguments['element'] === undefined ? [] : [entry.commandArguments['element']]));
}

test('opening sends the given open arguments unread, then reads the kinds, checks, draws and outlines', async () => {
  const { service, session } = await openedSession();
  assert.deepEqual(commands(service), ['open_vehicle', 'describe_element_kinds', 'check', 'get_preview', 'get_outline']);
  assert.deepEqual(service.log[0]?.commandArguments, vehicle.openArguments);
  const state = session.state;
  assert.equal(state.vehicle?.reference, vehicle);
  assert.equal(state.revision, 0);
  assert.equal(checkStatus(state), 'passed');
  assert.equal(state.preview?.projectRevision, 0);
  assert.deepEqual(
    state.elementKinds.map((kind) => kind.kind),
    fakeVehicle().kinds.map((kind) => kind.kind),
  );
  assert.deepEqual(
    state.outline?.elements.map((element) => element.kind),
    ['kind_body', 'kind_body', 'kind_frame', 'kind_spring', 'kind_carrier', 'kind_interface', 'kind_interface'],
  );
  assert.equal(state.selection, null);
});

test('a refused open keeps the vehicle that was open and says why', async () => {
  const { service, session } = await openedSession();
  const before = session.state;
  await session.open({ ...vehicle, openArguments: { vehicle_definition_path: '/missing.json' } });
  assert.equal(commands(service).at(-1), 'open_vehicle');
  assert.equal(session.state.outline, before.outline);
  assert.equal(session.state.vehicle, before.vehicle);
  assert.match(session.state.commandRefusal?.message ?? '', /missing/);
});

test('selecting reads the element’s properties; an element the outline does not have is not asked about and leaves nothing selected', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  assert.deepEqual(session.state.selection, body);
  assert.deepEqual(
    session.state.properties.map((property) => property.name),
    ['name', 'free_flag', 'size_quantity', 'offset_quantity', 'turn_quantity'],
  );
  const sent = service.log.length;
  await session.select({ kind: 'kind_body', name: 'missing' });
  assert.equal(service.log.length, sent);
  assert.equal(session.state.selection, null);
  assert.deepEqual(session.state.properties, []);
});

test('an accepted edit is followed by check, preview, outline and the selection’s properties', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  const sent = service.log.length;
  assert.equal(await session.setProperty(body, 'size_quantity', 12.5), true);
  assert.deepEqual(commands(service).slice(sent), ['set_property', 'check', 'get_preview', 'get_outline', 'get_properties']);
  const state = session.state;
  assert.equal(state.revision, 1);
  assert.equal(state.canUndo, true);
  assert.equal(valueOf(state, 'size_quantity'), 12.5);
  assert.equal(state.preview?.projectRevision, 1);
  assert.equal(previewIsStale(state), false);
});

test('references, tokens, booleans and curves are set by their values, and an optional reference to none', async () => {
  const { session } = await openedSession();
  await session.select(spring);
  assert.equal(await session.setProperty(spring, 'end_b', 'frame_a'), true);
  assert.equal(await session.setProperty(spring, 'axis_choice', 'axis_z'), true);
  assert.equal(await session.setProperty(spring, 'curve_quantity', [{ speed: 2, push: 8 }]), true);
  assert.deepEqual([valueOf(session.state, 'end_b'), valueOf(session.state, 'axis_choice'), valueOf(session.state, 'curve_quantity')], ['frame_a', 'axis_z', [{ speed: 2, push: 8 }]]);
  await session.select(body);
  assert.equal(await session.setProperty(body, 'free_flag', false), true);
  assert.equal(valueOf(session.state, 'free_flag'), false);
  const wheel = { kind: 'kind_interface', name: 'body_b.left' };
  await session.select(wheel);
  assert.equal(await session.setProperty(wheel, 'hinge_ref', null), true);
  assert.equal(valueOf(session.state, 'hinge_ref'), null);
  assert.equal(await session.setProperty(wheel, 'hinge_ref', 'no_such_hinge'), false);
  assert.equal(session.state.editRefusal?.property, 'hinge_ref');
});

test('an edit the check refuses keeps the older preview, which is then stale', async () => {
  const { service, session } = await openedSession();
  await session.select(spring);
  const sent = service.log.length;
  assert.equal(await session.setProperty(spring, 'rate_quantity', [-1, 2, 3]), true);
  assert.deepEqual(commands(service).slice(sent), ['set_property', 'check', 'get_outline', 'get_properties']);
  const state = session.state;
  assert.equal(checkStatus(state), 'failed');
  assert.equal(state.check?.report.refusals.length, 1);
  assert.equal(state.preview?.projectRevision, 0);
  assert.equal(previewIsStale(state), true);
});

test('a refused edit changes nothing, reads nothing again and is kept until the next edit', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  const before = session.state;
  const sent = service.log.length;
  assert.equal(await session.setProperty(body, 'size_quantity', [1, 2, 3]), false);
  assert.deepEqual(commands(service).slice(sent), ['set_property']);
  assert.equal(session.state.revision, before.revision);
  assert.equal(session.state.properties, before.properties);
  assert.deepEqual(session.state.editRefusal?.element, body);
  assert.equal(session.state.editRefusal?.property, 'size_quantity');
  assert.deepEqual(session.state.editRefusal?.value, [1, 2, 3]);
  await session.setProperty(body, 'size_quantity', 11);
  assert.equal(session.state.editRefusal, null);
});

test('a rename keeps the renamed element selected under its new name, and what named it follows', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  const sent = service.log.length;
  assert.equal(await session.setProperty(body, 'name', 'body_c'), true);
  const renamed = { kind: 'kind_body', name: 'body_c' };
  assert.deepEqual(session.state.selection, renamed);
  assert.equal(valueOf(session.state, 'name'), 'body_c');
  assert.deepEqual(elementsAskedAbout(service, sent), [body, renamed]);
  await session.select(frame);
  assert.equal(valueOf(session.state, 'body_ref'), 'body_c');
  const before = service.log.length;
  assert.equal(await session.setProperty(renamed, 'name', 'body_b'), false, 'a taken name is refused');
  assert.equal(session.state.editRefusal?.property, 'name');
  assert.deepEqual(commands(service).slice(before), ['set_property']);
  assert.deepEqual(session.state.selection, frame);
});

test('from the moment a rename is asked for until it is answered, the session says the element is being renamed', async () => {
  const { session } = await openedSession();
  await session.select(body);
  // The panel freezes the element's other fields on this: an edit queued now would carry the name being replaced.
  const renamed = session.setProperty(body, 'name', 'body_c');
  assert.deepEqual(session.state.renaming, body);
  assert.equal(await renamed, true);
  assert.equal(session.state.renaming, null);
  const resized = session.setProperty({ kind: 'kind_body', name: 'body_c' }, 'size_quantity', 31);
  assert.equal(session.state.renaming, null, 'an edit of anything but the name is no rename');
  assert.equal(await resized, true);
  const refused = session.setProperty({ kind: 'kind_body', name: 'body_c' }, 'name', 'body_b');
  assert.deepEqual(session.state.renaming, { kind: 'kind_body', name: 'body_c' });
  assert.equal(await refused, false);
  assert.equal(session.state.renaming, null, 'a refused rename is over too');
});

test('undo and redo add revisions and are followed by the same reading as an edit', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  await session.setProperty(body, 'size_quantity', 20);
  const sent = service.log.length;
  await session.undo();
  assert.deepEqual(commands(service).slice(sent), ['undo', 'check', 'get_preview', 'get_outline', 'get_properties']);
  assert.equal(session.state.revision, 2);
  assert.deepEqual([session.state.canUndo, session.state.canRedo], [false, true]);
  assert.equal(valueOf(session.state, 'size_quantity'), 10);
  await session.redo();
  assert.equal(session.state.revision, 3);
  assert.equal(valueOf(session.state, 'size_quantity'), 20);
  assert.equal(session.state.preview?.projectRevision, 3);
});

test('undo or redo drops a selection the project no longer has, and asks nothing about it', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  await session.setProperty(body, 'name', 'body_c');
  let sent = service.log.length;
  await session.undo();
  assert.equal(session.state.selection, null);
  assert.deepEqual(commands(service).slice(sent), ['undo', 'check', 'get_preview', 'get_outline']);
  assert.ok(outlineHas(session.state, body));
  await session.select(body);
  sent = service.log.length;
  await session.redo();
  assert.equal(session.state.selection, null, 'redo renamed the selection away again');
  assert.deepEqual(elementsAskedAbout(service, sent), []);
});

test('creating an element sends only what is given, then selects it and reads its properties', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  await session.startCreation('kind_frame');
  assert.equal(session.state.creatingKind, 'kind_frame');
  assert.equal(session.state.selection, null);
  const sent = service.log.length;
  assert.equal(await session.createElement('kind_frame', 'frame_b', { body_ref: 'body_b' }), null);
  assert.deepEqual(commands(service).slice(sent), ['add_element', 'check', 'get_preview', 'get_outline', 'get_properties']);
  assert.deepEqual(service.log[sent]?.commandArguments, { kind: 'kind_frame', name: 'frame_b', properties: { body_ref: 'body_b' } });
  const created = { kind: 'kind_frame', name: 'frame_b' };
  assert.deepEqual(session.state.selection, created);
  assert.equal(valueOf(session.state, 'body_ref'), 'body_b');
  assert.equal(session.state.creatingKind, null);
  assert.equal(session.state.revision, 1);
  assert.ok(outlineHas(session.state, created));
});

test('a refused creation changes nothing and names the property at fault, which the form shows', async () => {
  const { service, session } = await openedSession();
  await session.startCreation('kind_frame');
  const sent = service.log.length;
  const taken = await session.createElement('kind_frame', 'frame_a', { body_ref: 'body_a' });
  assert.equal(taken?.property, 'name');
  const missing = await session.createElement('kind_frame', 'frame_c', {});
  assert.equal(missing?.property, 'body_ref');
  const dangling = await session.createElement('kind_frame', 'frame_c', { body_ref: 'frame_a' });
  assert.equal(dangling?.property, 'body_ref');
  assert.deepEqual(commands(service).slice(sent), ['add_element', 'add_element', 'add_element']);
  assert.equal(session.state.revision, 0);
  assert.equal(session.state.creatingKind, 'kind_frame', 'the form stays open for correction');
});

test('a removal that takes the element alone also waits to be confirmed, and undo brings the element back', async () => {
  const { service, session } = await openedSession();
  await session.select(spring);
  let sent = service.log.length;
  await session.requestRemoval(spring);
  assert.deepEqual(commands(service).slice(sent), ['get_removal_consequences']);
  assert.deepEqual(session.state.pendingRemoval, { element: spring, removedElements: [spring] });
  assert.ok(outlineHas(session.state, spring));
  sent = service.log.length;
  await session.confirmRemoval();
  assert.deepEqual(commands(service).slice(sent), ['remove_element', 'check', 'get_preview', 'get_outline']);
  assert.equal(session.state.selection, null);
  assert.deepEqual(session.state.removedElements, [spring]);
  assert.equal(outlineHas(session.state, spring), false);
  await session.undo();
  assert.ok(outlineHas(session.state, spring));
  assert.equal(session.state.removedElements, null);
});

test('a removal that takes more waits to be confirmed: cancelling keeps everything, confirming removes all it named', async () => {
  const { service, session } = await openedSession();
  await session.select(otherBody);
  let sent = service.log.length;
  await session.requestRemoval(otherBody);
  assert.deepEqual(commands(service).slice(sent), ['get_removal_consequences']);
  const goes = [otherBody, spring, { kind: 'kind_carrier', name: 'body_b' }, { kind: 'kind_interface', name: 'body_b.left' }, { kind: 'kind_interface', name: 'body_b.right' }];
  assert.deepEqual(session.state.pendingRemoval, { element: otherBody, removedElements: goes });
  sent = service.log.length;
  await session.cancelRemoval();
  assert.equal(service.log.length, sent);
  assert.equal(session.state.pendingRemoval, null);
  assert.deepEqual(session.state.selection, otherBody);
  await session.requestRemoval(otherBody);
  sent = service.log.length;
  await session.confirmRemoval();
  assert.deepEqual(commands(service).slice(sent), ['remove_element', 'check', 'get_preview', 'get_outline']);
  assert.deepEqual(session.state.removedElements, goes);
  assert.ok(goes.every((element) => !outlineHas(session.state, element)));
  assert.equal(session.state.selection, null);
  assert.equal(session.state.pendingRemoval, null);
  assert.equal(session.state.revision, 1);
});

test('a removal the service refuses removes nothing and says why, about the element', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  const sent = service.log.length;
  await session.requestRemoval(body);
  assert.deepEqual(commands(service).slice(sent), ['get_removal_consequences']);
  assert.deepEqual(session.state.commandRefusal?.element, body);
  assert.match(session.state.commandRefusal?.message ?? '', /cannot be removed/);
  assert.equal(session.state.pendingRemoval, null);
  assert.deepEqual(session.state.selection, body);
  assert.equal(session.state.revision, 0);
});

test('nothing is asked about an element the outline no longer has, whatever asks', async () => {
  const { service, session } = await openedSession();
  await session.select(spring);
  await session.requestRemoval(spring);
  await session.confirmRemoval();
  const sent = service.log.length;
  // A stale drawing still shows the spring; a field of it may still commit.
  await session.select(spring);
  assert.equal(await session.setProperty(spring, 'axis_choice', 'axis_y'), false);
  await session.requestRemoval(spring);
  assert.equal(service.log.length, sent);
  assert.equal(session.state.selection, null);
});

test('actions asked for together run one after another', async () => {
  const { service, session } = await openedSession();
  const sent = service.log.length;
  await Promise.all([session.select(body), session.setProperty(body, 'size_quantity', 3), session.select(spring)]);
  assert.deepEqual(commands(service).slice(sent), [
    'get_properties',
    'set_property',
    'check',
    'get_preview',
    'get_outline',
    'get_properties',
    'get_properties',
  ]);
  assert.deepEqual(session.state.selection, spring);
});

test('a check of an earlier revision does not count as the current status', async () => {
  const { session } = await openedSession();
  const state = session.state;
  assert.equal(checkStatus({ ...state, revision: state.revision + 1 }), 'not_checked');
});

test('a failing server ends the session loudly and nothing more is sent', async () => {
  const service = new FakeVehicleModelingService({ '/fake/vehicle.json': fakeVehicle() });
  let reachable = true;
  const session = new VehicleEditingSession(async (command, commandArguments) => {
    if (!reachable) throw new Error('the modelling server is gone');
    return service.send(command, commandArguments);
  }, async () => null);
  await session.open(vehicle);
  reachable = false;
  await session.select(body);
  assert.match(session.state.failure ?? '', /gone/);
  reachable = true;
  const sent = service.log.length;
  await session.select(body);
  assert.equal(service.log.length, sent);
});

test('nothing asked for against the shown project while another vehicle opens is sent', async () => {
  const other = fakeVehicle();
  other.vehicleName = 'OTHER';
  const service = new FakeVehicleModelingService({ '/fake/vehicle.json': fakeVehicle(), '/fake/other.json': other });
  let gate: Promise<void> | null = null;
  let releaseVisualDefinition = (): void => {};
  const session = new VehicleEditingSession(service.send, async () => {
    await gate;
    return null;
  });
  await session.open(vehicle);
  await session.select(body);
  gate = new Promise((release) => {
    releaseVisualDefinition = release;
  });
  const otherVehicle = { ...vehicle, name: 'OTHER', openArguments: { ...vehicle.openArguments, vehicle_definition_path: '/fake/other.json' } };
  const opening = session.open(otherVehicle);
  const shownWhileOpening = session.state.opening;
  // The panels still show the first vehicle; everything asked for against it now must not reach the second.
  const edited = session.setProperty(body, 'size_quantity', 99);
  const created = session.createElement('kind_frame', 'frame_b', { body_ref: 'body_a' });
  const others = [session.select(spring), session.undo(), session.redo(), session.requestRemoval(spring), session.startCreation('kind_body')];
  releaseVisualDefinition();
  await opening;
  await Promise.all(others);
  const sinceOpen = service.log.slice(service.log.findIndex((entry) => entry.commandArguments['vehicle_definition_path'] === '/fake/other.json'));
  assert.deepEqual(
    sinceOpen.map((entry) => entry.command),
    ['open_vehicle', 'describe_element_kinds', 'check', 'get_preview', 'get_outline'],
  );
  await session.select(body);
  assert.equal(session.state.outline?.vehicleName, 'OTHER');
  assert.equal(session.state.revision, 0);
  assert.equal(valueOf(session.state, 'size_quantity'), 10);
  assert.equal(await edited, false);
  assert.equal(await created, null);
  assert.equal(session.state.creatingKind, null);
  assert.equal(shownWhileOpening, otherVehicle);
  assert.equal(session.state.opening, null);
});

test('actions asked for before an open still run first, against the project they were asked for', async () => {
  const other = fakeVehicle();
  other.vehicleName = 'OTHER';
  const service = new FakeVehicleModelingService({ '/fake/vehicle.json': fakeVehicle(), '/fake/other.json': other });
  const session = new VehicleEditingSession(service.send, async () => null);
  await session.open(vehicle);
  await session.select(body);
  const sent = service.log.length;
  const edited = session.setProperty(body, 'size_quantity', 12);
  const opening = session.open({ ...vehicle, openArguments: { ...vehicle.openArguments, vehicle_definition_path: '/fake/other.json' } });
  assert.equal(await edited, true);
  await opening;
  assert.deepEqual(
    service.log.slice(sent).map((entry) => entry.command),
    ['set_property', 'check', 'get_preview', 'get_outline', 'get_properties', 'open_vehicle', 'describe_element_kinds', 'check', 'get_preview', 'get_outline'],
  );
});
