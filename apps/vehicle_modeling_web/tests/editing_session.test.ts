import assert from 'node:assert/strict';
import { test } from 'node:test';

import type { ReferenceVehicle } from '../src/service/reference_vehicles.ts';
import { checkStatus, EditingSession, previewIsStale } from '../src/session/editing_session.ts';
import { FakeModelingService, fakeVehicle } from './fake_modeling_service.ts';

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
const spring = { kind: 'kind_spring', name: 'spring_a' };

async function openedSession() {
  const service = new FakeModelingService({ '/fake/vehicle.json': fakeVehicle() });
  const session = new EditingSession(service.send, async () => null);
  await session.open(vehicle);
  return { service, session };
}

function commands(service: FakeModelingService): string[] {
  return service.log.map((entry) => entry.command);
}

test('opening sends the given open arguments unread, then checks, draws and outlines', async () => {
  const { service, session } = await openedSession();
  assert.deepEqual(commands(service), ['open_vehicle', 'check', 'get_preview', 'get_outline']);
  assert.deepEqual(service.log[0]?.commandArguments, vehicle.openArguments);
  const state = session.state;
  assert.equal(state.vehicle?.reference, vehicle);
  assert.equal(state.revision, 0);
  assert.equal(checkStatus(state), 'passed');
  assert.equal(state.preview?.projectRevision, 0);
  assert.deepEqual(
    state.outline?.elements.map((element) => element.kind),
    ['kind_body', 'kind_body', 'kind_spring', 'kind_carrier', 'kind_interface', 'kind_interface'],
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
  assert.match(session.state.commandRefusal ?? '', /missing/);
});

test('selecting reads the element’s properties; a refused selection keeps the selection', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  assert.deepEqual(session.state.selection, body);
  assert.deepEqual(
    session.state.properties.map((property) => property.name),
    ['name', 'size_quantity', 'offset_quantity'],
  );
  await session.select({ kind: 'kind_body', name: 'missing' });
  assert.deepEqual(session.state.selection, body);
  assert.notEqual(session.state.commandRefusal, null);
  assert.equal(commands(service).filter((command) => command === 'get_properties').length, 2);
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
  assert.equal(state.properties.find((property) => property.name === 'size_quantity')?.value, 12.5);
  assert.equal(state.preview?.projectRevision, 1);
  assert.equal(previewIsStale(state), false);
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

test('undo and redo add revisions and are followed by the same reading as an edit', async () => {
  const { service, session } = await openedSession();
  await session.select(body);
  await session.setProperty(body, 'size_quantity', 20);
  const sent = service.log.length;
  await session.undo();
  assert.deepEqual(commands(service).slice(sent), ['undo', 'check', 'get_preview', 'get_outline', 'get_properties']);
  assert.equal(session.state.revision, 2);
  assert.deepEqual([session.state.canUndo, session.state.canRedo], [false, true]);
  assert.equal(session.state.properties.find((property) => property.name === 'size_quantity')?.value, 10);
  await session.redo();
  assert.equal(session.state.revision, 3);
  assert.equal(session.state.properties.find((property) => property.name === 'size_quantity')?.value, 20);
  assert.equal(session.state.preview?.projectRevision, 3);
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
  const service = new FakeModelingService({ '/fake/vehicle.json': fakeVehicle() });
  let reachable = true;
  const session = new EditingSession(async (command, commandArguments) => {
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
  const service = new FakeModelingService({ '/fake/vehicle.json': fakeVehicle(), '/fake/other.json': other });
  let gate: Promise<void> | null = null;
  let releaseVisualDefinition = (): void => {};
  const session = new EditingSession(service.send, async () => {
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
  const others = [session.select(spring), session.undo(), session.redo()];
  releaseVisualDefinition();
  await opening;
  await Promise.all(others);
  const sinceOpen = service.log.slice(service.log.findIndex((entry) => entry.commandArguments['vehicle_definition_path'] === '/fake/other.json'));
  assert.deepEqual(
    sinceOpen.map((entry) => entry.command),
    ['open_vehicle', 'check', 'get_preview', 'get_outline'],
  );
  await session.select(body);
  assert.equal(session.state.outline?.vehicleName, 'OTHER');
  assert.equal(session.state.revision, 0);
  assert.equal(session.state.properties.find((property) => property.name === 'size_quantity')?.value, 10);
  assert.equal(await edited, false);
  assert.equal(shownWhileOpening, otherVehicle);
  assert.equal(session.state.opening, null);
});

test('actions asked for before an open still run first, against the project they were asked for', async () => {
  const other = fakeVehicle();
  other.vehicleName = 'OTHER';
  const service = new FakeModelingService({ '/fake/vehicle.json': fakeVehicle(), '/fake/other.json': other });
  const session = new EditingSession(service.send, async () => null);
  await session.open(vehicle);
  await session.select(body);
  const sent = service.log.length;
  const edited = session.setProperty(body, 'size_quantity', 12);
  const opening = session.open({ ...vehicle, openArguments: { ...vehicle.openArguments, vehicle_definition_path: '/fake/other.json' } });
  assert.equal(await edited, true);
  await opening;
  assert.deepEqual(
    service.log.slice(sent).map((entry) => entry.command),
    ['set_property', 'check', 'get_preview', 'get_outline', 'get_properties', 'open_vehicle', 'check', 'get_preview', 'get_outline'],
  );
});
