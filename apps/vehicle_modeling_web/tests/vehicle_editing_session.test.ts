import assert from 'node:assert/strict';
import { test } from 'node:test';

import type { ReferenceVehicle } from '../src/service/fetch_reference_vehicles.ts';
import { rowPathOfSubject, rowWithKey } from '../src/service/find_model_tree_rows.ts';
import type { ElementReference, PropertySubject } from '../src/service/vehicle_modeling_service_protocol.ts';
import { checkStatus, creationFromOffers, VehicleEditingSession, previewIsStale, type OpenChoice, type SessionState } from '../src/session/vehicle_editing_session.ts';
import { FakeVehicleModelingService, fakeVehicle } from './fake_vehicle_modeling_service.ts';

// The session's sequencing against a service that follows the protocol: what
// is sent after each action, and that what the session shows is what the
// service answered last.

const projectsDirectory = '/fake/projects';

const vehicle: ReferenceVehicle = {
  name: 'FAKE',
  title: { en: 'Fake vehicle', zh: '假车辆' },
  openArguments: {
    vehicle_definition_path: '/fake/vehicle.json',
    startup_state_path: '/fake/startup.json',
    contact_record_path: '/fake/contact_record.json',
    visual_definition_path: null,
  },
};

function referenceVehicle(opened: ReferenceVehicle): OpenChoice {
  return { source: 'reference_vehicle', vehicle: opened };
}

function project(name: string): OpenChoice {
  return { source: 'vehicle_project', name };
}

const otherVehicle: ReferenceVehicle = { ...vehicle, name: 'OTHER', openArguments: { ...vehicle.openArguments, vehicle_definition_path: '/fake/other.json' } };

function serviceWithTwoVehicles(options: ConstructorParameters<typeof FakeVehicleModelingService>[1] = {}): FakeVehicleModelingService {
  const other = fakeVehicle();
  other.vehicleName = 'OTHER';
  return new FakeVehicleModelingService({ '/fake/vehicle.json': fakeVehicle(), '/fake/other.json': other }, { projectsDirectory, ...options });
}

/** An element as the subject of a selection or an edit. */
function subject(element: ElementReference): PropertySubject {
  return { element };
}

/** The keys of the subject's row and the rows above it, from the first level down. */
function rowKeysOf(state: SessionState, selected: PropertySubject): string[] {
  return (rowPathOfSubject(state.modelTree!, selected) ?? []).map((row) => row.key);
}

const body = { kind: 'kind_body', name: 'body_a' };
const otherBody = { kind: 'kind_body', name: 'body_b' };
const frame = { kind: 'kind_frame', name: 'frame_a' };
const spring = { kind: 'kind_spring', name: 'spring_a' };

/** A session started with nothing open and a link to the fake vehicle, which it followed. */
async function openedSession(service = serviceWithTwoVehicles()) {
  const session = new VehicleEditingSession(service.send);
  await session.start(projectsDirectory, { open: referenceVehicle(vehicle), select: null });
  return { service, session };
}

function commands(service: FakeVehicleModelingService): string[] {
  return service.log.map((entry) => entry.command);
}

function commandsSince(service: FakeVehicleModelingService, since: number): string[] {
  return commands(service).slice(since);
}

function valueOf(state: SessionState, property: string): unknown {
  return state.properties.find((each) => each.name === property)?.value;
}

function outlineHas(state: SessionState, element: ElementReference): boolean {
  return state.outline!.elements.some((each) => each.kind === element.kind && each.name === element.name);
}

/** Every element the service was asked about, by get_properties, set_property or a removal. */
function elementsAskedAbout(service: FakeVehicleModelingService, since: number): unknown[] {
  return service.log.slice(since).flatMap((entry) => {
    const asked = (entry.commandArguments['subject'] as { element?: unknown } | undefined)?.element ?? entry.commandArguments['element'];
    return asked === undefined ? [] : [asked];
  });
}

const openingCommands = ['describe_element_kinds', 'get_visual_definition', 'check', 'get_preview', 'get_outline', 'get_model_tree'];

test('with nothing open, the start lists the folder, follows the link and then selects what it names; it runs once', async () => {
  const service = serviceWithTwoVehicles();
  const session = new VehicleEditingSession(service.send);
  const link = { open: referenceVehicle(vehicle), select: body };
  await Promise.all([session.start(projectsDirectory, link), session.start(projectsDirectory, link)]);
  assert.deepEqual(commands(service), ['get_session', 'list_vehicle_projects', 'open_vehicle', ...openingCommands, 'get_properties']);
  assert.deepEqual(service.log[1]?.commandArguments, { directory: projectsDirectory }, 'the folder is passed as the server named it');
  assert.deepEqual(service.log[2]?.commandArguments, vehicle.openArguments, 'the open arguments are passed unread');
  const state = session.state;
  assert.equal(state.started, true);
  assert.deepEqual(state.vehicleProjects, []);
  assert.deepEqual(state.selection, subject(body));
  assert.equal(state.vehicleProjectFile, null);
  assert.equal(state.hasUnsavedChanges, false);
  assert.equal(state.linkNotFollowed, null);
  assert.equal(state.opening, null);
});

test('with nothing open, a link to a saved project opens it from the folder, tied to its file', async () => {
  const service = serviceWithTwoVehicles();
  const saved = fakeVehicle();
  saved.vehicleName = 'mine';
  service.savedProjects.set('mine', { vehicle: saved, visualDefinition: null });
  const session = new VehicleEditingSession(service.send);
  await session.start(projectsDirectory, { open: project('mine'), select: body });
  assert.deepEqual(commands(service), ['get_session', 'list_vehicle_projects', 'open_vehicle_project', ...openingCommands, 'get_properties']);
  assert.deepEqual(service.log[2]?.commandArguments, { directory: projectsDirectory, name: 'mine' });
  assert.deepEqual(session.state.vehicleProjectFile, { directory: projectsDirectory, name: 'mine' });
  assert.equal(session.state.outline?.vehicleName, 'mine');
  assert.deepEqual(session.state.selection, subject(body));
});

test('without a link the start lists the folder and opens nothing', async () => {
  const service = serviceWithTwoVehicles();
  const session = new VehicleEditingSession(service.send);
  await session.start(projectsDirectory, null);
  assert.deepEqual(commands(service), ['get_session', 'list_vehicle_projects']);
  assert.equal(session.state.started, true);
  assert.equal(session.state.vehicle, null);
});

test('opening sends the given open arguments unread, then reads the kinds, the visual definition, checks, draws and outlines', async () => {
  const { service, session } = await openedSession();
  const opened = commands(service).indexOf('open_vehicle');
  assert.deepEqual(commandsSince(service, opened), ['open_vehicle', ...openingCommands]);
  const state = session.state;
  assert.equal(state.revision, 0);
  assert.equal(checkStatus(state), 'passed');
  assert.equal(state.preview?.projectRevision, 0);
  assert.deepEqual(state.vehicle, { visualDefinition: null, visualDefinitionProblem: null });
  assert.deepEqual(
    state.elementKinds.map((kind) => kind.kind),
    fakeVehicle().kinds.map((kind) => kind.kind),
  );
  assert.deepEqual(
    state.outline?.elements.map((element) => element.kind),
    ['kind_body', 'kind_body', 'kind_body', 'kind_frame', 'kind_hinge', 'kind_spring', 'kind_carrier', 'kind_interface', 'kind_interface'],
  );
  assert.equal(state.selection, null);
});

test('a refused open keeps the vehicle that was open and says why', async () => {
  const { service, session } = await openedSession();
  const before = session.state;
  await session.open(referenceVehicle({ ...vehicle, openArguments: { vehicle_definition_path: '/missing.json', visual_definition_path: null } }));
  assert.equal(commands(service).at(-1), 'open_vehicle');
  assert.equal(session.state.outline, before.outline);
  assert.equal(session.state.vehicle, before.vehicle);
  assert.match(session.state.commandRefusal?.message ?? '', /missing/);
});

test('a reload resumes the open session as it stands: file, unsaved changes, kept drawing; nothing selected, the link not followed', async () => {
  const { service, session } = await openedSession();
  await session.startSaveAs();
  await session.saveAs('mine', false);
  await session.select(subject(spring));
  await session.setProperty(subject(spring), 'rate_quantity', [-1, 2, 3]);
  const before = session.state;
  assert.equal(previewIsStale(before), true, 'the check refused the edit and the older drawing is kept');

  const reloaded = new VehicleEditingSession(service.send);
  const sent = service.log.length;
  await reloaded.start(projectsDirectory, { open: referenceVehicle(otherVehicle), select: body });
  assert.deepEqual(commandsSince(service, sent), ['get_session', ...openingCommands], 'the link is not followed and the folder not listed');
  const state = reloaded.state;
  assert.deepEqual(state.outline, before.outline);
  assert.deepEqual([state.revision, state.canUndo, state.canRedo, state.hasUnsavedChanges], [before.revision, true, false, true]);
  assert.deepEqual(state.vehicleProjectFile, { directory: projectsDirectory, name: 'mine' });
  assert.equal(checkStatus(state), 'failed');
  assert.deepEqual(state.preview, before.preview, 'the drawing the service kept from before the refused check');
  assert.equal(state.selection, null);
  assert.deepEqual(state.linkNotFollowed, referenceVehicle(otherVehicle));
  assert.equal(state.vehicleProjectsDirectory, projectsDirectory);
});

test('Save without a file asks for a name; Save as writes, renames the vehicle and ties the session to the file; then Save writes there', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  await session.setProperty(subject(body), 'size_quantity', 12);
  assert.equal(session.state.hasUnsavedChanges, true);
  let sent = service.log.length;
  await session.save();
  assert.equal(service.log.length, sent, 'nothing is written without a file');
  assert.deepEqual(session.state.saveAs, { name: '', replaceQuestion: false, refusal: null, saving: false });

  sent = service.log.length;
  await session.saveAs('mine', false);
  assert.deepEqual(commandsSince(service, sent), ['list_vehicle_projects', 'save_vehicle_project_as', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties', 'list_vehicle_projects']);
  assert.deepEqual(service.log[sent + 1]?.commandArguments, { directory: projectsDirectory, name: 'mine' });
  let state = session.state;
  assert.equal(state.saveAs, null);
  assert.deepEqual(state.vehicleProjectFile, { directory: projectsDirectory, name: 'mine' });
  assert.equal(state.hasUnsavedChanges, false);
  assert.equal(state.outline?.vehicleName, 'mine');
  assert.equal(state.revision, 2, 'renaming the vehicle is one revision');
  assert.equal(state.preview?.projectRevision, 2);
  assert.deepEqual(state.vehicleProjects, ['mine']);
  assert.deepEqual(state.selection, subject(body));

  await session.setProperty(subject(body), 'size_quantity', 13);
  assert.equal(session.state.hasUnsavedChanges, true);
  sent = service.log.length;
  await session.save();
  assert.deepEqual(commandsSince(service, sent), ['save_vehicle_project']);
  state = session.state;
  assert.equal(state.hasUnsavedChanges, false);
  assert.equal(state.revision, 3, 'saving changes no revision');
  assert.equal((service.savedProjects.get('mine')?.vehicle.elements.find((each) => each.name === 'body_a')?.values['size_quantity']), 13);

  // Unsaved changes follow undo and redo: away from the saved state and back to it.
  await session.undo();
  assert.equal(session.state.hasUnsavedChanges, true);
  await session.redo();
  assert.equal(session.state.hasUnsavedChanges, false);
  // After save, undo and a new edit, the saved state cannot be reached any more: changes stay unsaved until a save.
  await session.undo();
  await session.setProperty(subject(body), 'size_quantity', 14);
  await session.undo();
  assert.deepEqual([session.state.hasUnsavedChanges, session.state.canRedo], [true, true]);
  await session.redo();
  assert.equal(session.state.hasUnsavedChanges, true);
});

test('Save as adds no undo step: undo after it keeps the new name, and Save then writes the project under that name', async () => {
  const { service, session } = await openedSession();
  await session.startSaveAs();
  await session.saveAs('mine', false);
  assert.deepEqual([session.state.revision, session.state.canUndo, session.state.canRedo], [1, false, false], 'one revision for the new name, nothing to undo');
  await session.select(subject(body));
  await session.setProperty(subject(body), 'size_quantity', 15);
  await session.startSaveAs();
  await session.saveAs('mine_again', false);
  assert.deepEqual([session.state.canUndo, session.state.canRedo], [true, false]);
  await session.undo();
  assert.equal(session.state.outline?.vehicleName, 'mine_again', 'undo brings no earlier name back');
  assert.equal(valueOf(session.state, 'size_quantity'), 10);
  assert.equal(session.state.hasUnsavedChanges, true);
  await session.save();
  assert.equal(service.savedProjects.get('mine_again')?.vehicle.vehicleName, 'mine_again');
});

test('Save as holds the project’s own name; saving under it again is no question about another project', async () => {
  const { service, session } = await openedSession();
  await session.startSaveAs();
  await session.saveAs('mine', false);
  await session.startSaveAs();
  assert.equal(session.state.saveAs?.name, 'mine');
  const sent = service.log.length;
  await session.saveAs('mine', false);
  assert.deepEqual(commandsSince(service, sent).slice(0, 2), ['list_vehicle_projects', 'save_vehicle_project_as']);
  assert.equal(session.state.saveAs, null);
  assert.equal(session.state.revision, 1, 'a vehicle that already has the name is not renamed again');
});

test('Save as onto another project of the folder asks first: another name leaves it, replacing writes over it', async () => {
  const { service, session } = await openedSession();
  await session.startSaveAs();
  await session.saveAs('other', false);
  await session.startSaveAs();
  await session.saveAs('mine', false);
  await session.select(subject(body));
  await session.setProperty(subject(body), 'size_quantity', 21);

  await session.startSaveAs();
  let sent = service.log.length;
  await session.saveAs('other', false);
  assert.deepEqual(commandsSince(service, sent), ['list_vehicle_projects'], 'nothing is written before the answer');
  assert.deepEqual(session.state.saveAs, { name: 'other', replaceQuestion: true, refusal: null, saving: false });
  session.chooseAnotherName();
  assert.deepEqual(session.state.saveAs, { name: 'other', replaceQuestion: false, refusal: null, saving: false });
  assert.deepEqual(session.state.vehicleProjectFile, { directory: projectsDirectory, name: 'mine' });

  sent = service.log.length;
  await session.saveAs('other', true);
  assert.equal(commandsSince(service, sent)[0], 'save_vehicle_project_as', 'the folder was listed before the question');
  assert.deepEqual(session.state.vehicleProjectFile, { directory: projectsDirectory, name: 'other' });
  assert.equal(session.state.hasUnsavedChanges, false);
  assert.equal(service.savedProjects.get('other')?.vehicle.elements.find((each) => each.name === 'body_a')?.values['size_quantity'], 21);
  assert.deepEqual(session.state.vehicleProjects, ['mine', 'other']);
});

test('a refused Save as keeps the dialog with the name typed and the service’s reason; nothing changes', async () => {
  const service = serviceWithTwoVehicles();
  const { session } = await openedSession(service);
  await session.startSaveAs();
  const before = session.state;
  await session.saveAs('not a name', false);
  assert.equal(session.state.saveAs?.name, 'not a name');
  assert.match(session.state.saveAs?.refusal ?? '', /not a project name/);
  assert.equal(session.state.saveAs?.saving, false);
  assert.deepEqual([session.state.revision, session.state.vehicleProjectFile, session.state.outline], [before.revision, null, before.outline]);

  service.projectsDirectoryWritable = false;
  await session.saveAs('fine_name', false);
  assert.deepEqual(session.state.saveAs, { name: 'fine_name', replaceQuestion: false, refusal: `cannot write into the folder '${projectsDirectory}'`, saving: false });
  assert.equal(service.savedProjects.size, 0);
  session.cancelSaveAs();
  assert.equal(session.state.saveAs, null);
});

test('opening anything while there are unsaved changes asks first: keeping on editing opens nothing, discarding opens it', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  await session.showOpenChoices();
  // An edit still on its way when the open is asked for counts.
  const edited = session.setProperty(subject(body), 'size_quantity', 30);
  const sent = service.log.length;
  await session.open(referenceVehicle(otherVehicle));
  assert.equal(await edited, true);
  assert.equal(commandsSince(service, sent).includes('open_vehicle'), false);
  assert.deepEqual(session.state.pendingOpen, referenceVehicle(otherVehicle));
  assert.equal(session.state.opening, null);
  session.keepEditing();
  assert.equal(session.state.pendingOpen, null);
  assert.equal(session.state.choosingWhatToOpen, false, 'keeping on editing goes back to the project');
  assert.equal(session.state.outline?.vehicleName, 'FAKE');

  await session.open(referenceVehicle(otherVehicle));
  await session.discardChangesAndOpen();
  assert.equal(commands(service).filter((command) => command === 'open_vehicle').length, 2);
  assert.equal(session.state.outline?.vehicleName, 'OTHER');
  assert.equal(session.state.pendingOpen, null);
  assert.equal(session.state.hasUnsavedChanges, false);

  // Without unsaved changes nothing is asked.
  await session.open(referenceVehicle(vehicle));
  assert.equal(session.state.pendingOpen, null);
  assert.equal(session.state.outline?.vehicleName, 'FAKE');
});

test('the choice of what to open lists the folder again over the session, and can be left without opening anything', async () => {
  const { service, session } = await openedSession();
  await session.startSaveAs();
  await session.saveAs('mine', false);
  const sent = service.log.length;
  await session.showOpenChoices();
  assert.equal(session.state.choosingWhatToOpen, true);
  assert.deepEqual(commandsSince(service, sent), ['list_vehicle_projects']);
  assert.deepEqual(session.state.vehicleProjects, ['mine']);
  session.leaveOpenChoices();
  assert.equal(session.state.choosingWhatToOpen, false);
  assert.equal(commandsSince(service, sent).length, 1);

  await session.showOpenChoices();
  await session.open(referenceVehicle(otherVehicle));
  assert.equal(session.state.choosingWhatToOpen, false, 'opening leaves the choice');
});

test('a saved project opens as it was saved, tied to its file; one its check refuses opens with no drawing', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(spring));
  await session.setProperty(subject(spring), 'rate_quantity', [-1, 2, 3]);
  await session.startSaveAs();
  await session.saveAs('refused_by_check', false);
  const saved = session.state.outline;
  await session.open(referenceVehicle(otherVehicle));

  const sent = service.log.length;
  await session.open(project('refused_by_check'));
  assert.deepEqual(commandsSince(service, sent), ['open_vehicle_project', ...openingCommands]);
  assert.deepEqual(service.log[sent]?.commandArguments, { directory: projectsDirectory, name: 'refused_by_check' });
  let state = session.state;
  assert.deepEqual(state.vehicleProjectFile, { directory: projectsDirectory, name: 'refused_by_check' });
  assert.deepEqual([state.revision, state.canUndo, state.hasUnsavedChanges], [0, false, false]);
  assert.deepEqual(state.outline, saved);
  assert.equal(checkStatus(state), 'failed');
  assert.equal(state.preview, null);
  assert.ok(state.check!.report.refusals.length > 0);
  await session.select(subject(spring));
  assert.deepEqual(valueOf(session.state, 'rate_quantity'), [-1, 2, 3]);

  await session.open(project('no_such_project'));
  state = session.state;
  assert.match(state.commandRefusal?.message ?? '', /no project named 'no_such_project'/);
  assert.deepEqual(state.vehicleProjectFile, { directory: projectsDirectory, name: 'refused_by_check' }, 'a refused open keeps the session');
});

test('a visual definition the view cannot draw from is reported, the bodies are drawn as markers, and editing goes on', async () => {
  const wheelVisual = { width_meters: 0.1, back_face_offset_meters: 0.05, flange_height_meters: 0.02, flange_thickness_meters: 0.02, rim_depth_meters: 0.03, web_thickness_meters: 0.02, hub_radius_meters: 0.1, hub_length_meters: 0.1 };
  const service = serviceWithTwoVehicles({
    visualDefinitions: { '/fake/visual.json': { length_unit: 'meter', parts: [], wheel_visual: wheelVisual }, '/fake/unusable.json': { length_unit: 'meter', parts: 'none' } },
  });
  const drawable = { ...vehicle, openArguments: { ...vehicle.openArguments, visual_definition_path: '/fake/visual.json' } };
  const session = new VehicleEditingSession(service.send);
  await session.start(projectsDirectory, { open: referenceVehicle(drawable), select: null });
  assert.notEqual(session.state.vehicle?.visualDefinition, null);
  assert.equal(session.state.vehicle?.visualDefinitionProblem, null);

  await session.open(referenceVehicle({ ...vehicle, openArguments: { ...vehicle.openArguments, visual_definition_path: '/fake/unusable.json' } }));
  assert.equal(session.state.vehicle?.visualDefinition, null);
  assert.match(session.state.vehicle?.visualDefinitionProblem ?? '', /parts is not an array/);
  assert.equal(session.state.failure, null);
  assert.equal(await session.setProperty(subject(body), 'size_quantity', 3), true);
  assert.equal(session.state.revision, 1);
  assert.match(session.state.vehicle?.visualDefinitionProblem ?? '', /parts is not an array/, 'the problem stays with the project it is about');
});

test('selecting reads the element’s properties; an element the model tree does not have is not asked about and leaves nothing selected', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  assert.deepEqual(session.state.selection, subject(body));
  assert.deepEqual(
    session.state.properties.map((property) => property.name),
    ['name', 'size_quantity', 'offset_quantity', 'turn_quantity'],
  );
  const sent = service.log.length;
  await session.select(subject({ kind: 'kind_body', name: 'missing' }));
  assert.equal(service.log.length, sent);
  assert.equal(session.state.selection, null);
  assert.deepEqual(session.state.properties, []);
});

test('an accepted edit is followed by check, preview, outline and the selection’s properties', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  const sent = service.log.length;
  assert.equal(await session.setProperty(subject(body), 'size_quantity', 12.5), true);
  assert.deepEqual(commandsSince(service, sent), ['set_property', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties']);
  const state = session.state;
  assert.equal(state.revision, 1);
  assert.equal(state.canUndo, true);
  assert.equal(state.hasUnsavedChanges, true);
  assert.equal(valueOf(state, 'size_quantity'), 12.5);
  assert.equal(state.preview?.projectRevision, 1);
  assert.equal(previewIsStale(state), false);
});

test('references, tokens and curves are set by their values, and an optional reference to none', async () => {
  const { session } = await openedSession();
  await session.select(subject(spring));
  assert.equal(await session.setProperty(subject(spring), 'end_b', 'frame_a'), true);
  assert.equal(await session.setProperty(subject(spring), 'axis_choice', 'axis_z'), true);
  assert.equal(await session.setProperty(subject(spring), 'curve_quantity', [{ speed: 2, push: 8 }]), true);
  assert.deepEqual([valueOf(session.state, 'end_b'), valueOf(session.state, 'axis_choice'), valueOf(session.state, 'curve_quantity')], ['frame_a', 'axis_z', [{ speed: 2, push: 8 }]]);
  const wheel = { kind: 'kind_interface', name: 'body_b.left' };
  await session.select(subject(wheel));
  assert.equal(await session.setProperty(subject(wheel), 'hinge_ref', null), true);
  assert.equal(valueOf(session.state, 'hinge_ref'), null);
  assert.equal(await session.setProperty(subject(wheel), 'hinge_ref', 'no_such_hinge'), false);
  assert.equal(session.state.editRefusal?.property, 'hinge_ref');
});

test('an edit the check refuses keeps the older preview, which is then stale', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(spring));
  const sent = service.log.length;
  assert.equal(await session.setProperty(subject(spring), 'rate_quantity', [-1, 2, 3]), true);
  assert.deepEqual(commandsSince(service, sent), ['set_property', 'check', 'get_outline', 'get_model_tree', 'get_properties']);
  const state = session.state;
  assert.equal(checkStatus(state), 'failed');
  assert.equal(state.check?.report.refusals.length, 1);
  assert.equal(state.preview?.projectRevision, 0);
  assert.equal(previewIsStale(state), true);
});

test('a refused edit changes nothing, reads nothing again and is kept until the next edit', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  const before = session.state;
  const sent = service.log.length;
  assert.equal(await session.setProperty(subject(body), 'size_quantity', [1, 2, 3]), false);
  assert.deepEqual(commandsSince(service, sent), ['set_property']);
  assert.equal(session.state.revision, before.revision);
  assert.equal(session.state.properties, before.properties);
  assert.deepEqual(session.state.editRefusal?.subject, subject(body));
  assert.equal(session.state.editRefusal?.property, 'size_quantity');
  assert.deepEqual(session.state.editRefusal?.value, [1, 2, 3]);
  await session.setProperty(subject(body), 'size_quantity', 11);
  assert.equal(session.state.editRefusal, null);
});

test('a rename keeps the renamed element selected under its new name, and what named it follows', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  const sent = service.log.length;
  assert.equal(await session.setProperty(subject(body), 'name', 'body_d'), true);
  const renamed = { kind: 'kind_body', name: 'body_d' };
  assert.deepEqual(session.state.selection, subject(renamed));
  assert.equal(valueOf(session.state, 'name'), 'body_d');
  assert.deepEqual(elementsAskedAbout(service, sent), [body, renamed]);
  await session.select(subject(frame));
  assert.equal(valueOf(session.state, 'body_ref'), 'body_d');
  const before = service.log.length;
  assert.equal(await session.setProperty(subject(renamed), 'name', 'body_b'), false, 'a taken name is refused');
  assert.equal(session.state.editRefusal?.property, 'name');
  assert.deepEqual(commandsSince(service, before), ['set_property']);
  assert.deepEqual(session.state.selection, subject(frame));
});

test('from the moment a rename is asked for until it is answered, the session says the element is being renamed', async () => {
  const { session } = await openedSession();
  await session.select(subject(body));
  // The panel freezes the element's other fields on this: an edit queued now would carry the name being replaced.
  const renamed = session.setProperty(subject(body), 'name', 'body_d');
  assert.deepEqual(session.state.renaming, body);
  assert.equal(await renamed, true);
  assert.equal(session.state.renaming, null);
  const resized = session.setProperty(subject({ kind: 'kind_body', name: 'body_d' }), 'size_quantity', 31);
  assert.equal(session.state.renaming, null, 'an edit of anything but the name is no rename');
  assert.equal(await resized, true);
  const refused = session.setProperty(subject({ kind: 'kind_body', name: 'body_d' }), 'name', 'body_b');
  assert.deepEqual(session.state.renaming, { kind: 'kind_body', name: 'body_d' });
  assert.equal(await refused, false);
  assert.equal(session.state.renaming, null, 'a refused rename is over too');
});

test('undo and redo add revisions and are followed by the same reading as an edit', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  await session.setProperty(subject(body), 'size_quantity', 20);
  const sent = service.log.length;
  await session.undo();
  assert.deepEqual(commandsSince(service, sent), ['undo', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties']);
  assert.equal(session.state.revision, 2);
  assert.deepEqual([session.state.canUndo, session.state.canRedo], [false, true]);
  assert.equal(session.state.hasUnsavedChanges, false, 'undo went back to the state it was opened in');
  assert.equal(valueOf(session.state, 'size_quantity'), 10);
  await session.redo();
  assert.equal(session.state.revision, 3);
  assert.equal(valueOf(session.state, 'size_quantity'), 20);
  assert.equal(session.state.preview?.projectRevision, 3);
});

test('undo or redo drops a selection the project no longer has, and asks nothing about it', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  await session.setProperty(subject(body), 'name', 'body_d');
  let sent = service.log.length;
  await session.undo();
  assert.equal(session.state.selection, null);
  assert.deepEqual(commandsSince(service, sent), ['undo', 'check', 'get_preview', 'get_outline', 'get_model_tree']);
  assert.ok(outlineHas(session.state, body));
  await session.select(subject(body));
  sent = service.log.length;
  await session.redo();
  assert.equal(session.state.selection, null, 'redo renamed the selection away again');
  assert.deepEqual(elementsAskedAbout(service, sent), []);
});

test('a row offering one kind opens its form at once, with what the row fixes; a row offering several asks for a kind first', async () => {
  const one = [{ kind: 'kind_a', fixedProperties: { owner_ref: 'a' } }];
  const several = [
    { kind: 'kind_a', fixedProperties: {} },
    { kind: 'kind_b', fixedProperties: {} },
  ];
  assert.deepEqual(creationFromOffers('a row', one), { rowKey: 'a row', offers: one, chosen: one[0] });
  assert.deepEqual(creationFromOffers('a row', several), { rowKey: 'a row', offers: several, chosen: null });

  const { service, session } = await openedSession();
  const forces = rowWithKey(session.state.modelTree!, 'domain/forces')!;
  await session.startCreation(forces.key, forces.creates);
  assert.deepEqual([session.state.creation?.rowKey, session.state.creation?.chosen], ['domain/forces', null]);
  await session.chooseCreationOffer(forces.creates[1]!);
  assert.equal(session.state.creation?.chosen?.kind, 'kind_bar');
  await session.cancelCreation();
  assert.equal(session.state.creation, null);
  assert.equal(service.log.filter((entry) => entry.command === 'add_element').length, 0, 'nothing is created by choosing');
});

test('creating sends what is given beside what the row fixes, then selects the element, read under the row it was created from', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  const framesOfB = rowWithKey(session.state.modelTree!, 'frames/body_b')!;
  await session.startCreation(framesOfB.key, framesOfB.creates);
  const offer = session.state.creation!.chosen!;
  assert.deepEqual(offer, { kind: 'kind_frame', fixedProperties: { body_ref: 'body_b' } });
  assert.equal(session.state.selection, null);
  const sent = service.log.length;
  assert.equal(await session.createElement(offer.kind, 'frame_b', { ...(offer.fixedProperties as Record<string, string>) }, null), null);
  assert.deepEqual(commandsSince(service, sent), ['add_element', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties']);
  assert.deepEqual(service.log[sent]?.commandArguments, { kind: 'kind_frame', name: 'frame_b', properties: { body_ref: 'body_b' } });
  const created = { kind: 'kind_frame', name: 'frame_b' };
  assert.deepEqual(session.state.selection, subject(created));
  assert.equal(valueOf(session.state, 'body_ref'), 'body_b');
  assert.equal(session.state.creation, null);
  assert.equal(session.state.revision, 1);
  assert.deepEqual(rowKeysOf(session.state, subject(created)), ['domain/bodies', 'element/kind_body/body_b', 'frames/body_b', 'element/kind_frame/frame_b']);
});

test('a refused creation changes nothing and names the property at fault, which the form shows', async () => {
  const { service, session } = await openedSession();
  const framesOfA = rowWithKey(session.state.modelTree!, 'frames/body_a')!;
  await session.startCreation(framesOfA.key, framesOfA.creates);
  const sent = service.log.length;
  const taken = await session.createElement('kind_frame', 'frame_a', { body_ref: 'body_a' }, null);
  assert.equal(taken?.property, 'name');
  const missing = await session.createElement('kind_frame', 'frame_c', {}, null);
  assert.equal(missing?.property, 'body_ref');
  const dangling = await session.createElement('kind_frame', 'frame_c', { body_ref: 'frame_a' }, null);
  assert.equal(dangling?.property, 'body_ref');
  assert.deepEqual(commandsSince(service, sent), ['add_element', 'add_element', 'add_element']);
  assert.equal(session.state.revision, 0);
  assert.equal(session.state.creation?.chosen?.kind, 'kind_frame', 'the form stays open for correction');
});

test('a removal that takes the element alone also waits to be confirmed, and undo brings the element back', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(spring));
  let sent = service.log.length;
  await session.requestRemoval(spring);
  assert.deepEqual(commandsSince(service, sent), ['get_removal_consequences']);
  assert.deepEqual(session.state.pendingRemoval, { element: spring, removedElements: [spring] });
  assert.ok(outlineHas(session.state, spring));
  sent = service.log.length;
  await session.confirmRemoval();
  assert.deepEqual(commandsSince(service, sent), ['remove_element', 'check', 'get_preview', 'get_outline', 'get_model_tree']);
  assert.equal(session.state.selection, null);
  assert.deepEqual(session.state.removedElements, [spring]);
  assert.equal(outlineHas(session.state, spring), false);
  await session.undo();
  assert.ok(outlineHas(session.state, spring));
  assert.equal(session.state.removedElements, null);
});

test('a removal that takes more waits to be confirmed: cancelling keeps everything, confirming removes all it named', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(otherBody));
  let sent = service.log.length;
  await session.requestRemoval(otherBody);
  assert.deepEqual(commandsSince(service, sent), ['get_removal_consequences']);
  const goes = [
    otherBody,
    { kind: 'kind_hinge', name: 'hinge_b' },
    spring,
    { kind: 'kind_carrier', name: 'body_b' },
    { kind: 'kind_interface', name: 'body_b.left' },
    { kind: 'kind_interface', name: 'body_b.right' },
  ];
  assert.deepEqual(session.state.pendingRemoval, { element: otherBody, removedElements: goes });
  sent = service.log.length;
  await session.cancelRemoval();
  assert.equal(service.log.length, sent);
  assert.equal(session.state.pendingRemoval, null);
  assert.deepEqual(session.state.selection, subject(otherBody));
  await session.requestRemoval(otherBody);
  sent = service.log.length;
  await session.confirmRemoval();
  assert.deepEqual(commandsSince(service, sent), ['remove_element', 'check', 'get_preview', 'get_outline', 'get_model_tree']);
  assert.deepEqual(session.state.removedElements, goes);
  assert.ok(goes.every((element) => !outlineHas(session.state, element)));
  assert.equal(session.state.selection, null);
  assert.equal(session.state.pendingRemoval, null);
  assert.equal(session.state.revision, 1);
});

test('a removal the service refuses, the element being gone already, asks nothing more and says why, about the element', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(spring));
  // Another page on the same service removed it meanwhile; this session has not read the project since.
  assert.ok((await service.send('remove_element', { element: spring })).ok);
  const sent = service.log.length;
  await session.requestRemoval(spring);
  assert.deepEqual(commandsSince(service, sent), ['get_removal_consequences']);
  assert.deepEqual(session.state.commandRefusal?.subject, subject(spring));
  assert.match(session.state.commandRefusal?.message ?? '', /no element named 'spring_a'/);
  assert.equal(session.state.pendingRemoval, null);
  assert.deepEqual(session.state.selection, subject(spring));
});

test('nothing is asked about an element the model tree no longer has, whatever asks', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(spring));
  await session.requestRemoval(spring);
  await session.confirmRemoval();
  const sent = service.log.length;
  // A stale drawing still shows the spring; a field of it may still commit.
  await session.select(subject(spring));
  assert.equal(await session.setProperty(subject(spring), 'axis_choice', 'axis_y'), false);
  await session.requestRemoval(spring);
  assert.equal(service.log.length, sent);
  assert.equal(session.state.selection, null);
});

test('actions asked for together run one after another', async () => {
  const { service, session } = await openedSession();
  const sent = service.log.length;
  await Promise.all([session.select(subject(body)), session.setProperty(subject(body), 'size_quantity', 3), session.select(subject(spring))]);
  assert.deepEqual(commandsSince(service, sent), ['get_properties', 'set_property', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties', 'get_properties']);
  assert.deepEqual(session.state.selection, subject(spring));
});

test('a check of an earlier revision does not count as the current status', async () => {
  const { session } = await openedSession();
  const state = session.state;
  assert.equal(checkStatus({ ...state, revision: state.revision + 1 }), 'not_checked');
});

test('a failing server ends the session loudly and nothing more is sent', async () => {
  const service = serviceWithTwoVehicles();
  let reachable = true;
  const session = new VehicleEditingSession(async (command, commandArguments) => {
    if (!reachable) throw new Error('the modelling server is gone');
    return service.send(command, commandArguments);
  });
  await session.start(projectsDirectory, { open: referenceVehicle(vehicle), select: null });
  reachable = false;
  await session.select(subject(body));
  assert.match(session.state.failure ?? '', /gone/);
  reachable = true;
  const sent = service.log.length;
  await session.select(subject(body));
  await session.save();
  assert.equal(service.log.length, sent);
});

test('nothing asked for against the shown project while another vehicle opens is sent', async () => {
  const service = serviceWithTwoVehicles();
  let gate: Promise<void> | null = null;
  let releaseOpen = (): void => {};
  const session = new VehicleEditingSession(async (command, commandArguments) => {
    if (command === 'get_visual_definition') await gate;
    return service.send(command, commandArguments);
  });
  await session.start(projectsDirectory, { open: referenceVehicle(vehicle), select: null });
  await session.select(subject(body));
  gate = new Promise((release) => {
    releaseOpen = release;
  });
  const opening = session.open(referenceVehicle(otherVehicle));
  const shownWhileOpening = session.state.opening;
  // The panels still show the first vehicle; everything asked for against it now must not reach the second.
  const edited = session.setProperty(subject(body), 'size_quantity', 99);
  const created = session.createElement('kind_frame', 'frame_b', { body_ref: 'body_a' }, null);
  const others = [session.select(subject(spring)), session.undo(), session.redo(), session.requestRemoval(spring), session.startCreation('domain/bodies', [{ kind: 'kind_body', fixedProperties: {} }]), session.save(), session.startSaveAs()];
  releaseOpen();
  await opening;
  await Promise.all(others);
  const sinceOpen = service.log.slice(service.log.findIndex((entry) => entry.commandArguments['vehicle_definition_path'] === '/fake/other.json'));
  assert.deepEqual(
    sinceOpen.map((entry) => entry.command),
    ['open_vehicle', ...openingCommands],
  );
  await session.select(subject(body));
  assert.equal(session.state.outline?.vehicleName, 'OTHER');
  assert.equal(session.state.revision, 0);
  assert.equal(valueOf(session.state, 'size_quantity'), 10);
  assert.equal(await edited, false);
  assert.equal(await created, null);
  assert.equal(session.state.creation, null);
  assert.equal(session.state.saveAs, null);
  assert.deepEqual(shownWhileOpening, referenceVehicle(otherVehicle));
  assert.equal(session.state.opening, null);
});

test('actions asked for before an open still run first, against the project they were asked for', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(body));
  await session.startSaveAs();
  await session.saveAs('mine', false);
  const sent = service.log.length;
  const edited = session.setProperty(subject(body), 'size_quantity', 12);
  const saved = session.save();
  const opening = session.open(referenceVehicle(otherVehicle));
  assert.equal(await edited, true);
  await saved;
  await opening;
  assert.deepEqual(commandsSince(service, sent), [
    'set_property',
    'check',
    'get_preview',
    'get_outline', 'get_model_tree',
    'get_properties',
    'save_vehicle_project',
    'open_vehicle',
    ...openingCommands,
  ]);
  assert.equal(session.state.pendingOpen, null, 'the save before it left nothing unsaved');
});

test('opening reads the model tree after the outline, and the tree has every element of the outline once', async () => {
  const { session } = await openedSession();
  const inTree: unknown[] = [];
  const collect = (rows: NonNullable<SessionState['modelTree']>): void => {
    for (const row of rows) {
      if (row.subject !== null && 'element' in row.subject) inTree.push(row.subject.element);
      collect(row.children);
    }
  };
  collect(session.state.modelTree!);
  const byKey = (each: unknown) => JSON.stringify(each);
  assert.deepEqual(inTree.map(byKey).sort(), session.state.outline!.elements.map(byKey).sort());
});

test('settings are selected and edited like an element, one undo step each, and stay selected through undo, redo and Save as', async () => {
  const { service, session } = await openedSession();
  const constant = { settings: 'setting_constant' };
  let sent = service.log.length;
  await session.select(constant);
  assert.deepEqual(commandsSince(service, sent), ['get_properties']);
  assert.deepEqual(service.log[sent]?.commandArguments, { subject: constant });
  assert.deepEqual(session.state.selection, constant);
  assert.deepEqual(
    session.state.properties.map((property) => [property.name, property.editable, property.value]),
    [['constant_quantity', true, null]],
    'a value not stated yet can be set',
  );

  sent = service.log.length;
  assert.equal(await session.setProperty(constant, 'constant_quantity', 9.81), true);
  assert.deepEqual(commandsSince(service, sent), ['set_property', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties']);
  assert.deepEqual(service.log[sent]?.commandArguments, { subject: constant, property: 'constant_quantity', value: 9.81 });
  assert.deepEqual([session.state.revision, session.state.canUndo, session.state.hasUnsavedChanges], [1, true, true]);
  assert.equal(valueOf(session.state, 'constant_quantity'), 9.81);
  await session.undo();
  assert.deepEqual(session.state.selection, constant, 'undo keeps the settings selected');
  assert.equal(valueOf(session.state, 'constant_quantity'), null);
  await session.redo();
  assert.equal(valueOf(session.state, 'constant_quantity'), 9.81);

  const identity = { settings: 'setting_identity' };
  await session.select(identity);
  await session.startSaveAs();
  await session.saveAs('mine', false);
  assert.deepEqual(session.state.selection, identity, 'Save as keeps the settings selected');
  assert.equal(valueOf(session.state, 'identity_text'), 'mine', 'and reads them again');
});

test('a refused settings edit is kept at its field, about the settings, and changes nothing', async () => {
  const { service, session } = await openedSession();
  const identity = { settings: 'setting_identity' };
  await session.select(identity);
  const sent = service.log.length;
  assert.equal(await session.setProperty(identity, 'identity_text', 'other'), false);
  assert.deepEqual(commandsSince(service, sent), ['set_property']);
  assert.deepEqual(session.state.editRefusal, { subject: identity, property: 'identity_text', value: 'other', message: "'identity_text' is read-only" });
  assert.equal(session.state.revision, 0);
  assert.deepEqual(session.state.selection, identity);
});

test('a selection survives every re-read of the tree, and a frame pointed at another body stays selected under it', async () => {
  const { session } = await openedSession();
  await session.select(subject(frame));
  assert.deepEqual(rowKeysOf(session.state, subject(frame)), ['domain/bodies', 'element/kind_body/body_a', 'frames/body_a', 'element/kind_frame/frame_a']);
  await session.setProperty(subject(body), 'size_quantity', 11);
  assert.deepEqual(session.state.selection, subject(frame), 'an edit elsewhere re-reads the tree and keeps the selection');
  assert.equal(await session.setProperty(subject(frame), 'body_ref', 'body_b'), true);
  assert.deepEqual(session.state.selection, subject(frame));
  assert.deepEqual(rowKeysOf(session.state, subject(frame)), ['domain/bodies', 'element/kind_body/body_b', 'frames/body_b', 'element/kind_frame/frame_a']);
  await session.undo();
  assert.deepEqual(rowKeysOf(session.state, subject(frame)), ['domain/bodies', 'element/kind_body/body_a', 'frames/body_a', 'element/kind_frame/frame_a']);
  assert.deepEqual(session.state.selection, subject(frame));
});

const freeBody = { kind: 'kind_body', name: 'body_c' };
const hinge = { kind: 'kind_hinge', name: 'hinge_b' };

/** A joint of a kind with its name and properties, as an attachment states it. */
function jointSetting(kind: string, name: string, properties: Record<string, unknown>) {
  return { joint: { kind, name, properties } };
}

test('a selected body is read with its attachment, and every other subject with none', async () => {
  const { session } = await openedSession();
  await session.select(subject(otherBody));
  assert.deepEqual([session.state.attachment?.joint, session.state.attachment?.heldAt], [hinge, frame], 'the joint it hangs on, and the frame at its other end');
  assert.deepEqual([session.state.attachment?.canBecomeFree, session.state.attachment?.canTakeJoint], [true, true]);
  await session.select(subject(body));
  assert.deepEqual(
    [session.state.attachment?.joint, session.state.attachment?.heldAt, session.state.attachment?.canBecomeFree, session.state.attachment?.canTakeJoint],
    [null, null, false, true],
    'no body is kept free: one that moves freely can take a joint',
  );
  assert.equal(session.state.attachment?.note, null, 'nothing is the matter with it');
  await session.select(subject(spring));
  assert.equal(session.state.attachment, null);
  await session.select({ settings: 'setting_constant' });
  assert.equal(session.state.attachment, null);
});

test('a body is created free or hanging on a joint, each in one edit, and is then selected', async () => {
  const { service, session } = await openedSession();
  const bodies = rowWithKey(session.state.modelTree!, 'domain/bodies')!;
  await session.startCreation(bodies.key, bodies.creates);
  let sent = service.log.length;
  assert.equal(await session.createElement('kind_body', 'body_free', { size_quantity: 3, offset_quantity: [1, 0, 0] }, { free: true }), null);
  assert.deepEqual(service.log[sent]?.commandArguments, {
    kind: 'kind_body',
    name: 'body_free',
    properties: { size_quantity: 3, offset_quantity: [1, 0, 0] },
    attachment: { free: true },
  });
  assert.deepEqual(session.state.selection, subject({ kind: 'kind_body', name: 'body_free' }));
  assert.deepEqual([session.state.attachment?.joint, session.state.attachment?.canBecomeFree], [null, false]);

  await session.startCreation(bodies.key, bodies.creates);
  sent = service.log.length;
  assert.equal(await session.createElement('kind_body', 'body_held', { size_quantity: 2 }, jointSetting('kind_weld', 'weld_held', { end_parent: 'frame_a', gap_quantity: 2 })), null);
  assert.deepEqual(commandsSince(service, sent), ['add_element', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties']);
  assert.equal(session.state.revision, 2, 'the body and its joint are one edit');
  assert.deepEqual(session.state.selection, subject({ kind: 'kind_body', name: 'body_held' }));
  assert.deepEqual(session.state.attachment?.joint, { kind: 'kind_weld', name: 'weld_held' });
  assert.equal(valueOf(session.state, 'offset_quantity'), null, 'a body held by a joint has no property only a free body has');
  assert.deepEqual(rowKeysOf(session.state, subject({ kind: 'kind_body', name: 'body_held' })), ['domain/bodies', 'element/kind_body/body_held']);
});

test('a free body takes a joint in one edit and stays selected; undo makes it free again', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(freeBody));
  assert.deepEqual([session.state.attachment?.canBecomeFree, session.state.attachment?.canTakeJoint, session.state.attachment?.joint], [false, true, null]);
  await session.startChangingAttachment();
  assert.deepEqual(session.state.attachmentChange?.body?.element, freeBody);
  assert.deepEqual(session.state.attachmentChange?.body?.jointProperties, [], 'it hangs on no joint to start from');
  assert.deepEqual(session.state.attachmentChange?.body?.attachment.framesOnBody, [freeBody]);
  const sent = service.log.length;
  const setting = jointSetting('kind_hinge', 'hinge_c', { end_parent: 'frame_a', end_child: 'body_c', axis_quantity: [1, 0, 0] });
  assert.equal(await session.setAttachment(setting), true);
  assert.deepEqual(commandsSince(service, sent), ['set_attachment', 'check', 'get_preview', 'get_outline', 'get_model_tree', 'get_properties']);
  assert.deepEqual(service.log[sent]?.commandArguments, { body: freeBody, attachment: setting });
  assert.deepEqual([session.state.selection, session.state.attachmentChange, session.state.revision], [subject(freeBody), null, 1]);
  assert.deepEqual(session.state.attachment?.joint, { kind: 'kind_hinge', name: 'hinge_c' });
  assert.equal(valueOf(session.state, 'offset_quantity'), null);
  await session.undo();
  assert.deepEqual([session.state.attachment?.joint, session.state.attachment?.canBecomeFree], [null, false]);
  assert.equal(outlineHas(session.state, { kind: 'kind_hinge', name: 'hinge_c' }), false);
  assert.deepEqual(valueOf(session.state, 'offset_quantity'), [0, 0, 0]);
});

test('a hanging body takes another joint in place of its own, starting from it, and becomes free; what names its joint moves or holds the change back', async () => {
  const { session } = await openedSession();
  await session.select(subject(otherBody));
  await session.startChangingAttachment();
  assert.deepEqual(
    session.state.attachmentChange?.body?.jointProperties.map((property) => [property.name, property.value]),
    [
      ['name', 'hinge_b'],
      ['end_parent', 'frame_a'],
      ['end_child', 'body_b'],
      ['axis_quantity', [0, 1, 0]],
    ],
  );
  // body_b.left names hinge_b, and may name hinges only.
  assert.equal(await session.setAttachment({ free: true }), false);
  assert.deepEqual(session.state.attachmentChange?.refusal?.subject, subject(otherBody));
  assert.match(session.state.attachmentChange?.refusal?.message ?? '', /body_b\.left/);
  assert.equal(await session.setAttachment(jointSetting('kind_weld', 'weld_b', { end_parent: 'frame_a', end_child: 'body_b' })), false);
  assert.match(session.state.attachmentChange?.refusal?.message ?? '', /body_b\.left/);
  assert.equal(session.state.revision, 0);
  assert.equal(await session.setAttachment(jointSetting('kind_hinge', 'hinge_b', { end_parent: 'body_a', end_child: 'body_b', axis_quantity: [0, 0, 1] })), true, 'the name of the joint it replaces');
  await session.select({ element: { kind: 'kind_interface', name: 'body_b.left' } });
  assert.equal(valueOf(session.state, 'hinge_ref'), 'hinge_b');
  assert.equal(await session.setProperty({ element: { kind: 'kind_interface', name: 'body_b.left' } }, 'hinge_ref', null), true);

  await session.select(subject(otherBody));
  await session.startChangingAttachment();
  assert.equal(await session.setAttachment(jointSetting('kind_weld', 'weld_b', { end_parent: 'frame_a', end_child: 'body_b' })), true);
  assert.deepEqual(session.state.attachment?.joint, { kind: 'kind_weld', name: 'weld_b' });
  assert.equal(outlineHas(session.state, hinge), false);
  await session.startChangingAttachment();
  assert.equal(await session.setAttachment({ free: true }), true);
  assert.deepEqual([session.state.attachment?.joint, session.state.attachment?.canBecomeFree], [null, false]);
  assert.equal(outlineHas(session.state, { kind: 'kind_weld', name: 'weld_b' }), false, 'its joint goes in the same edit');
  assert.deepEqual(valueOf(session.state, 'offset_quantity'), [0, 0, 0], 'it has the properties of a free body, at zero');
  assert.equal(session.state.revision, 4);
});

test('a refusal about the joint names the joint and the field at fault; one about the body names the body', async () => {
  const { session } = await openedSession();
  await session.select(subject(freeBody));
  await session.startChangingAttachment();
  const asked = (name: string, ends: Record<string, string>) => session.setAttachment(jointSetting('kind_hinge', name, ends));
  assert.equal(await asked('hinge_b', { end_parent: 'frame_a', end_child: 'body_c' }), false);
  assert.deepEqual([session.state.attachmentChange?.refusal?.subject, session.state.attachmentChange?.refusal?.property], [subject(hinge), 'name']);
  assert.equal(await asked('hinge_c', { end_parent: 'frame_a', end_child: 'frame_a' }), false);
  assert.deepEqual([session.state.attachmentChange?.refusal?.subject, session.state.attachmentChange?.refusal?.property], [subject({ kind: 'kind_hinge', name: 'hinge_c' }), 'end_child']);
  assert.equal(await asked('hinge_c', { end_parent: 'body_c', end_child: 'body_c' }), false, 'held at itself');
  assert.equal(session.state.attachmentChange?.refusal?.property, 'end_parent');
  assert.equal(session.state.revision, 0);
  assert.deepEqual(session.state.attachmentChange?.body?.element, freeBody, 'the change stays to be corrected');

  await session.select(subject(otherBody));
  await session.startChangingAttachment();
  const weld = jointSetting('kind_weld', 'weld_b', { end_parent: 'frame_a', end_child: 'body_b' });
  assert.equal(await session.setAttachment(weld), false, 'body_b.left names the hinge it hangs on, and may name no weld');
  assert.deepEqual([session.state.attachmentChange?.refusal?.subject, session.state.attachmentChange?.refusal?.property], [subject(otherBody), null]);
  await session.cancelAttachmentChange();
  assert.equal(session.state.attachmentChange, null);
});

test('a body left unattached by removing its joint says so in the tree, and has two ways out', async () => {
  const { session } = await openedSession();
  await session.select(subject(hinge));
  await session.requestRemoval(hinge);
  await session.confirmRemoval();
  const bodyRow = rowWithKey(session.state.modelTree!, 'element/kind_body/body_b')!;
  assert.notEqual(bodyRow.note, null, 'its row says it is not attached');
  assert.deepEqual(rowWithKey(session.state.modelTree!, 'domain/joints')!.attaches, [body, otherBody, freeBody], 'the free bodies and the unattached one');
  await session.select(subject(otherBody));
  assert.deepEqual([session.state.attachment?.joint, session.state.attachment?.canBecomeFree, session.state.attachment?.canTakeJoint], [null, true, true]);

  await session.startChangingAttachment();
  assert.equal(await session.setAttachment({ free: true }), true);
  assert.deepEqual([session.state.attachment?.canBecomeFree, rowWithKey(session.state.modelTree!, 'element/kind_body/body_b')!.note], [false, null]);
  await session.undo();
  assert.deepEqual([session.state.attachment?.canBecomeFree, session.state.attachment?.canTakeJoint], [true, true]);

  await session.startChangingAttachment();
  assert.equal(await session.setAttachment(jointSetting('kind_weld', 'weld_b', { end_parent: 'frame_a', end_child: 'body_b' })), true);
  assert.deepEqual(session.state.attachment?.joint, { kind: 'kind_weld', name: 'weld_b' });
  assert.equal(rowWithKey(session.state.modelTree!, 'element/kind_body/body_b')!.note, null);
});

test('the joints row’s "+" attaches one of the bodies it lists, read once chosen, and the selection stays where it was', async () => {
  const { service, session } = await openedSession();
  await session.select(subject(spring));
  const joints = rowWithKey(session.state.modelTree!, 'domain/joints')!;
  assert.deepEqual([joints.creates, joints.attaches], [[], [body, freeBody]], 'it creates nothing and attaches the bodies that move freely');
  await session.startAttaching(joints.key, joints.attaches);
  assert.deepEqual(session.state.attachmentChange, { fromRow: { rowKey: 'domain/joints', bodies: [body, freeBody] }, body: null, refusal: null });
  assert.deepEqual(session.state.selection, subject(spring));
  await session.cancelAttachmentChange();
  assert.equal(session.state.attachmentChange, null);

  await session.startAttaching(joints.key, joints.attaches);
  let sent = service.log.length;
  await session.chooseBodyToAttach(freeBody);
  assert.deepEqual(commandsSince(service, sent), ['get_properties']);
  assert.deepEqual(service.log[sent]?.commandArguments, { subject: subject(freeBody) });
  // Read anew: the assertion above narrowed the earlier reading to nothing.
  const chosen = session.state.attachmentChange as SessionState['attachmentChange'];
  assert.deepEqual([chosen?.body?.element, chosen?.body?.attachment.canTakeJoint], [freeBody, true]);
  sent = service.log.length;
  assert.equal(await session.setAttachment(jointSetting('kind_weld', 'weld_c', { end_parent: 'frame_a', end_child: 'body_c' })), true);
  assert.deepEqual(service.log[sent]?.commandArguments, { body: freeBody, attachment: jointSetting('kind_weld', 'weld_c', { end_parent: 'frame_a', end_child: 'body_c' }) });
  assert.deepEqual([session.state.selection, session.state.attachmentChange], [subject(spring), null]);
  const after = rowWithKey(session.state.modelTree!, 'domain/joints')!;
  assert.deepEqual([after.attaches, after.children.map((row) => row.key)], [[body], ['element/kind_hinge/hinge_b', 'element/kind_weld/weld_c']]);
});
