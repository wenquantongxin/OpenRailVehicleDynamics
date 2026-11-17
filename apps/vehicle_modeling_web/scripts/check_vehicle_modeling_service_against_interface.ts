// Drives the real modelling service through the modelling server the way the
// interface does, with the interface's own session, readers and drawing, and
// asserts the structure of what comes back, never its numbers. For each
// reference vehicle: open, outline, the properties of every element, the
// drawing with one wheel per contact interface and the visual definition from
// the service, an edit the check passes, an edit the check refuses, undo and
// redo. On GZ18, structural editing: a body, a frame on it and a force element
// from that frame to one the vehicle had are created; the body is renamed and
// the frame follows; removing the body takes the frame and the force element
// with it; undoing everything gives back the outline the vehicle opened with.
// Then a vehicle whose contact record does not fit it: it opens, its check
// refuses it, there is no drawing, and its outline and properties read. The
// model tree holds every element of each outline once; a frame is created
// through the offer of a body's row and stands under that body, also after the
// body is renamed; on GZ18 every editable number of the tree's settings is
// set, read back and undone, the wheels are drawn anew at a new nominal
// rolling radius, and a read-only property is refused. On GZ18 and IRW every
// rigid body is read with its attachment and nothing else is, and the row of
// the joints attaches the bodies that can take one and creates nothing; on
// GZ18 a body created with a joint hangs on it, and one created free that
// takes a joint and becomes free again reads free. Last,
// saving, in a temporary folder of projects removed at the end: GZ18 edited,
// saved as a project, with no undo step, the folder listed, IRW opened, the
// project opened again with its outline and the edited value as they were;
// unsaved changes through an edit and its undo, Save, and a name the service
// refuses; get_session at each stage.
// The reference vehicles are this app's catalogue; the vehicle that does not
// fit is made of the repository's IRW vehicle and GZ18 contact record. Kinds
// are told apart by the preview's drawings and properties by their
// descriptions: no kind and no property is named here.
// Exits non-zero at the first broken expectation.
//
// Usage: npm run check-service -- <service binary> <repository root>

import assert from 'node:assert/strict';
import { mkdtemp, readdir, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { readReferenceVehicleCatalogue, type ReferenceVehicleCatalogueEntry } from '../server/read_reference_vehicle_catalogue.ts';
import { startVehicleModelingHttpServer } from '../server/vehicle_modeling_http_server.ts';
import { createModelPalette } from '../src/scene/vehicle_model_view_colors_and_materials.ts';
import { assignPartsToDrawnBodies, VehiclePreviewScene } from '../src/scene/vehicle_preview_scene.ts';
import { fetchReferenceVehicles, type ReferenceVehicle } from '../src/service/fetch_reference_vehicles.ts';
import { fetchVehicleProjectsDirectory } from '../src/service/fetch_vehicle_projects_directory.ts';
import { rowPathOfSubject } from '../src/service/find_model_tree_rows.ts';
import { modelingServerCommandChannel, type SendCommand } from '../src/service/modeling_server_command_channel.ts';
import {
  elementKey,
  readServiceSessionDescription,
  sameElement,
  sameSubject,
  subjectElement,
  type Attachment,
  type AttachmentSetting,
  type ElementReference,
  type ModelTreeNode,
  type PropertyDescription,
  type PropertyReading,
  type ServiceSessionDescription,
} from '../src/service/vehicle_modeling_service_protocol.ts';
import { checkStatus, VehicleEditingSession, previewIsStale, type OpenChoice, type SessionState } from '../src/session/vehicle_editing_session.ts';
import type { ReferenceDescription } from '../src/ui/element_reference_picker_rules.ts';

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
    visual_definition_path: null,
  },
};

function referenceVehicle(vehicle: ReferenceVehicle): OpenChoice {
  return { source: 'reference_vehicle', vehicle };
}

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
    const state = await settled(session, session.select({ element: element }));
    for (const property of state.properties) {
      if (property.editable && property.valueKind === 'number') found.push({ element, property });
    }
  }
  return found;
}

/** Every row of the tree, at any depth, each after the rows above it. */
function treeRows(tree: readonly ModelTreeNode[]): ModelTreeNode[] {
  return tree.flatMap((row) => [row, ...treeRows(row.children)]);
}

function sessionAt(origin: string): VehicleEditingSession {
  return new VehicleEditingSession(modelingServerCommandChannel(origin));
}

/**
 * Reads the properties of every element: each has some, all in value kinds the
 * interface shows. A value the element does not hold is never editable, apart
 * from an optional reference that names none, which is.
 */
async function readEveryElement(session: VehicleEditingSession, elements: ElementReference[]): Promise<{ properties: number; absent: number; namingNone: number }> {
  const counts = { properties: 0, absent: 0, namingNone: 0 };
  for (const element of elements) {
    const state = await settled(session, session.select({ element: element }));
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
  let state = await settled(session, session.open(referenceVehicle(vehicle)));
  const outline = state.outline ?? fail('no outline after opening');
  assert.ok(outline.vehicleName.length > 0);
  assert.equal(state.revision, 0);
  assert.equal(state.hasUnsavedChanges, false);
  // The visual definition comes from the service, and the interface can draw from it.
  assert.ok(state.vehicle?.visualDefinition, `${vehicle.name} opens without a visual definition: ${state.vehicle?.visualDefinitionProblem}`);
  assert.equal(checkStatus(state), 'passed', `${vehicle.name} does not pass its check when opened: ${state.check?.report.refusals.join('; ')}`);
  const kinds = [...new Set(outline.elements.map((element) => element.kind))];
  // The outline lists the elements kind by kind: each kind is one run.
  assert.equal(kinds.length, outline.elements.filter((element, index) => outline.elements[index - 1]?.kind !== element.kind).length);
  // The model tree holds every element of the outline exactly once, whatever it nests under what.
  const inTree = treeRows(state.modelTree ?? fail('no model tree after opening')).flatMap((row) => {
    const element = subjectElement(row.subject);
    return element === null ? [] : [elementKey(element)];
  });
  assert.deepEqual([...inTree].sort(), outline.elements.map(elementKey).sort(), `${vehicle.name}'s model tree does not hold each element of the outline once`);
  const firstLevel = (state.modelTree ?? []).length;
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
  await settled(session, session.select({ element: edited.element }));
  assert.equal(await session.setProperty({ element: edited.element }, edited.property.name, doubled), true);
  state = session.state;
  assert.equal(state.revision, 1);
  assert.equal(state.canUndo, true);
  assert.equal(state.properties.find((property) => property.name === edited.property.name)?.value, doubled);
  assert.equal(checkStatus(state), 'passed', `doubling ${edited.property.name} was refused: ${state.check?.report.refusals.join('; ')}`);
  assert.equal(state.preview?.projectRevision, state.revision);

  // An edit the check refuses: the negative of an editable positive number, until one is refused.
  let refused: SessionState | null = null;
  for (const entry of numbers.filter((candidate) => candidate.property.value! > 0)) {
    await settled(session, session.select({ element: entry.element }));
    assert.equal(await session.setProperty({ element: entry.element }, entry.property.name, -entry.property.value!), true);
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
    `${vehicle.name}: ${outline.elements.length} elements in ${kinds.length} kinds, each once in a model tree of ${firstLevel} first-level rows and ${inTree.length} element rows,`,
    `  ${read.properties} properties read, ${read.absent} without a value and none of those editable,`,
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
  let state = await settled(session, session.open(referenceVehicle(vehicle)));
  const opened = state.outline ?? fail('no outline after opening');
  const preview = state.preview ?? fail('no preview after opening');
  const bodyKind = (preview.rigidBodies[0] ?? fail('nothing drawn as a rigid body')).element.kind;
  const frameKind = (preview.fixedFrames[0] ?? fail('nothing drawn as a fixed frame')).element.kind;
  const forceKind = (preview.forceElements[0] ?? fail('nothing drawn as a force element')).element.kind;
  const properties = (kind: string): PropertyDescription[] => (state.elementKinds.find((each) => each.kind === kind) ?? fail(`'${kind}' is not described`)).properties;
  const requiredReferences = (kind: string): ReferenceDescription[] =>
    properties(kind).filter((property): property is ReferenceDescription => property.valueKind === 'element_reference' && !property.optional);
  const created = async (kind: string, stem: string, given: Record<string, unknown>, attachment: AttachmentSetting | null = null): Promise<ElementReference> => {
    const name = unusedName(opened.elements, stem);
    const refusal = await session.createElement(kind, name, given as Record<string, string>, attachment);
    assert.equal(refusal, null, `creating a ${kind} was refused: ${refusal?.message}`);
    const element = { kind, name };
    assert.deepEqual(session.state.selection, { element }, 'a created element is selected');
    return element;
  };
  /** Whether the element's row stands under the owner's row in the model tree. */
  const under = (element: ElementReference, owner: ElementReference): boolean =>
    (rowPathOfSubject(session.state.modelTree!, { element }) ?? []).some((row) => sameSubject(row.subject, { element: owner }));

  // A body, given nothing but its name and that it moves freely: the service's defaults stand for the rest.
  const body = await created(bodyKind, 'structural_check_body', {}, { free: true });
  // A frame on it, made through the offer of a row under the body's own row, which fixes what puts it on the body.
  const frameReferences = requiredReferences(frameKind);
  assert.ok(frameReferences.length > 0 && frameReferences.every((reference) => reference.referencedKinds.includes(bodyKind)));
  const bodyRow = rowPathOfSubject(session.state.modelTree!, { element: body })?.at(-1) ?? fail('the new body has no row in the model tree');
  const offering = treeRows(bodyRow.children).find((row) => row.creates.some((offer) => offer.kind === frameKind)) ?? fail('no row under the new body offers a frame');
  await settled(session, session.startCreation(offering.key, offering.creates));
  const frameOffer = session.state.creation?.chosen ?? fail('the row under the body offers more than a frame');
  assert.ok(Object.keys(frameOffer.fixedProperties).length > 0, 'the frame offered under the body fixes nothing');
  const frame = await created(frameKind, 'structural_check_frame', frameOffer.fixedProperties);
  assert.ok(under(frame, body), 'the frame made through the body\'s offer does not stand under the body');
  // A force element from that frame to a frame the vehicle had.
  const ends = requiredReferences(forceKind);
  assert.ok(ends.length >= 2 && ends.every((end) => end.referencedKinds.includes(frameKind)), 'a force element names the frames at its ends');
  const vehicleFrame = opened.elements.find((element) => element.kind === frameKind)!;
  const force = await created(forceKind, 'structural_check_force', Object.fromEntries(ends.map((end, index) => [end.name, index === 0 ? frame.name : vehicleFrame.name])));

  // Renaming the selected body: the selection and the frame on it follow.
  const nameProperty = properties(bodyKind).find((property) => property.valueKind === 'element_name') ?? fail('a body has no name to set');
  const renamed = { kind: bodyKind, name: unusedName(opened.elements, 'structural_check_body_renamed') };
  await settled(session, session.select({ element: body }));
  assert.equal(await session.setProperty({ element: body }, nameProperty.name, renamed.name), true, `the rename was refused: ${session.state.editRefusal?.message}`);
  assert.deepEqual(session.state.selection, { element: renamed });
  assert.ok(under(frame, renamed), 'the frame does not stand under the renamed body');
  state = await settled(session, session.select({ element: frame }));
  for (const reference of frameReferences) assert.equal(state.properties.find((property) => property.name === reference.name)?.value, renamed.name);

  // Removing the body takes the frame and the force element with it; it waits to be confirmed.
  await settled(session, session.select({ element: renamed }));
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
    `${vehicle.name} structural editing: created ${bodyKind} '${body.name}', ${frameKind} '${frame.name}' through the offer of '${offering.title?.en}' under it`,
    `  (fixing ${Object.keys(frameOffer.fixedProperties).join(', ')}), and ${forceKind} '${force.name}' from it to '${vehicleFrame.name}';`,
    `  renamed the body to '${renamed.name}' and the frame followed, still under it in the tree; removed it with ${pending.removedElements.length - 1} elements that referred to it,`,
    `  five undos gave back the outline of ${opened.elements.length} elements the vehicle opened with; check ${checkStatus(state)} at revision ${state.revision}`,
  ].join('\n');
}

/** Whether an attachment reads free: it hangs on no joint, cannot become free and can take a joint. */
function readsFree(attachment: Attachment | null): boolean {
  return attachment !== null && attachment.joint === null && !attachment.canBecomeFree && attachment.canTakeJoint;
}

/**
 * How a rigid body is attached, through the interface's session, structure
 * only. The properties of every element of the kind the service describes as
 * attached come with an attachment, and no other subject's do, settings
 * included. The row listing the joints creates nothing and attaches the free
 * bodies that can take a joint and the unattached ones, and a joint kind is
 * not created alone. With the changes: a body created with a joint hangs on
 * it; a body created free takes a joint and becomes free again, and then reads
 * free, its joint gone; undoing the four edits gives back the outline the
 * vehicle opened with. The attached kind, the joint kinds and which property
 * names which end come from the service's description.
 */
async function checkAttachments(origin: string, vehicle: ReferenceVehicle, withChanges: boolean): Promise<string> {
  const session = sessionAt(origin);
  let state = await settled(session, session.open(referenceVehicle(vehicle)));
  const opened = state.outline ?? fail('no outline after opening');
  const attachedKind = state.elementKinds.find((kind) => kind.attachment !== null) ?? fail('no kind is described as attached');
  const description = attachedKind.attachment!;
  const attachments = new Map<string, Attachment>();
  for (const element of opened.elements) {
    state = await settled(session, session.select({ element }));
    if (element.kind === attachedKind.kind) attachments.set(elementKey(element), state.attachment ?? fail(`${element.name} is read without its attachment`));
    else assert.equal(state.attachment, null, `${element.kind} ${element.name} is read with an attachment`);
  }
  const settingsRows = treeRows(state.modelTree!).filter((row) => row.provided && row.subject !== null && 'settings' in row.subject);
  for (const row of settingsRows) {
    state = await settled(session, session.select(row.subject!));
    assert.equal(state.attachment, null, `the settings '${row.title?.en}' are read with an attachment`);
  }
  const read = [...attachments.values()];
  const hanging = read.filter((attachment) => attachment.joint !== null).length;
  const free = read.filter((attachment) => attachment.joint === null && !attachment.canBecomeFree).length;
  const keptFree = read.filter((attachment) => attachment.joint === null && !attachment.canBecomeFree && !attachment.canTakeJoint).length;
  for (const attachment of read) {
    assert.equal(attachment.heldAt !== null, attachment.joint !== null, 'a body is held at a frame other than exactly when it hangs on a joint');
    if (attachment.joint === null) continue;
    assert.ok(opened.elements.some((element) => sameElement(element, attachment.joint)), `a body hangs on '${attachment.joint.name}', which the outline does not have`);
    assert.ok(description.jointKinds.includes(attachment.joint.kind), `a body hangs on a ${attachment.joint.kind}, which is no joint kind`);
    assert.ok(opened.elements.some((element) => sameElement(element, attachment.heldAt)), `a body is held at '${attachment.heldAt!.name}', which the outline does not have`);
  }

  // The row listing the joints creates nothing; its "+" attaches the free bodies that can take a joint and the unattached ones.
  const isJoint = (row: ModelTreeNode): boolean => {
    const element = subjectElement(row.subject);
    return element !== null && description.jointKinds.includes(element.kind);
  };
  const jointsRow = treeRows(state.modelTree!).find((row) => row.children.some(isJoint)) ?? fail('no row lists the joints');
  assert.equal(jointsRow.children.filter(isJoint).length, hanging, 'the joints are not as many as the bodies hanging on them');
  assert.deepEqual(jointsRow.creates, [], 'the joints row creates something');
  const attachable = opened.elements.filter((element) => {
    const attachment = attachments.get(elementKey(element));
    return attachment !== undefined && attachment.canTakeJoint && attachment.joint === null;
  });
  assert.deepEqual(jointsRow.attaches, attachable, 'the joints row attaches other bodies than the free ones that can take a joint and the unattached ones');
  const lines = [
    `${vehicle.name} attachments: ${attachments.size} ${attachedKind.kind} each read with its attachment, ${opened.elements.length - attachments.size} other elements and ${settingsRows.length} settings with none;`,
    `  ${hanging} hanging on a joint, each held at a frame of the outline, ${free} free (${keptFree} of them kept free), ${attachments.size - hanging - free} neither;`,
    `  the row listing the ${hanging} joints creates nothing and attaches ${attachable.length}: ${attachable.map((element) => element.name).join(', ')}`,
  ];
  if (!withChanges) return lines.join('\n');

  // A joint is not created alone.
  const jointKind = description.jointKinds[0]!;
  const alone = await session.createElement(jointKind, unusedName(opened.elements, 'attachment_check_joint'), {}, null);
  assert.notEqual(alone, null, 'a joint was created alone');
  // A body created with a joint, held at a body that moves freely, hangs on it.
  const heldAt = opened.elements.find((element) => readsFree(attachments.get(elementKey(element)) ?? null)) ?? fail('no body reads free');
  const heldName = unusedName(opened.elements, 'attachment_check_held_body');
  const heldJoint = { kind: jointKind, name: unusedName(opened.elements, 'attachment_check_held_joint') };
  const heldRefusal = await session.createElement(attachedKind.kind, heldName, {}, { joint: { ...heldJoint, properties: { [description.otherEndProperty]: heldAt.name } } });
  assert.equal(heldRefusal, null, `creating a body with a joint was refused: ${heldRefusal?.message}`);
  assert.deepEqual(session.state.attachment?.joint, heldJoint, 'the body created with a joint does not hang on it');
  assert.deepEqual(session.state.attachment?.heldAt, heldAt, 'the body created with a joint is not held at the frame it was given');
  // A body created free takes a joint, and becomes free again: it reads free, its joint gone, and stays selected.
  const freeBody = { kind: attachedKind.kind, name: unusedName(opened.elements, 'attachment_check_free_body') };
  assert.equal(await session.createElement(freeBody.kind, freeBody.name, {}, { free: true }), null);
  assert.ok(readsFree(session.state.attachment), 'a body created free does not read free');
  await settled(session, session.startChangingAttachment());
  const takenJoint = { kind: jointKind, name: unusedName(opened.elements, 'attachment_check_taken_joint') };
  const taken = await session.setAttachment({ joint: { ...takenJoint, properties: { [description.bodyEndProperty]: freeBody.name, [description.otherEndProperty]: heldAt.name } } });
  assert.equal(taken, true, `taking a joint was refused: ${session.state.attachmentChange?.refusal?.message}`);
  assert.deepEqual([session.state.selection, session.state.attachment?.joint], [{ element: freeBody }, takenJoint]);
  await settled(session, session.startChangingAttachment());
  assert.equal(await session.setAttachment({ free: true }), true, `becoming free was refused: ${session.state.attachmentChange?.refusal?.message}`);
  assert.ok(readsFree(session.state.attachment), 'the body that took a joint and became free again does not read free');
  assert.ok(!session.state.outline!.elements.some((element) => sameElement(element, takenJoint)), 'the joint stays after the body became free');
  for (let step = 0; step < 4; step += 1) state = await settled(session, session.undo());
  assert.equal(state.canUndo, false);
  assert.deepEqual(state.outline!.elements, opened.elements);
  lines.push(
    `  a joint alone refused ('${alone?.message}'); '${heldName}' created hanging on ${jointKind} '${heldJoint.name}' held at '${heldAt.name}';`,
    `  '${freeBody.name}' created free, took ${jointKind} '${takenJoint.name}' and became free again, reading free with its joint gone; four undos gave back the outline`,
  );
  return lines.join('\n');
}

/**
 * The settings of the model tree through the interface's session: every
 * editable number of every settings row is set to another value, read back as
 * sent and undone, one step each; the one the drawing's wheels have as their
 * nominal rolling radius draws them anew at the new radius; a property the
 * service describes as read-only is refused, about its settings. Settings are
 * found as rows that select settings, and properties by their descriptions.
 */
async function checkSettings(origin: string, vehicle: ReferenceVehicle): Promise<string> {
  const session = sessionAt(origin);
  let state = await settled(session, session.open(referenceVehicle(vehicle)));
  const rows = treeRows(state.modelTree!).filter((row) => row.provided && row.subject !== null && 'settings' in row.subject);
  assert.ok(rows.length > 0, 'the model tree has no settings');
  const wheelRadius = (state.preview ?? fail('no drawing after opening')).wheels[0]?.nominalRollingRadiusMeters ?? fail('no wheel is drawn');
  const valueOf = (name: string): unknown => session.state.properties.find((property) => property.name === name)?.value;
  const edited: string[] = [];
  let wheelsRedrawnAt: number | null = null;
  let readOnly: { row: ModelTreeNode; property: string; value: unknown } | null = null;
  for (const row of rows) {
    state = await settled(session, session.select(row.subject));
    for (const property of state.properties) {
      if (!property.editable) {
        readOnly ??= { row, property: property.name, value: property.value };
        continue;
      }
      if (property.valueKind !== 'number') continue;
      const before = property.value;
      const changed = before === null || before === 0 ? 0.01 : before * 1.25;
      const revision = session.state.revision;
      assert.equal(await session.setProperty(row.subject!, property.name, changed), true, `${row.title?.en}: setting ${property.name} was refused: ${session.state.editRefusal?.message}`);
      assert.deepEqual([session.state.revision, valueOf(property.name)], [revision + 1, changed], `${row.title?.en}: ${property.name} did not read back as set`);
      if (before === wheelRadius) {
        const preview = session.state.preview ?? fail('no drawing after setting the radius');
        assert.equal(preview.projectRevision, session.state.revision, `the check refused a nominal rolling radius of ${changed}: ${session.state.check?.report.refusals.join('; ')}`);
        assert.ok(preview.wheels.every((wheel) => wheel.nominalRollingRadiusMeters === changed), 'the wheels are not drawn at the new radius');
        wheelsRedrawnAt = changed;
      }
      state = await settled(session, session.undo());
      assert.equal(valueOf(property.name), before, `${row.title?.en}: undo did not restore ${property.name}`);
      edited.push(`${row.title?.en} · ${property.name}`);
    }
  }
  assert.ok(edited.length > 0, 'no settings number can be set');
  assert.ok(wheelsRedrawnAt !== null, 'no settings number is the wheels\' nominal rolling radius in the drawing');
  const refusedOne = readOnly ?? fail('no settings property is read-only');
  const refused = await modelingServerCommandChannel(origin)('set_property', { subject: refusedOne.row.subject, property: refusedOne.property, value: refusedOne.value });
  if (refused.ok) fail(`${refusedOne.row.title?.en}: the read-only ${refusedOne.property} was set`);
  assert.ok(sameSubject(refused.refusal.subject, refusedOne.row.subject), 'the refusal is not about the settings');
  return [
    `${vehicle.name} settings: ${rows.length} settings rows (${rows.map((row) => row.title?.en).join(', ')});`,
    `  ${edited.length} editable numbers each set, read back and undone: ${edited.join(', ')};`,
    `  the wheels drawn anew at a nominal rolling radius of ${wheelsRedrawnAt} (from ${wheelRadius}); '${refusedOne.row.title?.en} · ${refusedOne.property}' refused as read-only:`,
    `  '${refused.refusal.message}'`,
  ].join('\n');
}

async function checkNonFittingVehicle(origin: string, vehicle: ReferenceVehicle): Promise<string> {
  const session = sessionAt(origin);
  const state = await settled(session, session.open(referenceVehicle(vehicle)));
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

/** What get_session says now. */
async function serviceSession(send: SendCommand): Promise<ServiceSessionDescription> {
  const answer = await send('get_session');
  if (!answer.ok) fail(`get_session was refused: ${answer.refusal.message}`);
  return readServiceSessionDescription(answer.result);
}

/** Opens the choice, discarding the unsaved changes the session asks about first. */
async function openedDiscardingChanges(session: VehicleEditingSession, choice: OpenChoice): Promise<SessionState> {
  await session.open(choice);
  if (session.state.pendingOpen !== null) await session.discardChangesAndOpen();
  return settled(session, Promise.resolve());
}

/** The saved project's name; the folder starts empty. */
const savedProjectName = 'check_service_saved_gz18';

/**
 * Saving and opening a project through the interface's session, in the
 * folder of projects the server was started with, which starts empty. The
 * edited value is found by its description, as in checkVehicle.
 */
async function checkProjectSaving(origin: string, gz18: ReferenceVehicle, irw: ReferenceVehicle): Promise<string> {
  const send = modelingServerCommandChannel(origin);
  const directory = await fetchVehicleProjectsDirectory(origin);
  const session = new VehicleEditingSession(send);
  // The checks before left a session open: the start resumes it, as a reload of the page would.
  let state = await settled(session, session.start(directory, null));
  assert.ok(state.vehicle !== null && state.outline !== null, 'the start did not resume the open session');
  assert.equal((await serviceSession(send)).open, true);

  state = await openedDiscardingChanges(session, referenceVehicle(gz18));
  assert.deepEqual(await serviceSession(send), { open: true, vehicleProjectFile: null });
  assert.equal(state.hasUnsavedChanges, false);
  const opened = state.outline ?? fail('no outline after opening');
  const numbers = await editableNumbers(session, opened.elements.filter((element, index) => opened.elements.findIndex((other) => other.kind === element.kind) === index));
  const edited = numbers.find((entry) => entry.property.value! > 0) ?? fail('no editable positive number');
  const doubled = 2 * edited.property.value!;
  assert.equal(await session.setProperty({ element: edited.element }, edited.property.name, doubled), true);
  assert.equal(session.state.hasUnsavedChanges, true);
  const { revision: revisionBeforeSaving, canUndo: canUndoBeforeSaving, canRedo: canRedoBeforeSaving } = session.state;

  // A name the service refuses stays at the dialog with its reason, and nothing is written.
  await session.startSaveAs();
  await session.saveAs('not a project name', false);
  const refusedDialog = session.state.saveAs ?? fail('a refused Save as closed its dialog');
  const refusal = refusedDialog.refusal ?? fail('the service took a name with spaces in it');
  assert.equal(refusedDialog.name, 'not a project name');
  assert.deepEqual(await readdir(directory), [], 'a refused Save as wrote into the folder');

  // Save as: written, the vehicle renamed in one revision, the session tied to the file.
  state = await settled(session, session.saveAs(savedProjectName, false));
  assert.equal(state.saveAs, null, `Save as was refused: ${state.saveAs?.refusal}`);
  assert.deepEqual(state.vehicleProjectFile, { directory, name: savedProjectName });
  assert.equal(state.hasUnsavedChanges, false);
  assert.equal(state.outline?.vehicleName, savedProjectName);
  assert.equal(state.revision, revisionBeforeSaving + 1);
  assert.deepEqual(await serviceSession(send), { open: true, vehicleProjectFile: { directory, name: savedProjectName } });
  assert.equal((await readdir(directory)).length, 1, 'the folder holds more than the one project file');
  const savedOutline = state.outline!;
  const savedCheck = checkStatus(state);

  // The name is the document's identity, not an edit: no undo step, and undoing the edit before keeps the name.
  assert.deepEqual([state.canUndo, state.canRedo], [canUndoBeforeSaving, canRedoBeforeSaving], 'Save as changed what can be undone or redone');
  state = await settled(session, session.undo());
  assert.equal(state.outline?.vehicleName, savedProjectName, 'undo after Save as brought an earlier name back');
  assert.equal(state.hasUnsavedChanges, true);
  state = await settled(session, session.redo());
  assert.equal(state.hasUnsavedChanges, false, 'redo back to the saved state left changes unsaved');

  // The folder lists the project.
  state = await settled(session, session.showOpenChoices());
  assert.deepEqual(state.vehicleProjects, [savedProjectName]);
  session.leaveOpenChoices();

  // IRW, from its files: no file, nothing to ask about after the save.
  await session.open(referenceVehicle(irw));
  state = await settled(session, Promise.resolve());
  assert.equal(state.pendingOpen, null, 'the session asked about unsaved changes right after saving');
  assert.deepEqual(await serviceSession(send), { open: true, vehicleProjectFile: null });
  assert.notEqual(state.outline?.vehicleName, savedProjectName);

  // The project again, as it was saved.
  state = await settled(session, session.open({ source: 'vehicle_project', name: savedProjectName }));
  assert.deepEqual(state.vehicleProjectFile, { directory, name: savedProjectName });
  assert.deepEqual([state.revision, state.canUndo, state.hasUnsavedChanges], [0, false, false]);
  assert.deepEqual(state.outline, savedOutline);
  assert.equal(checkStatus(state), savedCheck);
  state = await settled(session, session.select({ element: edited.element }));
  assert.equal(state.properties.find((property) => property.name === edited.property.name)?.value, doubled);
  assert.deepEqual(await serviceSession(send), { open: true, vehicleProjectFile: { directory, name: savedProjectName } });

  // Unsaved changes follow undo; Save writes to the file and leaves nothing unsaved.
  const tripled = 3 * edited.property.value!;
  assert.equal(await session.setProperty({ element: edited.element }, edited.property.name, tripled), true);
  assert.equal(session.state.hasUnsavedChanges, true);
  state = await settled(session, session.undo());
  assert.equal(state.hasUnsavedChanges, false, 'undoing the one edit since opening left changes unsaved');
  assert.equal(await session.setProperty({ element: edited.element }, edited.property.name, tripled), true);
  state = await settled(session, session.save());
  assert.equal(state.hasUnsavedChanges, false);
  assert.equal((await readdir(directory)).length, 1);
  assert.deepEqual(await serviceSession(send), { open: true, vehicleProjectFile: { directory, name: savedProjectName } });

  return [
    `GZ18 saved as a project: ${edited.element.name}'s ${edited.property.name} doubled, a name with spaces refused at the dialog ('${refusal}'),`,
    `  saved as '${savedProjectName}' at revision ${revisionBeforeSaving + 1} with the vehicle renamed and no undo step: undo kept the name, redo was saved again,`,
    `  the folder listed [${savedProjectName}], IRW opened with no file,`,
    `  the project opened again at revision 0 with its ${savedOutline.elements.length} elements and the doubled value, check ${savedCheck};`,
    `  changes unsaved after an edit and none after its undo, Save left nothing unsaved; get_session named the file at each stage`,
  ].join('\n');
}

// The reference vehicles exactly as serve:reference offers them, and the one that does not fit; any free port; a
// temporary folder of projects, removed at the end.
const referenceVehicles = await readReferenceVehicleCatalogue(fileURLToPath(new URL('../reference_vehicle_catalogue.json', import.meta.url)));
const vehicleProjectsDirectory = await mkdtemp(join(tmpdir(), 'orvd-check-service-projects-'));
try {
  const options = {
    servicePath: resolve(servicePath),
    host: '127.0.0.1',
    port: 0,
    staticDirectory: fileURLToPath(new URL('../dist', import.meta.url)),
    referenceVehicles: [...referenceVehicles, nonFittingVehicle],
    vehicleProjectsDirectory,
    terminal: null,
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
    const named = (name: string): ReferenceVehicle => vehicles.find((vehicle) => vehicle.name === name) ?? fail(`the catalogue has no ${name}`);
    console.log(await checkAttachments(origin, named('GZ18'), true));
    console.log(await checkAttachments(origin, named('IRW'), false));
    console.log(await checkStructuralEditing(origin, named('GZ18')));
    console.log(await checkSettings(origin, named('GZ18')));
    console.log(await checkProjectSaving(origin, named('GZ18'), named('IRW')));
    console.log('check-service: passed');
  } catch (error) {
    console.error(`check-service: ${error instanceof Error ? error.message : String(error)}`);
    process.exitCode = 1;
  } finally {
    await server.close();
  }
} finally {
  await rm(vehicleProjectsDirectory, { recursive: true, force: true });
}
