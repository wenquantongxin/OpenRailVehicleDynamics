// The modelling service's answers, as the interface reads them. The protocol
// is stated at the top of tools/vehicle_modeling_service/vehicle_modeling_service.cc.
// Answers come from our own service, so they are read leniently: only the
// fields the interface uses are read, unknown fields are ignored, and a field
// that is used but missing or of the wrong shape fails loudly.

export interface ElementReference {
  kind: string;
  name: string;
}

/** A name for people, in English and Chinese, as the service gives it. */
export interface BilingualTitle {
  en: string;
  zh: string;
}

/**
 * What has properties: an element, or settings that are not an element. The
 * settings word is the service's; the interface passes it on and never reads it.
 */
export type PropertySubject = { element: ElementReference } | { settings: string };

export type Vector3 = [number, number, number];
/** Three rows of a rotation matrix. */
export type Rotation = [Vector3, Vector3, Vector3];

/** The open session after an accepted command, as every accepted answer states it. */
export interface SessionAfterCommand {
  revision: number;
  canUndo: boolean;
  canRedo: boolean;
  /** The project differs from the state it was last saved in or, before any save, the state it was opened in. */
  hasUnsavedChanges: boolean;
}

/** An accepted command's answer: the session after it, and the command's own result. */
export interface Accepted {
  ok: true;
  /** Null while no vehicle is open, when only get_session and list_vehicle_projects are accepted. */
  session: SessionAfterCommand | null;
  result: unknown;
}

export interface Refusal {
  /** What the refusal concerns, when it concerns a subject or a property of one; for add_element, the element that was to be created. */
  subject: PropertySubject | null;
  property: string | null;
  message: string;
}

export type ServiceAnswer = Accepted | { ok: false; refusal: Refusal };

/** A project file as the service names it: the folder of projects it lies in, and the project's name. */
export interface VehicleProjectFile {
  directory: string;
  name: string;
}

/** What get_session says is open: nothing, or a vehicle with the project file the session is associated with, if any. */
export type ServiceSessionDescription = { open: false } | { open: true; vehicleProjectFile: VehicleProjectFile | null };

export interface Outline {
  vehicleName: string;
  /** Every element, kind by kind in the service's order. */
  elements: ElementReference[];
}

/** What a row's "+" creates: an element kind, with the properties the row fixes for it. */
export interface CreationOffer {
  kind: string;
  fixedProperties: Readonly<Record<string, unknown>>;
}

/**
 * A row of the model tree, as the service describes it: the interface draws
 * what it is given and knows none of the rows.
 */
export interface ModelTreeNode {
  /** Unique in the tree, and the same for the same row across edits, a rename of its element excepted. */
  key: string;
  /** Null for an element's row, which shows the element's name. */
  title: BilingualTitle | null;
  /** What selecting the row selects; null for a row that only groups. */
  subject: PropertySubject | null;
  /** False while what the row stands for is not provided yet. */
  provided: boolean;
  /** One line shown with the row. */
  note: BilingualTitle | null;
  creates: CreationOffer[];
  /** The bodies the row's "+" attaches, in outline order. */
  attaches: ElementReference[];
  children: ModelTreeNode[];
}

export interface CheckReport {
  passed: boolean;
  refusals: string[];
  information: string[];
  coverageStatements: string[];
}

/** One row of a curve: a number for each of the property's row fields. */
export type CurvePoint = Readonly<Record<string, number>>;

/** The value kind of a curve, a list of rows of numbers. */
export const curveValueKind = 'saturated_piecewise_linear_damper_curve';

/**
 * A property value by its value kind, beside what the kind says about the
 * value: the kinds a reference may name and whether it may name none, the
 * choices of a token, the fields of a curve's rows. A value is null where the
 * subject holds no value, and where an optional reference names none.
 */
export type PropertyValue =
  | { valueKind: 'number'; value: number | null }
  | { valueKind: 'vector3'; value: Vector3 | null }
  | { valueKind: 'rotation'; value: Rotation | null }
  /** The element's own name; setting it renames the element. */
  | { valueKind: 'element_name'; value: string | null }
  | { valueKind: 'element_reference'; referencedKinds: string[]; optional: boolean; value: string | null }
  | { valueKind: 'token'; choices: string[]; value: string | null }
  | { valueKind: typeof curveValueKind; rowFields: string[]; value: CurvePoint[] | null }
  /** A text shown as it is: never an element's own name and never a reference. */
  | { valueKind: 'text'; value: string | null };

export type PropertyReading = {
  name: string;
  editable: boolean;
  /** The section the property is shown in; consecutive properties with the same section form one. */
  section: BilingualTitle | null;
} & PropertyValue;

type WithoutValue<Reading> = Reading extends unknown ? Omit<Reading, 'value'> : never;

/** What a property is, without a value: as describe_element_kinds states it for a kind. */
export type PropertyDescription = WithoutValue<PropertyReading>;

/**
 * How the elements of a kind are attached, as describe_element_kinds states
 * it for the one kind that is: free in the world, or by a joint of one of the
 * joint kinds, whose one end is on the element and whose other end names the
 * frame it is held at.
 */
export interface AttachmentDescription {
  title: BilingualTitle;
  freeTitle: BilingualTitle;
  /** What the joint an element hangs on is shown under. */
  jointTitle: BilingualTitle;
  /** What the frame it is held at is shown under. */
  heldAtTitle: BilingualTitle;
  jointKinds: string[];
  /** The joint's property naming its end on the attached element. */
  bodyEndProperty: string;
  /** The joint's property naming the frame the element is held at. */
  otherEndProperty: string;
  /** The element's properties only one that moves freely has. */
  freeOnlyProperties: string[];
}

/** An element kind, its name for people, its properties in the order get_properties returns them, and how its elements are attached. */
export interface ElementKindDescription {
  kind: string;
  title: BilingualTitle;
  properties: PropertyDescription[];
  /** Null for every kind but the one whose elements are attached. */
  attachment: AttachmentDescription | null;
}

/** How an element is attached now and what can be changed, as get_properties states it with its properties. */
export interface Attachment {
  /** One short line saying how it is held, naming no element. */
  summary: BilingualTitle;
  /** One line more: what is the matter, or why something cannot be changed. */
  note: BilingualTitle | null;
  /** The joint it hangs on. */
  joint: ElementReference | null;
  /** The frame at that joint's other end: an element of a kind a frame is, a body or a fixed frame. */
  heldAt: ElementReference | null;
  canBecomeFree: boolean;
  canTakeJoint: boolean;
  /** What the joint's end on it may name: itself, then its frames. */
  framesOnBody: ElementReference[];
  /** What the joint's other end may name. */
  framesItCanBeHeldAt: ElementReference[];
}

/** What an attachment is set to: moving freely, or a joint of a kind, with its name and properties. */
export type AttachmentSetting = { free: true } | { joint: { kind: string; name: string; properties: Readonly<Record<string, unknown>> } };

/** A body or fixed frame drawn at its pose; `rotationDrawingFromLocal` takes its own components to the drawing frame. */
export interface PosedDrawing {
  element: ElementReference;
  rotationDrawingFromLocal: Rotation;
  originInDrawingFrameMeters: Vector3;
}

export interface RigidBodyDrawing extends PosedDrawing {
  /** The body name its shapes are stated under in a visual definition; null for a body that has no shapes. */
  visualDefinitionBodyName: string | null;
}

export interface JointDrawing {
  element: ElementReference;
  locationInDrawingFrameMeters: Vector3;
}

export interface RevoluteJointDrawing extends JointDrawing {
  /** A unit vector. */
  axisInDrawingFrame: Vector3;
}

export interface ForceElementDrawing {
  element: ElementReference;
  referenceEndInDrawingFrameMeters: Vector3;
  oppositeEndInDrawingFrameMeters: Vector3;
}

/** A wheel of a contact interface (`element`), turning with its wheel body. */
export interface WheelDrawing {
  element: ElementReference;
  wheelBody: ElementReference;
  /** The centre of the nominal rolling circle. */
  datumInDrawingFrameMeters: Vector3;
  /** A unit vector along the spin axis, from the wheel's back face to its outer face. */
  outwardSpinAxisInDrawingFrame: Vector3;
  nominalRollingRadiusMeters: number;
}

/**
 * What the 3D view draws, in the drawing frame: +x forward, +y right, +z down,
 * metres. Every drawing names the element it shows. projectRevision is the
 * revision it was computed from.
 */
export interface Preview {
  projectRevision: number;
  rigidBodies: RigidBodyDrawing[];
  fixedFrames: PosedDrawing[];
  revoluteJoints: RevoluteJointDrawing[];
  ballRpyJoints: JointDrawing[];
  weldJoints: JointDrawing[];
  forceElements: ForceElementDrawing[];
  wheels: WheelDrawing[];
  notDrawableElements: ElementReference[];
}

function field(object: unknown, key: string, what: string): unknown {
  if (typeof object !== 'object' || object === null || !(key in object)) {
    throw new Error(`modelling service answer: ${what}.${key} is missing`);
  }
  return (object as Record<string, unknown>)[key];
}

function number(value: unknown, what: string): number {
  if (typeof value !== 'number') throw new Error(`modelling service answer: ${what} is not a number`);
  return value;
}

function text(value: unknown, what: string): string {
  if (typeof value !== 'string') throw new Error(`modelling service answer: ${what} is not a string`);
  return value;
}

function truth(value: unknown, what: string): boolean {
  if (typeof value !== 'boolean') throw new Error(`modelling service answer: ${what} is not true or false`);
  return value;
}

function list(value: unknown, what: string): unknown[] {
  if (!Array.isArray(value)) throw new Error(`modelling service answer: ${what} is not a list`);
  return value;
}

function vector(value: unknown, what: string): Vector3 {
  const entries = list(value, what);
  if (entries.length !== 3) throw new Error(`modelling service answer: ${what} does not have three entries`);
  return [number(entries[0], what), number(entries[1], what), number(entries[2], what)];
}

function rotation(value: unknown, what: string): Rotation {
  const rows = list(value, what);
  if (rows.length !== 3) throw new Error(`modelling service answer: ${what} does not have three rows`);
  return [vector(rows[0], `${what} row 0`), vector(rows[1], `${what} row 1`), vector(rows[2], `${what} row 2`)];
}

function texts(value: unknown, what: string): string[] {
  return list(value, what).map((entry) => text(entry, what));
}

export function readElement(value: unknown, what: string): ElementReference {
  return { kind: text(field(value, 'kind', what), `${what}.kind`), name: text(field(value, 'name', what), `${what}.name`) };
}

function nullable<T>(value: unknown, read: (present: unknown) => T): T | null {
  return value === null ? null : read(value);
}

function readTitle(value: unknown, what: string): BilingualTitle {
  return { en: text(field(value, 'en', what), `${what}.en`), zh: text(field(value, 'zh', what), `${what}.zh`) };
}

/** A subject: an element, or settings by the service's word for them. */
export function readSubject(value: unknown, what: string): PropertySubject {
  if (typeof value === 'object' && value !== null && 'element' in value) {
    return { element: readElement(field(value, 'element', what), `${what}.element`) };
  }
  return { settings: text(field(value, 'settings', what), `${what}.settings`) };
}

export function readAnswer(json: unknown): ServiceAnswer {
  if (field(json, 'ok', 'answer') === true) {
    // While nothing is open the four session fields are null; the revision says which it is.
    const revision = field(json, 'revision', 'answer');
    return {
      ok: true,
      session:
        revision === null
          ? null
          : {
              revision: number(revision, 'answer.revision'),
              canUndo: truth(field(json, 'can_undo', 'answer'), 'answer.can_undo'),
              canRedo: truth(field(json, 'can_redo', 'answer'), 'answer.can_redo'),
              hasUnsavedChanges: truth(field(json, 'has_unsaved_changes', 'answer'), 'answer.has_unsaved_changes'),
            },
      result: field(json, 'result', 'answer'),
    };
  }
  const error = field(json, 'error', 'answer');
  return {
    ok: false,
    refusal: {
      subject: nullable(field(error, 'subject', 'error'), (subject) => readSubject(subject, 'error.subject')),
      property: nullable(field(error, 'property', 'error'), (property) => text(property, 'error.property')),
      message: text(field(error, 'message', 'error'), 'error.message'),
    },
  };
}

export function readServiceSessionDescription(result: unknown): ServiceSessionDescription {
  if (!truth(field(result, 'open', 'get_session'), 'get_session.open')) {
    return { open: false };
  }
  return {
    open: true,
    vehicleProjectFile: nullable(field(result, 'vehicle_project_file', 'get_session'), (file) => ({
      directory: text(field(file, 'directory', 'vehicle_project_file'), 'vehicle_project_file.directory'),
      name: text(field(file, 'name', 'vehicle_project_file'), 'vehicle_project_file.name'),
    })),
  };
}

/** The names of the projects in a folder of projects, sorted, as list_vehicle_projects gives them. */
export function readVehicleProjectNames(result: unknown): string[] {
  return texts(field(result, 'vehicle_projects', 'list_vehicle_projects'), 'list_vehicle_projects.vehicle_projects');
}

/** The visual definition document the project carries, unread: a JSON object, or null where it carries none. */
export function readVisualDefinitionDocument(result: unknown): unknown {
  return field(result, 'visual_definition', 'get_visual_definition');
}

export function readOutline(result: unknown): Outline {
  return {
    vehicleName: text(field(result, 'vehicle_name', 'outline'), 'outline.vehicle_name'),
    elements: list(field(result, 'elements', 'outline'), 'outline.elements').map((element) => readElement(element, 'outline element')),
  };
}

export function readCheckReport(result: unknown): CheckReport {
  return {
    passed: truth(field(result, 'passed', 'check'), 'check.passed'),
    refusals: texts(field(result, 'refusals', 'check'), 'check.refusals'),
    information: texts(field(result, 'information', 'check'), 'check.information'),
    coverageStatements: texts(field(result, 'coverage_statements', 'check'), 'check.coverage_statements'),
  };
}

/** A curve's rows, each read by the property's row fields. */
function curve(value: unknown, rowFields: readonly string[], what: string): CurvePoint[] {
  return list(value, what).map((row) => Object.fromEntries(rowFields.map((rowField) => [rowField, number(field(row, rowField, what), `${what}.${rowField}`)])));
}

function propertyDescription(property: unknown): PropertyDescription {
  const name = text(field(property, 'name', 'property'), 'property.name');
  const valueKind = text(field(property, 'value_kind', name), `${name}.value_kind`);
  const described = {
    name,
    editable: truth(field(property, 'editable', name), `${name}.editable`),
    section: nullable(field(property, 'section', name), (section) => readTitle(section, `${name}.section`)),
  };
  switch (valueKind) {
    case 'number':
    case 'vector3':
    case 'rotation':
    case 'element_name':
    case 'text':
      return { ...described, valueKind };
    case 'element_reference':
      return {
        ...described,
        valueKind,
        referencedKinds: texts(field(property, 'referenced_kinds', name), `${name}.referenced_kinds`),
        optional: truth(field(property, 'optional', name), `${name}.optional`),
      };
    case 'token':
      return { ...described, valueKind, choices: texts(field(property, 'choices', name), `${name}.choices`) };
    case curveValueKind:
      return { ...described, valueKind, rowFields: texts(field(property, 'row_fields', name), `${name}.row_fields`) };
    default:
      throw new Error(`modelling service answer: ${name} has the value kind '${valueKind}', which the interface cannot show`);
  }
}

function propertyReading(property: unknown): PropertyReading {
  const description = propertyDescription(property);
  const value = field(property, 'value', description.name);
  const what = description.name;
  switch (description.valueKind) {
    case 'number':
      return { ...description, value: nullable(value, (present) => number(present, what)) };
    case 'vector3':
      return { ...description, value: nullable(value, (present) => vector(present, what)) };
    case 'rotation':
      return { ...description, value: nullable(value, (present) => rotation(present, what)) };
    case 'element_name':
    case 'element_reference':
    case 'token':
    case 'text':
      return { ...description, value: nullable(value, (present) => text(present, what)) };
    case curveValueKind:
      return { ...description, value: nullable(value, (present) => curve(present, description.rowFields, what)) };
  }
}

export function readProperties(result: unknown): PropertyReading[] {
  return list(field(result, 'properties', 'properties'), 'properties').map(propertyReading);
}

function elementList(value: unknown, what: string): ElementReference[] {
  return list(value, what).map((element) => readElement(element, what));
}

/** The attachment get_properties states beside a subject's properties; null for every subject but an attached element. */
export function readAttachment(result: unknown): Attachment | null {
  return nullable(field(result, 'attachment', 'properties'), (attachment) => ({
    summary: readTitle(field(attachment, 'summary', 'attachment'), 'attachment.summary'),
    note: nullable(field(attachment, 'note', 'attachment'), (note) => readTitle(note, 'attachment.note')),
    joint: nullable(field(attachment, 'joint', 'attachment'), (joint) => readElement(joint, 'attachment.joint')),
    heldAt: nullable(field(attachment, 'held_at', 'attachment'), (heldAt) => readElement(heldAt, 'attachment.held_at')),
    canBecomeFree: truth(field(attachment, 'can_become_free', 'attachment'), 'attachment.can_become_free'),
    canTakeJoint: truth(field(attachment, 'can_take_joint', 'attachment'), 'attachment.can_take_joint'),
    framesOnBody: elementList(field(attachment, 'frames_on_body', 'attachment'), 'attachment.frames_on_body'),
    framesItCanBeHeldAt: elementList(field(attachment, 'frames_it_can_be_held_at', 'attachment'), 'attachment.frames_it_can_be_held_at'),
  }));
}

function attachmentDescription(value: unknown, kind: string): AttachmentDescription {
  const what = `${kind}.attachment`;
  return {
    title: readTitle(field(value, 'title', what), `${what}.title`),
    freeTitle: readTitle(field(value, 'free_title', what), `${what}.free_title`),
    jointTitle: readTitle(field(value, 'joint_title', what), `${what}.joint_title`),
    heldAtTitle: readTitle(field(value, 'held_at_title', what), `${what}.held_at_title`),
    jointKinds: texts(field(value, 'joint_kinds', what), `${what}.joint_kinds`),
    bodyEndProperty: text(field(value, 'body_end_property', what), `${what}.body_end_property`),
    otherEndProperty: text(field(value, 'other_end_property', what), `${what}.other_end_property`),
    freeOnlyProperties: texts(field(value, 'free_only_properties', what), `${what}.free_only_properties`),
  };
}

export function readElementKinds(result: unknown): ElementKindDescription[] {
  return list(field(result, 'element_kinds', 'describe_element_kinds'), 'element_kinds').map((entry) => {
    const kind = text(field(entry, 'kind', 'element kind'), 'element kind.kind');
    return {
      kind,
      title: readTitle(field(entry, 'title', kind), `${kind}.title`),
      properties: list(field(entry, 'properties', kind), `${kind}.properties`).map(propertyDescription),
      attachment: nullable(field(entry, 'attachment', kind), (attachment) => attachmentDescription(attachment, kind)),
    };
  });
}

function modelTreeNode(node: unknown): ModelTreeNode {
  const key = text(field(node, 'key', 'model tree row'), 'model tree row.key');
  return {
    key,
    title: nullable(field(node, 'title', key), (title) => readTitle(title, `${key}.title`)),
    subject: nullable(field(node, 'subject', key), (subject) => readSubject(subject, `${key}.subject`)),
    provided: truth(field(node, 'provided', key), `${key}.provided`),
    note: nullable(field(node, 'note', key), (note) => readTitle(note, `${key}.note`)),
    creates: list(field(node, 'creates', key), `${key}.creates`).map((offer) => ({
      kind: text(field(offer, 'kind', `${key} offer`), `${key} offer.kind`),
      fixedProperties: field(offer, 'fixed_properties', `${key} offer`) as Readonly<Record<string, unknown>>,
    })),
    attaches: elementList(field(node, 'attaches', key), `${key}.attaches`),
    children: list(field(node, 'children', key), `${key}.children`).map(modelTreeNode),
  };
}

/** The model tree as get_model_tree describes it: its first-level rows, each with the rows under it. */
export function readModelTree(result: unknown): ModelTreeNode[] {
  return list(field(result, 'nodes', 'get_model_tree'), 'get_model_tree.nodes').map(modelTreeNode);
}

/** The element an accepted add_element created. */
export function readCreatedElement(result: unknown): ElementReference {
  return readElement(field(result, 'element', 'add_element'), 'add_element.element');
}

/** The subject an accepted set_property leaves: an element under its new name after a rename. */
export function readEditedSubject(result: unknown): PropertySubject {
  return readSubject(field(result, 'subject', 'set_property'), 'set_property.subject');
}

/** What remove_element removed, or get_removal_consequences says it would: the element and everything with it. */
export function readRemovedElements(result: unknown): ElementReference[] {
  return list(field(result, 'removed_elements', 'removal'), 'removal.removed_elements').map((element) => readElement(element, 'removed element'));
}

function posed(value: unknown, rotationKey: string, what: string): PosedDrawing {
  return {
    element: readElement(field(value, 'element', what), `${what}.element`),
    rotationDrawingFromLocal: rotation(field(value, rotationKey, what), `${what}.${rotationKey}`),
    originInDrawingFrameMeters: vector(field(value, 'origin_in_drawing_frame_meters', what), `${what}.origin_in_drawing_frame_meters`),
  };
}

function joint(value: unknown, what: string): JointDrawing {
  return {
    element: readElement(field(value, 'element', what), `${what}.element`),
    locationInDrawingFrameMeters: vector(field(value, 'location_in_drawing_frame_meters', what), `${what}.location_in_drawing_frame_meters`),
  };
}

/** The preview of the last check that passed, or null before any check has passed. */
export function readPreview(result: unknown): Preview | null {
  const preview = field(result, 'preview', 'get_preview');
  if (preview === null) {
    return null;
  }
  const drawings = (key: string) => list(field(preview, key, 'preview'), `preview.${key}`);
  return {
    projectRevision: number(field(preview, 'project_revision', 'preview'), 'preview.project_revision'),
    rigidBodies: drawings('rigid_bodies').map((body) => ({
      ...posed(body, 'rotation_drawing_from_body', 'rigid body drawing'),
      visualDefinitionBodyName: nullable(field(body, 'visual_definition_body_name', 'rigid body drawing'), (name) =>
        text(name, 'rigid body drawing.visual_definition_body_name'),
      ),
    })),
    fixedFrames: drawings('fixed_frames').map((frame) => posed(frame, 'rotation_drawing_from_frame', 'fixed frame drawing')),
    revoluteJoints: drawings('revolute_joints').map((revolute) => ({
      ...joint(revolute, 'revolute joint drawing'),
      axisInDrawingFrame: vector(field(revolute, 'axis_in_drawing_frame', 'revolute joint drawing'), 'revolute joint drawing.axis_in_drawing_frame'),
    })),
    ballRpyJoints: drawings('ball_rpy_joints').map((ball) => joint(ball, 'ball-RPY joint drawing')),
    weldJoints: drawings('weld_joints').map((weld) => joint(weld, 'weld joint drawing')),
    forceElements: drawings('force_elements').map((force) => ({
      element: readElement(field(force, 'element', 'force element drawing'), 'force element drawing.element'),
      referenceEndInDrawingFrameMeters: vector(
        field(force, 'reference_end_in_drawing_frame_meters', 'force element drawing'),
        'force element drawing.reference_end_in_drawing_frame_meters',
      ),
      oppositeEndInDrawingFrameMeters: vector(
        field(force, 'opposite_end_in_drawing_frame_meters', 'force element drawing'),
        'force element drawing.opposite_end_in_drawing_frame_meters',
      ),
    })),
    wheels: drawings('wheels').map((wheel) => ({
      element: readElement(field(wheel, 'element', 'wheel drawing'), 'wheel drawing.element'),
      wheelBody: readElement(field(wheel, 'wheel_body', 'wheel drawing'), 'wheel drawing.wheel_body'),
      datumInDrawingFrameMeters: vector(field(wheel, 'datum_in_drawing_frame_meters', 'wheel drawing'), 'wheel drawing.datum_in_drawing_frame_meters'),
      outwardSpinAxisInDrawingFrame: vector(field(wheel, 'outward_spin_axis_in_drawing_frame', 'wheel drawing'), 'wheel drawing.outward_spin_axis_in_drawing_frame'),
      nominalRollingRadiusMeters: number(field(wheel, 'nominal_rolling_radius_meters', 'wheel drawing'), 'wheel drawing.nominal_rolling_radius_meters'),
    })),
    notDrawableElements: drawings('not_drawable_elements').map((element) => readElement(element, 'not drawable element')),
  };
}

/** One string per element, for maps and deep links: kinds hold no colon, so the first colon separates. */
export function elementKey(element: ElementReference): string {
  return `${element.kind}:${element.name}`;
}

export function sameElement(a: ElementReference | null, b: ElementReference | null): boolean {
  return a !== null && b !== null && a.kind === b.kind && a.name === b.name;
}

/** The element a subject is, or null for settings. */
export function subjectElement(subject: PropertySubject | null): ElementReference | null {
  return subject !== null && 'element' in subject ? subject.element : null;
}

export function sameSubject(a: PropertySubject | null, b: PropertySubject | null): boolean {
  if (a === null || b === null) return false;
  if ('element' in a) return 'element' in b && sameElement(a.element, b.element);
  return 'settings' in b && a.settings === b.settings;
}

/** One string per subject, for keys: an element's, or the settings word's. */
export function subjectKey(subject: PropertySubject): string {
  return 'element' in subject ? `element/${elementKey(subject.element)}` : `settings/${subject.settings}`;
}
