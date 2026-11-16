// The modelling service's answers, as the interface reads them. The protocol
// is stated at the top of tools/vehicle_modeling_service/vehicle_modeling_service.cc.
// Answers come from our own service, so they are read leniently: only the
// fields the interface uses are read, unknown fields are ignored, and a field
// that is used but missing or of the wrong shape fails loudly.

export interface ElementReference {
  kind: string;
  name: string;
}

export type Vector3 = [number, number, number];
/** Three rows of a rotation matrix. */
export type Rotation = [Vector3, Vector3, Vector3];

/** The session after an accepted command, and the command's own result. */
export interface Accepted {
  ok: true;
  revision: number;
  canUndo: boolean;
  canRedo: boolean;
  result: unknown;
}

export interface Refusal {
  /** What the refusal concerns, when it concerns an element or a property. */
  element: ElementReference | null;
  property: string | null;
  message: string;
}

export type ServiceAnswer = Accepted | { ok: false; refusal: Refusal };

export interface Outline {
  vehicleName: string;
  /** Every element, kind by kind in the service's order. */
  elements: ElementReference[];
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
 * element holds no value, and where an optional reference names none.
 */
export type PropertyValue =
  | { valueKind: 'number'; value: number | null }
  | { valueKind: 'vector3'; value: Vector3 | null }
  | { valueKind: 'boolean'; value: boolean | null }
  | { valueKind: 'rotation'; value: Rotation | null }
  /** The element's own name; setting it renames the element. */
  | { valueKind: 'element_name'; value: string | null }
  | { valueKind: 'element_reference'; referencedKinds: string[]; optional: boolean; value: string | null }
  | { valueKind: 'token'; choices: string[]; value: string | null }
  | { valueKind: typeof curveValueKind; rowFields: string[]; value: CurvePoint[] | null };

export type PropertyReading = { name: string; editable: boolean } & PropertyValue;

type WithoutValue<Reading> = Reading extends unknown ? Omit<Reading, 'value'> : never;

/** What a property is, without a value: as describe_element_kinds states it for a kind. */
export type PropertyDescription = WithoutValue<PropertyReading>;

/** An element kind and its properties, in the order get_properties returns them. */
export interface ElementKindDescription {
  kind: string;
  properties: PropertyDescription[];
}

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

export function readAnswer(json: unknown): ServiceAnswer {
  if (field(json, 'ok', 'answer') === true) {
    return {
      ok: true,
      revision: number(field(json, 'revision', 'answer'), 'answer.revision'),
      canUndo: truth(field(json, 'can_undo', 'answer'), 'answer.can_undo'),
      canRedo: truth(field(json, 'can_redo', 'answer'), 'answer.can_redo'),
      result: field(json, 'result', 'answer'),
    };
  }
  const error = field(json, 'error', 'answer');
  return {
    ok: false,
    refusal: {
      element: nullable(field(error, 'element', 'error'), (element) => readElement(element, 'error.element')),
      property: nullable(field(error, 'property', 'error'), (property) => text(property, 'error.property')),
      message: text(field(error, 'message', 'error'), 'error.message'),
    },
  };
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
  const described = { name, editable: truth(field(property, 'editable', name), `${name}.editable`) };
  switch (valueKind) {
    case 'number':
    case 'vector3':
    case 'boolean':
    case 'rotation':
    case 'element_name':
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
    case 'boolean':
      return { ...description, value: nullable(value, (present) => truth(present, what)) };
    case 'rotation':
      return { ...description, value: nullable(value, (present) => rotation(present, what)) };
    case 'element_name':
    case 'element_reference':
    case 'token':
      return { ...description, value: nullable(value, (present) => text(present, what)) };
    case curveValueKind:
      return { ...description, value: nullable(value, (present) => curve(present, description.rowFields, what)) };
  }
}

export function readProperties(result: unknown): PropertyReading[] {
  return list(field(result, 'properties', 'properties'), 'properties').map(propertyReading);
}

export function readElementKinds(result: unknown): ElementKindDescription[] {
  return list(field(result, 'element_kinds', 'describe_element_kinds'), 'element_kinds').map((entry) => ({
    kind: text(field(entry, 'kind', 'element kind'), 'element kind.kind'),
    properties: list(field(entry, 'properties', 'element kind'), 'element kind.properties').map(propertyDescription),
  }));
}

/** The element an accepted set_property or add_element leaves: after a rename, under its new name. */
export function readEditedElement(result: unknown): ElementReference {
  return readElement(field(result, 'element', 'edit result'), 'edit result.element');
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
