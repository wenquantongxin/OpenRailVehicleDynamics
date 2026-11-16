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

/** One point of a damper curve: its fields as the service names them, in order. */
export type CurvePoint = Readonly<Record<string, number>>;

/** A property value by its value kind; null where the element holds no value. */
export type PropertyValue =
  | { valueKind: 'number'; value: number | null }
  | { valueKind: 'vector3'; value: Vector3 | null }
  | { valueKind: 'text' | 'wheel_side'; value: string | null }
  | { valueKind: 'boolean'; value: boolean | null }
  | { valueKind: 'rotation'; value: Rotation | null }
  | { valueKind: 'saturated_piecewise_linear_damper_curve'; value: CurvePoint[] | null };

export type PropertyReading = { name: string; editable: boolean } & PropertyValue;

/** A body or fixed frame drawn at its pose; `rotationDrawingFromLocal` takes its own components to the drawing frame. */
export interface PosedDrawing {
  element: ElementReference;
  rotationDrawingFromLocal: Rotation;
  originInDrawingFrameMeters: Vector3;
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
  rigidBodies: PosedDrawing[];
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

function curve(value: unknown, what: string): CurvePoint[] {
  return list(value, what).map((point) => {
    if (typeof point !== 'object' || point === null) throw new Error(`modelling service answer: a point of ${what} is not an object`);
    return Object.fromEntries(Object.entries(point).map(([key, entry]) => [key, number(entry, `${what}.${key}`)]));
  });
}

function propertyValue(valueKind: string, value: unknown, what: string): PropertyValue {
  switch (valueKind) {
    case 'number':
      return { valueKind, value: nullable(value, (present) => number(present, what)) };
    case 'vector3':
      return { valueKind, value: nullable(value, (present) => vector(present, what)) };
    case 'text':
    case 'wheel_side':
      return { valueKind, value: nullable(value, (present) => text(present, what)) };
    case 'boolean':
      return { valueKind, value: nullable(value, (present) => truth(present, what)) };
    case 'rotation':
      return { valueKind, value: nullable(value, (present) => rotation(present, what)) };
    case 'saturated_piecewise_linear_damper_curve':
      return { valueKind, value: nullable(value, (present) => curve(present, what)) };
    default:
      throw new Error(`modelling service answer: ${what} has the value kind '${valueKind}', which the interface cannot show`);
  }
}

export function readProperties(result: unknown): PropertyReading[] {
  return list(field(result, 'properties', 'properties'), 'properties').map((property) => {
    const name = text(field(property, 'name', 'property'), 'property.name');
    return {
      name,
      editable: truth(field(property, 'editable', name), `${name}.editable`),
      ...propertyValue(text(field(property, 'value_kind', name), `${name}.value_kind`), field(property, 'value', name), name),
    };
  });
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
    rigidBodies: drawings('rigid_bodies').map((body) => posed(body, 'rotation_drawing_from_body', 'rigid body drawing')),
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
