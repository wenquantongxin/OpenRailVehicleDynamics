// The parametric visual definition of a vehicle, read as the scene viewer
// reads it (apps/scene_viewer_web/src/record/visual_definition.ts) and trimmed
// to what the modelling view draws: the parts and the wheel section. Each part
// is stated in its own body's frame; sizes are schematic and carry no physics.
// An appearance names a display role; the view decides how a role looks. Of
// wheel_visual only the eight lengths of the section are read: nothing turns
// in this view, so there is no rim marker, and nothing is labelled. Labels and
// display_bindings are not read: the modelling view gives no body a role.

export type Appearance = 'shell' | 'glass' | 'floor' | 'structure' | 'accent' | 'dark';

const appearances: ReadonlySet<string> = new Set(['shell', 'glass', 'floor', 'structure', 'accent', 'dark']);

interface SolidPartBase {
  name: string;
  bodyName: string;
  appearance: Appearance;
}

export interface BoxPart extends SolidPartBase {
  kind: 'box';
  centerInBodyFrameMeters: [number, number, number];
  sizeMeters: [number, number, number];
  cornerRadiusMeters: number;
}

export interface CylinderPart extends SolidPartBase {
  kind: 'cylinder';
  startInBodyFrameMeters: [number, number, number];
  endInBodyFrameMeters: [number, number, number];
  radiusMeters: number;
}

export interface SpherePart extends SolidPartBase {
  kind: 'sphere';
  centerInBodyFrameMeters: [number, number, number];
  radiusMeters: number;
}

export interface AxesPart {
  kind: 'axes';
  name: string;
  bodyName: string;
  originInBodyFrameMeters: [number, number, number];
  lengthMeters: number;
}

export type VisualPart = BoxPart | CylinderPart | SpherePart | AxesPart;

/** A schematic wheel section; the tape circle lies on the wheel's datum and the flange on its back face. */
export interface WheelSectionSizes {
  widthMeters: number;
  backFaceOffsetMeters: number;
  flangeHeightMeters: number;
  flangeThicknessMeters: number;
  rimDepthMeters: number;
  webThicknessMeters: number;
  hubRadiusMeters: number;
  hubLengthMeters: number;
}

export interface VisualDefinition {
  parts: VisualPart[];
  wheelSection: WheelSectionSizes;
}

function triple(value: unknown, what: string): [number, number, number] {
  if (!Array.isArray(value) || value.length !== 3 || !value.every((entry) => Number.isFinite(entry))) {
    throw new Error(`visual definition: ${what} is not a finite three-vector`);
  }
  return [value[0] as number, value[1] as number, value[2] as number];
}

function positiveNumber(value: unknown, what: string): number {
  if (typeof value !== 'number' || !(value > 0)) {
    throw new Error(`visual definition: ${what} is not a positive number`);
  }
  return value;
}

function nonNegativeNumber(value: unknown, what: string): number {
  if (typeof value !== 'number' || !(value >= 0)) {
    throw new Error(`visual definition: ${what} is not a non-negative number`);
  }
  return value;
}

function appearance(value: unknown, what: string): Appearance {
  if (typeof value !== 'string' || !appearances.has(value)) {
    throw new Error(`visual definition: ${what} is not one of ${[...appearances].join(', ')}`);
  }
  return value as Appearance;
}

/** Reads the parts; which bodies they name is checked against the drawn vehicle, where the body names are known. */
export function parseVisualDefinition(textValue: string): VisualDefinition {
  const json = JSON.parse(textValue) as Record<string, unknown>;
  if (json['length_unit'] !== 'meter') throw new Error('visual definition: length_unit must be meter');
  if (!Array.isArray(json['parts'])) throw new Error('visual definition: parts is not an array');
  const parts = json['parts'].map((entry: Record<string, unknown>, index): VisualPart => {
    const name = typeof entry['name'] === 'string' ? entry['name'] : `part_${index}`;
    const bodyName = entry['body_name'];
    if (typeof bodyName !== 'string') {
      throw new Error(`visual definition: part '${name}' names no body`);
    }
    if (entry['shape'] === 'axes') {
      return {
        kind: 'axes',
        name,
        bodyName,
        originInBodyFrameMeters: triple(entry['origin_in_body_frame_meters'], `${name}.origin`),
        lengthMeters: positiveNumber(entry['length_meters'], `${name}.length`),
      };
    }
    const base = { name, bodyName, appearance: appearance(entry['appearance'], `${name}.appearance`) };
    switch (entry['shape']) {
      case 'box':
        return {
          ...base,
          kind: 'box',
          centerInBodyFrameMeters: triple(entry['center_in_body_frame_meters'], `${name}.center`),
          sizeMeters: triple(entry['size_meters'], `${name}.size`),
          cornerRadiusMeters:
            entry['corner_radius_meters'] === undefined ? 0 : nonNegativeNumber(entry['corner_radius_meters'], `${name}.corner_radius`),
        };
      case 'cylinder':
        return {
          ...base,
          kind: 'cylinder',
          startInBodyFrameMeters: triple(entry['start_in_body_frame_meters'], `${name}.start`),
          endInBodyFrameMeters: triple(entry['end_in_body_frame_meters'], `${name}.end`),
          radiusMeters: positiveNumber(entry['radius_meters'], `${name}.radius`),
        };
      case 'sphere':
        return {
          ...base,
          kind: 'sphere',
          centerInBodyFrameMeters: triple(entry['center_in_body_frame_meters'], `${name}.center`),
          radiusMeters: positiveNumber(entry['radius_meters'], `${name}.radius`),
        };
      default:
        throw new Error(`visual definition: part '${name}' has unknown shape '${String(entry['shape'])}'`);
    }
  });
  const wheel = json['wheel_visual'];
  if (typeof wheel !== 'object' || wheel === null || Array.isArray(wheel)) throw new Error('visual definition: wheel_visual is missing or not an object');
  const length = (key: string): number => positiveNumber((wheel as Record<string, unknown>)[key], `wheel_visual.${key}`);
  const wheelSection: WheelSectionSizes = {
    widthMeters: length('width_meters'),
    backFaceOffsetMeters: length('back_face_offset_meters'),
    flangeHeightMeters: length('flange_height_meters'),
    flangeThicknessMeters: length('flange_thickness_meters'),
    rimDepthMeters: length('rim_depth_meters'),
    webThicknessMeters: length('web_thickness_meters'),
    hubRadiusMeters: length('hub_radius_meters'),
    hubLengthMeters: length('hub_length_meters'),
  };
  if (wheelSection.backFaceOffsetMeters >= wheelSection.widthMeters) {
    throw new Error('visual definition: the wheel back face must lie within the wheel width');
  }
  return { parts, wheelSection };
}
