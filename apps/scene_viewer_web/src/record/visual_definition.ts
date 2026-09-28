// The parametric visual definition copied into the record. Each part is
// stated in its own body's frame; sizes are schematic and carry no physics.
// An appearance names a display role; the viewer decides how a role looks.

export type Appearance = 'shell' | 'glass' | 'floor' | 'structure' | 'accent' | 'dark';

const appearances: ReadonlySet<string> = new Set(['shell', 'glass', 'floor', 'structure', 'accent', 'dark']);

export interface BilingualName {
  en: string;
  zh: string;
}

export interface PartLabel extends BilingualName {
  /** Where the leader line starts, in the part's body frame; null means the part's own centre. */
  anchorInBodyFrameMeters: [number, number, number] | null;
}

interface SolidPartBase {
  name: string;
  bodyName: string;
  appearance: Appearance;
  label: PartLabel | null;
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

/** A schematic wheel section; the tape circle lies on the placement datum. */
export interface WheelVisual {
  widthMeters: number;
  backFaceOffsetMeters: number;
  flangeHeightMeters: number;
  flangeThicknessMeters: number;
  rimDepthMeters: number;
  webThicknessMeters: number;
  hubRadiusMeters: number;
  hubLengthMeters: number;
  rimMarker: boolean;
  /** Wheels that carry a label, by wheel-rail interface name. */
  labels: WheelLabel[];
}

export interface WheelLabel extends BilingualName {
  interfaceName: string;
}

export interface VisualDefinition {
  vehicleName: string;
  displayName: BilingualName;
  parts: VisualPart[];
  wheelVisual: WheelVisual;
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

function text(value: unknown, what: string): string {
  if (typeof value !== 'string' || value.trim() === '') {
    throw new Error(`visual definition: ${what} is not a non-empty string`);
  }
  return value;
}

function bilingual(value: unknown, what: string): BilingualName {
  const entry = value as Record<string, unknown> | undefined;
  if (entry === undefined || entry === null || typeof entry !== 'object') {
    throw new Error(`visual definition: ${what} is missing`);
  }
  return { en: text(entry['en'], `${what}.en`), zh: text(entry['zh'], `${what}.zh`) };
}

function label(value: unknown, what: string): PartLabel | null {
  if (value === undefined) {
    return null;
  }
  const names = bilingual(value, what);
  const anchor = (value as Record<string, unknown>)['anchor_in_body_frame_meters'];
  return { ...names, anchorInBodyFrameMeters: anchor === undefined ? null : triple(anchor, `${what}.anchor`) };
}

function wheelLabels(value: unknown): WheelLabel[] {
  if (value === undefined) {
    return [];
  }
  if (!Array.isArray(value)) {
    throw new Error('visual definition: wheel_visual.labels is not an array');
  }
  return value.map((entry, index) => {
    const what = `wheel_visual.labels[${index}]`;
    const names = bilingual(entry, what);
    return { ...names, interfaceName: text((entry as Record<string, unknown>)['interface_name'], `${what}.interface_name`) };
  });
}

function appearance(value: unknown, what: string): Appearance {
  if (typeof value !== 'string' || !appearances.has(value)) {
    throw new Error(`visual definition: ${what} is not one of ${[...appearances].join(', ')}`);
  }
  return value as Appearance;
}

export function parseVisualDefinition(textValue: string, bodyNames: ReadonlySet<string>): VisualDefinition {
  const json = JSON.parse(textValue) as Record<string, unknown>;
  const partsJson = json['parts'];
  if (!Array.isArray(partsJson)) {
    throw new Error('visual definition: parts is not an array');
  }
  const parts: VisualPart[] = partsJson.map((entry, index) => {
    const part = entry as Record<string, unknown>;
    const name = typeof part['name'] === 'string' ? part['name'] : `part_${index}`;
    const bodyName = part['body_name'];
    if (typeof bodyName !== 'string' || !bodyNames.has(bodyName)) {
      throw new Error(`visual definition: part '${name}' binds to unknown body '${String(bodyName)}'`);
    }
    if (part['shape'] === 'axes') {
      return {
        kind: 'axes',
        name,
        bodyName,
        originInBodyFrameMeters: triple(part['origin_in_body_frame_meters'], `${name}.origin`),
        lengthMeters: positiveNumber(part['length_meters'], `${name}.length`),
      };
    }
    const base = {
      name,
      bodyName,
      appearance: appearance(part['appearance'], `${name}.appearance`),
      label: label(part['label'], `${name}.label`),
    };
    switch (part['shape']) {
      case 'box':
        return {
          ...base,
          kind: 'box',
          centerInBodyFrameMeters: triple(part['center_in_body_frame_meters'], `${name}.center`),
          sizeMeters: triple(part['size_meters'], `${name}.size`),
          cornerRadiusMeters:
            part['corner_radius_meters'] === undefined
              ? 0
              : nonNegativeNumber(part['corner_radius_meters'], `${name}.corner_radius`),
        };
      case 'cylinder':
        return {
          ...base,
          kind: 'cylinder',
          startInBodyFrameMeters: triple(part['start_in_body_frame_meters'], `${name}.start`),
          endInBodyFrameMeters: triple(part['end_in_body_frame_meters'], `${name}.end`),
          radiusMeters: positiveNumber(part['radius_meters'], `${name}.radius`),
        };
      case 'sphere':
        return {
          ...base,
          kind: 'sphere',
          centerInBodyFrameMeters: triple(part['center_in_body_frame_meters'], `${name}.center`),
          radiusMeters: positiveNumber(part['radius_meters'], `${name}.radius`),
        };
      default:
        throw new Error(`visual definition: part '${name}' has unknown shape '${String(part['shape'])}'`);
    }
  });
  const wheel = json['wheel_visual'] as Record<string, unknown> | undefined;
  if (wheel === undefined) {
    throw new Error('visual definition: wheel_visual is missing');
  }
  const wheelVisual: WheelVisual = {
    widthMeters: positiveNumber(wheel['width_meters'], 'wheel_visual.width_meters'),
    backFaceOffsetMeters: positiveNumber(wheel['back_face_offset_meters'], 'wheel_visual.back_face_offset_meters'),
    flangeHeightMeters: positiveNumber(wheel['flange_height_meters'], 'wheel_visual.flange_height_meters'),
    flangeThicknessMeters: positiveNumber(wheel['flange_thickness_meters'], 'wheel_visual.flange_thickness_meters'),
    rimDepthMeters: positiveNumber(wheel['rim_depth_meters'], 'wheel_visual.rim_depth_meters'),
    webThicknessMeters: positiveNumber(wheel['web_thickness_meters'], 'wheel_visual.web_thickness_meters'),
    hubRadiusMeters: positiveNumber(wheel['hub_radius_meters'], 'wheel_visual.hub_radius_meters'),
    hubLengthMeters: positiveNumber(wheel['hub_length_meters'], 'wheel_visual.hub_length_meters'),
    rimMarker: wheel['rim_marker'] === true,
    labels: wheelLabels(wheel['labels']),
  };
  if (wheelVisual.backFaceOffsetMeters >= wheelVisual.widthMeters) {
    throw new Error('visual definition: the wheel back face must lie within the wheel width');
  }
  return {
    vehicleName: text(json['vehicle_name'], 'vehicle_name'),
    displayName: bilingual(json['display_name'], 'display_name'),
    parts,
    wheelVisual,
  };
}
