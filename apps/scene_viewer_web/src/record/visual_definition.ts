// The parametric visual definition copied into the record. Each part is
// stated in its own body's frame; sizes are schematic and carry no physics.
// An appearance names a display role; the viewer decides how a role looks.
// The display bindings name the bodies the cards, camera and labels treat as
// carbody, bogie frames and carriers; they are read here field by field, and
// resolved against the record in one place, `vehicle_display_bindings.ts`.

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

/** A body given a display role and a name. */
export interface DisplayBodyBinding {
  bodyName: string;
  displayName: BilingualName;
}

/** A bogie frame with the bodies shown as its running gear; wheels follow their carrier and are not listed. */
export interface BogieDisplayBinding extends DisplayBodyBinding {
  memberBodyNames: string[];
}

/** The display name of a carrier body named by the record's wheel placements, in display order. */
export interface CarrierDisplayBinding {
  carrierBodyName: string;
  displayName: BilingualName;
}

export interface DisplayBindings {
  /** null when the definition states that no body plays the carbody. */
  carbody: DisplayBodyBinding | null;
  /** In display order from end 1; may be empty. */
  bogies: BogieDisplayBinding[];
  /** In display order; must name every carrier of the record exactly once. */
  carriers: CarrierDisplayBinding[];
}

export interface VisualDefinition {
  vehicleName: string;
  displayName: BilingualName;
  parts: VisualPart[];
  wheelVisual: WheelVisual;
  displayBindings: DisplayBindings;
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

function object(value: unknown, what: string): Record<string, unknown> {
  if (value === undefined || value === null || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error(`visual definition: ${what} is missing or not an object`);
  }
  return value as Record<string, unknown>;
}

function array(value: unknown, what: string): unknown[] {
  if (!Array.isArray(value)) {
    throw new Error(`visual definition: ${what} is not an array`);
  }
  return value;
}

function bilingual(value: unknown, what: string): BilingualName {
  const entry = object(value, what);
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
  return array(value, 'wheel_visual.labels').map((entry, index) => {
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

function displayBodyBinding(value: unknown, what: string): DisplayBodyBinding {
  const entry = object(value, what);
  return { bodyName: text(entry['body_name'], `${what}.body_name`), displayName: bilingual(entry['display_name'], `${what}.display_name`) };
}

function displayBindings(value: unknown): DisplayBindings {
  const entry = object(value, 'display_bindings');
  if (!('carbody' in entry)) {
    throw new Error('visual definition: display_bindings.carbody must be given, as a binding or null');
  }
  const carbody = entry['carbody'] === null ? null : displayBodyBinding(entry['carbody'], 'display_bindings.carbody');
  const bogies = array(entry['bogies'], 'display_bindings.bogies').map((bogie, index) => {
    const what = `display_bindings.bogies[${index}]`;
    const members = array(object(bogie, what)['member_body_names'], `${what}.member_body_names`).map((name, member) =>
      text(name, `${what}.member_body_names[${member}]`),
    );
    return { ...displayBodyBinding(bogie, what), memberBodyNames: members };
  });
  const carriers = array(entry['carriers'], 'display_bindings.carriers').map((carrier, index) => {
    const what = `display_bindings.carriers[${index}]`;
    const fields = object(carrier, what);
    return {
      carrierBodyName: text(fields['carrier_body_name'], `${what}.carrier_body_name`),
      displayName: bilingual(fields['display_name'], `${what}.display_name`),
    };
  });
  return { carbody, bogies, carriers };
}

export function parseVisualDefinition(textValue: string, bodyNames: ReadonlySet<string>): VisualDefinition {
  const json = JSON.parse(textValue) as Record<string, unknown>;
  const parts: VisualPart[] = array(json['parts'], 'parts').map((entry, index) => {
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
            part['corner_radius_meters'] === undefined ? 0 : nonNegativeNumber(part['corner_radius_meters'], `${name}.corner_radius`),
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
  const wheel = object(json['wheel_visual'], 'wheel_visual');
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
    displayBindings: displayBindings(json['display_bindings']),
  };
}
