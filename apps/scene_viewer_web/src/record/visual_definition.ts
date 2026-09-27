// The parametric visual definition copied into the record. Each part is
// stated in its own body's frame; sizes are schematic and carry no physics.

export interface BoxPart {
  kind: 'box';
  name: string;
  bodyName: string;
  centerInBodyFrameMeters: [number, number, number];
  sizeMeters: [number, number, number];
}

export interface CylinderPart {
  kind: 'cylinder';
  name: string;
  bodyName: string;
  startInBodyFrameMeters: [number, number, number];
  endInBodyFrameMeters: [number, number, number];
  radiusMeters: number;
}

export interface AxesPart {
  kind: 'axes';
  name: string;
  bodyName: string;
  originInBodyFrameMeters: [number, number, number];
  lengthMeters: number;
}

export type VisualPart = BoxPart | CylinderPart | AxesPart;

export interface VisualDefinition {
  vehicleName: string;
  parts: VisualPart[];
  wheelVisual: { widthMeters: number; rimMarker: boolean };
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

export function parseVisualDefinition(text: string, bodyNames: ReadonlySet<string>): VisualDefinition {
  const json = JSON.parse(text) as Record<string, unknown>;
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
    switch (part['shape']) {
      case 'box':
        return {
          kind: 'box',
          name,
          bodyName,
          centerInBodyFrameMeters: triple(part['center_in_body_frame_meters'], `${name}.center`),
          sizeMeters: triple(part['size_meters'], `${name}.size`),
        };
      case 'cylinder':
        return {
          kind: 'cylinder',
          name,
          bodyName,
          startInBodyFrameMeters: triple(part['start_in_body_frame_meters'], `${name}.start`),
          endInBodyFrameMeters: triple(part['end_in_body_frame_meters'], `${name}.end`),
          radiusMeters: positiveNumber(part['radius_meters'], `${name}.radius`),
        };
      case 'axes':
        return {
          kind: 'axes',
          name,
          bodyName,
          originInBodyFrameMeters: triple(part['origin_in_body_frame_meters'], `${name}.origin`),
          lengthMeters: positiveNumber(part['length_meters'], `${name}.length`),
        };
      default:
        throw new Error(`visual definition: part '${name}' has unknown shape '${String(part['shape'])}'`);
    }
  });
  const wheelVisual = json['wheel_visual'] as Record<string, unknown> | undefined;
  if (wheelVisual === undefined) {
    throw new Error('visual definition: wheel_visual is missing');
  }
  return {
    vehicleName: typeof json['vehicle_name'] === 'string' ? json['vehicle_name'] : '',
    parts,
    wheelVisual: {
      widthMeters: positiveNumber(wheelVisual['width_meters'], 'wheel_visual.width_meters'),
      rimMarker: wheelVisual['rim_marker'] === true,
    },
  };
}
