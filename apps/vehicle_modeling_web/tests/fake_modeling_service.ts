import type { SendCommand } from '../src/service/command_channel.ts';
import { readAnswer } from '../src/service/protocol.ts';

// An in-memory stand-in for orvd_vehicle_modeling_service that follows its
// protocol: revisions that only grow, undo and redo, a check that refuses any
// negative number, and a preview that only a passing check replaces. Kinds and
// names are made up on purpose: the interface must not depend on them.

export interface FakeProperty {
  name: string;
  value_kind: string;
  editable: boolean;
  value: unknown;
}

export interface FakeElement {
  kind: string;
  name: string;
  properties: FakeProperty[];
}

export interface FakeVehicle {
  vehicleName: string;
  elements: FakeElement[];
  /** The preview lists, without project_revision. */
  drawings: Record<string, unknown>;
}

function negativeNumbers(value: unknown): boolean {
  return typeof value === 'number' ? value < 0 : Array.isArray(value) && value.some(negativeNumbers);
}

export class FakeModelingService {
  readonly log: { command: string; commandArguments: Record<string, unknown> }[] = [];
  private readonly vehicles: ReadonlyMap<string, FakeVehicle>;
  private vehicle: FakeVehicle | null = null;
  private elements: FakeElement[] = [];
  private revision = 0;
  private undoSteps: FakeElement[][] = [];
  private redoSteps: FakeElement[][] = [];
  private preview: Record<string, unknown> | null = null;

  /** Vehicles by the vehicle_definition_path that opens them. */
  constructor(vehicles: Record<string, FakeVehicle>) {
    this.vehicles = new Map(Object.entries(vehicles));
  }

  readonly send: SendCommand = async (command, commandArguments = {}) => {
    this.log.push({ command, commandArguments: { ...commandArguments } });
    await Promise.resolve();
    return readAnswer(this.answer(command, commandArguments));
  };

  private accepted(result: unknown): unknown {
    return { ok: true, revision: this.revision, can_undo: this.undoSteps.length > 0, can_redo: this.redoSteps.length > 0, result };
  }

  private refused(message: string, element: unknown = null, property: unknown = null): unknown {
    return { ok: false, error: { element, property, message } };
  }

  private snapshot(): FakeElement[] {
    return structuredClone(this.elements);
  }

  private answer(command: string, commandArguments: Record<string, unknown>): unknown {
    if (command === 'open_vehicle') {
      const opened = this.vehicles.get(String(commandArguments['vehicle_definition_path']));
      if (opened === undefined) return this.refused(`cannot read '${String(commandArguments['vehicle_definition_path'])}'`);
      this.vehicle = opened;
      this.elements = structuredClone(opened.elements);
      this.revision = 0;
      this.undoSteps = [];
      this.redoSteps = [];
      this.preview = null;
      return this.accepted({});
    }
    if (this.vehicle === null) return this.refused('no vehicle is open; send open_vehicle first');
    const target = commandArguments['element'] as { kind: string; name: string } | undefined;
    const element = this.elements.find((candidate) => candidate.kind === target?.kind && candidate.name === target?.name);
    switch (command) {
      case 'get_outline':
        return this.accepted({
          vehicle_name: this.vehicle.vehicleName,
          elements: this.elements.map(({ kind, name }) => ({ kind, name })),
          element_groups: [],
          unplaced_free_bodies: [],
          contact_record_present: true,
        });
      case 'get_properties':
        return element === undefined ? this.refused(`no element named '${target?.name}'`, target) : this.accepted({ properties: element.properties });
      case 'set_property': {
        const name = String(commandArguments['property']);
        const property = element?.properties.find((candidate) => candidate.name === name);
        if (element === undefined || property === undefined || !property.editable) return this.refused(`'${name}' cannot be set`, target, name);
        const value = commandArguments['value'];
        if ((property.value_kind === 'number') !== (typeof value === 'number')) return this.refused(`'${name}' holds a ${property.value_kind}`, target, name);
        this.undoSteps.push(this.snapshot());
        this.redoSteps = [];
        property.value = value;
        this.revision += 1;
        return this.accepted({});
      }
      case 'undo':
      case 'redo': {
        const [from, to] = command === 'undo' ? [this.undoSteps, this.redoSteps] : [this.redoSteps, this.undoSteps];
        const restored = from.pop();
        if (restored === undefined) return this.refused(`there is no step to ${command}`);
        to.push(this.snapshot());
        this.elements = restored;
        this.revision += 1;
        return this.accepted({});
      }
      case 'check': {
        const refusals = this.elements.flatMap((each) =>
          each.properties.filter((property) => negativeNumbers(property.value)).map((property) => `${each.name}: ${property.name} is negative`),
        );
        if (refusals.length === 0) this.preview = { project_revision: this.revision, ...this.vehicle.drawings };
        return this.accepted({ passed: refusals.length === 0, refusals, information: ['a fake check'], coverage_statements: ['It covers nothing.'] });
      }
      case 'get_preview':
        return this.accepted({ preview: this.preview });
      default:
        return this.refused(`'${command}' is not a command`);
    }
  }
}

const identity = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
];

/** A vehicle of made-up element kinds; one body shares its name with an element of another kind. */
export function fakeVehicle(): FakeVehicle {
  return {
    vehicleName: 'FAKE',
    elements: [
      {
        kind: 'kind_body',
        name: 'body_a',
        properties: [
          { name: 'name', value_kind: 'text', editable: false, value: 'body_a' },
          { name: 'size_quantity', value_kind: 'number', editable: true, value: 10 },
          { name: 'offset_quantity', value_kind: 'vector3', editable: true, value: [0, 0, 0] },
        ],
      },
      { kind: 'kind_body', name: 'body_b', properties: [{ name: 'size_quantity', value_kind: 'number', editable: true, value: 4 }] },
      {
        kind: 'kind_spring',
        name: 'spring_a',
        properties: [
          { name: 'rate_quantity', value_kind: 'vector3', editable: true, value: [1, 2, 3] },
          { name: 'curve_quantity', value_kind: 'saturated_piecewise_linear_damper_curve', editable: false, value: [{ speed: 0, push: 0 }, { speed: 1, push: 5 }] },
        ],
      },
      { kind: 'kind_carrier', name: 'body_b', properties: [{ name: 'carrier_quantity', value_kind: 'text', editable: false, value: 'body_b' }] },
      { kind: 'kind_interface', name: 'body_b.left', properties: [{ name: 'side', value_kind: 'wheel_side', editable: false, value: 'left' }] },
      { kind: 'kind_interface', name: 'body_b.right', properties: [{ name: 'side', value_kind: 'wheel_side', editable: false, value: 'right' }] },
    ],
    drawings: {
      rigid_bodies: [
        { element: { kind: 'kind_body', name: 'body_a' }, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [0, 0, -1] },
        { element: { kind: 'kind_body', name: 'body_b' }, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [5, 0, -0.5] },
      ],
      fixed_frames: [],
      revolute_joints: [],
      ball_rpy_joints: [],
      weld_joints: [],
      force_elements: [
        { element: { kind: 'kind_spring', name: 'spring_a' }, reference_end_in_drawing_frame_meters: [0, 0, -1], opposite_end_in_drawing_frame_meters: [5, 0, -0.5] },
      ],
      wheels: [
        {
          element: { kind: 'kind_interface', name: 'body_b.left' },
          wheel_body: { kind: 'kind_body', name: 'body_b' },
          datum_in_drawing_frame_meters: [5, -0.75, -0.43],
          outward_spin_axis_in_drawing_frame: [0, -1, 0],
          nominal_rolling_radius_meters: 0.43,
        },
        {
          element: { kind: 'kind_interface', name: 'body_b.right' },
          wheel_body: { kind: 'kind_body', name: 'body_b' },
          datum_in_drawing_frame_meters: [5, 0.75, -0.43],
          outward_spin_axis_in_drawing_frame: [0, 1, 0],
          nominal_rolling_radius_meters: 0.43,
        },
      ],
      not_drawable_elements: [],
    },
  };
}
