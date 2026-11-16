import type { SendCommand } from '../src/service/modeling_server_command_channel.ts';
import { readAnswer } from '../src/service/vehicle_modeling_service_protocol.ts';

// An in-memory stand-in for orvd_vehicle_modeling_service that follows its
// protocol: kinds described with their properties, revisions that only grow,
// undo and redo, a check that refuses any negative number, and a preview that
// only a passing check replaces, of the elements that still exist. Edits are
// refused as the real service refuses them: a missing target, a name already
// taken in its kind, a reference to nothing of a kind it may name, a required
// reference left out. Removing an element removes everything that refers to
// it, and a rename updates every reference to the renamed element. Kinds and
// names are made up on purpose: the interface must not depend on them.

export interface FakePropertyDescription {
  name: string;
  value_kind: string;
  editable: boolean;
  referenced_kinds?: string[];
  optional?: boolean;
  choices?: string[];
  row_fields?: string[];
}

export interface FakeKind {
  kind: string;
  properties: FakePropertyDescription[];
}

export interface FakeElement {
  kind: string;
  name: string;
  /** Values by property name, apart from the element's own name. */
  values: Record<string, unknown>;
  /** Why the element cannot be removed, where it cannot. */
  removalRefused?: string;
}

export interface FakeVehicle {
  vehicleName: string;
  kinds: FakeKind[];
  elements: FakeElement[];
  /** The preview lists, without project_revision. */
  drawings: Record<string, unknown>;
}

interface Element {
  kind: string;
  name: string;
}

type Refused = { refused: { element: unknown; property: unknown; message: string } };

function negativeNumbers(value: unknown): boolean {
  if (typeof value === 'number') return value < 0;
  if (Array.isArray(value)) return value.some(negativeNumbers);
  return typeof value === 'object' && value !== null && Object.values(value).some(negativeNumbers);
}

const identity = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
];

/** The value a new element holds for a property it is not given. */
function defaultValue(property: FakePropertyDescription): unknown {
  switch (property.value_kind) {
    case 'number':
      return 0;
    case 'vector3':
      return [0, 0, 0];
    case 'boolean':
      return false;
    case 'rotation':
      return identity;
    case 'token':
      return property.choices![0];
    case 'saturated_piecewise_linear_damper_curve':
      return [];
    default:
      return null;
  }
}

export class FakeVehicleModelingService {
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
    this.log.push({ command, commandArguments: structuredClone({ ...commandArguments }) });
    await Promise.resolve();
    return readAnswer(this.answer(command, commandArguments));
  };

  private accepted(result: unknown): unknown {
    return { ok: true, revision: this.revision, can_undo: this.undoSteps.length > 0, can_redo: this.redoSteps.length > 0, result };
  }

  private refused(message: string, element: unknown = null, property: unknown = null): Refused {
    return { refused: { element, property, message } };
  }

  private kindOf(kind: string): FakeKind | undefined {
    return this.vehicle?.kinds.find((each) => each.kind === kind);
  }

  private find(element: Element | undefined): FakeElement | undefined {
    return this.elements.find((candidate) => candidate.kind === element?.kind && candidate.name === element?.name);
  }

  /** The elements kind by kind in the described order, each kind in the order its elements were added. */
  private outline(): FakeElement[] {
    const order = this.vehicle!.kinds.map((each) => each.kind);
    return [...this.elements].sort((a, b) => order.indexOf(a.kind) - order.indexOf(b.kind));
  }

  /** Whether the value fits the property, or why not. */
  private judge(property: FakePropertyDescription, value: unknown): string | null {
    switch (property.value_kind) {
      case 'number':
        return typeof value === 'number' ? null : `'${property.name}' holds a number`;
      case 'vector3':
        return Array.isArray(value) && value.length === 3 && value.every((entry) => typeof entry === 'number') ? null : `'${property.name}' holds three numbers`;
      case 'boolean':
        return typeof value === 'boolean' ? null : `'${property.name}' holds true or false`;
      case 'element_name':
        return typeof value === 'string' && value !== '' ? null : 'a name is not empty';
      case 'token':
        return property.choices!.includes(value as string) ? null : `'${String(value)}' is not one of ${property.choices!.join(', ')}`;
      case 'element_reference':
        if (value === null) return property.optional === true ? null : `'${property.name}' must name an element`;
        return this.elements.some((each) => property.referenced_kinds!.includes(each.kind) && each.name === value)
          ? null
          : `no ${property.referenced_kinds!.join(' or ')} is named '${String(value)}'`;
      case 'saturated_piecewise_linear_damper_curve':
        return Array.isArray(value) && value.every((row) => property.row_fields!.every((rowField) => typeof (row as Record<string, unknown>)[rowField] === 'number'))
          ? null
          : `'${property.name}' holds rows of ${property.row_fields!.join(' and ')}`;
      default:
        return `'${property.name}' cannot be set`;
    }
  }

  private nameTaken(kind: string, name: string): boolean {
    return this.elements.some((each) => each.kind === kind && each.name === name);
  }

  /** The element and everything that refers to it, directly or through another, in outline order. */
  private removalOf(element: FakeElement): FakeElement[] {
    const removed = new Set<FakeElement>([element]);
    let grown = true;
    while (grown) {
      grown = false;
      for (const candidate of this.elements) {
        if (removed.has(candidate)) continue;
        const refers = this.kindOf(candidate.kind)!.properties.some(
          (property) =>
            property.value_kind === 'element_reference' &&
            [...removed].some((target) => property.referenced_kinds!.includes(target.kind) && candidate.values[property.name] === target.name),
        );
        if (refers) {
          removed.add(candidate);
          grown = true;
        }
      }
    }
    return this.outline().filter((each) => removed.has(each));
  }

  private edit(change: () => void): void {
    this.undoSteps.push(structuredClone(this.elements));
    this.redoSteps = [];
    change();
    this.revision += 1;
  }

  private answer(command: string, commandArguments: Record<string, unknown>): unknown {
    const outcome = this.outcome(command, commandArguments);
    if (typeof outcome === 'object' && outcome !== null && 'refused' in outcome) {
      return { ok: false, error: (outcome as Refused).refused };
    }
    return outcome;
  }

  private outcome(command: string, commandArguments: Record<string, unknown>): unknown {
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
    const target = commandArguments['element'] as Element | undefined;
    const element = this.find(target);
    switch (command) {
      case 'describe_element_kinds':
        return this.accepted({ element_kinds: this.vehicle.kinds });
      case 'get_outline':
        return this.accepted({
          vehicle_name: this.vehicle.vehicleName,
          elements: this.outline().map(({ kind, name }) => ({ kind, name })),
          element_groups: [],
          unplaced_free_bodies: [],
          contact_record_present: true,
        });
      case 'get_properties': {
        if (element === undefined) return this.refused(`no element named '${target?.name}'`, target);
        const properties = this.kindOf(element.kind)!.properties.map((property) => {
          const value = property.value_kind === 'element_name' ? element.name : (element.values[property.name] ?? null);
          return { ...property, editable: property.editable && (value !== null || property.value_kind === 'element_reference'), value };
        });
        return this.accepted({ properties });
      }
      case 'set_property': {
        const name = String(commandArguments['property']);
        const property = element === undefined ? undefined : this.kindOf(element.kind)!.properties.find((candidate) => candidate.name === name);
        if (element === undefined || property === undefined || !property.editable) return this.refused(`'${name}' cannot be set`, target, name);
        const value = commandArguments['value'];
        const wrong = this.judge(property, value);
        if (wrong !== null) return this.refused(wrong, target, name);
        if (property.value_kind === 'element_name' && value !== element.name && this.nameTaken(element.kind, value as string)) {
          return this.refused(`the name '${String(value)}' is taken`, target, name);
        }
        this.edit(() => {
          const edited = this.find(element)!;
          if (property.value_kind !== 'element_name') {
            edited.values[name] = value;
            return;
          }
          for (const other of this.elements) {
            for (const reference of this.kindOf(other.kind)!.properties) {
              if (reference.value_kind === 'element_reference' && reference.referenced_kinds!.includes(edited.kind) && other.values[reference.name] === edited.name) {
                other.values[reference.name] = value;
              }
            }
          }
          edited.name = value as string;
        });
        return this.accepted({ element: { kind: element.kind, name: property.value_kind === 'element_name' ? value : element.name } });
      }
      case 'add_element': {
        const kind = this.kindOf(String(commandArguments['kind']));
        const name = commandArguments['name'];
        const created = { kind: commandArguments['kind'], name };
        if (kind === undefined) return this.refused(`'${String(commandArguments['kind'])}' is not an element kind`, created);
        const nameProperty = kind.properties.find((property) => property.value_kind === 'element_name')!.name;
        if (typeof name !== 'string' || name === '') return this.refused('a name is not empty', created, nameProperty);
        if (this.nameTaken(kind.kind, name)) return this.refused(`the name '${name}' is taken`, created, nameProperty);
        const given = commandArguments['properties'] as Record<string, unknown>;
        for (const [propertyName, value] of Object.entries(given)) {
          const property = kind.properties.find((candidate) => candidate.name === propertyName);
          if (property === undefined || !property.editable) return this.refused(`'${propertyName}' cannot be given`, created, propertyName);
          const wrong = this.judge(property, value);
          if (wrong !== null) return this.refused(wrong, created, propertyName);
        }
        const missing = kind.properties.find((property) => property.value_kind === 'element_reference' && property.optional !== true && !(property.name in given));
        if (missing !== undefined) return this.refused(`'${missing.name}' must name an element`, created, missing.name);
        const values = Object.fromEntries(kind.properties.filter((property) => property.value_kind !== 'element_name').map((property) => [property.name, defaultValue(property)]));
        this.edit(() => this.elements.push({ kind: kind.kind, name, values: { ...values, ...structuredClone(given) } }));
        return this.accepted({ element: created });
      }
      case 'get_removal_consequences':
      case 'remove_element': {
        if (element === undefined) return this.refused(`no element named '${target?.name}'`, target);
        if (element.removalRefused !== undefined) return this.refused(element.removalRefused, target);
        const removed = this.removalOf(element);
        if (command === 'remove_element') this.edit(() => (this.elements = this.elements.filter((each) => !removed.some((gone) => gone.kind === each.kind && gone.name === each.name))));
        return this.accepted({ removed_elements: removed.map(({ kind, name }) => ({ kind, name })) });
      }
      case 'undo':
      case 'redo': {
        const [from, to] = command === 'undo' ? [this.undoSteps, this.redoSteps] : [this.redoSteps, this.undoSteps];
        const restored = from.pop();
        if (restored === undefined) return this.refused(`there is no step to ${command}`);
        to.push(structuredClone(this.elements));
        this.elements = restored;
        this.revision += 1;
        return this.accepted({});
      }
      case 'check': {
        const refusals = this.elements.flatMap((each) =>
          Object.entries(each.values)
            .filter(([, value]) => negativeNumbers(value))
            .map(([property]) => `${each.name}: ${property} is negative`),
        );
        if (refusals.length === 0) this.preview = { project_revision: this.revision, ...this.drawingsOfElements() };
        return this.accepted({ passed: refusals.length === 0, refusals, information: ['a fake check'], coverage_statements: ['It covers nothing.'] });
      }
      case 'get_preview':
        return this.accepted({ preview: this.preview });
      default:
        return this.refused(`'${command}' is not a command`);
    }
  }

  /** The vehicle's drawings of the elements that exist now. */
  private drawingsOfElements(): Record<string, unknown> {
    const exists = (drawn: unknown): boolean => this.find(drawn as Element) !== undefined;
    return Object.fromEntries(
      Object.entries(this.vehicle!.drawings).map(([list, drawings]) => [
        list,
        (drawings as Record<string, unknown>[]).filter((drawing) => exists(list === 'not_drawable_elements' ? drawing : drawing['element'])),
      ]),
    );
  }
}

const reference = (name: string, referencedKinds: string[], optional = false): FakePropertyDescription => ({
  name,
  value_kind: 'element_reference',
  editable: true,
  referenced_kinds: referencedKinds,
  optional,
});

const ownName: FakePropertyDescription = { name: 'name', value_kind: 'element_name', editable: true };

/**
 * A vehicle of made-up element kinds, one of them without elements. One body
 * shares its name with a carrier, an element of another kind; a frame and a
 * spring refer to the bodies, and the carrier and its interfaces to the body
 * they turn with.
 */
export function fakeVehicle(): FakeVehicle {
  return {
    vehicleName: 'FAKE',
    kinds: [
      {
        kind: 'kind_body',
        properties: [
          ownName,
          { name: 'free_flag', value_kind: 'boolean', editable: true },
          { name: 'size_quantity', value_kind: 'number', editable: true },
          { name: 'offset_quantity', value_kind: 'vector3', editable: true },
          { name: 'turn_quantity', value_kind: 'rotation', editable: false },
        ],
      },
      { kind: 'kind_frame', properties: [ownName, reference('body_ref', ['kind_body'])] },
      { kind: 'kind_hinge', properties: [ownName, reference('end_a', ['kind_frame', 'kind_body']), reference('end_b', ['kind_frame', 'kind_body'])] },
      {
        kind: 'kind_spring',
        properties: [
          ownName,
          reference('end_a', ['kind_frame', 'kind_body']),
          reference('end_b', ['kind_frame', 'kind_body']),
          { name: 'rate_quantity', value_kind: 'vector3', editable: true },
          { name: 'axis_choice', value_kind: 'token', editable: true, choices: ['axis_x', 'axis_y', 'axis_z'] },
          { name: 'curve_quantity', value_kind: 'saturated_piecewise_linear_damper_curve', editable: true, row_fields: ['speed', 'push'] },
        ],
      },
      { kind: 'kind_carrier', properties: [ownName, reference('carrier_body', ['kind_body'])] },
      {
        kind: 'kind_interface',
        properties: [
          ownName,
          reference('carrier_ref', ['kind_carrier']),
          reference('wheel_body_ref', ['kind_body']),
          reference('hinge_ref', ['kind_hinge'], true),
          { name: 'side', value_kind: 'token', editable: true, choices: ['left', 'right'] },
        ],
      },
    ],
    elements: [
      {
        kind: 'kind_body',
        name: 'body_a',
        values: { free_flag: true, size_quantity: 10, offset_quantity: [0, 0, 0], turn_quantity: identity },
        removalRefused: 'body_a is what every other place is measured from, so it cannot be removed',
      },
      { kind: 'kind_body', name: 'body_b', values: { free_flag: false, size_quantity: 4, offset_quantity: [0, 0, 0], turn_quantity: identity } },
      { kind: 'kind_frame', name: 'frame_a', values: { body_ref: 'body_a' } },
      {
        kind: 'kind_spring',
        name: 'spring_a',
        values: {
          end_a: 'frame_a',
          end_b: 'body_b',
          rate_quantity: [1, 2, 3],
          axis_choice: 'axis_x',
          curve_quantity: [
            { speed: 0, push: 0 },
            { speed: 1, push: 5 },
          ],
        },
      },
      { kind: 'kind_carrier', name: 'body_b', values: { carrier_body: 'body_b' } },
      { kind: 'kind_interface', name: 'body_b.left', values: { carrier_ref: 'body_b', wheel_body_ref: 'body_b', hinge_ref: null, side: 'left' } },
      { kind: 'kind_interface', name: 'body_b.right', values: { carrier_ref: 'body_b', wheel_body_ref: 'body_b', hinge_ref: null, side: 'right' } },
    ],
    drawings: {
      rigid_bodies: [
        { element: { kind: 'kind_body', name: 'body_a' }, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [0, 0, -1], visual_definition_body_name: 'body_a' },
        { element: { kind: 'kind_body', name: 'body_b' }, rotation_drawing_from_body: identity, origin_in_drawing_frame_meters: [5, 0, -0.5], visual_definition_body_name: 'body_b' },
      ],
      fixed_frames: [{ element: { kind: 'kind_frame', name: 'frame_a' }, rotation_drawing_from_frame: identity, origin_in_drawing_frame_meters: [0, 0, -1] }],
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
