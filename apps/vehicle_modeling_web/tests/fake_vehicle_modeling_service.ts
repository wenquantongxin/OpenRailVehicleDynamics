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
//
// Projects are kept in memory by name, in the one folder the fake is given:
// another folder cannot be read, a folder made unwritable refuses every save,
// and a name is refused as the service refuses it. Saving as renames the
// vehicle in the whole undo and redo history, with one revision and no undo
// step: the name is the document's identity, not an edit. Unsaved changes
// follow undo and redo: every edit makes a project state of its own, and the
// session has unsaved changes while its state is not the one last saved or
// opened.
// get_session and list_vehicle_projects are answered while nothing is open,
// with the session's fields null.

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

/** What an edit changes and undo restores, with the state's own number: no two edits make the same state. */
interface ProjectState {
  vehicleName: string;
  elements: FakeElement[];
  stateNumber: number;
}

/** A project as saved: the vehicle as it was then, and the visual definition document it carries. */
export interface FakeSavedProject {
  vehicle: FakeVehicle;
  visualDefinition: unknown;
}

type Refused = { refused: { element: unknown; property: unknown; message: string } };

/** The service's rule for a project name, which the interface never applies: it shows the refusal. */
const projectName = /^[A-Za-z0-9._-]+$/;

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
  /** The projects of the folder, by name. */
  readonly savedProjects = new Map<string, FakeSavedProject>();
  /** Whether saving into the folder succeeds; a folder that cannot be written refuses every save. */
  projectsDirectoryWritable = true;
  private readonly vehicles: ReadonlyMap<string, FakeVehicle>;
  private readonly visualDefinitions: ReadonlyMap<string, unknown>;
  private readonly projectsDirectory: string;
  /** The open vehicle's kinds and drawings; null while nothing is open. */
  private vehicle: FakeVehicle | null = null;
  private vehicleName = '';
  private elements: FakeElement[] = [];
  private stateNumber = 0;
  private stateNumbersGiven = 0;
  /** The state last saved or, before any save, opened. */
  private savedStateNumber = 0;
  private visualDefinition: unknown = null;
  private projectFile: { directory: string; name: string } | null = null;
  private revision = 0;
  private undoSteps: ProjectState[] = [];
  private redoSteps: ProjectState[] = [];
  private preview: Record<string, unknown> | null = null;

  /**
   * Vehicles by the vehicle_definition_path that opens them, visual definition
   * documents by the visual_definition_path that reads them, and the one
   * folder of projects.
   */
  constructor(vehicles: Record<string, FakeVehicle>, options: { visualDefinitions?: Record<string, unknown>; projectsDirectory?: string } = {}) {
    this.vehicles = new Map(Object.entries(vehicles));
    this.visualDefinitions = new Map(Object.entries(options.visualDefinitions ?? {}));
    this.projectsDirectory = options.projectsDirectory ?? '/fake/projects';
  }

  readonly send: SendCommand = async (command, commandArguments = {}) => {
    this.log.push({ command, commandArguments: structuredClone({ ...commandArguments }) });
    await Promise.resolve();
    return readAnswer(this.answer(command, commandArguments));
  };

  private accepted(result: unknown): unknown {
    if (this.vehicle === null) {
      return { ok: true, revision: null, can_undo: null, can_redo: null, has_unsaved_changes: null, result };
    }
    return {
      ok: true,
      revision: this.revision,
      can_undo: this.undoSteps.length > 0,
      can_redo: this.redoSteps.length > 0,
      has_unsaved_changes: this.stateNumber !== this.savedStateNumber,
      result,
    };
  }

  /** Makes the given project the open one: revision 0, no history, no preview, nothing unsaved. */
  private openProject(vehicle: FakeVehicle, visualDefinition: unknown, projectFile: { directory: string; name: string } | null): void {
    this.vehicle = structuredClone(vehicle);
    this.vehicleName = vehicle.vehicleName;
    this.elements = structuredClone(vehicle.elements);
    this.visualDefinition = structuredClone(visualDefinition);
    this.projectFile = projectFile;
    this.stateNumber = ++this.stateNumbersGiven;
    this.savedStateNumber = this.stateNumber;
    this.revision = 0;
    this.undoSteps = [];
    this.redoSteps = [];
    this.preview = null;
  }

  /** Writes the open project as the named one; the caller has checked the folder and the name. */
  private saveProject(name: string, vehicleName: string): void {
    const vehicle = { ...this.vehicle!, vehicleName, elements: this.elements };
    this.savedProjects.set(name, structuredClone({ vehicle, visualDefinition: this.visualDefinition }));
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

  private projectState(): ProjectState {
    return structuredClone({ vehicleName: this.vehicleName, elements: this.elements, stateNumber: this.stateNumber });
  }

  private edit(change: () => void): void {
    this.undoSteps.push(this.projectState());
    this.redoSteps = [];
    change();
    this.stateNumber = ++this.stateNumbersGiven;
    this.revision += 1;
  }

  private answer(command: string, commandArguments: Record<string, unknown>): unknown {
    const outcome = this.outcome(command, commandArguments);
    if (typeof outcome === 'object' && outcome !== null && 'refused' in outcome) {
      return { ok: false, error: (outcome as Refused).refused };
    }
    return outcome;
  }

  /** Why the folder cannot be read, or null. */
  private unreadableDirectory(directory: unknown): string | null {
    return directory === this.projectsDirectory ? null : `cannot read the folder '${String(directory)}'`;
  }

  /** The open-anywhere commands: the session, the folder, and the two opens. Null for every other command. */
  private sessionOutcome(command: string, commandArguments: Record<string, unknown>): unknown {
    switch (command) {
      case 'get_session':
        return this.accepted(this.vehicle === null ? { open: false } : { open: true, vehicle_project_file: this.projectFile });
      case 'list_vehicle_projects': {
        const unreadable = this.unreadableDirectory(commandArguments['directory']);
        return unreadable !== null ? this.refused(unreadable) : this.accepted({ vehicle_projects: [...this.savedProjects.keys()].sort() });
      }
      case 'open_vehicle': {
        const path = String(commandArguments['vehicle_definition_path']);
        const opened = this.vehicles.get(path);
        if (opened === undefined) return this.refused(`cannot read '${path}'`);
        if (!('visual_definition_path' in commandArguments)) return this.refused('visual_definition_path is missing');
        const visualPath = commandArguments['visual_definition_path'];
        const visualDefinition = visualPath === null ? null : this.visualDefinitions.get(String(visualPath));
        if (visualDefinition === undefined) return this.refused(`cannot read '${String(visualPath)}'`);
        this.openProject(opened, visualDefinition, null);
        return this.accepted({});
      }
      case 'open_vehicle_project': {
        const directory = String(commandArguments['directory']);
        const name = String(commandArguments['name']);
        const unreadable = this.unreadableDirectory(directory);
        const saved = this.savedProjects.get(name);
        if (unreadable !== null) return this.refused(unreadable);
        if (saved === undefined) return this.refused(`the folder holds no project named '${name}'`);
        this.openProject(saved.vehicle, saved.visualDefinition, { directory, name });
        return this.accepted({});
      }
      default:
        return null;
    }
  }

  private outcome(command: string, commandArguments: Record<string, unknown>): unknown {
    const sessionOutcome = this.sessionOutcome(command, commandArguments);
    if (sessionOutcome !== null) return sessionOutcome;
    if (this.vehicle === null) return this.refused('no vehicle is open; send open_vehicle first');
    const target = commandArguments['element'] as Element | undefined;
    const element = this.find(target);
    switch (command) {
      case 'save_vehicle_project': {
        if (this.projectFile === null) return this.refused('the session has no project file; save it as a project first');
        if (!this.projectsDirectoryWritable) return this.refused(`cannot write into the folder '${this.projectFile.directory}'`);
        this.saveProject(this.projectFile.name, this.vehicleName);
        this.savedStateNumber = this.stateNumber;
        return this.accepted({});
      }
      case 'save_vehicle_project_as': {
        const directory = String(commandArguments['directory']);
        const name = String(commandArguments['name']);
        const unreadable = this.unreadableDirectory(directory);
        if (unreadable !== null) return this.refused(unreadable);
        if (!projectName.test(name)) return this.refused(`'${name}' is not a project name: use only letters, digits, '.', '_' and '-'`);
        if (!this.projectsDirectoryWritable) return this.refused(`cannot write into the folder '${directory}'`);
        // Written first; only then does the session's project take the name. The name is the document's identity,
        // not an edit: it changes in every state undo and redo hold, with one revision and no undo step.
        this.saveProject(name, name);
        if (this.vehicleName !== name) {
          for (const state of [...this.undoSteps, ...this.redoSteps]) state.vehicleName = name;
          this.vehicleName = name;
          this.revision += 1;
        }
        this.projectFile = { directory, name };
        this.savedStateNumber = this.stateNumber;
        return this.accepted({});
      }
      case 'get_visual_definition':
        return this.accepted({ visual_definition: this.visualDefinition });
      case 'describe_element_kinds':
        return this.accepted({ element_kinds: this.vehicle.kinds });
      case 'get_outline':
        return this.accepted({
          vehicle_name: this.vehicleName,
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
        to.push(this.projectState());
        ({ vehicleName: this.vehicleName, elements: this.elements, stateNumber: this.stateNumber } = restored);
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
