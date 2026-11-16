import type { SendCommand } from '../service/modeling_server_command_channel.ts';
import {
  readCheckReport,
  readEditedElement,
  readElementKinds,
  readOutline,
  readPreview,
  readProperties,
  readRemovedElements,
  sameElement,
  type Accepted,
  type CheckReport,
  type CurvePoint,
  type ElementKindDescription,
  type ElementReference,
  type Outline,
  type Preview,
  type PropertyReading,
  type Refusal,
  type Vector3,
} from '../service/vehicle_modeling_service_protocol.ts';
import type { ReferenceVehicle } from '../service/fetch_reference_vehicles.ts';
import type { VisualDefinition } from '../visual/read_vehicle_visual_definition.ts';

// The editing session as the interface shows it. The service's project is the
// only editable truth: this holds the service's latest answers, beside the
// visual definition the vehicle is drawn with, and every accepted command is
// followed by reading again what the interface shows. Actions run one after
// another in the order they are asked for, so each one starts from the state
// the previous one left. From the moment an open is asked for until it has
// finished, nothing asked for against the shown project is sent: it would
// reach the project being opened instead.
//
// The selection stays true to the project: after every edit, undo or redo the
// outline is read again, a selection the outline no longer has is dropped, and
// a renamed selection follows its new name. Nothing is ever asked about an
// element the outline does not have. A rename changes the name every other
// edit of the element is sent under, so from the moment it is asked for until
// it is answered the session says the element is being renamed, and the
// element's other properties take no input. Every removal says what it takes
// and waits to be confirmed.

export interface OpenedVehicle {
  reference: ReferenceVehicle;
  visualDefinition: VisualDefinition | null;
}

/** A check report with the revision it was made at. */
export interface CheckOutcome {
  revision: number;
  report: CheckReport;
}

/** A value as set_property and add_element take it, of the property's value kind; null names none where a reference is optional. */
export type EditValue = number | Vector3 | boolean | string | null | readonly CurvePoint[];

export interface EditRefusal {
  element: ElementReference;
  property: string;
  value: EditValue;
  message: string;
}

/** Sets one property of an element; resolves to whether the service accepted the edit. */
export type SetProperty = (element: ElementReference, property: string, value: EditValue) => Promise<boolean>;

/** A removal waiting for the user to confirm it. */
export interface PendingRemoval {
  element: ElementReference;
  /** Everything the removal takes, the element itself included, in outline order. */
  removedElements: ElementReference[];
}

export interface SessionState {
  vehicle: OpenedVehicle | null;
  revision: number;
  canUndo: boolean;
  canRedo: boolean;
  outline: Outline | null;
  /** Every element kind with its properties, as the service describes them; empty while no vehicle is open. */
  elementKinds: ElementKindDescription[];
  check: CheckOutcome | null;
  /** The preview of the last check that passed; older than the project when its revision is lower. */
  preview: Preview | null;
  selection: ElementReference | null;
  /** The selection's properties at the current revision; empty without a selection. */
  properties: PropertyReading[];
  /** The kind whose creation form the property panel shows; nothing is selected meanwhile. */
  creatingKind: string | null;
  /** The element whose rename is on its way to the service; its other properties take no input meanwhile. */
  renaming: ElementReference | null;
  /** A removal waiting to be confirmed. */
  pendingRemoval: PendingRemoval | null;
  /** What the latest removal took, until the next action. */
  removedElements: ElementReference[] | null;
  /** The latest refused edit, until the next edit or open. */
  editRefusal: EditRefusal | null;
  /** The latest refused open, removal, undo or redo, in the service's words, until the next action. */
  commandRefusal: Refusal | null;
  /** The server or the service failed; the session cannot be trusted to match the project any more. */
  failure: string | null;
  /** The vehicle asked to be opened last, until no open is left to finish; the shown project takes no actions meanwhile. */
  opening: ReferenceVehicle | null;
}

export type LoadVisualDefinition = (vehicle: ReferenceVehicle) => Promise<VisualDefinition | null>;

const closedSession: SessionState = {
  vehicle: null,
  revision: 0,
  canUndo: false,
  canRedo: false,
  outline: null,
  elementKinds: [],
  check: null,
  preview: null,
  selection: null,
  properties: [],
  creatingKind: null,
  renaming: null,
  pendingRemoval: null,
  removedElements: null,
  editRefusal: null,
  commandRefusal: null,
  failure: null,
  opening: null,
};

/** What the panel shows once an action has changed what it is about. */
const nothingPending: Pick<SessionState, 'creatingKind' | 'pendingRemoval' | 'removedElements' | 'commandRefusal'> = {
  creatingKind: null,
  pendingRemoval: null,
  removedElements: null,
  commandRefusal: null,
};

export type CheckStatus = 'passed' | 'failed' | 'not_checked';

/** The status of the current revision: a check of an earlier revision does not count. */
export function checkStatus(state: SessionState): CheckStatus {
  if (state.check === null || state.check.revision !== state.revision) {
    return 'not_checked';
  }
  return state.check.report.passed ? 'passed' : 'failed';
}

/** The preview shows an older project than the current revision. */
export function previewIsStale(state: SessionState): boolean {
  return state.preview !== null && state.preview.projectRevision < state.revision;
}

function outlineHas(outline: Outline | null, element: ElementReference): boolean {
  return outline !== null && outline.elements.some((each) => sameElement(each, element));
}

export class VehicleEditingSession {
  private current: SessionState = closedSession;
  private readonly listeners = new Set<() => void>();
  private actions: Promise<void> = Promise.resolve();
  /** Opens asked for and not yet finished. */
  private pendingOpens = 0;
  private readonly send: SendCommand;
  private readonly loadVisualDefinition: LoadVisualDefinition;

  constructor(send: SendCommand, loadVisualDefinition: LoadVisualDefinition) {
    this.send = send;
    this.loadVisualDefinition = loadVisualDefinition;
  }

  get state(): SessionState {
    return this.current;
  }

  readonly subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };

  open(vehicle: ReferenceVehicle): Promise<void> {
    if (this.current.failure !== null) {
      return Promise.resolve();
    }
    this.pendingOpens += 1;
    this.publish({ opening: vehicle });
    return this.enqueue(async () => {
      const opened = await this.send('open_vehicle', vehicle.openArguments);
      let outcome: Partial<SessionState>;
      if (opened.ok) {
        const visualDefinition = await this.loadVisualDefinition(vehicle);
        const elementKinds = readElementKinds((await this.accepted('describe_element_kinds')).result);
        outcome = { ...closedSession, vehicle: { reference: vehicle, visualDefinition }, elementKinds, ...(await this.readProject(null, null)) };
      } else {
        // A refused open keeps the session that was open.
        outcome = { commandRefusal: opened.refusal };
      }
      this.pendingOpens -= 1;
      this.publish({ ...outcome, opening: this.pendingOpens > 0 ? this.current.opening : null });
    });
  }

  /** Selects an element and reads its properties; an element the outline does not have leaves nothing selected. */
  select(element: ElementReference | null): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      if (element === null || !outlineHas(this.current.outline, element)) {
        this.publish({ ...nothingPending, selection: null, properties: [] });
        return;
      }
      const answer = await this.accepted('get_properties', { element });
      this.publish({ ...nothingPending, ...envelope(answer), selection: element, properties: readProperties(answer.result) });
    });
  }

  /** Shows the creation form of a kind instead of the selection. */
  startCreation(kind: string): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      this.publish({ ...nothingPending, selection: null, properties: [], creatingKind: kind });
    });
  }

  cancelCreation(): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      this.publish({ creatingKind: null });
    });
  }

  /**
   * Creates an element and selects it. Resolves to the service's refusal, or
   * to null once the element is created (or when nothing was sent, because a
   * vehicle is opening).
   */
  createElement(kind: string, name: string, properties: Readonly<Record<string, EditValue>>): Promise<Refusal | null> {
    let refusal: Refusal | null = null;
    return this.enqueueOnShownProject(async () => {
      const answer = await this.send('add_element', { kind, name, properties });
      if (!answer.ok) {
        refusal = answer.refusal;
        return;
      }
      this.publish({ ...nothingPending, ...(await this.readProject(readEditedElement(answer.result), this.current.preview)) });
    }).then(() => refusal);
  }

  /**
   * Resolves to whether the service accepted the edit. A renamed selection
   * stays selected under its new name; until the rename is answered the
   * element is being renamed.
   */
  setProperty(element: ElementReference, property: string, value: EditValue): Promise<boolean> {
    let accepted = false;
    const renames = this.pendingOpens === 0 && this.isOwnName(element.kind, property);
    if (renames) this.publish({ renaming: element });
    return this.enqueueOnShownProject(async () => {
      if (!outlineHas(this.current.outline, element)) return;
      const answer = await this.send('set_property', { element, property, value });
      if (!answer.ok) {
        this.publish({ editRefusal: { element, property, value, message: answer.refusal.message }, commandRefusal: null });
        return;
      }
      const selection = sameElement(this.current.selection, element) ? readEditedElement(answer.result) : this.current.selection;
      this.publish({ ...nothingPending, ...(await this.readProject(selection, this.current.preview)), editRefusal: null });
      accepted = true;
    }).then(() => {
      if (renames) this.publish({ renaming: null });
      return accepted;
    });
  }

  /** Asks the service what removing the element would take; the removal then waits for confirmRemoval or cancelRemoval. */
  requestRemoval(element: ElementReference): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      if (!outlineHas(this.current.outline, element)) return;
      const answer = await this.send('get_removal_consequences', { element });
      if (!answer.ok) {
        this.publish({ ...nothingPending, commandRefusal: answer.refusal });
        return;
      }
      this.publish({ ...nothingPending, pendingRemoval: { element, removedElements: readRemovedElements(answer.result) } });
    });
  }

  confirmRemoval(): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      const pending = this.current.pendingRemoval;
      if (pending === null) return;
      const answer = await this.send('remove_element', { element: pending.element });
      if (!answer.ok) {
        this.publish({ ...nothingPending, commandRefusal: answer.refusal });
        return;
      }
      const removedElements = readRemovedElements(answer.result);
      this.publish({ ...nothingPending, ...(await this.readProject(this.current.selection, this.current.preview)), removedElements });
    });
  }

  cancelRemoval(): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      this.publish({ pendingRemoval: null });
    });
  }

  undo(): Promise<void> {
    return this.step('undo');
  }

  redo(): Promise<void> {
    return this.step('redo');
  }

  /** Forgets the latest refused edit, once its field is reverted. */
  dismissEditRefusal(): void {
    this.publish({ editRefusal: null });
  }

  /** Whether the kind's property is its elements' own name, as the service describes it. */
  private isOwnName(kind: string, property: string): boolean {
    const described = this.current.elementKinds.find((each) => each.kind === kind)?.properties.find((each) => each.name === property);
    return described?.valueKind === 'element_name';
  }

  private step(command: 'undo' | 'redo'): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      const answer = await this.send(command);
      if (!answer.ok) {
        this.publish({ ...nothingPending, commandRefusal: answer.refusal });
        return;
      }
      this.publish({ ...nothingPending, ...(await this.readProject(this.current.selection, this.current.preview)) });
    });
  }

  /**
   * Everything shown after the project changed: the check, the preview when
   * the check passed (otherwise the one given, which the service kept), the
   * outline and the selection's properties, when the outline still has the
   * selection.
   */
  private async readProject(selection: ElementReference | null, keptPreview: Preview | null): Promise<Partial<SessionState>> {
    const checked = await this.accepted('check');
    const report = readCheckReport(checked.result);
    const preview = report.passed ? readPreview((await this.accepted('get_preview')).result) : keptPreview;
    const outlined = await this.accepted('get_outline');
    const outline = readOutline(outlined.result);
    const kept = selection !== null && outlineHas(outline, selection) ? selection : null;
    let last = outlined;
    let properties: PropertyReading[] = [];
    if (kept !== null) {
      last = await this.accepted('get_properties', { element: kept });
      properties = readProperties(last.result);
    }
    return { ...envelope(last), check: { revision: checked.revision, report }, preview, outline, selection: kept, properties };
  }

  /** A command the service has no reason to refuse; a refusal means the interface and the service disagree. */
  private async accepted(command: string, commandArguments?: Readonly<Record<string, unknown>>): Promise<Accepted> {
    const answer = await this.send(command, commandArguments);
    if (!answer.ok) {
      throw new Error(`the modelling service refused '${command}': ${answer.refusal.message}`);
    }
    return answer;
  }

  /** Queues an action on the shown project, unless a vehicle is opening: then it is not sent at all. */
  private enqueueOnShownProject(action: () => Promise<void>): Promise<void> {
    return this.pendingOpens > 0 ? Promise.resolve() : this.enqueue(action);
  }

  private enqueue(action: () => Promise<void>): Promise<void> {
    const run = this.actions.then(async () => {
      if (this.current.failure !== null) {
        return;
      }
      try {
        await action();
      } catch (error) {
        this.publish({ failure: error instanceof Error ? error.message : String(error), opening: null });
      }
    });
    this.actions = run;
    return run;
  }

  private publish(changes: Partial<SessionState>): void {
    this.current = { ...this.current, ...changes };
    for (const listener of this.listeners) {
      listener();
    }
  }
}

function envelope(answer: Accepted): Pick<SessionState, 'revision' | 'canUndo' | 'canRedo'> {
  return { revision: answer.revision, canUndo: answer.canUndo, canRedo: answer.canRedo };
}
