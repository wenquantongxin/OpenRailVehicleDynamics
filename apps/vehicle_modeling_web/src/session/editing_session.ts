import type { SendCommand } from '../service/command_channel.ts';
import {
  readCheckReport,
  readOutline,
  readPreview,
  readProperties,
  type Accepted,
  type CheckReport,
  type ElementReference,
  type Outline,
  type Preview,
  type PropertyReading,
  type Vector3,
} from '../service/protocol.ts';
import type { ReferenceVehicle } from '../service/reference_vehicles.ts';
import type { VisualDefinition } from '../visual/visual_definition.ts';

// The editing session as the interface shows it. The service's project is the
// only editable truth: this holds the service's latest answers, beside the
// visual definition the vehicle is drawn with, and every accepted command is
// followed by reading again what the interface shows. Actions run one after
// another in the order they are asked for, so each one starts from the state
// the previous one left. From the moment an open is asked for until it has
// finished, nothing asked for against the shown project is sent: it would
// reach the project being opened instead.

export interface OpenedVehicle {
  reference: ReferenceVehicle;
  visualDefinition: VisualDefinition | null;
}

/** A check report with the revision it was made at. */
export interface CheckOutcome {
  revision: number;
  report: CheckReport;
}

export type EditValue = number | Vector3;

export interface EditRefusal {
  element: ElementReference;
  property: string;
  value: EditValue;
  message: string;
}

export interface SessionState {
  vehicle: OpenedVehicle | null;
  revision: number;
  canUndo: boolean;
  canRedo: boolean;
  outline: Outline | null;
  check: CheckOutcome | null;
  /** The preview of the last check that passed; older than the project when its revision is lower. */
  preview: Preview | null;
  selection: ElementReference | null;
  /** The selection's properties at the current revision; empty without a selection. */
  properties: PropertyReading[];
  /** The latest refused edit, until the next edit or open. */
  editRefusal: EditRefusal | null;
  /** The latest refused open, selection, undo or redo, in the service's words, until the next action. */
  commandRefusal: string | null;
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
  check: null,
  preview: null,
  selection: null,
  properties: [],
  editRefusal: null,
  commandRefusal: null,
  failure: null,
  opening: null,
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

export class EditingSession {
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
        outcome = { ...closedSession, vehicle: { reference: vehicle, visualDefinition }, ...(await this.readProject(null, null)) };
      } else {
        // A refused open keeps the session that was open.
        outcome = { commandRefusal: opened.refusal.message };
      }
      this.pendingOpens -= 1;
      this.publish({ ...outcome, opening: this.pendingOpens > 0 ? this.current.opening : null });
    });
  }

  select(element: ElementReference | null): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      if (element === null) {
        this.publish({ selection: null, properties: [], commandRefusal: null });
        return;
      }
      const answer = await this.send('get_properties', { element });
      if (!answer.ok) {
        this.publish({ commandRefusal: answer.refusal.message });
        return;
      }
      this.publish({ ...envelope(answer), selection: element, properties: readProperties(answer.result), commandRefusal: null });
    });
  }

  /** Resolves to whether the service accepted the edit. */
  setProperty(element: ElementReference, property: string, value: EditValue): Promise<boolean> {
    let accepted = false;
    return this.enqueueOnShownProject(async () => {
      const answer = await this.send('set_property', { element, property, value });
      if (!answer.ok) {
        this.publish({ editRefusal: { element, property, value, message: answer.refusal.message }, commandRefusal: null });
        return;
      }
      this.publish({ ...(await this.readProject(this.current.selection, this.current.preview)), editRefusal: null, commandRefusal: null });
      accepted = true;
    }).then(() => accepted);
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

  private step(command: 'undo' | 'redo'): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      const answer = await this.send(command);
      if (!answer.ok) {
        this.publish({ commandRefusal: answer.refusal.message });
        return;
      }
      this.publish({ ...(await this.readProject(this.current.selection, this.current.preview)), commandRefusal: null });
    });
  }

  /**
   * Everything shown after the project changed: the check, the preview when
   * the check passed (otherwise the one given, which the service kept), the
   * outline and the selection's properties.
   */
  private async readProject(selection: ElementReference | null, keptPreview: Preview | null): Promise<Partial<SessionState>> {
    const checked = await this.accepted('check');
    const report = readCheckReport(checked.result);
    const preview = report.passed ? readPreview((await this.accepted('get_preview')).result) : keptPreview;
    const outlined = await this.accepted('get_outline');
    let last = outlined;
    let properties: PropertyReading[] = [];
    if (selection !== null) {
      last = await this.accepted('get_properties', { element: selection });
      properties = readProperties(last.result);
    }
    return {
      ...envelope(last),
      check: { revision: checked.revision, report },
      preview,
      outline: readOutline(outlined.result),
      selection,
      properties,
    };
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
