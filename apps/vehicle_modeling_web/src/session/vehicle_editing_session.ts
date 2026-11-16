import type { ReferenceVehicle } from '../service/fetch_reference_vehicles.ts';
import type { SendCommand } from '../service/modeling_server_command_channel.ts';
import {
  readCheckReport,
  readEditedElement,
  readElementKinds,
  readOutline,
  readPreview,
  readProperties,
  readRemovedElements,
  readServiceSessionDescription,
  readVehicleProjectNames,
  readVisualDefinitionDocument,
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
  type SessionAfterCommand,
  type Vector3,
  type VehicleProjectFile,
} from '../service/vehicle_modeling_service_protocol.ts';
import { readVisualDefinition, type VisualDefinition } from '../visual/read_vehicle_visual_definition.ts';

// The editing session as the interface shows it. The service's project is the
// only editable truth: this holds the service's latest answers, beside the
// visual definition the vehicle is drawn with, and every accepted command is
// followed by reading again what the interface shows. Actions run one after
// another in the order they are asked for, so each one starts from the state
// the previous one left. From the moment an open is asked for until it has
// finished, nothing asked for against the shown project is sent: it would
// reach the project being opened instead.
//
// The session starts from what the service has open: an open session is
// resumed as it stands, so that reloading the page loses nothing, and a link
// to a vehicle or a project is followed only when nothing is open. Opening
// anything while the project has unsaved changes asks first. Save writes to
// the project file the session is associated with; without one it is Save as,
// which lists the folder of projects just before saving and asks before
// replacing another project. The folder is a string from the server, handed
// to the service as it is: which file a project is and whether a name is
// acceptable, the service says.
//
// The selection stays true to the project: after every edit, undo or redo the
// outline is read again, a selection the outline no longer has is dropped, and
// a renamed selection follows its new name. Nothing is ever asked about an
// element the outline does not have. A rename changes the name every other
// edit of the element is sent under, so from the moment it is asked for until
// it is answered the session says the element is being renamed, and the
// element's other properties take no input. Every removal says what it takes
// and waits to be confirmed.

/** A vehicle the session has open and what it is drawn with: a new one for every open and every resumption. */
export interface OpenedVehicle {
  visualDefinition: VisualDefinition | null;
  /** Why the visual definition the project carries cannot be drawn from; the bodies are then drawn as their markers. */
  visualDefinitionProblem: string | null;
}

/** What can be opened: a reference vehicle from its files, or a project saved in the folder of projects. */
export type OpenChoice = { source: 'reference_vehicle'; vehicle: ReferenceVehicle } | { source: 'vehicle_project'; name: string };

/** A link the page was opened with: followed only when nothing is open. */
export interface StartLink {
  open: OpenChoice;
  /** The element to select once it is open. */
  select: ElementReference | null;
}

/** Save as, while its dialog is shown. */
export interface SaveAsDialog {
  /** The name the dialog holds: the project's own at first, then the one last asked for. */
  name: string;
  /** The name is another project in the folder, which saving would replace: the dialog asks first. */
  replaceQuestion: boolean;
  /** The service's refusal of the name or of the folder, shown at the field. */
  refusal: string | null;
  /** The listing of the folder or the save is on its way. */
  saving: boolean;
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
  /** The service has been asked what is open; until then nothing is shown as open or as closed. */
  started: boolean;
  /** The folder of projects as the server named it; null until the start. */
  vehicleProjectsDirectory: string | null;
  /** The projects in the folder as listed last; null until listed. */
  vehicleProjects: string[] | null;
  /** Null while no vehicle is open. */
  vehicle: OpenedVehicle | null;
  /** The project file the session is associated with: the one it was opened from or last saved as. */
  vehicleProjectFile: VehicleProjectFile | null;
  /** The project differs from the state it was last saved in or, before any save, the state it was opened in. */
  hasUnsavedChanges: boolean;
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
  /** The latest refused open, listing, save, removal, undo or redo, in the service's words, until the next action. */
  commandRefusal: Refusal | null;
  /** The server or the service failed; the session cannot be trusted to match the project any more. */
  failure: string | null;
  /** What was asked to be opened last, until no open is left to finish; the shown project takes no actions meanwhile. */
  opening: OpenChoice | null;
  /** The reference vehicles and projects are shown over the open session, to open another. */
  choosingWhatToOpen: boolean;
  /** A choice to open made while the project has unsaved changes, waiting until they are discarded or kept. */
  pendingOpen: OpenChoice | null;
  saveAs: SaveAsDialog | null;
  /** The start link that was not followed because a session was open, which was resumed instead. */
  linkNotFollowed: OpenChoice | null;
}

const closedSession: SessionState = {
  started: false,
  vehicleProjectsDirectory: null,
  vehicleProjects: null,
  vehicle: null,
  vehicleProjectFile: null,
  hasUnsavedChanges: false,
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
  choosingWhatToOpen: false,
  pendingOpen: null,
  saveAs: null,
  linkNotFollowed: null,
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

/** The vehicle as drawn with the document it carries: one the view cannot draw from is a display problem, and the bodies are drawn as markers. */
function vehicleDrawnWith(document: unknown): OpenedVehicle {
  if (document === null) {
    return { visualDefinition: null, visualDefinitionProblem: null };
  }
  try {
    return { visualDefinition: readVisualDefinition(document), visualDefinitionProblem: null };
  } catch (error) {
    return { visualDefinition: null, visualDefinitionProblem: error instanceof Error ? error.message : String(error) };
  }
}

export class VehicleEditingSession {
  private current: SessionState = closedSession;
  private readonly listeners = new Set<() => void>();
  private actions: Promise<void> = Promise.resolve();
  /** Opens asked for and not yet finished. */
  private pendingOpens = 0;
  private startAsked = false;
  private readonly send: SendCommand;

  constructor(send: SendCommand) {
    this.send = send;
  }

  get state(): SessionState {
    return this.current;
  }

  readonly subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };

  /**
   * Starts from what the service has open, once: an open session is resumed
   * as it stands, with nothing selected, and the link is not followed; only
   * when nothing is open is the folder of projects listed and the link
   * followed.
   */
  start(vehicleProjectsDirectory: string, link: StartLink | null): Promise<void> {
    if (this.startAsked) {
      return Promise.resolve();
    }
    this.startAsked = true;
    this.publish({ vehicleProjectsDirectory });
    return this.enqueue(async () => {
      const described = readServiceSessionDescription((await this.accepted('get_session')).result);
      if (described.open) {
        this.publish({ ...(await this.readOpenedProject(described.vehicleProjectFile)), linkNotFollowed: link?.open ?? null });
        return;
      }
      await this.listVehicleProjects();
      this.publish({ started: true });
      if (link === null) return;
      this.publish({ opening: link.open });
      const opened = await this.openNow(link.open);
      this.publish({ opening: null });
      if (opened && link.select !== null) await this.selectNow(link.select);
    });
  }

  /**
   * Opens a reference vehicle or a saved project. While the project has
   * unsaved changes nothing is opened: the choice waits until the changes are
   * discarded or kept.
   */
  open(choice: OpenChoice): Promise<void> {
    return this.openUnlessChangesAreUnsaved(choice, false);
  }

  /** Opens the choice that waited, and the unsaved changes are lost. */
  discardChangesAndOpen(): Promise<void> {
    const choice = this.current.pendingOpen;
    if (choice === null) {
      return Promise.resolve();
    }
    this.publish({ pendingOpen: null });
    return this.openUnlessChangesAreUnsaved(choice, true);
  }

  /** Forgets the choice that waited and goes back to the project, which stays as it is. */
  keepEditing(): void {
    this.publish({ pendingOpen: null, choosingWhatToOpen: false });
  }

  /** Shows the reference vehicles and the projects over the open session, the folder listed again. */
  showOpenChoices(): Promise<void> {
    this.publish({ choosingWhatToOpen: true, commandRefusal: null });
    return this.enqueueOnShownProject(() => this.listVehicleProjects());
  }

  /** Goes back to the open session without opening anything. */
  leaveOpenChoices(): void {
    this.publish({ choosingWhatToOpen: false, pendingOpen: null });
  }

  /** Writes the project to the file the session is associated with; without one, shows the Save as dialog. */
  save(): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      if (this.current.vehicle === null) return;
      if (this.current.vehicleProjectFile === null) {
        this.publish({ saveAs: this.newSaveAsDialog() });
        return;
      }
      const answer = await this.send('save_vehicle_project');
      this.publish(answer.ok ? { ...envelope(answer), commandRefusal: null } : { commandRefusal: answer.refusal });
    });
  }

  /** Shows the Save as dialog, holding the project's name when the session has a file. */
  startSaveAs(): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      if (this.current.vehicle !== null) this.publish({ saveAs: this.newSaveAsDialog() });
    });
  }

  cancelSaveAs(): void {
    this.publish({ saveAs: null });
  }

  /** Answers the question about replacing another project with no: the dialog takes a name again. */
  chooseAnotherName(): void {
    const dialog = this.current.saveAs;
    if (dialog !== null) this.publish({ saveAs: { ...dialog, replaceQuestion: false } });
  }

  /**
   * Saves the project as the named project of the folder. The folder is
   * listed just before: a name that is another project there is asked about
   * first, unless replacing it is the answer this call carries. A refusal is
   * kept at the dialog with the name. After saving, the session is associated
   * with the new file and shows the project under its new name.
   */
  saveAs(name: string, replacingAnotherProject: boolean): Promise<void> {
    return this.enqueueOnShownProject(async () => {
      if (this.current.saveAs === null) return;
      const directory = this.projectsDirectory();
      const dialog = (changes: Partial<SaveAsDialog>): Partial<SessionState> => ({ saveAs: { name, replaceQuestion: false, refusal: null, saving: false, ...changes } });
      this.publish(dialog({ saving: true }));
      if (!replacingAnotherProject) {
        const listed = await this.send('list_vehicle_projects', { directory });
        if (!listed.ok) {
          this.publish(dialog({ refusal: listed.refusal.message }));
          return;
        }
        const vehicleProjects = readVehicleProjectNames(listed.result);
        const ownFile = this.current.vehicleProjectFile;
        const another = vehicleProjects.includes(name) && !(ownFile !== null && ownFile.directory === directory && ownFile.name === name);
        this.publish({ vehicleProjects, ...(another ? dialog({ replaceQuestion: true }) : {}) });
        if (another) return;
      }
      const saved = await this.send('save_vehicle_project_as', { directory, name });
      if (!saved.ok) {
        this.publish(dialog({ refusal: saved.refusal.message }));
        return;
      }
      // Saving as renamed the vehicle, which is one more revision: check, draw and outline it again.
      this.publish({ ...(await this.readProject(this.current.selection, this.current.preview)), vehicleProjectFile: { directory, name }, saveAs: null, commandRefusal: null });
      await this.listVehicleProjects();
    });
  }

  /** Selects an element and reads its properties; an element the outline does not have leaves nothing selected. */
  select(element: ElementReference | null): Promise<void> {
    return this.enqueueOnShownProject(() => this.selectNow(element));
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

  private projectsDirectory(): string {
    const directory = this.current.vehicleProjectsDirectory;
    if (directory === null) {
      throw new Error('the session has not started, so it knows no folder of projects');
    }
    return directory;
  }

  private newSaveAsDialog(): SaveAsDialog {
    return { name: this.current.vehicleProjectFile?.name ?? '', replaceQuestion: false, refusal: null, saving: false };
  }

  private openUnlessChangesAreUnsaved(choice: OpenChoice, discardingChanges: boolean): Promise<void> {
    if (this.current.failure !== null) {
      return Promise.resolve();
    }
    this.pendingOpens += 1;
    this.publish({ opening: choice });
    return this.enqueue(async () => {
      try {
        // Decided here, after everything asked for before: an edit on its way counts.
        if (!discardingChanges && this.current.vehicle !== null && this.current.hasUnsavedChanges) {
          this.publish({ pendingOpen: choice });
          return;
        }
        await this.openNow(choice);
      } finally {
        this.pendingOpens -= 1;
        this.publish({ opening: this.pendingOpens > 0 ? this.current.opening : null });
      }
    });
  }

  /** Opens the choice; resolves to whether it opened. A refused open keeps the session that was open. */
  private async openNow(choice: OpenChoice): Promise<boolean> {
    const file = choice.source === 'vehicle_project' ? { directory: this.projectsDirectory(), name: choice.name } : null;
    const opened = choice.source === 'reference_vehicle' ? await this.send('open_vehicle', choice.vehicle.openArguments) : await this.send('open_vehicle_project', { ...file });
    if (!opened.ok) {
      this.publish({ commandRefusal: opened.refusal });
      return false;
    }
    this.publish(await this.readOpenedProject(file));
    return true;
  }

  /**
   * Everything shown of a project the session shows from now on, opened or
   * resumed, with nothing selected. The preview is asked for whatever the
   * check says: a resumed project may have one the service kept from before a
   * check that refused it.
   */
  private async readOpenedProject(vehicleProjectFile: VehicleProjectFile | null): Promise<SessionState> {
    const elementKinds = readElementKinds((await this.accepted('describe_element_kinds')).result);
    const vehicle = vehicleDrawnWith(readVisualDefinitionDocument((await this.accepted('get_visual_definition')).result));
    const checked = await this.accepted('check');
    const preview = readPreview((await this.accepted('get_preview')).result);
    const outlined = await this.accepted('get_outline');
    const { vehicleProjectsDirectory, vehicleProjects, opening } = this.current;
    return {
      ...closedSession,
      ...envelope(outlined),
      started: true,
      vehicleProjectsDirectory,
      vehicleProjects,
      opening,
      vehicle,
      vehicleProjectFile,
      elementKinds,
      check: { revision: openSession(checked).revision, report: readCheckReport(checked.result) },
      preview,
      outline: readOutline(outlined.result),
    };
  }

  private async listVehicleProjects(): Promise<void> {
    const listed = await this.send('list_vehicle_projects', { directory: this.projectsDirectory() });
    this.publish(listed.ok ? { vehicleProjects: readVehicleProjectNames(listed.result) } : { commandRefusal: listed.refusal });
  }

  private async selectNow(element: ElementReference | null): Promise<void> {
    if (element === null || !outlineHas(this.current.outline, element)) {
      this.publish({ ...nothingPending, selection: null, properties: [] });
      return;
    }
    const answer = await this.accepted('get_properties', { element });
    this.publish({ ...nothingPending, ...envelope(answer), selection: element, properties: readProperties(answer.result) });
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
    return { ...envelope(last), check: { revision: openSession(checked).revision, report }, preview, outline, selection: kept, properties };
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

/** The open session an accepted answer describes; an answer that says nothing is open means the service and the interface disagree. */
function openSession(answer: Accepted): SessionAfterCommand {
  if (answer.session === null) {
    throw new Error('the modelling service answered as if no vehicle were open');
  }
  return answer.session;
}

function envelope(answer: Accepted): Pick<SessionState, 'revision' | 'canUndo' | 'canRedo' | 'hasUnsavedChanges'> {
  const { revision, canUndo, canRedo, hasUnsavedChanges } = openSession(answer);
  return { revision, canUndo, canRedo, hasUnsavedChanges };
}
