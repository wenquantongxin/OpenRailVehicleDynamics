import { loadBundleFromUrl, ResourceReadError } from '../record/load_bundle.ts';
import { composeSceneDefinition, type SceneAssets } from '../record/scene_assets.ts';
import type { SceneDefinition } from '../record/scene_definition.ts';
import { decodeMessage, descriptionSemanticallyEqual, stateMatchesDescription, type DescriptionMessage, type StateMessage } from '../stream/protocol.ts';

export interface LiveConnectionStatus { state: 'connecting' | 'loading' | 'connected' | 'reconnecting' | 'error'; message: string }
export interface LiveConnectionCallbacks {
  ready(definition: SceneDefinition, description: DescriptionMessage, state: StateMessage): void;
  state(state: StateMessage): void;
  status(status: LiveConnectionStatus): void;
  disconnected(): void;
}

/** One observer connection. A description, its resources and first state publish together. */
export class LiveConnection {
  private socket: WebSocket | null = null;
  private stopped = false;
  private retry: ReturnType<typeof setTimeout> | null = null;
  private retryDelay = 250;
  private generation = 0;
  private loading: AbortController | null = null;
  private description: DescriptionMessage | null = null;
  private lastDescription: DescriptionMessage | null = null;
  private definition: SceneDefinition | null = null;
  private pending: StateMessage | null = null;
  private lastState: { runId: bigint; sequence: bigint; time: number } | null = null;
  private installed = false;
  private blockedRun: bigint | null = null;
  private readonly url: URL;
  private readonly callbacks: LiveConnectionCallbacks;
  private readonly selectedBundle: string | null;
  private readonly makeSocket: (url: string) => WebSocket;
  private failure: string | null = null;

  constructor(callbacks: LiveConnectionCallbacks, selectedBundle: string | null,
    pageUrl: string = window.location.href, makeSocket: (url: string) => WebSocket = (url) => new WebSocket(url)) {
    this.callbacks = callbacks;
    this.selectedBundle = selectedBundle;
    this.makeSocket = makeSocket;
    this.url = new URL('/live', pageUrl);
    this.url.protocol = this.url.protocol === 'https:' ? 'wss:' : 'ws:';
  }
  start(): void { this.connect(); }
  dispose(): void {
    this.stopped = true;
    this.generation++;
    this.loading?.abort();
    if (this.retry !== null) clearTimeout(this.retry);
    this.socket?.close();
    this.socket = null;
  }
  private connect(): void {
    if (this.stopped) return;
    if (this.failure === null) this.callbacks.status({ state: this.retryDelay === 250 ? 'connecting' : 'reconnecting', message: '' });
    const socket = this.makeSocket(this.url.href);
    this.socket = socket;
    socket.binaryType = 'arraybuffer';
    socket.onmessage = (event) => {
      if (socket !== this.socket || this.stopped) return;
      try {
        if (!(event.data instanceof ArrayBuffer)) throw new Error('Expected an OSTS binary message');
        const message = decodeMessage(new Uint8Array(event.data));
        if (message.kind === 'description') this.acceptDescription(message);
        else this.acceptState(message);
      } catch (error) { this.fail(error); }
    };
    socket.onerror = () => { /* close schedules the next connection */ };
    socket.onclose = () => {
      if (socket !== this.socket || this.stopped) return;
      this.socket = null;
      this.generation++;
      this.loading?.abort();
      this.description = null;
      this.pending = null;
      this.installed = false;
      this.callbacks.disconnected();
      if (this.failure === null) this.callbacks.status({ state: 'reconnecting', message: '' });
      this.retry = setTimeout(() => { this.retry = null; this.connect(); }, this.retryDelay);
      this.retryDelay = Math.min(5000, this.retryDelay * 2);
    };
  }
  private acceptDescription(message: DescriptionMessage): void {
    if (this.blockedRun === message.header.runId) return;
    const sameRun = this.lastDescription?.header.runId === message.header.runId;
    if (sameRun && !descriptionSemanticallyEqual(this.lastDescription!, message)) {
      throw new Error('Scene description changed within one run');
    }
    if (this.description?.header.runId === message.header.runId) return;
    this.blockedRun = null;
    if (!sameRun) this.failure = null;
    if (this.failure === null) this.callbacks.status({ state: 'loading', message: '' });
    this.callbacks.disconnected();
    this.description = message;
    this.lastDescription = message;
    if (!sameRun) this.definition = null;
    this.pending = null;
    this.installed = false;
    this.loading?.abort();
    const generation = ++this.generation;
    if (this.definition !== null) return;
    const controller = new AbortController();
    this.loading = controller;
    const empty: SceneAssets = { worldFrame: message.description.worldFrame, lengthUnit: 'meter', angleUnit: 'radian',
      track: null, visualDefinitionFile: null, visualDefinitionText: null };
    const named = message.resourceBundle === null ? null : `/bundles/${encodeURIComponent(message.resourceBundle)}/`;
    const base = named === null ? Promise.resolve(empty) : loadBundleFromUrl(named, controller.signal);
    const selected = this.selectedBundle === null ? Promise.resolve(null) : loadBundleFromUrl(this.selectedBundle, controller.signal);
    Promise.all([base, selected]).then(([assets, override]) => {
      if (this.stopped || controller.signal.aborted || generation !== this.generation) return;
      this.definition = composeSceneDefinition(message.description, assets, override);
      this.publishReady();
    }).catch((error: unknown) => {
      if (!this.stopped && !controller.signal.aborted && generation === this.generation) {
        this.fail(error, error instanceof ResourceReadError);
      }
    });
  }
  private acceptState(message: StateMessage): void {
    if (this.description === null || message.header.runId !== this.description.header.runId || this.blockedRun === message.header.runId) return;
    if (!stateMatchesDescription(message.state, this.description.description)) throw new Error('State layout disagrees with its scene description');
    const { runId, sequence, simulationTimeSeconds: time } = message.header;
    if (this.lastState?.runId === runId) {
      if (sequence < this.lastState.sequence || (sequence === this.lastState.sequence && this.installed)) return;
      if (time < this.lastState.time || (sequence === this.lastState.sequence && time !== this.lastState.time)) {
        throw new Error('Simulation time moved backwards; waiting for a new run');
      }
    }
    this.lastState = { runId, sequence, time };
    if (this.installed) {
      this.callbacks.state(message);
      return;
    }
    this.pending = message;
    this.publishReady();
  }
  private publishReady(): void {
    if (this.installed || this.description === null || this.definition === null || this.pending === null) return;
    this.callbacks.ready(this.definition, this.description, this.pending);
    this.pending = null;
    this.installed = true;
    this.retryDelay = 250;
    this.failure = null;
    this.callbacks.status({ state: 'connected', message: '' });
  }
  private fail(error: unknown, retryable = false): void {
    if (!retryable) this.blockedRun = this.description?.header.runId ?? this.lastDescription?.header.runId ?? null;
    this.loading?.abort();
    this.callbacks.disconnected();
    this.failure = error instanceof Error ? error.message : String(error);
    this.callbacks.status({ state: 'error', message: this.failure });
    if (retryable) this.socket?.close();
  }
}
