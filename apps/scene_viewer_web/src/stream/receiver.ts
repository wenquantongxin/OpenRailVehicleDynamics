import { decodeDatagramHeader, decodeMessage, descriptionSemanticallyEqual, fragmentPayloadBytes, headerBytes, stateMatchesDescription } from './protocol.ts';
import type { DescriptionMessage, OstsHeader, StateMessage } from './protocol.ts';

interface PartialMessage { header: OstsHeader; bytes: Uint8Array; received: Set<number>; }
interface Run {
  description: DescriptionMessage | null;
  state: StateMessage | null;
  pendingState: StateMessage | null;
  sequences: Map<number, bigint>;
  retired: boolean;
}
export interface StreamUpdate { description: DescriptionMessage; state: StateMessage; changedRun: boolean; }

/** Bounded UDP reassembly, semantic admission and one active display channel. */
export class StreamReceiver {
  private readonly partials = new Map<string, PartialMessage>();
  private readonly runs = new Map<bigint, Run>();
  private activeId: bigint | null = null;
  get partialCount(): number { return this.partials.size; }
  get runCount(): number { return this.runs.size; }
  get current(): StreamUpdate | null {
    const run = this.activeId === null ? null : this.runs.get(this.activeId);
    return run?.description && run.state ? { description: run.description, state: run.state, changedRun: false } : null;
  }
  private run(id: bigint): Run {
    const known = this.runs.get(id);
    if (known) return known;
    while (this.runs.size >= 16) {
      const oldest = [...this.runs.keys()].find((key) => key !== this.activeId)!;
      this.runs.delete(oldest);
      for (const [key, partial] of this.partials) if (partial.header.runId === oldest) this.partials.delete(key);
    }
    const created: Run = { description: null, state: null, pendingState: null, sequences: new Map(), retired: false };
    this.runs.set(id, created);
    return created;
  }
  private clearOlder(header: OstsHeader): void {
    for (const [key, partial] of this.partials) {
      const other = partial.header;
      if (other.runId === header.runId && other.kind === header.kind && other.sequence <= header.sequence) this.partials.delete(key);
    }
  }
  feed(datagram: Uint8Array): StreamUpdate | null {
    const header = decodeDatagramHeader(datagram);
    const known = this.runs.get(header.runId);
    if (header.sequence <= (known?.sequences.get(header.kind) ?? -1n)) return null;
    const key = `${header.runId}/${header.kind}/${header.sequence}`;
    let partial = this.partials.get(key);
    if (!partial) {
      if (this.partials.size >= 8) this.partials.delete(this.partials.keys().next().value!);
      partial = { header, bytes: new Uint8Array(headerBytes + header.payloadBytes), received: new Set() };
      this.partials.set(key, partial);
    }
    if (partial.header.payloadBytes !== header.payloadBytes || partial.header.simulationTimeSeconds !== header.simulationTimeSeconds) {
      this.partials.delete(key);
      throw new Error('Conflicting OSTS fragments');
    }
    const start = headerBytes + header.fragmentIndex * fragmentPayloadBytes;
    const body = datagram.subarray(headerBytes);
    if (partial.received.has(header.fragmentIndex)) {
      if (!body.every((value, index) => partial!.bytes[start + index] === value)) {
        this.partials.delete(key);
        throw new Error('Conflicting duplicate OSTS fragment');
      }
      return null;
    }
    if (header.fragmentIndex === 0) partial.bytes.set(datagram.subarray(0, headerBytes));
    partial.bytes.set(body, start);
    partial.received.add(header.fragmentIndex);
    if (partial.received.size !== header.fragmentCount) return null;
    this.partials.delete(key);
    const message = decodeMessage(partial.bytes);
    const run = this.run(header.runId);
    if (message.kind === 'description') {
      if (run.description && !descriptionSemanticallyEqual(run.description, message)) throw new Error('Description changed within an OSTS run');
      run.description = message;
      run.sequences.set(1, header.sequence);
      this.clearOlder(header);
      if (run.pendingState) {
        const pending = run.pendingState;
        run.pendingState = null;
        if (stateMatchesDescription(pending.state, message.description)) {
          run.state = pending;
          run.sequences.set(2, pending.header.sequence);
          this.clearOlder(pending.header);
        }
      }
    } else {
      if (!run.description) {
        if (!run.pendingState || message.header.sequence > run.pendingState.header.sequence) run.pendingState = message;
        return null;
      }
      if (!stateMatchesDescription(message.state, run.description.description)) throw new Error('OSTS state counts disagree with description');
      run.state = message;
      run.sequences.set(2, header.sequence);
      this.clearOlder(header);
    }
    if (!run.description || !run.state || run.retired) return null;
    const changedRun = this.activeId !== header.runId;
    if (changedRun) {
      // Map order is the first complete valid message, including not-yet-ready runs.
      for (const [id, earlier] of this.runs) {
        if (id === header.runId) break;
        earlier.retired = true;
      }
      this.activeId = header.runId;
    } else if (message.kind === 'description') return null;
    return { description: run.description, state: run.state, changedRun };
  }
}
