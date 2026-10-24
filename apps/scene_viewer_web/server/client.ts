import type { StreamUpdate } from '../src/stream/receiver.ts';

export interface BinarySender {
  readonly readyState: number;
  readonly bufferedAmount: number;
  send(bytes: Uint8Array, callback: (error?: Error) => void): void;
  terminate(): void;
}
/** One in-flight send plus one replaceable latest state; descriptions precede their states. */
export class StreamClient {
  private pending: StreamUpdate | null = null;
  private writing = false;
  private descriptionRun: bigint | null = null;
  private socket: BinarySender;
  constructor(socket: BinarySender) { this.socket = socket; }
  publish(update: StreamUpdate): void { this.pending = update; this.flush(); }
  private flush(): void {
    if (this.writing || this.socket.readyState !== 1 || !this.pending) return;
    if (this.socket.bufferedAmount > 2 * 1024 * 1024) { this.pending = null; this.socket.terminate(); return; }
    const update = this.pending;
    const needsDescription = this.descriptionRun !== update.description.header.runId;
    const bytes = needsDescription ? update.description.bytes : update.state.bytes;
    if (!needsDescription) this.pending = null;
    this.writing = true;
    this.socket.send(bytes, (error) => {
      this.writing = false;
      if (error) { this.pending = null; this.socket.terminate(); return; }
      if (needsDescription) this.descriptionRun = update.description.header.runId;
      this.flush();
    });
  }
}
