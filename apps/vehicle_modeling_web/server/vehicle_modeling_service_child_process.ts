import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { createInterface } from 'node:readline';

// The modelling service as a child process. The service answers every input
// line with exactly one output line, in order, so lines are exchanged strictly
// one at a time: a request line is written only after the previous one has
// been answered, and the next line the service writes is the answer to the
// line just sent. Its standard error is passed through for people to read.

export class VehicleModelingServiceChildProcess {
  private readonly child;
  private pendingAnswer: ((line: string) => void) | null = null;
  private exchanges: Promise<unknown> = Promise.resolve();
  private state: 'running' | 'stopping' | 'ended' = 'running';

  /** onEnd is called once if the service ends or cannot be started, unless stop() asked for it. */
  constructor(servicePath: string, onEnd: (ending: string) => void) {
    this.child = spawn(servicePath, [], { stdio: ['pipe', 'pipe', 'inherit'] });
    const end = (ending: string): void => {
      if (this.state === 'running') {
        this.state = 'ended';
        onEnd(ending);
      }
    };
    this.child.on('error', (error) => end(`could not be run: ${error.message}`));
    this.child.on('exit', (code, signal) => end(signal === null ? `exited with code ${code}` : `was ended by ${signal}`));
    createInterface({ input: this.child.stdout }).on('line', (line) => {
      const answer = this.pendingAnswer;
      if (answer === null) {
        throw new Error(`the modelling service wrote a line no request asked for: ${line}`);
      }
      this.pendingAnswer = null;
      answer(line);
    });
  }

  /** Sends one request line once every earlier exchange is answered; resolves with its answer line. */
  exchange(requestLine: string): Promise<string> {
    const answered = this.exchanges.then(
      () =>
        new Promise<string>((resolve) => {
          this.pendingAnswer = resolve;
          this.child.stdin.write(`${requestLine}\n`);
        }),
    );
    this.exchanges = answered;
    return answered;
  }

  /** Ends the service the way the protocol provides: by ending its input. */
  async stop(): Promise<void> {
    if (this.state !== 'running') {
      return;
    }
    this.state = 'stopping';
    const exited = once(this.child, 'exit');
    this.child.stdin.end();
    await exited;
  }
}
