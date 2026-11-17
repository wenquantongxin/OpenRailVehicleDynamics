import { createInterface } from 'node:readline';

// A conversation with the person who started the server, at the terminal it
// runs in: a question asked and answered by one line, and a line told. There
// is none when standard input or standard output is not a terminal, so that a
// script or a test never waits for an answer.

export interface TerminalConversation {
  /** Asks once; the line answered, or null when the input ends, or Ctrl+C is pressed, before an answer. */
  ask(question: string): Promise<string | null>;
  tell(line: string): void;
}

export function terminalConversation(): TerminalConversation | null {
  if (process.stdin.isTTY !== true || process.stdout.isTTY !== true) return null;
  return {
    ask: (question) =>
      new Promise((answered) => {
        const lines = createInterface({ input: process.stdin, output: process.stdout });
        let answer: string | null = null;
        lines.on('SIGINT', () => lines.close());
        lines.question(question, (line) => {
          answer = line;
          lines.close();
        });
        lines.once('close', () => {
          // Without an answer the cursor still stands after the question.
          if (answer === null) process.stdout.write('\n');
          answered(answer);
        });
      }),
    tell: (line) => {
      process.stdout.write(`${line}\n`);
    },
  };
}
