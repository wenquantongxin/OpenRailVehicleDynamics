import { readAnswer, type ServiceAnswer } from './vehicle_modeling_service_protocol.ts';

/** Sends one command of the service protocol with its arguments and returns the service's answer. */
export type SendCommand = (command: string, commandArguments?: Readonly<Record<string, unknown>>) => Promise<ServiceAnswer>;

/**
 * Commands go to the modelling server at serverOrigin, which relays them to
 * the service one at a time. A refusal is an answer; an unreachable server or
 * an answer to another request is a failure and throws.
 */
export function modelingServerCommandChannel(serverOrigin: string): SendCommand {
  let nextId = 1;
  return async (command, commandArguments = {}) => {
    const id = nextId++;
    const response = await fetch(`${serverOrigin}/api/command`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ id, command, ...commandArguments }),
    });
    if (!response.ok) {
      throw new Error(`the modelling server answered '${command}' with HTTP ${response.status}`);
    }
    const answer: unknown = await response.json();
    // The service echoes the id; a different one means the relay crossed answers.
    if ((answer as { id?: unknown }).id !== id) {
      throw new Error(`the answer to '${command}' carries another request's id`);
    }
    return readAnswer(answer);
  };
}
