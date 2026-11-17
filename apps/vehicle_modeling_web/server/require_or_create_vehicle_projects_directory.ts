import { mkdir, stat } from 'node:fs/promises';

import type { TerminalConversation } from './terminal_conversation.ts';

// The folder of projects, settled before the server starts anything. An
// existing directory is taken as it is. A missing folder is created, parents
// included, when the person who started the server is asked once at its
// terminal and answers yes; any other answer, the end of the input included,
// or no terminal to ask at, refuses it and creates nothing. A path that exists
// and is not a directory is refused without a question.

/** Whether an answer is yes: y or yes, in any case. */
function answersYes(answer: string | null): boolean {
  return answer !== null && /^(y|yes)$/i.test(answer.trim());
}

export async function requireOrCreateVehicleProjectsDirectory(path: string, terminal: TerminalConversation | null): Promise<void> {
  const refusal = new Error(`the folder of vehicle projects '${path}' is not an existing directory`);
  const found = await stat(path).catch((error: NodeJS.ErrnoException) => (error.code === 'ENOENT' ? null : 'unusable'));
  if (found === 'unusable' || (found !== null && !found.isDirectory())) throw refusal;
  if (found !== null) return;
  if (terminal === null) throw refusal;
  const answer = await terminal.ask(`The folder of vehicle projects '${path}' does not exist. Create it? 工程文件夹不存在，是否新建？ [y/N] `);
  if (!answersYes(answer)) throw refusal;
  await mkdir(path, { recursive: true });
  terminal.tell(`Created the folder of vehicle projects '${path}'. 已新建工程文件夹。`);
}
