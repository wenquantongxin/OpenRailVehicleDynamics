import type { OpenChoice, SessionState } from '../session/vehicle_editing_session.ts';

// How the interface's sentences name what can be opened and what is being
// edited: a starting point and a project by their names; what the session
// edits is a project, a new one until it is saved to a file.

export interface BilingualText {
  en: string;
  zh: string;
}

/** The name alone, for a short label. */
export function openChoiceName(choice: OpenChoice): string {
  return choice.source === 'starting_point' ? choice.startingPoint.name : choice.name;
}

/** The choice in a sentence, saying whether it is a starting point or a project. */
export function openChoiceInSentence(choice: OpenChoice): BilingualText {
  const name = openChoiceName(choice);
  return choice.source === 'starting_point' ? { en: `the starting point ${name}`, zh: `起点 ${name}` } : { en: `the project ${name}`, zh: `工程 ${name}` };
}

/** What the session edits, in a sentence: the project of its file, otherwise the new project by the outline's name. */
export function editedProjectInSentence(state: SessionState): BilingualText {
  if (state.vehicleProjectFile !== null) {
    return { en: `the project ${state.vehicleProjectFile.name}`, zh: `工程 ${state.vehicleProjectFile.name}` };
  }
  const name = state.outline?.vehicleName ?? '';
  return { en: `the new project ${name}`, zh: `新工程 ${name}` };
}
