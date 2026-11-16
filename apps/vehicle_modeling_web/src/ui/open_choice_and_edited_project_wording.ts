import type { OpenChoice, SessionState } from '../session/vehicle_editing_session.ts';

// How the interface's sentences name what can be opened and what is being
// edited: a reference vehicle by its catalogue name, a project by its own.

export interface BilingualText {
  en: string;
  zh: string;
}

/** The name alone, for a short label. */
export function openChoiceName(choice: OpenChoice): string {
  return choice.source === 'reference_vehicle' ? choice.vehicle.name : choice.name;
}

/** The choice in a sentence, saying whether it is a reference vehicle or a project. */
export function openChoiceInSentence(choice: OpenChoice): BilingualText {
  const name = openChoiceName(choice);
  return choice.source === 'reference_vehicle' ? { en: `the reference vehicle ${name}`, zh: `参考车辆 ${name}` } : { en: `the project ${name}`, zh: `工程 ${name}` };
}

/** What the session edits, in a sentence: the project when the session has a file, otherwise the vehicle by the outline's name. */
export function editedProjectInSentence(state: SessionState): BilingualText {
  if (state.vehicleProjectFile !== null) {
    return { en: `the project ${state.vehicleProjectFile.name}`, zh: `工程 ${state.vehicleProjectFile.name}` };
  }
  const name = state.outline?.vehicleName ?? '';
  return { en: `the vehicle ${name}`, zh: `车辆 ${name}` };
}
