import type { BilingualTitle } from '../service/vehicle_modeling_service_protocol.ts';

/** A kind's English title inside a sentence: its first letter in lower case, as a common noun reads there. */
export function kindTitleInASentence(title: BilingualTitle): string {
  return title.en.charAt(0).toLowerCase() + title.en.slice(1);
}
