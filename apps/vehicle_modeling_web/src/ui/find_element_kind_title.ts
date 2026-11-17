import type { BilingualTitle, ElementKindDescription } from '../service/vehicle_modeling_service_protocol.ts';

/** A kind's name for people, as describe_element_kinds gives it; every kind the service names, it describes. */
export function elementKindTitle(kinds: readonly ElementKindDescription[], kind: string): BilingualTitle {
  const described = kinds.find((each) => each.kind === kind);
  if (described === undefined) {
    throw new Error(`the modelling service names the kind '${kind}', which it does not describe`);
  }
  return described.title;
}
