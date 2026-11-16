import type { ElementReference, PropertyDescription } from '../service/vehicle_modeling_service_protocol.ts';

// What a reference field offers, apart from its component so that the rule
// can be tested without a page: the outline's elements of the kinds the
// property may name, filtered by what is typed, and none where the property is
// optional. A vehicle has well over a hundred frames, so typing narrows them.

export type ReferenceDescription = Extract<PropertyDescription, { valueKind: 'element_reference' }>;

/**
 * The choices for a filter text, in outline order; null stands for naming
 * none. None comes first, and only while nothing is typed, so that what is
 * typed always leads to an element.
 */
export function referenceChoices(elements: readonly ElementReference[], description: ReferenceDescription, filter: string): (ElementReference | null)[] {
  const needle = filter.trim().toLowerCase();
  const named = elements.filter((element) => description.referencedKinds.includes(element.kind) && element.name.toLowerCase().includes(needle));
  return description.optional && needle === '' ? [null, ...named] : named;
}

/** Whether a choice needs its kind beside its name: where the property may name more than one kind. */
export function choicesShowKind(description: ReferenceDescription): boolean {
  return description.referencedKinds.length > 1;
}
