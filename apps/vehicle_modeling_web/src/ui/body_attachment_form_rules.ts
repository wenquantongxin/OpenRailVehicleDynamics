import { curveValueKind, subjectElement, type AttachmentDescription, type ElementKindDescription, type PropertyReading, type Refusal } from '../service/vehicle_modeling_service_protocol.ts';
import { curveEntryOf } from './editable_curve_table_rules.ts';
import { blankEntries, type FormEntries, type FormEntry } from './element_creation_form_rules.ts';

// The rules of the fields that state how a body is attached, apart from their
// components so that they can be tested without a page. A body moves freely,
// or hangs on a joint of one of the kinds its attachment names, whose fields
// are its kind's: its end on the body is asked for only when the body exists
// already, for a new body is that end itself. The body's properties only a
// free body has are asked for only while free is chosen. A joint that takes
// another's place starts from the other's values wherever its kind has a
// property of the same name and value kind, the name included, and a form
// that changes from one joint kind to another keeps what was entered the same
// way.

/** What a body is to be: moving freely, or held by a joint of a kind. */
export type AttachmentChoice = { free: true } | { jointKind: string };

/** The choices offered, in order: free when it is offered, then each joint kind when one is. */
export function attachmentChoices(description: AttachmentDescription, offersFree: boolean, offersJoint: boolean): AttachmentChoice[] {
  return [...(offersFree ? [{ free: true } as const] : []), ...(offersJoint ? description.jointKinds.map((jointKind) => ({ jointKind })) : [])];
}

export function sameAttachmentChoice(a: AttachmentChoice | null, b: AttachmentChoice): boolean {
  return a !== null && ('free' in a ? 'free' in b : 'jointKind' in b && a.jointKind === b.jointKind);
}

/** The body's kind as its creation asks for it: without the properties only a free body has, unless free is chosen. */
export function bodyKindAsked(kind: ElementKindDescription, description: AttachmentDescription, choice: AttachmentChoice): ElementKindDescription {
  return 'free' in choice ? kind : { ...kind, properties: kind.properties.filter((property) => !description.freeOnlyProperties.includes(property.name)) };
}

/** A joint kind as its fields ask for it: without its end on the body for a body not created yet. */
export function jointKindAsked(jointKind: ElementKindDescription, description: AttachmentDescription, asksBodyEnd: boolean): ElementKindDescription {
  return asksBodyEnd ? jointKind : { ...jointKind, properties: jointKind.properties.filter((property) => property.name !== description.bodyEndProperty) };
}

/** A blank field holding what another field holds, where the two hold the same kind of value. */
function carriedEntry(blank: FormEntry, kept: FormEntry): FormEntry {
  switch (blank.valueKind) {
    case 'number':
    case 'vector3':
      return kept.valueKind === blank.valueKind ? { ...blank, texts: kept.texts } : blank;
    case 'element_name':
      return kept.valueKind === 'element_name' ? { ...blank, text: kept.text } : blank;
    case 'element_reference':
      return kept.valueKind === 'element_reference' ? { ...blank, chosen: kept.chosen } : blank;
    case 'token':
      return kept.valueKind === 'token' && kept.chosen !== undefined && blank.choices.includes(kept.chosen) ? { ...blank, chosen: kept.chosen } : blank;
    case curveValueKind:
      return kept.valueKind === curveValueKind && kept.rowFields.join() === blank.rowFields.join() ? { ...blank, rows: kept.rows } : blank;
  }
}

/** A value read, as the field of its value kind would hold it; null for a value no field holds. */
function entryOfReading(reading: PropertyReading): FormEntry | null {
  switch (reading.valueKind) {
    case 'number':
      return reading.value === null ? null : { valueKind: 'number', texts: [String(reading.value)] };
    case 'vector3':
      return reading.value === null ? null : { valueKind: 'vector3', texts: reading.value.map(String) };
    case 'element_name':
      return reading.value === null ? null : { valueKind: 'element_name', text: reading.value };
    case 'element_reference':
      return { valueKind: 'element_reference', description: reading, chosen: reading.value };
    case 'token':
      return reading.value === null ? null : { valueKind: 'token', choices: reading.choices, chosen: reading.value };
    case curveValueKind:
      return reading.value === null ? null : { valueKind: curveValueKind, rowFields: reading.rowFields, rows: curveEntryOf(reading.value, reading.rowFields) };
    case 'rotation':
    case 'text':
      return null;
  }
}

/** The fields of a kind, starting from another element's values wherever the kind has a property of the same name and value kind. */
export function entriesStartingFrom(kind: ElementKindDescription, readings: readonly PropertyReading[]): FormEntries {
  return Object.fromEntries(
    Object.entries(blankEntries(kind, {})).map(([name, blank]) => {
      const reading = readings.find((each) => each.name === name && each.valueKind === blank.valueKind);
      const held = reading === undefined ? null : entryOfReading(reading);
      return [name, held === null ? blank : carriedEntry(blank, held)];
    }),
  );
}

/** The fields of another kind, keeping what was entered wherever it has a property of the same name and value kind. */
export function entriesCarriedOver(kind: ElementKindDescription, entries: FormEntries): FormEntries {
  return Object.fromEntries(
    Object.entries(blankEntries(kind, {})).map(([name, blank]) => {
      const kept = entries[name];
      return [name, kept === undefined ? blank : carriedEntry(blank, kept)];
    }),
  );
}

/** The field's entry with a choice made, where it is a reference with no choice made yet. */
export function withReferenceChosen(entries: FormEntries, property: string, chosen: string): FormEntries {
  const entry = entries[property];
  return entry?.valueKind === 'element_reference' && entry.chosen === undefined ? { ...entries, [property]: { ...entry, chosen } } : entries;
}

/** Whether a refusal is about the joint of the kind given, rather than the body: shown at the joint's field it names, or with the attachment. */
export function refusalAboutJoint(refusal: Refusal, jointKind: string | null): boolean {
  return jointKind !== null && subjectElement(refusal.subject)?.kind === jointKind;
}
