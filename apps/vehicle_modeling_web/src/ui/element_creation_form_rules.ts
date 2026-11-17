import { curveValueKind, type ElementKindDescription, type PropertyDescription, type PropertyReading, type Refusal } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditValue } from '../session/vehicle_editing_session.ts';
import { numberFromText } from './decimal_number_text_reading.ts';
import { readCurveEntry, type CurveEntry } from './editable_curve_table_rules.ts';
import type { ReferenceDescription } from './element_reference_picker_rules.ts';

// The creation form's rules, apart from its component so that they can be
// tested without a page. The form asks for the element's name and every
// property its kind lets be edited, each empty at first, apart from the
// properties the row's offer fixes: those are shown with their values, cannot
// be changed, and are always sent. Of the rest only what is filled in is
// sent: the service's defaults stand for it. The name is sent on its own, the
// other properties by name.

/**
 * What one field of the form holds, beside what its input needs to know of
 * the property. A choice not made yet is undefined; null names none.
 */
export type FormEntry =
  | { valueKind: 'number' | 'vector3'; texts: readonly string[] }
  | { valueKind: 'element_name'; text: string }
  | { valueKind: 'element_reference'; description: ReferenceDescription; chosen: string | null | undefined }
  | { valueKind: 'token'; choices: readonly string[]; chosen: string | undefined }
  | { valueKind: 'boolean'; chosen: boolean | undefined }
  | { valueKind: typeof curveValueKind; rowFields: readonly string[]; rows: CurveEntry };

/** The form's fields by property name. */
export type FormEntries = Readonly<Record<string, FormEntry>>;

/** The properties the offer fixes, by name, with their values. */
export type FixedProperties = Readonly<Record<string, unknown>>;

/** The properties the form shows: every one the kind lets be edited, the name included, in the kind's order; a fixed one is shown, not entered. */
export function formProperties(kind: ElementKindDescription): PropertyDescription[] {
  return kind.properties.filter((property) => property.editable);
}

/** A fixed property as the form shows it: with its value, and not editable. */
export function fixedPropertyReading(property: PropertyDescription, fixed: FixedProperties): PropertyReading {
  return { ...property, editable: false, value: fixed[property.name] } as PropertyReading;
}

function blankEntry(property: PropertyDescription): FormEntry {
  switch (property.valueKind) {
    case 'number':
      return { valueKind: 'number', texts: [''] };
    case 'vector3':
      return { valueKind: 'vector3', texts: ['', '', ''] };
    case 'element_name':
      return { valueKind: 'element_name', text: '' };
    case 'element_reference':
      return { valueKind: 'element_reference', description: property, chosen: undefined };
    case 'token':
      return { valueKind: 'token', choices: property.choices, chosen: undefined };
    case 'boolean':
      return { valueKind: 'boolean', chosen: undefined };
    case curveValueKind:
      return { valueKind: curveValueKind, rowFields: property.rowFields, rows: [] };
    case 'rotation':
    case 'text':
      throw new Error(`${property.name}: a ${property.valueKind} is described as editable, which the creation form cannot enter`);
  }
}

export function blankEntries(kind: ElementKindDescription, fixed: FixedProperties): FormEntries {
  return Object.fromEntries(formProperties(kind).filter((property) => !(property.name in fixed)).map((property) => [property.name, blankEntry(property)]));
}

export type FormReading =
  /** A field holds a text that is no number: the property and its input, counted as the field counts them. */
  | { outcome: 'unreadable'; property: string; at: number }
  | { outcome: 'ready'; name: string; properties: Record<string, EditValue> };

/** What add_element is sent for the form as it is filled in: the fixed properties, and what is entered. */
export function readCreationForm(kind: ElementKindDescription, fixed: FixedProperties, entries: FormEntries): FormReading {
  let name = '';
  const properties: Record<string, EditValue> = { ...(fixed as Record<string, EditValue>) };
  for (const property of formProperties(kind)) {
    if (property.name in fixed) continue;
    const entry = entries[property.name]!;
    switch (entry.valueKind) {
      case 'element_name':
        name = entry.text;
        break;
      case 'number':
      case 'vector3': {
        if (entry.texts.every((text) => text.trim() === '')) break;
        const values = entry.texts.map(numberFromText);
        const at = values.indexOf(null);
        if (at >= 0) return { outcome: 'unreadable', property: property.name, at };
        properties[property.name] = entry.valueKind === 'number' ? values[0]! : (values as [number, number, number]);
        break;
      }
      case 'element_reference':
      case 'token':
      case 'boolean':
        if (entry.chosen !== undefined) properties[property.name] = entry.chosen;
        break;
      case curveValueKind: {
        if (entry.rows.length === 0) break;
        const reading = readCurveEntry(entry.rows, entry.rowFields, []);
        if (reading.outcome === 'unreadable') return { outcome: 'unreadable', property: property.name, at: reading.at };
        if (reading.outcome === 'changed') properties[property.name] = reading.value;
        break;
      }
    }
  }
  return { outcome: 'ready', name, properties };
}

/** The field a refusal of the creation is shown at; null shows it at the form. */
export function refusedField(kind: ElementKindDescription, refusal: Refusal): string | null {
  return formProperties(kind).some((property) => property.name === refusal.property) ? refusal.property : null;
}
