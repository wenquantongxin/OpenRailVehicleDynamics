import type { EntryReading } from './typed_property_field_commit_rules.ts';

// What a number field's text means. A field shows a number with its full
// precision, String(value), and accepts a decimal number in the notation
// JavaScript and JSON share, scientific notation included. Whether a value is
// physically sound is the service's to judge, not the field's.

const decimalNumber = /^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$/;

/** The finite number the text states, or null when it states none. */
export function numberFromText(text: string): number | null {
  const trimmed = text.trim();
  if (!decimalNumber.test(trimmed)) {
    return null;
  }
  const value = Number(trimmed);
  return Number.isFinite(value) ? value : null;
}

/** Reads the texts of a field's inputs against the values they show: one for a number, three for a vector. */
export function readNumberEntry(texts: readonly string[], current: readonly number[]): EntryReading<number[]> {
  const values = texts.map(numberFromText);
  const at = values.indexOf(null);
  if (at >= 0) {
    return { outcome: 'unreadable', at };
  }
  return values.every((value, index) => value === current[index]) ? { outcome: 'unchanged' } : { outcome: 'changed', value: values as number[] };
}
