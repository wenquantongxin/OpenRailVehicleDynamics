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

export type DraftReading = { outcome: 'unchanged' } | { outcome: 'not_a_number'; position: number } | { outcome: 'changed'; values: number[] };

/** Reads the texts of a field's inputs against the values they show: one for a number, three for a vector. */
export function readDraft(texts: readonly string[], current: readonly number[]): DraftReading {
  const values = texts.map(numberFromText);
  const position = values.indexOf(null);
  if (position >= 0) {
    return { outcome: 'not_a_number', position };
  }
  return values.every((value, index) => value === current[index]) ? { outcome: 'unchanged' } : { outcome: 'changed', values: values as number[] };
}
