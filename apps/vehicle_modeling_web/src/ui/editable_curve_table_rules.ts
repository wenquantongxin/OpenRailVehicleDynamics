import type { CurvePoint } from '../service/vehicle_modeling_service_protocol.ts';
import { numberFromText } from './decimal_number_text_reading.ts';
import type { EntryReading } from './typed_property_field_commit_rules.ts';

// What a curve table holds while it is edited: a text for each of the
// property's row fields in each row. Rows are added at the end and removed
// anywhere, the first row of an empty curve included, and the whole curve is
// read at once, so that it is committed as one edit.

/** The texts of a curve table, a row of texts per curve row, in the order of the row fields. */
export type CurveEntry = readonly (readonly string[])[];

/** The texts a curve shows: every number with its full precision. */
export function curveEntryOf(curve: readonly CurvePoint[], rowFields: readonly string[]): CurveEntry {
  return curve.map((row) => rowFields.map((rowField) => String(row[rowField])));
}

export function withRowAdded(entry: CurveEntry, rowFields: readonly string[]): CurveEntry {
  return [...entry, rowFields.map(() => '')];
}

export function withRowRemoved(entry: CurveEntry, row: number): CurveEntry {
  return entry.filter((_, index) => index !== row);
}

export function withCellText(entry: CurveEntry, row: number, column: number, text: string): CurveEntry {
  return entry.map((texts, rowIndex) => (rowIndex === row ? texts.map((each, columnIndex) => (columnIndex === column ? text : each)) : texts));
}

/**
 * Reads the whole table against the curve it shows: a text that is no number
 * is pointed at by its place counted row by row, and a table that states the
 * curve shown is unchanged.
 */
export function readCurveEntry(entry: CurveEntry, rowFields: readonly string[], current: readonly CurvePoint[]): EntryReading<CurvePoint[]> {
  const rows: CurvePoint[] = [];
  for (const [rowIndex, texts] of entry.entries()) {
    const values = texts.map(numberFromText);
    const column = values.indexOf(null);
    if (column >= 0) return { outcome: 'unreadable', at: rowIndex * rowFields.length + column };
    rows.push(Object.fromEntries(rowFields.map((rowField, index) => [rowField, values[index]!])));
  }
  const unchanged = rows.length === current.length && rows.every((row, index) => rowFields.every((rowField) => row[rowField] === current[index]![rowField]));
  return unchanged ? { outcome: 'unchanged' } : { outcome: 'changed', value: rows };
}
