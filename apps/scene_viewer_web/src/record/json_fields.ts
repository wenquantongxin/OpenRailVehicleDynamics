/** Shape checks shared by the description and resource parsers; no I/O. */
export function object(value: unknown, what: string): Record<string, unknown> {
  if (value === null || typeof value !== 'object' || Array.isArray(value)) throw new Error(`${what} must be an object`);
  return value as Record<string, unknown>;
}
export function array(value: unknown, what: string): unknown[] {
  if (!Array.isArray(value)) throw new Error(`${what} must be an array`);
  return value;
}
export function number(value: unknown, what: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) throw new Error(`${what} must be finite`);
  return value;
}
export function integer(value: unknown, what: string): number {
  const parsed = number(value, what);
  if (!Number.isSafeInteger(parsed) || parsed < 0) throw new Error(`${what} must be a non-negative integer`);
  return parsed;
}
export function text(value: unknown, what: string): string {
  if (typeof value !== 'string') throw new Error(`${what} must be a string`);
  return value;
}
export function string(value: unknown, what: string): string {
  if (typeof value !== 'string' || value.trim() === '') throw new Error(`${what} must be a non-empty string`);
  return value;
}
export function nullableString(value: unknown, what: string): string | null {
  return value === null ? null : string(value, what);
}
export function triple(value: unknown, what: string): [number, number, number] {
  const entries = array(value, what);
  if (entries.length !== 3) throw new Error(`${what} must have three entries`);
  return [number(entries[0], what), number(entries[1], what), number(entries[2], what)];
}
export function exact(value: unknown, expected: string, what: string): string {
  if (value !== expected) throw new Error(`${what} must be '${expected}'`);
  return expected;
}
export function unique(names: string[], what: string): void {
  if (new Set(names).size !== names.length) throw new Error(`${what} contains duplicate names`);
}
