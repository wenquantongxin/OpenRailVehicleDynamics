// The rules of a field that commits what is typed into it to the service,
// apart from its component so that they can be tested without a page: what is
// entered and not yet accepted, which input a commit could not read, and
// whether a commit is on its way to the service. While a commit is on its way
// the inputs take no typing, Escape included, so that nothing typed is lost
// when the answer decides what the field shows. What an entry means is the
// field's own: numbers, a name, the rows of a curve.

/** How a commit reads an entry against the value the field shows. */
export type EntryReading<Value> = { outcome: 'unchanged' } | { outcome: 'unreadable'; at: number } | { outcome: 'changed'; value: Value };

export interface FieldState<Entry> {
  /** What is entered and not yet accepted; null while the field shows its value. */
  entry: Entry | null;
  /** The input whose text a commit could not read. */
  unreadableAt: number | null;
  /** A commit is on its way to the service. */
  sending: boolean;
}

export const restingField: FieldState<never> = { entry: null, unreadableAt: null, sending: false };

/** Whether the inputs take typing now. */
export function takesTyping(state: FieldState<unknown>): boolean {
  return !state.sending;
}

export function afterTyping<Entry>(state: FieldState<Entry>, entry: Entry): FieldState<Entry> {
  return takesTyping(state) ? { ...state, entry } : state;
}

/**
 * What a commit does: the value to send, if any; whether the field goes back
 * to its value because nothing changed; and the state meanwhile.
 */
export function afterCommit<Entry, Value>(
  state: FieldState<Entry>,
  read: (entry: Entry) => EntryReading<Value>,
): { state: FieldState<Entry>; send: { value: Value } | null; reverted: boolean } {
  if (state.entry === null || state.sending) return { state, send: null, reverted: false };
  const reading = read(state.entry);
  if (reading.outcome === 'unreadable') return { state: { ...state, unreadableAt: reading.at }, send: null, reverted: false };
  if (reading.outcome === 'unchanged') return { state: restingField, send: null, reverted: true };
  return { state: { ...state, unreadableAt: null, sending: true }, send: { value: reading.value }, reverted: false };
}

/** The state once the service has answered the commit: an accepted value is shown as it is read back; a refused one stays. */
export function afterAnswer<Entry>(state: FieldState<Entry>, accepted: boolean): FieldState<Entry> {
  return accepted ? restingField : { ...state, sending: false };
}
