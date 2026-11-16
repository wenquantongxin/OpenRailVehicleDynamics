import { readDraft } from './number_entry.ts';

// One number field's rules, apart from its component so that they can be
// tested without a page: the texts its inputs show, which input does not hold
// a number, and whether a commit of it is on its way to the service. While a
// commit is on its way the inputs take no typing, Escape included, so that
// nothing typed is lost when the answer decides what the field shows.

export interface NumberFieldState {
  /** The texts entered and not yet accepted; null while the field shows its values. */
  draft: string[] | null;
  /** The input whose text a commit could not read as a number. */
  notANumberAt: number | null;
  /** A commit is on its way to the service. */
  sending: boolean;
}

export const restingField: NumberFieldState = { draft: null, notANumberAt: null, sending: false };

/** Whether the inputs take typing now. */
export function takesTyping(state: NumberFieldState): boolean {
  return !state.sending;
}

export function afterTyping(state: NumberFieldState, texts: string[]): NumberFieldState {
  return takesTyping(state) ? { ...state, draft: texts } : state;
}

/**
 * What a commit does: the values to send, if any; whether the field goes back
 * to its values because nothing changed; and the state meanwhile.
 */
export function afterCommit(state: NumberFieldState, current: readonly number[]): { state: NumberFieldState; send: number[] | null; reverted: boolean } {
  if (state.draft === null || state.sending) return { state, send: null, reverted: false };
  const reading = readDraft(state.draft, current);
  if (reading.outcome === 'not_a_number') return { state: { ...state, notANumberAt: reading.position }, send: null, reverted: false };
  if (reading.outcome === 'unchanged') return { state: restingField, send: null, reverted: true };
  return { state: { ...state, notANumberAt: null, sending: true }, send: reading.values, reverted: false };
}

/** The state once the service has answered the commit: an accepted value is shown as it is read back; a refused one stays. */
export function afterAnswer(state: NumberFieldState, accepted: boolean): NumberFieldState {
  return accepted ? restingField : { ...state, sending: false };
}
