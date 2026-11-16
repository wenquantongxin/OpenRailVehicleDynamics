import { useRef, useState } from 'react';

import { afterAnswer, afterCommit, afterTyping, restingField, takesTyping, type EntryReading, type FieldState } from './typed_property_field_commit_rules.ts';

// The two ways a property panel field commits to the service. A typed field
// commits what is typed on Enter or when focus leaves it, and Escape reverts
// it. A choice commits as soon as it is made.

export interface TypedFieldCommit<Entry> {
  field: FieldState<Entry>;
  /** Takes typing as a change of the latest entry, null while the field shows its value. */
  type: (change: (entry: Entry | null) => Entry) => void;
  commit: () => Promise<void>;
  revert: () => void;
}

export function useTypedFieldCommit<Entry, Value>(options: {
  /** The refused value's entry, when the field is shown after its edit was refused, so that it can be corrected. */
  refusedEntry: Entry | null;
  read: (entry: Entry) => EntryReading<Value>;
  send: (value: Value) => Promise<boolean>;
  /** Called when the field goes back to its value, such as to forget a refusal. */
  onRevert: () => void;
}): TypedFieldCommit<Entry> {
  const [field, showField] = useState<FieldState<Entry>>(() => (options.refusedEntry === null ? restingField : { ...restingField, entry: options.refusedEntry }));
  // Every event reads the field as the last one left it, rendered yet or not.
  const latest = useRef(field);
  const setField = (next: FieldState<Entry>): void => {
    latest.current = next;
    showField(next);
  };
  const revert = (): void => {
    if (!takesTyping(latest.current)) return;
    setField(restingField);
    options.onRevert();
  };
  return {
    field,
    type: (change) => setField(afterTyping(latest.current, change(latest.current.entry))),
    commit: async () => {
      const step = afterCommit(latest.current, options.read);
      if (step.reverted) {
        revert();
        return;
      }
      setField(step.state);
      if (step.send === null) return;
      const accepted = await options.send(step.send.value);
      setField(afterAnswer(latest.current, accepted));
    },
    revert,
  };
}

/** A choice made and on its way to the service, shown meanwhile; no other is taken until the answer. */
export function useChoiceCommit<Value>(send: (value: Value) => Promise<boolean>): { pending: { value: Value } | null; choose: (value: Value) => void } {
  const [pending, setPending] = useState<{ value: Value } | null>(null);
  const sending = useRef(false);
  return {
    pending,
    choose: (value) => {
      if (sending.current) return;
      sending.current = true;
      setPending({ value });
      void send(value).finally(() => {
        sending.current = false;
        setPending(null);
      });
    },
  };
}
