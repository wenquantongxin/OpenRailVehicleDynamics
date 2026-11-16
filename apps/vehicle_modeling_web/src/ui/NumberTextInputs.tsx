import type { KeyboardEvent } from 'react';

import { identifierLabel } from './identifier_display_labels.ts';

/** The input of a number, or the three of a vector, as texts. */
export function NumberTextInputs(props: {
  propertyName: string;
  texts: readonly string[];
  /** Whether each input's text differs from the value shown. */
  changed: readonly boolean[];
  unreadableAt: number | null;
  refused: boolean;
  readOnly: boolean;
  onText: (position: number, text: string) => void;
  onKeyDown?: (event: KeyboardEvent<HTMLInputElement>) => void;
}) {
  const label = identifierLabel(props.propertyName);
  return (
    <>
      {props.texts.map((text, position) => (
        <input
          key={position}
          type="text"
          inputMode="decimal"
          spellCheck={false}
          autoComplete="off"
          aria-label={props.texts.length === 1 ? label : `${label} [${position}]`}
          aria-invalid={props.unreadableAt === position || props.refused}
          className={props.changed[position] ? 'changed' : ''}
          readOnly={props.readOnly}
          value={text}
          onChange={(event) => props.onText(position, event.target.value)}
          onKeyDown={props.onKeyDown}
        />
      ))}
    </>
  );
}
