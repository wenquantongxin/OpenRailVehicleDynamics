import { useEffect, useRef } from 'react';

import { identifierLabel } from './identifier_display_labels.ts';

// The input of a true-or-false property: a checkbox taking a change the
// moment it is made. Undefined is a creation form's field with no choice yet,
// shown neither ticked nor clear.

export function BooleanCheckbox(props: { propertyName: string; value: boolean | null | undefined; disabled: boolean; refused: boolean; onChoose: (value: boolean) => void }) {
  const box = useRef<HTMLInputElement>(null);
  useEffect(() => {
    if (box.current !== null) box.current.indeterminate = props.value === undefined;
  }, [props.value]);
  return (
    <label className="boolean-field">
      <input
        ref={box}
        type="checkbox"
        aria-label={identifierLabel(props.propertyName)}
        aria-invalid={props.refused}
        checked={props.value === true}
        disabled={props.disabled}
        onChange={(event) => props.onChoose(event.target.checked)}
      />
      {props.value === undefined ? (
        <span className="not-given">
          not given <span className="zh">未填</span>
        </span>
      ) : (
        <span className="value">{String(props.value === true)}</span>
      )}
    </label>
  );
}
