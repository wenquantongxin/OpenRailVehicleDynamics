import type { KeyboardEvent } from 'react';

import { angleInputLabel, rollPitchYawCaptions } from './roll_pitch_yaw_angle_captions.ts';
import { RollPitchYawAngleCaption } from './RollPitchYawAngleCaption.tsx';

/**
 * The line saying how the three angles of a rotation turn, and their inputs,
 * as texts, one to a line after its caption. Rendered inside the angle field,
 * whose messages follow the inputs.
 */
export function RollPitchYawAngleInputs(props: {
  texts: readonly string[];
  /** Whether each input's text differs from the angle shown. */
  changed: readonly boolean[];
  unreadableAt: number | null;
  refused: boolean;
  readOnly: boolean;
  onText: (position: number, text: string) => void;
  onKeyDown?: (event: KeyboardEvent<HTMLInputElement>) => void;
}) {
  return (
    <>
      <p className="angle-order-note">
        Turned about the fixed axes: x first, then y, then z. <span className="zh">绕固定轴转动：先 x，再 y，后 z。</span>
      </p>
      {rollPitchYawCaptions.map((caption, position) => (
        <label className="angle-field" key={caption.axis}>
          <RollPitchYawAngleCaption caption={caption} />
          <input
            type="text"
            inputMode="decimal"
            spellCheck={false}
            autoComplete="off"
            aria-label={angleInputLabel(caption)}
            aria-invalid={props.unreadableAt === position || props.refused}
            className={props.changed[position] ? 'changed' : ''}
            readOnly={props.readOnly}
            value={props.texts[position]}
            onChange={(event) => props.onText(position, event.target.value)}
            onKeyDown={props.onKeyDown}
          />
        </label>
      ))}
    </>
  );
}
