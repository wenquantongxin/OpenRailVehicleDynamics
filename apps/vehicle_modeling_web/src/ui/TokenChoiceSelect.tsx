import { identifierLabel } from './identifier_display_labels.ts';
import { shownChoice } from './token_choice_select_rules.ts';

// The input of a token: a select of the choices the service names, taking a
// choice the moment it is made. A value that is none of the choices shows no
// choice; undefined is a creation form's field with no choice yet.

export function TokenChoiceSelect(props: {
  propertyName: string;
  choices: readonly string[];
  value: string | null | undefined;
  disabled: boolean;
  refused: boolean;
  onChoose: (token: string) => void;
}) {
  const shown = shownChoice(props.value, props.choices);
  return (
    <select
      className={`token-select ${shown === undefined ? 'unchosen' : ''}`}
      aria-label={identifierLabel(props.propertyName)}
      aria-invalid={props.refused}
      value={shown ?? ''}
      disabled={props.disabled}
      onChange={(event) => props.onChoose(event.target.value)}
    >
      {shown === undefined && (
        <option value="" disabled>
          Choose · 选择
        </option>
      )}
      {props.choices.map((choice) => (
        <option key={choice} value={choice}>
          {choice}
        </option>
      ))}
    </select>
  );
}
