import type { ChoicePropertyEditorProps } from './choice_property_editor_props.ts';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { useChoiceCommit } from './property_field_commit_hooks.ts';
import { TokenChoiceSelect } from './TokenChoiceSelect.tsx';

// The selected subject's token, one of the choices the service names. A
// choice is committed the moment it is made; while it is on its way the field
// shows it and takes no other, and neither does the field of an element whose
// rename is on its way. A refused choice leaves the token as it was and shows
// the service's reason.

export function TokenPropertyEditor(props: ChoicePropertyEditorProps & { propertyName: string; choices: readonly string[]; value: string | null }) {
  const { pending, choose } = useChoiceCommit<string>((token) => props.onCommit(props.subject, props.propertyName, token));
  return (
    <div className="choice-field">
      <TokenChoiceSelect
        propertyName={props.propertyName}
        choices={props.choices}
        value={pending === null ? props.value : pending.value}
        disabled={pending !== null || props.frozen}
        refused={props.refusal !== null}
        onChoose={choose}
      />
      {props.refusal !== null && <FieldRefusalMessage message={props.refusal.message} />}
    </div>
  );
}
