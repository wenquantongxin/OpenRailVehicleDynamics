import { BooleanCheckbox } from './BooleanCheckbox.tsx';
import type { ChoicePropertyEditorProps } from './choice_property_editor_props.ts';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { useChoiceCommit } from './property_field_commit_hooks.ts';

// The selected subject's true-or-false property. A change is committed the
// moment it is made; while it is on its way the box shows it and takes no
// other, and neither does the box of an element whose rename is on its way. A
// refused change leaves the value as it was and shows the service's reason.

export function BooleanPropertyEditor(props: ChoicePropertyEditorProps & { propertyName: string; value: boolean | null }) {
  const { pending, choose } = useChoiceCommit<boolean>((value) => props.onCommit(props.subject, props.propertyName, value));
  return (
    <div className="choice-field">
      <BooleanCheckbox
        propertyName={props.propertyName}
        value={pending === null ? props.value : pending.value}
        disabled={pending !== null || props.frozen}
        refused={props.refusal !== null}
        onChoose={choose}
      />
      {props.refusal !== null && <FieldRefusalMessage message={props.refusal.message} />}
    </div>
  );
}
