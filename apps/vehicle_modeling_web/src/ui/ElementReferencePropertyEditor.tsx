import type { ElementKindDescription, ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { ChoicePropertyEditorProps } from './choice_property_editor_props.ts';
import type { ReferenceDescription } from './element_reference_picker_rules.ts';
import { ElementReferencePicker } from './ElementReferencePicker.tsx';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { useChoiceCommit } from './property_field_commit_hooks.ts';

// The selected subject's reference to an element. A choice is committed
// the moment it is made; while it is on its way the field shows it and takes
// no other, and neither does the field of an element whose rename is on its
// way. A refused choice leaves the reference as it was and shows the
// service's reason.

export function ElementReferencePropertyEditor(
  props: ChoicePropertyEditorProps & { description: ReferenceDescription; value: string | null; elements: readonly ElementReference[]; kinds: readonly ElementKindDescription[] },
) {
  const { pending, choose } = useChoiceCommit<string | null>((name) => props.onCommit(props.subject, props.description.name, name));
  return (
    <div className="choice-field">
      <ElementReferencePicker
        description={props.description}
        elements={props.elements}
        kinds={props.kinds}
        value={pending === null ? props.value : pending.value}
        disabled={pending !== null || props.frozen}
        refused={props.refusal !== null}
        onChoose={choose}
      />
      {props.refusal !== null && <FieldRefusalMessage message={props.refusal.message} />}
    </div>
  );
}
