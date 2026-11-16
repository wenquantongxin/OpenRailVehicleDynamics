import type { ElementReference, Vector3 } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';
import { readNumberEntry } from './decimal_number_text_reading.ts';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { NumberTextInputs } from './NumberTextInputs.tsx';
import { useTypedFieldCommit } from './property_field_commit_hooks.ts';
import { takesTyping } from './typed_property_field_commit_rules.ts';
import { UnreadableNumberMessage } from './UnreadableNumberMessage.tsx';

// An editable number or vector of the selected element. It commits on Enter
// or when focus leaves the field, and nothing is sent when its value did not
// change. A refused edit keeps the text entered and shows the service's reason.

export function NumberAndVectorPropertyEditor(props: {
  element: ElementReference;
  propertyName: string;
  /** One number, or the three of a vector. */
  current: readonly number[];
  refusal: EditRefusal | null;
  /** Takes no input: the element's rename is on its way. */
  frozen: boolean;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
}) {
  const { element, propertyName, current, refusal } = props;
  const shown = current.map(String);
  const { field, type, commit, revert } = useTypedFieldCommit<readonly string[], number[]>({
    // A field shown again after its edit was refused starts from the refused value.
    refusedEntry: refusal === null ? null : [refusal.value].flat().map(String),
    read: (texts) => readNumberEntry(texts, current),
    send: (values) => props.onCommit(element, propertyName, current.length === 1 ? values[0]! : (values as Vector3)),
    onRevert: () => {
      if (refusal !== null) props.onDismissRefusal();
    },
  });
  const texts = field.entry ?? shown;
  return (
    <div
      className={`number-fields count-${texts.length}`}
      onBlur={(event) => {
        // Moving between a vector's own inputs is not leaving the field.
        if (!event.currentTarget.contains(event.relatedTarget as Node | null)) void commit();
      }}
    >
      <NumberTextInputs
        propertyName={propertyName}
        texts={texts}
        changed={texts.map((text, position) => field.entry !== null && text !== shown[position])}
        unreadableAt={field.unreadableAt}
        refused={refusal !== null}
        readOnly={!takesTyping(field) || props.frozen}
        onText={(position, text) => type((entry) => (entry ?? shown).map((each, index) => (index === position ? text : each)))}
        onKeyDown={(event) => {
          if (event.key === 'Enter') void commit();
          else if (event.key === 'Escape') revert();
        }}
      />
      {field.unreadableAt !== null && <UnreadableNumberMessage />}
      {refusal !== null && <FieldRefusalMessage message={refusal.message} />}
    </div>
  );
}
