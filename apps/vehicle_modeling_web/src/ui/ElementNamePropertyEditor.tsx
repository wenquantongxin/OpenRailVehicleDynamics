import type { ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { identifierLabel } from './identifier_display_labels.ts';
import { useTypedFieldCommit } from './property_field_commit_hooks.ts';
import { takesTyping } from './typed_property_field_commit_rules.ts';

// The selected element's own name. Committing a new one renames the element,
// and the service updates everything that names it; the selection follows it.
// Whether a name may be taken is the service's to judge.

export function ElementNamePropertyEditor(props: {
  element: ElementReference;
  propertyName: string;
  current: string;
  refusal: EditRefusal | null;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
}) {
  const { element, propertyName, current, refusal } = props;
  const { field, type, commit, revert } = useTypedFieldCommit<string, string>({
    refusedEntry: refusal === null ? null : String(refusal.value),
    read: (text) => (text === current ? { outcome: 'unchanged' } : { outcome: 'changed', value: text }),
    send: (name) => props.onCommit(element, propertyName, name),
    onRevert: () => {
      if (refusal !== null) props.onDismissRefusal();
    },
  });
  const text = field.entry ?? current;
  return (
    <div className="text-field">
      <input
        type="text"
        spellCheck={false}
        autoComplete="off"
        aria-label={identifierLabel(propertyName)}
        aria-invalid={refusal !== null}
        className={field.entry !== null && text !== current ? 'changed' : ''}
        readOnly={!takesTyping(field)}
        value={text}
        onChange={(event) => {
          const typed = event.target.value;
          type(() => typed);
        }}
        onBlur={() => void commit()}
        onKeyDown={(event) => {
          if (event.key === 'Enter') void commit();
          else if (event.key === 'Escape') revert();
        }}
      />
      {refusal !== null && <FieldRefusalMessage message={refusal.message} />}
    </div>
  );
}
