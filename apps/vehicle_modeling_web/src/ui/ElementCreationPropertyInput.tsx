import { curveValueKind, type ElementReference, type PropertyDescription } from '../service/vehicle_modeling_service_protocol.ts';
import { BooleanCheckbox } from './BooleanCheckbox.tsx';
import { withCellText, withRowAdded, withRowRemoved } from './editable_curve_table_rules.ts';
import { EditableCurveTable } from './EditableCurveTable.tsx';
import type { FormEntry } from './element_creation_form_rules.ts';
import { ElementReferencePicker } from './ElementReferencePicker.tsx';
import { identifierLabel } from './identifier_display_labels.ts';
import { NumberTextInputs } from './NumberTextInputs.tsx';
import { TokenChoiceSelect } from './TokenChoiceSelect.tsx';

/** The input of one property in the creation form, by the property's value kind; it holds what is entered, unsent. */
export function ElementCreationPropertyInput(props: { property: PropertyDescription; entry: FormEntry; elements: readonly ElementReference[]; disabled: boolean; unreadableAt: number | null; refused: boolean; onEntry: (entry: FormEntry) => void }) {
  const { property, entry, disabled, refused, onEntry } = props;
  switch (entry.valueKind) {
    case 'element_name':
      return (
        <div className="text-field">
          <input
            type="text"
            spellCheck={false}
            autoComplete="off"
            aria-label={identifierLabel(property.name)}
            aria-invalid={refused}
            readOnly={disabled}
            value={entry.text}
            onChange={(event) => onEntry({ ...entry, text: event.target.value })}
          />
        </div>
      );
    case 'number':
    case 'vector3':
      return (
        <div className={`number-fields count-${entry.texts.length}`}>
          <NumberTextInputs
            propertyName={property.name}
            texts={entry.texts}
            changed={entry.texts.map(() => false)}
            unreadableAt={props.unreadableAt}
            refused={refused}
            readOnly={disabled}
            onText={(position, text) => onEntry({ ...entry, texts: entry.texts.map((each, index) => (index === position ? text : each)) })}
          />
        </div>
      );
    case 'element_reference':
      return (
        <ElementReferencePicker
          description={entry.description}
          elements={props.elements}
          value={entry.chosen}
          disabled={disabled}
          refused={refused}
          onChoose={(name) => onEntry({ ...entry, chosen: name })}
        />
      );
    case 'token':
      return (
        <TokenChoiceSelect
          propertyName={property.name}
          choices={entry.choices}
          value={entry.chosen}
          disabled={disabled}
          refused={refused}
          onChoose={(token) => onEntry({ ...entry, chosen: token })}
        />
      );
    case 'boolean':
      return <BooleanCheckbox propertyName={property.name} value={entry.chosen} disabled={disabled} refused={refused} onChoose={(value) => onEntry({ ...entry, chosen: value })} />;
    case curveValueKind:
      return (
        <EditableCurveTable
          propertyName={property.name}
          rowFields={entry.rowFields}
          entry={entry.rows}
          changed={() => false}
          unreadableAt={props.unreadableAt}
          refused={refused}
          readOnly={disabled}
          onCellText={(row, column, text) => onEntry({ ...entry, rows: withCellText(entry.rows, row, column, text) })}
          onAddRow={() => onEntry({ ...entry, rows: withRowAdded(entry.rows, entry.rowFields) })}
          onRemoveRow={(row) => onEntry({ ...entry, rows: withRowRemoved(entry.rows, row) })}
        />
      );
  }
}
