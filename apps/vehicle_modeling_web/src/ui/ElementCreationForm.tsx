import { useState } from 'react';

import { curveValueKind, type ElementKindDescription, type ElementReference, type PropertyDescription, type Refusal } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditValue } from '../session/vehicle_editing_session.ts';
import { BooleanCheckbox } from './BooleanCheckbox.tsx';
import { withCellText, withRowAdded, withRowRemoved } from './editable_curve_table_rules.ts';
import { EditableCurveTable } from './EditableCurveTable.tsx';
import { blankEntries, formProperties, readCreationForm, refusedField, type FormEntries, type FormEntry } from './element_creation_form_rules.ts';
import { valueKindLabel } from './ElementPropertyRow.tsx';
import { ElementReferencePicker } from './ElementReferencePicker.tsx';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { identifierLabel } from './identifier_display_labels.ts';
import { NumberTextInputs } from './NumberTextInputs.tsx';
import { TokenChoiceSelect } from './TokenChoiceSelect.tsx';
import { UnreadableNumberMessage } from './UnreadableNumberMessage.tsx';

// The form that creates an element of a kind: its name and every property the
// kind lets be edited, each empty at first. Only what is filled in is sent,
// and the service's defaults stand for the rest. A refusal is shown at the
// property it names, or at the form when it names none, and what was entered
// stays for correction. Once the element is created the panel selects it.

export type CreateElement = (kind: string, name: string, properties: Readonly<Record<string, EditValue>>) => Promise<Refusal | null>;

function FormInput(props: { property: PropertyDescription; entry: FormEntry; elements: readonly ElementReference[]; disabled: boolean; unreadableAt: number | null; refused: boolean; onEntry: (entry: FormEntry) => void }) {
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

export function ElementCreationForm(props: { kind: ElementKindDescription; elements: readonly ElementReference[]; onCreate: CreateElement; onCancel: () => void }) {
  const { kind } = props;
  const [entries, setEntries] = useState<FormEntries>(() => blankEntries(kind));
  const [unreadable, setUnreadable] = useState<{ property: string; at: number } | null>(null);
  const [refusal, setRefusal] = useState<Refusal | null>(null);
  const [sending, setSending] = useState(false);
  const refusedAt = refusal === null ? null : refusedField(kind, refusal);

  const create = async (): Promise<void> => {
    const reading = readCreationForm(kind, entries);
    if (reading.outcome === 'unreadable') {
      setUnreadable({ property: reading.property, at: reading.at });
      return;
    }
    setUnreadable(null);
    setSending(true);
    const refused = await props.onCreate(kind.kind, reading.name, reading.properties);
    // Once the element is created the panel shows it instead; a refusal keeps the form as it was filled in.
    setSending(false);
    setRefusal(refused);
  };

  return (
    <form
      className="creation-form"
      onSubmit={(event) => {
        event.preventDefault();
        if (!sending) void create();
      }}
    >
      <div className="property-list">
        <p className="form-note">
          Only what is filled in is sent; the service&rsquo;s defaults stand for the rest.
          <span className="zh">只发送已填写的项，其余取服务的默认值。</span>
        </p>
        {formProperties(kind).map((property) => (
          <div className="property" data-property={property.name} key={property.name}>
            <div className="property-head">
              <span className="property-name">{identifierLabel(property.name)}</span>
              <span className="property-kind">{valueKindLabel(property)}</span>
            </div>
            <FormInput
              property={property}
              entry={entries[property.name]!}
              elements={props.elements}
              disabled={sending}
              unreadableAt={unreadable?.property === property.name ? unreadable.at : null}
              refused={refusedAt === property.name}
              onEntry={(entry) => setEntries((previous) => ({ ...previous, [property.name]: entry }))}
            />
            {unreadable?.property === property.name && <UnreadableNumberMessage />}
            {refusal !== null && refusedAt === property.name && <FieldRefusalMessage message={refusal.message} />}
          </div>
        ))}
      </div>
      <div className="form-actions">
        {refusal !== null && refusedAt === null && (
          <p className="message refused" role="alert">
            <b>
              Not created <span className="zh">未能创建</span>
            </b>
            <span>{refusal.message}</span>
          </p>
        )}
        <div className="form-buttons">
          <button type="submit" className="chip primary" disabled={sending}>
            <span className="en">Create</span>
            <span className="zh">创建</span>
          </button>
          <button type="button" className="chip" disabled={sending} onClick={props.onCancel}>
            <span className="en">Cancel</span>
            <span className="zh">取消</span>
          </button>
        </div>
      </div>
    </form>
  );
}
