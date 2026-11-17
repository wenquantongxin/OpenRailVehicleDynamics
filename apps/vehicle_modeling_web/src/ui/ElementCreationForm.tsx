import { useState } from 'react';

import type { ElementKindDescription, ElementReference, Refusal } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditValue } from '../session/vehicle_editing_session.ts';
import { blankEntries, fixedPropertyReading, formProperties, readCreationForm, refusedField, type FixedProperties, type FormEntries } from './element_creation_form_rules.ts';
import { ElementCreationPropertyInput } from './ElementCreationPropertyInput.tsx';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { identifierLabel } from './identifier_display_labels.ts';
import { ReadOnlyPropertyValue } from './ReadOnlyPropertyValue.tsx';
import { valueKindLabel } from './SubjectPropertyRow.tsx';
import { UnreadableNumberMessage } from './UnreadableNumberMessage.tsx';

// The form that creates an element of a kind: its name and every property the
// kind lets be edited, each empty at first, except those the row's offer
// fixes, which are shown with their values and cannot be changed. Only what is
// filled in is sent beside the fixed properties, and the service's defaults
// stand for the rest. A refusal is shown at the property it names, or at the
// form when it names none, and what was entered stays for correction. Once
// the element is created the panel selects it.

export type CreateElement = (kind: string, name: string, properties: Readonly<Record<string, EditValue>>) => Promise<Refusal | null>;

export function ElementCreationForm(props: {
  kind: ElementKindDescription;
  fixedProperties: FixedProperties;
  elements: readonly ElementReference[];
  /** The kinds' descriptions, whose titles name the kinds a reference may name. */
  kinds: readonly ElementKindDescription[];
  onCreate: CreateElement;
  onCancel: () => void;
}) {
  const { kind, fixedProperties } = props;
  const [entries, setEntries] = useState<FormEntries>(() => blankEntries(kind, fixedProperties));
  const [unreadable, setUnreadable] = useState<{ property: string; at: number } | null>(null);
  const [refusal, setRefusal] = useState<Refusal | null>(null);
  const [sending, setSending] = useState(false);
  const refusedAt = refusal === null ? null : refusedField(kind, refusal);

  const create = async (): Promise<void> => {
    const reading = readCreationForm(kind, fixedProperties, entries);
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
        {formProperties(kind).map((property) =>
          property.name in fixedProperties ? (
            <div className="property read-only fixed" data-property={property.name} key={property.name}>
              <div className="property-head">
                <span className="property-name">{identifierLabel(property.name)}</span>
                <span className="property-kind">{valueKindLabel(property)}</span>
              </div>
              <ReadOnlyPropertyValue property={fixedPropertyReading(property, fixedProperties)} />
              <p className="fixed-note">
                Fixed by the row it is created under <span className="zh">由所在行确定</span>
              </p>
              {refusal !== null && refusedAt === property.name && <FieldRefusalMessage message={refusal.message} />}
            </div>
          ) : (
            <div className="property" data-property={property.name} key={property.name}>
              <div className="property-head">
                <span className="property-name">{identifierLabel(property.name)}</span>
                <span className="property-kind">{valueKindLabel(property)}</span>
              </div>
              <ElementCreationPropertyInput
                property={property}
                entry={entries[property.name]!}
                elements={props.elements}
                kinds={props.kinds}
                disabled={sending}
                unreadableAt={unreadable?.property === property.name ? unreadable.at : null}
                refused={refusedAt === property.name}
                onEntry={(entry) => setEntries((previous) => ({ ...previous, [property.name]: entry }))}
              />
              {unreadable?.property === property.name && <UnreadableNumberMessage />}
              {refusal !== null && refusedAt === property.name && <FieldRefusalMessage message={refusal.message} />}
            </div>
          ),
        )}
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
