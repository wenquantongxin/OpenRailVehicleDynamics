import { useState } from 'react';

import type { AttachmentSetting, ElementKindDescription, ElementReference, Refusal } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditValue } from '../session/vehicle_editing_session.ts';
import { AttachmentChoiceButtons } from './AttachmentChoiceButtons.tsx';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { attachmentChoices, bodyKindAsked, entriesCarriedOver, jointKindAsked, refusalAboutJoint, type AttachmentChoice } from './body_attachment_form_rules.ts';
import { blankEntries, fixedPropertyReading, formProperties, readCreationForm, refusedField, type FixedProperties, type FormEntries } from './element_creation_form_rules.ts';
import { ElementCreationPropertyInput } from './ElementCreationPropertyInput.tsx';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { identifierLabel } from './identifier_display_labels.ts';
import { JointFormFields } from './JointFormFields.tsx';
import { ReadOnlyPropertyValue } from './ReadOnlyPropertyValue.tsx';
import { valueKindLabel } from './SubjectPropertyRow.tsx';
import { UnreadableNumberMessage } from './UnreadableNumberMessage.tsx';

// The form that creates an element of a kind: its name and every property the
// kind lets be edited, each empty at first, except those the row's offer
// fixes, which are shown with their values and cannot be changed. Only what is
// filled in is sent beside the fixed properties, and the service's defaults
// stand for the rest. A kind whose elements are attached asks, after the
// name, how the new one is attached: free at first, or a joint of a kind,
// whose fields follow, its end on the element being the element itself; the
// properties only a free element has are asked for only while free is
// chosen. A refusal is shown at the property it names, of the element or of
// its joint, or at the form or the attachment when it names none, and what was
// entered stays for correction. Once the element is created the panel selects
// it.

export type CreateElement = (kind: string, name: string, properties: Readonly<Record<string, EditValue>>, attachment: AttachmentSetting | null) => Promise<Refusal | null>;

export function ElementCreationForm(props: {
  kind: ElementKindDescription;
  fixedProperties: FixedProperties;
  elements: readonly ElementReference[];
  /** The kinds' descriptions, whose titles name the kinds a reference may name and the joint kinds. */
  kinds: readonly ElementKindDescription[];
  onCreate: CreateElement;
  onCancel: () => void;
}) {
  const { kind, fixedProperties, kinds } = props;
  const description = kind.attachment;
  const [entries, setEntries] = useState<FormEntries>(() => blankEntries(kind, fixedProperties));
  const [choice, setChoice] = useState<AttachmentChoice>({ free: true });
  const [jointEntries, setJointEntries] = useState<FormEntries>({});
  const [unreadable, setUnreadable] = useState<{ property: string; at: number; ofJoint: boolean } | null>(null);
  const [refusal, setRefusal] = useState<Refusal | null>(null);
  const [sending, setSending] = useState(false);
  const askedKind = description === null ? kind : bodyKindAsked(kind, description, choice);
  const jointKind = description === null || 'free' in choice ? null : jointKindAsked(kinds.find((each) => each.kind === choice.jointKind)!, description, false);
  const aboutJoint = refusal !== null && refusalAboutJoint(refusal, jointKind?.kind ?? null);
  const refusedAt = refusal === null || aboutJoint ? null : refusedField(askedKind, refusal);
  const jointRefusedAt = refusal !== null && aboutJoint && jointKind !== null ? refusedField(jointKind, refusal) : null;

  const chooseAttachment = (chosen: AttachmentChoice): void => {
    setChoice(chosen);
    setUnreadable(null);
    if ('free' in chosen || description === null) return;
    const chosenKind = jointKindAsked(kinds.find((each) => each.kind === chosen.jointKind)!, description, false);
    setJointEntries((previous) => (Object.keys(previous).length > 0 ? entriesCarriedOver(chosenKind, previous) : blankEntries(chosenKind, {})));
  };

  const create = async (): Promise<void> => {
    const reading = readCreationForm(askedKind, fixedProperties, entries);
    if (reading.outcome === 'unreadable') {
      setUnreadable({ property: reading.property, at: reading.at, ofJoint: false });
      return;
    }
    let attachment: AttachmentSetting | null = description === null ? null : { free: true };
    if (jointKind !== null) {
      const jointReading = readCreationForm(jointKind, {}, jointEntries);
      if (jointReading.outcome === 'unreadable') {
        setUnreadable({ property: jointReading.property, at: jointReading.at, ofJoint: true });
        return;
      }
      attachment = { joint: { kind: jointKind.kind, name: jointReading.name, properties: jointReading.properties } };
    }
    setUnreadable(null);
    setSending(true);
    const refused = await props.onCreate(kind.kind, reading.name, reading.properties, attachment);
    // Once the element is created the panel shows it instead; a refusal keeps the form as it was filled in.
    setSending(false);
    setRefusal(refused);
  };

  const attachmentFields = description !== null && (
    <div className="property attachment-field">
      <div className="property-head">
        <span className="attachment-title">
          <BilingualTitleText title={description.title} />
        </span>
      </div>
      <AttachmentChoiceButtons description={description} kinds={kinds} choices={attachmentChoices(description, true, true)} chosen={choice} disabled={sending} onChoose={chooseAttachment} />
      {refusal !== null && aboutJoint && jointRefusedAt === null && <FieldRefusalMessage message={refusal.message} />}
      {jointKind !== null && (
        <JointFormFields
          jointKind={jointKind}
          description={description}
          entries={jointEntries}
          bodyEndChoices={null}
          otherEndChoices={null}
          elements={props.elements}
          kinds={kinds}
          disabled={sending}
          unreadable={unreadable?.ofJoint === true ? unreadable : null}
          refusal={jointRefusedAt === null || refusal === null ? null : { property: jointRefusedAt, message: refusal.message }}
          onEntry={(property, entry) => setJointEntries((previous) => ({ ...previous, [property]: entry }))}
        />
      )}
    </div>
  );

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
        {formProperties(askedKind).map((property) => (
          <div className="creation-field" key={property.name}>
            {property.name in fixedProperties ? (
              <div className="property read-only fixed" data-property={property.name}>
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
              <div className="property" data-property={property.name}>
                <div className="property-head">
                  <span className="property-name">{identifierLabel(property.name)}</span>
                  <span className="property-kind">{valueKindLabel(property)}</span>
                </div>
                <ElementCreationPropertyInput
                  property={property}
                  entry={entries[property.name]!}
                  elements={props.elements}
                  kinds={kinds}
                  disabled={sending}
                  unreadableAt={unreadable?.ofJoint === false && unreadable.property === property.name ? unreadable.at : null}
                  refused={refusedAt === property.name}
                  onEntry={(entry) => setEntries((previous) => ({ ...previous, [property.name]: entry }))}
                />
                {unreadable?.ofJoint === false && unreadable.property === property.name && <UnreadableNumberMessage />}
                {refusal !== null && refusedAt === property.name && <FieldRefusalMessage message={refusal.message} />}
              </div>
            )}
            {property.valueKind === 'element_name' && attachmentFields}
          </div>
        ))}
      </div>
      <div className="form-actions">
        {refusal !== null && !aboutJoint && refusedAt === null && (
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
