import { useState } from 'react';

import type { AttachmentDescription, AttachmentSetting, ElementKindDescription, ElementReference, Refusal } from '../service/vehicle_modeling_service_protocol.ts';
import type { BodyToAttach } from '../session/vehicle_editing_session.ts';
import { AttachmentChoiceButtons } from './AttachmentChoiceButtons.tsx';
import { AttachmentJointAndHeldAtLinks } from './AttachmentJointAndHeldAtLinks.tsx';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import {
  attachmentChoices,
  entriesCarriedOver,
  entriesStartingFrom,
  refusalAboutJoint,
  withReferenceChosen,
  type AttachmentChoice,
} from './body_attachment_form_rules.ts';
import { CurrentJointRemovalOrReplacementLine } from './CurrentJointRemovalOrReplacementLine.tsx';
import { readCreationForm, refusedField, type FormEntries } from './element_creation_form_rules.ts';
import { FieldRefusalMessage } from './FieldRefusalMessage.tsx';
import { JointFormFields } from './JointFormFields.tsx';

// A change of how a body is attached: what it is now, with the joint it hangs
// on and the frame it is held at as links, then what its attachment allows,
// free and each joint kind, one of which is chosen. A joint kind asks for the
// joint: its end on the body among the body and its frames, the body first,
// its other end among the frames the body can be held at, and the kind's other
// properties, all starting from the joint the body hangs on wherever the kind
// has a property of the same name and value kind. Apply sends the change as
// one edit; a refusal about the joint is shown at its field, any other with
// the attachment.

export function AttachmentChangeForm(props: {
  description: AttachmentDescription;
  kinds: readonly ElementKindDescription[];
  body: BodyToAttach;
  /** Whether becoming free is offered, where the attachment allows it. */
  offersFree: boolean;
  /** The outline's elements, which a reference other than the joint's ends may name. */
  elements: readonly ElementReference[];
  refusal: Refusal | null;
  onSelectElement: (element: ElementReference) => void;
  onApply: (setting: AttachmentSetting) => Promise<boolean>;
  onCancel: () => void;
}) {
  const { description, kinds, body } = props;
  const { attachment } = body;
  const [chosen, setChosen] = useState<AttachmentChoice | null>(null);
  const [entries, setEntries] = useState<FormEntries>({});
  const [unreadable, setUnreadable] = useState<{ property: string; at: number } | null>(null);
  const [sending, setSending] = useState(false);
  const [refusalSetAside, setRefusalSetAside] = useState<Refusal | null>(null);
  const choices = attachmentChoices(description, props.offersFree && attachment.canBecomeFree, attachment.canTakeJoint);
  const jointKind = chosen !== null && 'jointKind' in chosen ? (kinds.find((kind) => kind.kind === chosen.jointKind) ?? null) : null;
  const refusal = props.refusal !== refusalSetAside ? props.refusal : null;
  const aboutJoint = refusal !== null && refusalAboutJoint(refusal, jointKind?.kind ?? null);
  const fieldRefused = aboutJoint && jointKind !== null ? refusedField(jointKind, refusal) : null;

  const choose = (choice: AttachmentChoice): void => {
    setChosen(choice);
    setUnreadable(null);
    setRefusalSetAside(props.refusal);
    if ('free' in choice) return;
    const kind = kinds.find((each) => each.kind === choice.jointKind)!;
    const start = Object.keys(entries).length > 0 ? entriesCarriedOver(kind, entries) : entriesStartingFrom(kind, body.jointProperties);
    setEntries(withReferenceChosen(start, description.bodyEndProperty, body.element.name));
  };

  const apply = async (): Promise<void> => {
    if (chosen === null) return;
    let setting: AttachmentSetting = { free: true };
    if (jointKind !== null) {
      const reading = readCreationForm(jointKind, {}, entries);
      if (reading.outcome === 'unreadable') {
        setUnreadable({ property: reading.property, at: reading.at });
        return;
      }
      setting = { joint: { kind: jointKind.kind, name: reading.name, properties: reading.properties } };
    }
    setUnreadable(null);
    setSending(true);
    // Accepted, the change is over and the form leaves; refused, it stays as it was filled in.
    await props.onApply(setting);
    setSending(false);
  };

  return (
    <form
      className="attachment-change"
      onSubmit={(event) => {
        event.preventDefault();
        if (!sending) void apply();
      }}
    >
      <div className="property-list">
        <section className="property-section">
          <h3 className="property-section-title">
            <BilingualTitleText title={description.title} />
          </h3>
          <div className="attachment-body">
            <p className="attachment-now">
              <span className="attachment-label">
                Now <span className="zh">现在</span>
              </span>
              <span className="attachment-summary">
                <BilingualTitleText title={attachment.summary} />
              </span>
            </p>
            <AttachmentJointAndHeldAtLinks description={description} attachment={attachment} kinds={kinds} onSelectElement={props.onSelectElement} />
            <p className="attachment-label">
              Change to <span className="zh">改为</span>
            </p>
            <AttachmentChoiceButtons description={description} kinds={kinds} choices={choices} chosen={chosen} disabled={sending} onChoose={choose} />
            {chosen !== null && attachment.joint !== null && <CurrentJointRemovalOrReplacementLine joint={attachment.joint} removed={'free' in chosen} />}
            {refusal !== null && fieldRefused === null && <FieldRefusalMessage message={refusal.message} />}
          </div>
        </section>
        {jointKind !== null && (
          <section className="property-section">
            <h3 className="property-section-title">
              <BilingualTitleText title={jointKind.title} />
            </h3>
            <JointFormFields
              jointKind={jointKind}
              description={description}
              entries={entries}
              bodyEndChoices={attachment.framesOnBody}
              otherEndChoices={attachment.framesItCanBeHeldAt}
              elements={props.elements}
              kinds={kinds}
              disabled={sending}
              unreadable={unreadable}
              refusal={fieldRefused === null || refusal === null ? null : { property: fieldRefused, message: refusal.message }}
              onEntry={(property, entry) => setEntries((previous) => ({ ...previous, [property]: entry }))}
            />
          </section>
        )}
      </div>
      <div className="form-actions">
        <div className="form-buttons">
          <button type="submit" className="chip primary" disabled={sending || chosen === null}>
            <span className="en">Apply</span>
            <span className="zh">应用</span>
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
