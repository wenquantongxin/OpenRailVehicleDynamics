import type { Attachment, AttachmentDescription, ElementKindDescription, ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { AttachmentJointAndHeldAtLinks } from './AttachmentJointAndHeldAtLinks.tsx';
import { BilingualTitleText } from './BilingualTitleText.tsx';

// How the selected body is attached, shown before its property sections: the
// short line saying how it is held, the line saying what is the matter when
// something is, the joint it hangs on and the frame it is held at as links
// that select them, and Change… when it can become free or take a joint.

export function BodyAttachmentSummary(props: {
  description: AttachmentDescription;
  attachment: Attachment;
  kinds: readonly ElementKindDescription[];
  onSelectElement: (element: ElementReference) => void;
  onChange: () => void;
}) {
  const { attachment } = props;
  return (
    <section className="property-section attachment-section">
      <h3 className="property-section-title">
        <BilingualTitleText title={props.description.title} />
      </h3>
      <div className="attachment-body">
        <p className="attachment-summary">
          <BilingualTitleText title={attachment.summary} />
        </p>
        {attachment.note !== null && (
          <p className="attachment-note">
            <BilingualTitleText title={attachment.note} />
          </p>
        )}
        <AttachmentJointAndHeldAtLinks description={props.description} attachment={attachment} kinds={props.kinds} onSelectElement={props.onSelectElement} />
        {(attachment.canBecomeFree || attachment.canTakeJoint) && (
          <div className="attachment-actions">
            <button type="button" className="chip attachment-change-button" onClick={props.onChange}>
              <span className="en">Change…</span>
              <span className="zh">更改…</span>
            </button>
          </div>
        )}
      </div>
    </section>
  );
}
