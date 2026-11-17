import type { Attachment, AttachmentDescription, ElementKindDescription, ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { ElementLinkButton } from './ElementLinkButton.tsx';
import { elementKindTitle } from './find_element_kind_title.ts';

// What a hanging body hangs on, under the line saying how it is held: the
// joint, and the frame at the joint's other end, each in a row of its own
// under the title the service gives it, each a link that selects the element.
// A long name breaks after its underscores and dots.

export function AttachmentJointAndHeldAtLinks(props: {
  description: AttachmentDescription;
  attachment: Attachment;
  kinds: readonly ElementKindDescription[];
  onSelectElement: (element: ElementReference) => void;
}) {
  const rows = [
    { title: props.description.jointTitle, element: props.attachment.joint },
    { title: props.description.heldAtTitle, element: props.attachment.heldAt },
  ].filter((row): row is { title: typeof row.title; element: ElementReference } => row.element !== null);
  if (rows.length === 0) return null;
  return (
    <dl className="attachment-links">
      {rows.map(({ title, element }) => (
        <div className="attachment-link-row" key={title.en}>
          <dt>
            <BilingualTitleText title={title} />
          </dt>
          <dd>
            <ElementLinkButton element={element} kindTitle={elementKindTitle(props.kinds, element.kind)} onSelect={props.onSelectElement} />
          </dd>
        </div>
      ))}
    </dl>
  );
}
