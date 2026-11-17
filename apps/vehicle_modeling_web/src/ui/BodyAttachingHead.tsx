import type { AttachmentDescription, ElementReference, ModelTreeNode } from '../service/vehicle_modeling_service_protocol.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { WrappableElementName } from './WrappableElementName.tsx';

// The head of the property panel while a row's "+" attaches a body: the
// attachment's title and the row's, then the body chosen, or the request to
// choose one.

export function BodyAttachingHead(props: { description: AttachmentDescription; fromRow: ModelTreeNode | null; body: ElementReference | null }) {
  return (
    <div className="selection-head creation-head">
      <div className="selection-title">
        <span className="selection-kind">
          <BilingualTitleText title={props.description.title} />
          {props.fromRow?.title != null && (
            <span className="creation-row">
              <BilingualTitleText title={props.fromRow.title} />
            </span>
          )}
        </span>
        {props.body !== null ? (
          <span className="selection-name">
            <WrappableElementName name={props.body.name} />
          </span>
        ) : (
          <span className="creation-kind">
            Choose what to attach <span className="zh">选择挂接对象</span>
          </span>
        )}
      </div>
    </div>
  );
}
