import { subjectElement, type BilingualTitle, type ModelTreeNode } from '../service/vehicle_modeling_service_protocol.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { WrappableElementName } from './WrappableElementName.tsx';

// The head of the property panel while it creates an element: the row whose
// "+" was used, by its title or its element's name, and the kind being
// created, or the request to choose one.

export function ElementCreationHead(props: { fromRow: ModelTreeNode | null; kindTitle: BilingualTitle | null }) {
  const { fromRow } = props;
  const fromElement = subjectElement(fromRow?.subject ?? null);
  return (
    <div className="selection-head creation-head">
      <div className="selection-title">
        <span className="selection-kind">
          New element <span className="zh">新建元素</span>
          {fromRow?.title != null && (
            <span className="creation-row">
              <BilingualTitleText title={fromRow.title} />
            </span>
          )}
          {fromElement !== null && (
            <span className="creation-row element-name">
              <WrappableElementName name={fromElement.name} />
            </span>
          )}
        </span>
        <span className="creation-kind">
          {props.kindTitle !== null ? (
            <BilingualTitleText title={props.kindTitle} />
          ) : (
            <>
              Choose what to create <span className="zh">选择新建的类型</span>
            </>
          )}
        </span>
      </div>
    </div>
  );
}
