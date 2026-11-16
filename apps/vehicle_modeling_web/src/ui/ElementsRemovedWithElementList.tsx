import { elementKey, sameElement, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { identifierLabel } from './identifier_display_labels.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

/** What a removal takes besides the element: how many elements refer to it, each of them listed, and that undo brings them back. */
export function ElementsRemovedWithElementList({ element, removedElements }: { element: ElementReference; removedElements: readonly ElementReference[] }) {
  const others = removedElements.length - 1;
  return (
    <>
      <p className="removal-question">
        Removing this {identifierLabel(element.kind)} also removes the {others} {others === 1 ? 'element' : 'elements'} that refer to it, directly or through
        one another.
        <span className="zh">删除该元素将一并删除直接或间接引用它的 {others} 个元素。</span>
      </p>
      <ul className="removed-list">
        {removedElements.map((each) => (
          <li key={elementKey(each)} className={sameElement(each, element) ? 'removed-itself' : ''}>
            <span className="removed-kind">{identifierLabel(each.kind)}</span>
            <span className="removed-name">
              <WrappableElementName name={each.name} />
            </span>
          </li>
        ))}
      </ul>
      <p className="removal-note">
        Undo brings them all back. <span className="zh">撤销可全部恢复。</span>
      </p>
    </>
  );
}
