import { elementKey, sameElement, type ElementKindDescription, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { elementKindTitle } from './find_element_kind_title.ts';
import { kindTitleInASentence } from './kind_title_in_a_sentence.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

/** What a removal takes besides the element: how many elements refer to it, each of them listed, and that undo brings them back. */
export function ElementsRemovedWithElementList(props: { element: ElementReference; removedElements: readonly ElementReference[]; kinds: readonly ElementKindDescription[] }) {
  const { element, removedElements, kinds } = props;
  const others = removedElements.length - 1;
  const kindTitle = elementKindTitle(kinds, element.kind);
  return (
    <>
      <p className="removal-question">
        Removing this {kindTitleInASentence(kindTitle)} also removes the {others} {others === 1 ? 'element' : 'elements'} that refer to it, directly or
        through one another.
        <span className="zh">删除该{kindTitle.zh}将一并删除直接或间接引用它的 {others} 个元素。</span>
      </p>
      <ul className="removed-list">
        {removedElements.map((each) => (
          <li key={elementKey(each)} className={sameElement(each, element) ? 'removed-itself' : ''}>
            <span className="removed-kind">
              <BilingualTitleText title={elementKindTitle(kinds, each.kind)} />
            </span>
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
