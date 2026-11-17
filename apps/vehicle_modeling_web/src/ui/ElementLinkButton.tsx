import type { BilingualTitle, ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

/** An element named in a message, by its kind's title and its name; choosing it selects the element. */
export function ElementLinkButton({ element, kindTitle, onSelect }: { element: ElementReference; kindTitle: BilingualTitle; onSelect: (element: ElementReference) => void }) {
  return (
    <button type="button" className="element-link" onClick={() => onSelect(element)} title={`${kindTitle.en} · ${element.name}`}>
      <span className="link-kind">{kindTitle.en}</span> <WrappableElementName name={element.name} />
    </button>
  );
}
