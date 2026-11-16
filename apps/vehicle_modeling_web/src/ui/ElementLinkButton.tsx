import type { ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { identifierLabel } from './identifier_display_labels.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

/** An element named in a message, by kind and name; choosing it selects the element. */
export function ElementLinkButton({ element, onSelect }: { element: ElementReference; onSelect: (element: ElementReference) => void }) {
  return (
    <button type="button" className="element-link" onClick={() => onSelect(element)} title={`${identifierLabel(element.kind)} · ${element.name}`}>
      <span className="link-kind">{identifierLabel(element.kind)}</span> <WrappableElementName name={element.name} />
    </button>
  );
}
