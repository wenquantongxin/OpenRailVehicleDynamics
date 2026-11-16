import { useEffect, useRef } from 'react';

import { elementKey, sameElement, type ElementReference, type PropertyReading } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';
import { ElementPropertyRow } from './ElementPropertyRow.tsx';

// The selected element's properties, in the service's order, each in the
// editor of its value kind. A refused edit is shown at its property. While the
// element's rename is on its way its other properties take no input: an edit
// made meanwhile would be sent under the name the rename replaces.

/** Asks the list to focus one property's field once it shows that element. */
export interface PropertyFocusRequest {
  element: ElementReference;
  property: string;
  serial: number;
}

export function ElementPropertyList(props: {
  selection: ElementReference;
  properties: PropertyReading[];
  /** The outline's elements, which a reference may name. */
  elements: readonly ElementReference[];
  editRefusal: EditRefusal | null;
  /** The element whose rename is on its way, if any. */
  renaming: ElementReference | null;
  focusRequest: PropertyFocusRequest | null;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
}) {
  const { selection, editRefusal, focusRequest } = props;
  const beingRenamed = sameElement(props.renaming, selection);
  const body = useRef<HTMLDivElement>(null);
  const focusedSerial = useRef(0);

  useEffect(() => {
    if (focusRequest === null || focusRequest.serial === focusedSerial.current || !sameElement(focusRequest.element, selection)) return;
    const input = body.current?.querySelector<HTMLInputElement | HTMLSelectElement>(`[data-property="${CSS.escape(focusRequest.property)}"] :is(input, select)`);
    if (input === null || input === undefined) return;
    focusedSerial.current = focusRequest.serial;
    input.focus();
    if (input instanceof HTMLInputElement) input.select();
  }, [focusRequest, selection, props.properties]);

  return (
    <div className="property-list" ref={body}>
      {props.properties.map((property) => (
        <ElementPropertyRow
          key={`${elementKey(selection)}/${property.name}`}
          element={selection}
          property={property}
          elements={props.elements}
          refusal={editRefusal !== null && sameElement(editRefusal.element, selection) && editRefusal.property === property.name ? editRefusal : null}
          frozen={beingRenamed && property.valueKind !== 'element_name'}
          onCommit={props.onCommit}
          onDismissRefusal={props.onDismissRefusal}
        />
      ))}
    </div>
  );
}
