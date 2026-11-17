import { useEffect, useRef } from 'react';

import { sameElement, sameSubject, subjectElement, subjectKey, type ElementKindDescription, type ElementReference, type PropertyReading, type PropertySubject } from '../service/vehicle_modeling_service_protocol.ts';
import type { EditRefusal, SetProperty } from '../session/vehicle_editing_session.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { propertySections } from './property_section_rules.ts';
import { SubjectPropertyRow } from './SubjectPropertyRow.tsx';

// The selected subject's properties, in the service's order and in the
// sections their descriptions state, each in the editor of its value kind. A
// refused edit is shown at its property. While an element's rename is on its
// way its other properties take no input: an edit made meanwhile would be sent
// under the name the rename replaces.

/** Asks the list to focus one property's field once it shows that subject. */
export interface PropertyFocusRequest {
  subject: PropertySubject;
  property: string;
  serial: number;
}

export function SubjectPropertyList(props: {
  selection: PropertySubject;
  properties: PropertyReading[];
  /** The outline's elements, which a reference may name. */
  elements: readonly ElementReference[];
  /** The kinds' descriptions, whose titles name the kinds a reference may name. */
  kinds: readonly ElementKindDescription[];
  editRefusal: EditRefusal | null;
  /** The element whose rename is on its way, if any. */
  renaming: ElementReference | null;
  focusRequest: PropertyFocusRequest | null;
  onCommit: SetProperty;
  onDismissRefusal: () => void;
}) {
  const { selection, editRefusal, focusRequest } = props;
  const beingRenamed = sameElement(props.renaming, subjectElement(selection));
  const body = useRef<HTMLDivElement>(null);
  const focusedSerial = useRef(0);

  useEffect(() => {
    if (focusRequest === null || focusRequest.serial === focusedSerial.current || !sameSubject(focusRequest.subject, selection)) return;
    const input = body.current?.querySelector<HTMLInputElement | HTMLSelectElement>(`[data-property="${CSS.escape(focusRequest.property)}"] :is(input, select)`);
    if (input === null || input === undefined) return;
    focusedSerial.current = focusRequest.serial;
    input.focus();
    if (input instanceof HTMLInputElement) input.select();
  }, [focusRequest, selection, props.properties]);

  return (
    <div className="property-list" ref={body}>
      {propertySections(props.properties).map((section, index) => (
        <section className="property-section" key={index}>
          {section.title !== null && (
            <h3 className="property-section-title">
              <BilingualTitleText title={section.title} />
            </h3>
          )}
          {section.properties.map((property) => (
            <SubjectPropertyRow
              key={`${subjectKey(selection)}/${property.name}`}
              subject={selection}
              property={property}
              elements={props.elements}
              kinds={props.kinds}
              refusal={editRefusal !== null && sameSubject(editRefusal.subject, selection) && editRefusal.property === property.name ? editRefusal : null}
              frozen={beingRenamed && property.valueKind !== 'element_name'}
              onCommit={props.onCommit}
              onDismissRefusal={props.onDismissRefusal}
            />
          ))}
        </section>
      ))}
    </div>
  );
}
